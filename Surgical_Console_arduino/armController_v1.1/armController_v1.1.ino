#include <ros.h>
#include <std_msgs/UInt8.h>
#include <sensor_handler/sensorData.h>
#include <SPI.h>

#define LEFT

/*
 * Arduino Pin Connections
 * SPI MOSI                 Pin 11
 * SPI MISO                 Pin 12
 * SPI SCLK:                Pin 13
 * 
 * AMT22 Pin Connections
 * Vdd (5V):                Pin  1
 * SPI SCLK:                Pin  2
 * SPI MOSI:                Pin  3
 * GND:                     Pin  4
 * SPI MISO:                Pin  5
 * SPI Chip Select:         Pin  6
 * 
 */

/* SPI pins */
#define SPI_MOSI        11
#define SPI_MISO        12
#define SPI_SCLK        13
#define P_ENC_MIN       8

/* Encoder chip select (CS) pins */
#define P_ENC_0 8
#define P_ENC_1 7
#define P_ENC_2 6
#define P_ENC_3 5
#define P_ENC_4 4
#define P_ENC_5 3
#define P_ENC_6 2
#define MAX_ENC         7  //how many enc connected

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* Additional function pin */
#define P_ACTIVITY_LED 10
#define P_GRIPPER      A0
/* Ideal ros loop rate */
#define LOOP_FREQ 1000

uint8_t arm_inst; 

ros::NodeHandle nh;

void arm_inst_cb(const std_msgs::UInt8& msg) {
  arm_inst = msg.data;
  
}

sensor_handler::sensorData arm_data;
#ifdef LEFT
  ros::Subscriber<std_msgs::UInt8> arm_inst_sub("l_arm_instruction", &arm_inst_cb);
  ros::Publisher arm_data_pub("l_arm_data", &arm_data);
#else
  ros::Subscriber<std_msgs::UInt8> arm_inst_sub("r_arm_instruction", &arm_inst_cb);
  ros::Publisher arm_data_pub("r_arm_data", &arm_data);
#endif

void setup()
{
  /* Create array to store encoder cs pin */
  uint8_t p_enc[MAX_ENC];
  for (int i=0; i<MAX_ENC; i++) {
    p_enc[i] = P_ENC_MIN-i;
  }
  
  /* Set the modes for the SPI IO */
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  for (int i=0; i<MAX_ENC; i++) {
    pinMode(p_enc[i], OUTPUT);
  }
  
  /* Get the CS line high which is the default inactive state */
  for (int i=0; i<MAX_ENC; i++) {
    digitalWrite(p_enc[i], HIGH);
  }

  //set the clockrate. Uno clock rate is 16Mhz, divider of 32 gives 500 kHz.
  //500 kHz is a good speed for our test environment
  //SPI.setClockDivider(SPI_CLOCK_DIV2);   // 8 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV8);   // 2 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV64);  // 250 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV128); // 125 kHz

  //start SPI bus
  SPI.begin();
  
  /* Initialize activity-LED pin */
  pinMode(P_ACTIVITY_LED, OUTPUT);
  digitalWrite(P_ACTIVITY_LED, HIGH);

  /* ROS initialization */
  nh.initNode();
  nh.advertise(arm_data_pub);
  nh.subscribe(arm_inst_sub);
}

void loop()
{
  /* Create array to store encoder CS pin */
  uint8_t p_enc[MAX_ENC];
  uint16_t encPos[MAX_ENC];
  for (int i=0; i<MAX_ENC; i++) {
    p_enc[i] = P_ENC_MIN-i;
  }

  /* Create a 16 bit variable to hold the encoders position */
  uint16_t encoderPosition;
  /* Create a variable to attemp another read if error occurred */
  uint8_t attempts;


  while (1) {
    /* Mode selection section */
    if ((arm_inst&1) == 1) {
      /* Enter calibration mode */
      for (int i=0; i<MAX_ENC; i++) {
        if (((arm_inst>>(i+1))&1) == 1) {
          setZeroSPI(p_enc[i]);
        }
      }
      arm_inst = 0;
    }
    else {
      /* Enter default data reading mode */
      for (int i=0; i<MAX_ENC; i++) {
        //set attemps counter at 0 so we can try again if we get bad position    
        attempts = 0;
    
        //this function gets the encoder position and returns it as a uint16_t
        //send the function either res12 or res14 for your encoders resolution
        encoderPosition = getPositionSPI(p_enc[i], RES14); 
    
        //if the position returned was 0xFFFF we know that there was an error calculating the checksum
        //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
        while (encoderPosition == 0xFFFF && ++attempts < 3)
        {
          encoderPosition = getPositionSPI(p_enc[i], RES14); //try again
        }
        arm_data.enc_position[i] = encoderPosition;
      }
      uint16_t gripper_val = analogRead(P_GRIPPER);
      arm_data.gripper_data = gripper_val;
    }
    
    /* ROS data publish section */
    arm_data_pub.publish(&arm_data);
    nh.spinOnce();
    digitalWrite(P_ACTIVITY_LED, HIGH-(digitalRead(P_ACTIVITY_LED))); /* Blink LED for sign of activity */
    delay(1000/LOOP_FREQ);
  }
}

/*
 * This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4. 
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 */
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

/*
 * This function does the SPI transfer. sendByte is the byte to transmit. 
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.  
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
 * This function takes the pin number of the desired device as an input
 */
void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_RESET, encoder, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}
