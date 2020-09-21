// *************************
// *        UI node        *
// *      TA192001004      *
// *************************

/* PINS:
    LCD:
        SCL in pin A5      / SCL PIN IN MEGA
        SDA in pin A4      / SDA PIN IN MEGA
    Sel BUTTON:
        PIN 10 (active high)     
    Limit Switch:
        PIN 11 (repo)       PIN 2
        PIN 12 (Start stop)  PIN3 (LSW2)
    FSR:
    1. ANALOG 0
    2. ANALOG 1 
    Encoder:
        CLK: 8              PIN 53 on mega2560 [PB0]
        DT 9                PIN 52 on mega2560 [PB1]
*/
    // CONSTANTS
// #define reposPin              11           //LSW_1 for repositioning
// #define startStopPin          12           //LSW_2 for start and stop
#define reposPin              2           //LSW_1 for repositioning 
#define startStopPin          3           //LSW_2 for start and stop
#define selPin                10           // push button for menu. 
#define unit                  0.01f;       //Incremental unit
#define calib_time            5000         //Calibration timer
#define reset_time            20000        //Menu idle handler
#define gripper_timer         3000         //Timer for gripper calibration
#define interval              500          //blink time
#define user1                 "dr.A"
#define user2                 "dr.B"
#define user3                 "dr.C"
#define HOME                    0
#define TITLE                   99
#define MAIN                     1
#define load_menu                2
#define save_menu                3
#define multiplier_menu          4
#define reset_menu               5
#define encoder_menu             6
#define gripper_menu             7
#define power_menu               8

#define LOOP_FREQ             1000

// #define LCD_I2C

// LIBRARY
    //General
#ifdef LCD_I2C
  #include <LiquidCrystal_I2C.h>
  #include <Wire.h>
#else
  #include <LiquidCrystal.h>
#endif

    //RosSerial
#include <ros.h>
#include <data_handler/ui_msgs.h>
#include <data_handler/Profile.h>
#include <std_msgs/UInt8.h>

    // PROCESS VARIABLES
uint8_t     repoReading;                    
uint8_t     startStopReading; 
uint8_t     rst =                    1;
uint8_t     butReading;
bool        running  =           false; 
uint8_t     start_stop =             1;   //output for startstop. 0 = stop 1= start.
int         ledState =             LOW; 
uint8_t     grip_count=              0;  //gripper calib menu
uint8_t     gripCalib_menu=          0;  //0 = close, 1= open, 3= done
float       prevMul                   ;
int         i                         ;   // counter loop
uint8_t     firstRun=                0; 

    // LCD menu variables
uint8_t     counter =                0; 
uint8_t     page =                   1;
uint8_t     Ready =                  1;
uint8_t     submenu =              TITLE;
uint8_t     last_counter =           0; 
bool        clk_State;
bool        Last_State; 
bool        dt_State;  
uint8_t     pushed =                 0;
uint8_t     enLim =                  0;
String      stats =          "Stopped";
uint8_t     encoderStat_r;
uint8_t     encoderStat_l;
uint8_t     sub_mul_menu =           0;
    // Output   
float     mul                      ;    //Scaling
float     increment=            0.1;    // increment buat Multiplier
uint8_t   calib    =              0;    //data calibration right arm sensor; 0 = no calib, 1-7 pilih sensor.
uint8_t   arm      =              0;    //left/right arm (0:R, 1:L)
uint8_t   power     =             0;
uint8_t   calibGrip =             0;    //1: closed reading. 2: open reading
    //Flag System
struct flag_choose
{
    uint8_t scale;     // flagChooseScale
    uint8_t repo;      // flag repositioning
} flag;
struct calib_status     //symbol for LCD display
{
    char id1;
    char id2;
    char id3;
    char id4;
    char id5;
    char id6;
    char id7;
}sensorStat;
struct profile_variables
{
    float multi;
    bool  getMul = false;

}profile;

    // LCD variables
unsigned long lcdrefresh = 0;  // To store time for lcd to refresh 
unsigned long lastLimit  = 0;  // time for limit switch refresh display
unsigned long lastStop   = 0;  //time for start stop limit switch to refresh display
unsigned long lastCalib  = 0;  //time for calibration encoder and gripper
unsigned long lastIdle   = 0;
// for blinking LED
unsigned long previousMillis = 0; 
uint8_t arrow[8] = {0x0, 0x04 ,0x02, 0x01, 0x02, 0x04, 0x00, 0x00}; // arrow icon for LCD display
    // Uncomment for OBJECT LCD i2c
// LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // LCD I2C initialization
    //LCD object 
//  const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7; //Uncomment for new board
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  
    // ROS
// Set up the ROS nodehandle and publisher/subscriber
ros::NodeHandle n;
data_handler::ui_msgs UI_msg;
data_handler::Profile prof_msg;
std_msgs::UInt8 preload_msg;
//the publishers
ros::Publisher pub_ui("UI", &UI_msg);
ros::Publisher pub_prof("profile_instruction", &prof_msg);
ros::Publisher pub_preload("preload", &preload_msg);
void encCb_r(const std_msgs::UInt8& encStat_msg){
    encoderStat_r = encStat_msg.data;
}
void encCb_l(const std_msgs::UInt8& encStat_l_msg){
    encoderStat_l = encStat_l_msg.data;
}
void loadProfCb(const data_handler::Profile& prof){
    profile.multi = prof.prof_value;
    profile.getMul = true;
}
ros::Subscriber<std_msgs::UInt8> sub_encStatR("r_flag_sensor", &encCb_r );
ros::Subscriber<std_msgs::UInt8> sub_encStatL("l_flag_sensor", &encCb_l);
ros::Subscriber<data_handler::Profile> sub_prof("load_profile", &loadProfCb);

    // INITIALIZATION
void setup() {
    n.initNode();
    n.advertise(pub_ui);
    n.advertise(pub_prof);
    n.advertise(pub_preload);
    n.subscribe(sub_encStatR);
    n.subscribe(sub_encStatL);
    n.subscribe(sub_prof);
        //PinMode
    pinMode(reposPin, INPUT);
    pinMode(selPin, INPUT);
    pinMode(startStopPin, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
        //Setup init flagChoose
    flag.scale = 0;
    flag.repo  = 1;
        //setup status sensor for calib
    negate_selection();
        // setup LCD
    lcd.backlight(); 
    lcd.begin(20,4);
    lcd.createChar(1, arrow);   //Create the arrow symbol
        //interrupt encoder
    PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
    PCMSK0 |= (1 << PCINT0);  //Set pin PB0 trigger an interrupt on state change. (PIN 53)
    PCMSK0 |= (1 << PCINT1);  //Set pin PB1 trigger an interrupt on state change. (PIN 52) 
    DDRB &= B11111100;        //PB0, PB1 as input for the encoder clock and data pins 
    Last_State =   (PINB & B00000001); //pin 8 state (clock pin)
}
void loop() {
    while (firstRun == 0){
            //Init multiplier before profile load
        lcd.setCursor(0,0);
        lcd.print("Waiting for data");
        lcd.setCursor(0,1);
        lcd.print("PrevMul: ");
        lcd.print(profile.multi);    
        preload_msg.data = 1;
        pub_preload.publish(&preload_msg);
        n.spinOnce();
        if (profile.getMul == true ){
            mul = profile.multi;
            lcd.clear();
            firstRun = 1;
            lcd.setCursor(0,0);
            lcd.print("Data retreived");
            lcd.setCursor(0,1);
            lcd.print("PrevMul: ");
            lcd.print(profile.multi);  
            delay(1000);
            lcd.clear();
        }
        delay(1);
    }
        //pin Reading
    repoReading = digitalRead(reposPin);
    startStopReading = digitalRead(startStopPin);
        //subscriber encoder connection
    if(encoderStat_r != 0 || encoderStat_l != 0){
        if(encoderStat_r != 0 && encoderStat_l == 0){
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(" Right Arm Offline");
            lcd.setCursor(0,1);
            lcd.print(" Check Connection");
            
        }
        else if(encoderStat_r == 0 && encoderStat_l != 0){
            lcd.clear();
            lcd.setCursor(0,1);
            lcd.print(" Left Arm Offline");
            lcd.setCursor(0,2);
            lcd.print(" Check Connection");
        }
        else if (encoderStat_r !=0 && encoderStat_l != 0){
            lcd.clear();
            lcd.setCursor(0,1);
            lcd.print(" L+R Arm Offline");
            lcd.setCursor(0,2);
            lcd.print(" Check Connection");
        }
        start_stop = 1;
        stats = "Stopped";
        if (millis() - previousMillis >= 150) {
            previousMillis = millis();
            if (ledState == LOW) {
                ledState = HIGH;
            } 
            else {
                ledState = LOW;
            }
            digitalWrite(LED_BUILTIN, ledState);
        }
        power = 0;
        UI_msg.sys_flag   = (rst<<3)| (flag.repo<<2) | (power<<1) | start_stop;    
            //publishing the message to the topic
        pub_ui.publish( &UI_msg);
        n.spinOnce();
        submenu = HOME;
        delay(150);
        return;
    }
        //LED blinker every interval
    if (millis() - previousMillis >= interval) {
        previousMillis = millis();
        if (ledState == LOW) {
            ledState = HIGH;
        } 
        else {
            ledState = LOW;
        }
        digitalWrite(LED_BUILTIN, ledState);
    }
        //Repositioning function
    if ((running == true) && (submenu == HOME)){
        repos_function();
    }
        //LCD MENU    
    if( pushed || (millis()-lcdrefresh > 500)){
        Ready=1;
            //Limit Switch toggle startStop
        if((startStopReading == HIGH) && (millis()-lastStop>=1000)){
            running = not(running); //toggle state
            if(running == true && submenu == HOME){
                start_stop = 0; // started
                stats = "Running";
                if(firstRun == 1){
                  rst= 0;
                  firstRun = 2;
                }
            }
            else{
                start_stop = 1; // stopped
                stats = "Stopped";
            }
            lastStop = millis();
        }  
        if(submenu == TITLE){
            title_display();
            counter=0;
        }
        else if (submenu == HOME){
            flag.scale = 0;
            if (running == true){
                repos_function();
            }
            home_display();
            lastIdle = millis();
        }
            //Main menu
        else if(submenu == MAIN){  
            flag.scale = 0;
            //if startstop foot switch is pressed
            if (running == true){
                submenu = HOME;
                lcd.clear();
                lcd.setCursor(0,0);   
                lcd.print(" MAIN MENU DISABLED ");
                lcd.setCursor(0,1);
                lcd.print("  STOP THE SYSTEM");
                lcd.setCursor(0,3);  
                lcd.print("     Returning.");   
                delay(1000);
                lcd.clear();
                lcdrefresh = millis();
            }
            //if idle for 20 secs, return to main screen
            if(last_counter == counter){
                if((millis() - lastIdle) >= reset_time){
                    if (flag.repo == 1)
                        submenu = HOME;
                    pushed = 0;
                    counter = 0;
                    lastIdle = millis();
                }
            }
            if(0 <= counter && counter < 5){     
                lcd.clear();
                lcd.setCursor(0,0);   
                lcd.print("Multi: ");
                lcd.print(mul*flag.repo);  
                lcd.print("  ");    
                lcd.print(stats); 
                lcd.setCursor(0,1);
                lcd.write(1);
                lcd.print("Load Profile");
                lcd.setCursor(0,2);  
                lcd.print(" Save Profile");   
                lcd.setCursor(0,3);
                lcd.print(" Change Multiplier");
                lcd.print(" V");    
                lcdrefresh = millis();
                page=1;
                if(pushed)                     
                {  
                    pushed = 1;
                }
            }
            else if(5 <= counter && counter < 10){
                lcd.setCursor(0,0);   
                lcd.print("Multi: ");
                lcd.print(mul*flag.repo);  
                lcd.print("  ");    
                lcd.print(stats);
                lcd.setCursor(0,1);
                lcd.print(" Load Profile");
                lcd.setCursor(0,2); 
                lcd.write(1);
                lcd.print("Save Profile");   
                lcd.setCursor(0,3);
                lcd.print(" Change Multiplier");
                lcd.print(" V");   
                lcdrefresh = millis();       
                page=2;
                if(pushed)
                {
                    pushed = 1;
                    lcd.clear();
                }     
            }
            else if (10 <= counter && counter < 15) {
                lcd.clear();
                lcd.setCursor(0,0);   
                lcd.print("Multi: ");
                lcd.print(mul*flag.repo);  
                lcd.print("  ");    
                lcd.print(stats);
                lcd.setCursor(0,1);                   
                lcd.print(" Load Profile");
                lcd.setCursor(0,2); 
                lcd.print(" Save Profile");   
                lcd.setCursor(0,3);
                lcd.write(1);
                lcd.print("Change Multiplier");
                lcd.print(" V");
                lcdrefresh = millis();
                page=3;
                if(pushed)
                {
                    pushed = 1;
                    lcd.clear();
                } 
            }
                //second part
            else if (15<= counter && counter < 20){                    
                lcd.clear();
                lcd.setCursor(0,0);   
                lcd.print("Multi: ");
                lcd.print(mul*flag.repo);  
                lcd.print("  ");    
                lcd.print(stats);
                lcd.setCursor(0,1);
                lcd.write(1);
                lcd.print("Reset Actuator");
                lcd.print("    ^");
                lcd.setCursor(0,2);
                lcd.print(" Calibrate Encoder");
                lcd.setCursor(0,3);
                lcd.print(" Calibrate Gripper");
                lcd.print(" V");
                lcdrefresh=millis();
                page=4;
                if(pushed)
                {
                    pushed = 1;
                    lcd.clear();
                }  
            }
            else if (20 <= counter && counter < 25){
                lcd.setCursor(0,0);   
                lcd.print("Multi: ");
                lcd.print(mul*flag.repo);
                lcd.print("  "); 
                lcd.print(stats);
                lcd.setCursor(0,1);
                lcd.print(" Reset Actuator");
                lcd.print("    ^");
                lcd.setCursor(0,2);
                lcd.write(1);
                lcd.print("Calibrate Encoder");
                lcd.setCursor(0,3);
                lcd.print(" Calibrate Gripper");
                lcd.print(" V");
                lcdrefresh = millis();
                page=5;
                if(pushed)
                {
                    pushed = 1;
                    lcd.clear();
                }  
            }
            else if (25 <= counter && counter < 30){
                lcd.clear();
                lcd.setCursor(0,0);   
                lcd.print("Multi: ");
                lcd.print(mul*flag.repo);
                lcd.print("  ");
                lcd.print(stats);
                lcd.setCursor(0,1);
                lcd.print(" Reset Actuator");
                lcd.print("    ^");
                lcd.setCursor(0,2);
                lcd.print(" Calibrate Encoder");
                lcd.setCursor(0,3);
                lcd.write(1);
                lcd.print("Calibrate Gripper");
                lcd.print(" V");
                lcdrefresh = millis();
                page=6;
                if(pushed)
                {
                    pushed = 1;
                    lcd.clear();
                }  
            }
                //last part
            else if (30 <= counter && counter < 40){
                lcd.clear();
                lcd.setCursor(0,0);   
                lcd.print("Multi: ");
                lcd.print(mul*flag.repo);
                lcd.print("  ");
                lcd.print(stats);
                lcd.setCursor(0,1);
                lcd.write(1);
                lcd.print("Power Off");
                lcd.print("         ^");
                lcdrefresh = millis();
                page=7;
                if(pushed)
                {
                    pushed = 1;
                    lcd.clear();
                }  
            }
        }//submenu = MAIN     
            //Menu for load profile
        else if(submenu == load_menu){ 
            flag.scale = 0;
            if(0 <= counter && counter < 4){ 
                if( ( millis()-lcdrefresh ) >= 310 ){   
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Load Profile :");  
                    lcd.setCursor(0,1);
                    lcd.write(1);
                    lcd.print(user1);
                    lcd.setCursor(1,2);
                    lcd.print(user2);
                    lcd.setCursor(1,3);
                    lcd.print(user3);
                    lcd.print("        V");
                    page = 1;
                        //Publisher
                    prof_msg.prof_ID = 0;
                    prof_msg.load= 1;
                    pub_prof.publish(&prof_msg);
                    lcdrefresh = millis();  
                }    
            }
            else if(4 <= counter && counter < 8){ 
                if( ( millis()-lcdrefresh ) >= 310 ){   
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Load Profile :");  
                    lcd.setCursor(1,1);
                    lcd.print(user1);
                    lcd.setCursor(0,2);
                    lcd.write(1);
                    lcd.print(user2);
                    lcd.setCursor(1,3);
                    lcd.print(user3);
                    lcd.print("        V");
                    page = 2;
                        //Publisher
                    prof_msg.prof_ID = 1;
                    prof_msg.load= 1;
                    pub_prof.publish(&prof_msg);
                    lcdrefresh = millis();  
                }    
            }
            else if(8 <= counter && counter < 12){ 
                if( ( millis()-lcdrefresh ) >= 310 ){              
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Load Profile :");  
                    lcd.setCursor(1,1);
                    lcd.print(user1);
                    lcd.setCursor(1,2);
                    lcd.print(user2);
                    lcd.setCursor(0,3);
                    lcd.write(1);
                    lcd.print(user3);
                    lcd.print("        V");
                    page = 3;
                    //Publisher
                    prof_msg.prof_ID = 2;
                    prof_msg.load= 1;
                    pub_prof.publish(&prof_msg);
                    lcdrefresh = millis();  
                }    
            }
            else if(12 <= counter && counter < 40){ 
                if( ( millis()-lcdrefresh ) >= 310 ){              
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Load Profile :");  
                    lcd.setCursor(0,1);
                    lcd.write(1);
                    lcd.print("Exit");  
                    page = 4;
                    lcdrefresh = millis();  
                }    
            }
        }
            //Menu for save profile
        else if(submenu == save_menu){
            flag.scale = 0;
            if(0 <= counter && counter < 4){ 
                if( ( millis()-lcdrefresh ) >= 310 ){   
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Save Profile :");  
                    lcd.setCursor(0,1);
                    lcd.write(1);
                    lcd.print(user1);
                    lcd.setCursor(1,2);
                    lcd.print(user2);
                    lcd.setCursor(1,3);
                    lcd.print(user3);
                    lcd.print("        V");
                    page = 1;
                    lcdrefresh = millis();  
                }    
            }
            else if(4 <= counter && counter < 8){ 
                if( ( millis()-lcdrefresh ) >= 310 ){   
                    lcd.clear();           
                    lcd.setCursor(0,0);
                    lcd.print("Save Profile :");  
                    lcd.setCursor(1,1);
                    lcd.print(user1);
                    lcd.setCursor(0,2);
                    lcd.write(1);
                    lcd.print(user2);
                    lcd.setCursor(1,3);
                    lcd.print(user3);
                    lcd.print("        V");
                    page = 2;
                    lcdrefresh = millis();  
                }    
            }
            else if(8 <= counter && counter < 12){ 
                if( ( millis()-lcdrefresh ) >= 310 ){              
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Save Profile :");  
                    lcd.setCursor(1,1);
                    lcd.print(user1);
                    lcd.setCursor(1,2);
                    lcd.print(user2);
                    lcd.setCursor(0,3);
                    lcd.write(1);
                    lcd.print(user3);
                    lcd.print("        V");
                    page = 3;
                    lcdrefresh = millis();  
                }    
            }
            else if(12 <= counter && counter < 40){ 
                if( ( millis()-lcdrefresh ) >= 310 ){              
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Save Profile :");  
                    lcd.setCursor(0,1);
                    lcd.write(1);
                    lcd.print("Exit");  
                    page = 4;
                    lcdrefresh = millis();  
                }    
            }
        }
            //Menu for changing multiplier
        else if(submenu == multiplier_menu){  
            flag.scale = 1;
            if(sub_mul_menu == 0){        
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Multiplier :");       
                lcd.setCursor(7,2);
                lcd.print(mul*flag.repo);   
                lcd.setCursor(7,3);
                lcd.print("^");
                page = 1;
                lcdrefresh = millis();  
            }
            else if (sub_mul_menu == 1){
                lcd.setCursor(0,0);
                lcd.print("  Multiplier :");       
                lcd.setCursor(7,2);
                lcd.print(mul*flag.repo);   
                lcd.setCursor(7,3);
                lcd.print("  ^");
                page = 2;
                lcdrefresh = millis();
            }
            else if(sub_mul_menu == 2){
                lcd.setCursor(0,0);
                lcd.print("  Multiplier :");       
                lcd.setCursor(7,2);
                lcd.print(mul*flag.repo);   
                lcd.setCursor(7,3);
                lcd.print("   ^");
                page = 3;
                lcdrefresh = millis();
            }
        }
            //Menu for calibrating encoder
        else if (submenu == encoder_menu){ 
            flag.scale = 0;
            if((0 <= counter) && counter < 3){
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear(); 
                    lcd.setCursor(1,0);  
                    lcd.print("Choose Sensor ID");
                    lcd.setCursor(0,1);   
                    lcd.write(1);
                    lcd.print("Sensor ID 1  ");
                    lcd.print(sensorStat.id1);
                    lcd.setCursor(0,2);  
                    lcd.print(" Sensor ID 2  ");
                    lcd.print(sensorStat.id2);
                    lcd.setCursor(0,3);  
                    lcd.print(" Sensor ID 3  ");
                    lcd.print(sensorStat.id3);
                    page = 1;
                    lcdrefresh = millis();  
                } 
            } 
            else if (3 <= counter && counter < 6){
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(1,0);  
                    lcd.print("Choose Sensor ID");
                    lcd.setCursor(0,1);   
                    lcd.print(" Sensor ID 1  ");
                    lcd.print(sensorStat.id1);
                    lcd.setCursor(0,2);  
                    lcd.write(1);
                    lcd.print("Sensor ID 2  ");
                    lcd.print(sensorStat.id2);
                    lcd.setCursor(0,3);  
                    lcd.print(" Sensor ID 3  ");
                    lcd.print(sensorStat.id3);
                    page = 2; 
                    lcdrefresh = millis();  
                } 
            }
            else if (6 <= counter && counter < 9){
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(1,0);  
                    lcd.print("Choose Sensor ID");
                    lcd.setCursor(0,1);   
                    lcd.print(" Sensor ID 1  ");
                    lcd.print(sensorStat.id1);
                    lcd.setCursor(0,2);  
                    lcd.print(" Sensor ID 2  ");
                    lcd.print(sensorStat.id2);
                    lcd.setCursor(0,3);  
                    lcd.write(1);
                    lcd.print("Sensor ID 3  ");
                    lcd.print(sensorStat.id3);
                    page = 3;    
                    lcdrefresh = millis();  
                } 
            }
                // NEXT PAGE
            else if (9 <= counter && counter < 12){
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0);  
                    lcd.write(1);
                    lcd.print("Sensor ID 4  ");
                    lcd.print(sensorStat.id4);
                    lcd.setCursor(0,1);   
                    lcd.print(" Sensor ID 5  ");
                    lcd.print(sensorStat.id5);
                    lcd.setCursor(0,2);  
                    lcd.print(" Sensor ID 6  ");
                    lcd.print(sensorStat.id6);
                    lcd.setCursor(0,3);  
                    lcd.print(" Sensor ID 7  ");
                    lcd.print(sensorStat.id7);
                    page = 4;    
                    lcdrefresh = millis();  
                } 
            }
            else if (12 <= counter && counter < 15){
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0);  
                    lcd.print(" Sensor ID 4  ");
                    lcd.print(sensorStat.id4);
                    lcd.setCursor(0,1);   
                    lcd.write(1);
                    lcd.print("Sensor ID 5  ");
                    lcd.print(sensorStat.id5);
                    lcd.setCursor(0,2);  
                    lcd.print(" Sensor ID 6  ");
                    lcd.print(sensorStat.id6);
                    lcd.setCursor(0,3);  
                    lcd.print(" Sensor ID 7  ");
                    lcd.print(sensorStat.id7);
                    page = 5;   
                    lcdrefresh = millis();  
                } 
            }
            else if (15 <= counter && counter < 18){
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0);  
                    lcd.print(" Sensor ID 4  ");
                    lcd.print(sensorStat.id4);
                    lcd.setCursor(0,1);   
                    lcd.print(" Sensor ID 5  ");
                    lcd.print(sensorStat.id5);
                    lcd.setCursor(0,2);  
                    lcd.write(1);
                    lcd.print("Sensor ID 6  ");
                    lcd.print(sensorStat.id6);
                    lcd.setCursor(0,3);  
                    lcd.print(" Sensor ID 7  ");
                    lcd.print(sensorStat.id7);
                    page = 6;
                    lcdrefresh = millis();  
                } 
            }
            else if (18 <= counter && counter < 21){
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0);  
                    lcd.print(" Sensor ID 4  ");
                    lcd.print(sensorStat.id4);
                    lcd.setCursor(0,1);   
                    lcd.print(" Sensor ID 5  ");
                    lcd.print(sensorStat.id5);
                    lcd.setCursor(0,2);  
                    lcd.print(" Sensor ID 6  ");
                    lcd.print(sensorStat.id6);
                    lcd.setCursor(0,3);  
                    lcd.write(1);
                    lcd.print("Sensor ID 7  ");
                    lcd.print(sensorStat.id7);
                    page = 7;  
                    lcdrefresh = millis();  
                } 
            }
            else if (21 <= counter && counter < 24)  {
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0);  
                    lcd.write(1);
                    lcd.print("All Sensor");
                    lcd.setCursor(0,1);
                    lcd.print(" Calibrate Right");
                    lcd.setCursor(0,2);
                    lcd.print(" Calibrate Left");
                    lcd.setCursor(0,3);
                    lcd.print(" Exit");
                    page = 8; 
                    lcdrefresh = millis();  
                } 
            }    
            else if (24 <= counter && counter < 27)  {
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0);
                    lcd.print(" All Sensor");
                    lcd.setCursor(0,1);
                    lcd.write(1);
                    lcd.print("Calibrate Right");
                    lcd.setCursor(0,2);
                    lcd.print(" Calibrate Left");
                    lcd.setCursor(0,3);
                    lcd.print(" Exit");
                    page = 9; 
                    lcdrefresh = millis();  
                } 
            }
            else if (27 <= counter && counter < 30)  {
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0); 
                    lcd.print(" All Sensor");
                    lcd.setCursor(0,1);
                    lcd.print(" Calibrate Right");
                    lcd.setCursor(0,2);
                    lcd.write(1);
                    lcd.print("Calibrate Left");
                    lcd.setCursor(0,3);
                    lcd.print(" Exit");
                    page = 10; 
                    lcdrefresh = millis();  
                } 
            }
            else if (30 <= counter && counter < 40)  {
                if(( millis()-lcdrefresh ) >= 310 ){ 
                    lcd.clear();  
                    lcd.setCursor(0,0); 
                    lcd.print(" All Sensor");
                    lcd.setCursor(0,1);
                    lcd.print(" Calibrate Right");
                    lcd.setCursor(0,2);
                    lcd.print(" Calibrate Left");
                    lcd.setCursor(0,3);
                    lcd.write(1);
                    lcd.print("Exit");
                    page = 11; 
                    lcdrefresh = millis();  
                } 
            }
        }//submenu 3 change calib
            //Menu for calibrating Gripper
        else if (submenu == gripper_menu){
            if(( millis()-lcdrefresh ) >= 310 ){ 
                lcd.clear();  
                if (gripCalib_menu == 0){
                    calibGrip = 0;
                    flag.scale = 0;
                    if((0 <= counter) && counter < 3){
                            lcd.setCursor(1,0);  
                            lcd.print("Choose Gripper:");
                            lcd.setCursor(0,1);   
                            lcd.write(1);
                            lcd.print("Right Gripper");
                            lcd.setCursor(0,2);  
                            lcd.print(" Left Gripper  ");
                            arm = 0;
                            gripCalib_menu = 0;
                    } 
                    else if (3 <= counter && counter < 40){
                            lcd.setCursor(1,0);  
                            lcd.print("Choose Gripper:");
                            lcd.setCursor(0,1);   
                            lcd.print(" Right Gripper");
                            lcd.setCursor(0,2);  
                            lcd.write(1);
                            lcd.print("Left Gripper ");
                            arm = 1;
                            gripCalib_menu = 0; 
                    }
                    lcdrefresh = millis();
                }
                else if(gripCalib_menu == 1){
                    calibGrip = 4;
                    lcd.setCursor(0,0); 
                    lcd.print(" Calibrate Gripper");    
                    lcd.setCursor(0,2);   
                    lcd.print("Please Close Gripper"); 
                    lcd.setCursor(0,3);
                    lcd.print("Step Repos Switch");
                    if(repoReading == 1 && (millis() - lastCalib > 1000)){
                        lcd.clear();
                        lastCalib = millis();
                        while(grip_count <100){
                            calibGrip = 1;
                            lcd.setCursor(0,2);
                            lcd.print("Calibrating..");
                            UI_msg.calibArm = arm;
                            UI_msg.calibGrip = calibGrip;
                            UI_msg.sys_flag   = (rst<<3)| (flag.repo<<2) | (power<<1) | start_stop;  
                            pub_ui.publish(&UI_msg);
                            grip_count++;
                        }
                        gripCalib_menu = 2;
                        grip_count = 0;
                    }
                    lcdrefresh = millis();
                }
                else if(gripCalib_menu == 2){
                    calibGrip = 4;
                    lcd.clear();
                    lcd.setCursor(0,0); 
                    lcd.print(" Calibrate Gripper");  
                    lcd.setCursor(0,2);   
                    lcd.print("   Open Gripper"); 
                    lcd.setCursor(0,3);
                    lcd.print("Step Repos Switch");
                    if(repoReading == 1 && (millis() - lastCalib > 1000)){
                        lcd.clear();
                        lastCalib = millis();
                        while(grip_count <100){
                            calibGrip = 2;
                            lcd.setCursor(0,2);
                            lcd.print("Calibrating..");
                            UI_msg.calibArm = arm;
                            UI_msg.calibGrip = calibGrip;
                            UI_msg.sys_flag   = (rst<<3)| (flag.repo<<2) | (power<<1) | start_stop;  
                            pub_ui.publish(&UI_msg);
                            grip_count++;
                        }
                        gripCalib_menu = 3;
                        grip_count = 0;
                    }
                    lcdrefresh = millis();
                }
                else if(gripCalib_menu == 3){
                    lcd.clear();
                    calibGrip = 3;
                    lcd.setCursor(0,0);
                    lcd.print("Gripper Calibration");
                    lcd.setCursor(0,1);
                    lcd.print("     Complete");
                    lcd.setCursor(0,2);
                    lcd.print("   Press Button");
                    flag.scale =0;
                    lcdrefresh = millis();
                }
                
            }
        }
            //Menu for reset actuator
        else if (submenu == reset_menu){
            if(( millis()-lcdrefresh ) >= 310 ){ 
                lcd.clear();  
                lcd.setCursor(0,1); 
                lcd.print("  Press Button to");       
                lcd.setCursor(0,2);   
                lcd.print("  Reset Actuator");    
                flag.scale =0;
                lcdrefresh = millis();  
            }  
            
        }// submenu 4 for rst prompt
            //Menu for power down.
        else if (submenu == power_menu){ 
            if(( millis()-lcdrefresh ) >= 310 ){ 
                lcd.clear();  
                lcd.setCursor(0,1); 
                lcd.print("  Press Button to");       
                lcd.setCursor(0,2);      
                lcd.print("     Power Down") ;       
                flag.scale =0;
                lcdrefresh = millis();  
            }  
        }
    }//end of the MENU prints on the LCD
        //Selector Button Condition
    // last_counter = counter; 
    if(!digitalRead(selPin))
    {
        if(submenu == load_menu){
            if(page == 1){//load prof 1
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                mul = profile.multi;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Profile 1 loaded");
                lcd.setCursor(0,2);
                lcd.print("Username  : ");
                lcd.print(user1);
                lcd.setCursor(0,3);
                lcd.print("Multiplier: ");
                lcd.print(mul); 
                delay(2000);
                lcd.clear();
            }
            else if(page == 2){//load prof 2
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                mul = profile.multi;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Profile 2 loaded");
                lcd.setCursor(0,2);
                lcd.print("Username  : ");
                lcd.print(user2);
                lcd.setCursor(0,3);
                lcd.print("Multiplier: ");
                lcd.print(mul);  
                delay(2000);
                lcd.clear();

            }
            else if(page == 3){//load prof 3
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                mul = profile.multi;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Profile 3 loaded");
                lcd.setCursor(0,2);
                lcd.print("Username  : ");
                lcd.print(user3);
                lcd.setCursor(0,3);
                lcd.print("Multiplier: ");
                lcd.print(mul);   
                delay(2000);
                lcd.clear();

            }
            else if(page == 4){//exit
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                mul = mul;
                lcd.clear();
                lcd.setCursor(5,1);
                lcd.print(" Exiting");
                delay(2000);
                lcd.clear();

            }
        }
            //saving profile 
        else if(submenu == save_menu){
            if(page == 1){//save prof 1
                submenu = save_menu;
                counter=0;
                pushed = 0;
                Ready=0;
                //publisher
                prof_msg.prof_value = mul;
                prof_msg.prof_ID = 0;
                prof_msg.load = 0;
                pub_prof.publish(&prof_msg);
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Profile 1 saved");
                lcd.setCursor(0,2);
                lcd.print("Username  : ");
                lcd.print(user1);
                lcd.setCursor(0,3);
                lcd.print("Multiplier: ");
                lcd.print(mul);   
                delay(2000);
                lcd.clear();

            }
            else if(page == 2){//save prof 2
                submenu = save_menu;
                counter=0;
                pushed = 0;
                Ready=0;
                //publisher
                prof_msg.prof_value = mul;
                prof_msg.prof_ID = 1;
                prof_msg.load = 0;
                pub_prof.publish(&prof_msg);
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Profile 2 saved");
                lcd.setCursor(0,2);
                lcd.print("Username  : ");
                lcd.print(user2);
                lcd.setCursor(0,3);
                lcd.print("Multiplier: ");
                lcd.print(mul);   
                delay(2000);
                lcd.clear();

            }
            else if(page == 3){//save prof 3
                submenu = save_menu;
                counter=0;
                pushed = 0;
                Ready=0;
                //publisher
                prof_msg.prof_value = mul;
                prof_msg.prof_ID = 2;
                prof_msg.load = 0;
                pub_prof.publish(&prof_msg);
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Profile 3 saved");
                lcd.setCursor(0,2);
                lcd.print("Username  : ");
                lcd.print(user3);
                lcd.setCursor(0,3);
                lcd.print("Multiplier: ");
                lcd.print(mul);  
                delay(2000);
                lcd.clear();

            }
            else if(page == 4){//exit
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                lcd.clear();
                lcd.setCursor(5,1);
                lcd.print(" Exiting");
                delay(2000);
                lcd.clear();

            }
        }
            // changing scale
        else if(submenu == multiplier_menu){   
            if  (page == 1){
                submenu = multiplier_menu;
                counter = 0;
                pushed = 0;
                sub_mul_menu = 1;
                delay(15);
            } 
            else if (page == 2){
                submenu = multiplier_menu;
                counter = 0;
                pushed = 0;
                sub_mul_menu = 2;
                delay(15);
            }
            else if(page==3){
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("  Scale Changed :");
                lcd.setCursor(5,2);  
                lcd.print(mul); 
                lcd.print(" times");
                UI_msg.Multi = mul *flag.repo;
                UI_msg.sys_flag   = (rst<<3)| (flag.repo<<2) | (power<<1) | start_stop;  
                pub_ui.publish(&UI_msg);
                sub_mul_menu = 0;
                delay(2000);
                lcd.clear();
            }
        }

        else if(submenu == encoder_menu) {
            if(page==1) {
                submenu=6;
                counter=0;
                pushed = 0;
                Ready=0;       
                calib = 0b00000010 | calib;       
                sensorStat.id1 = '+';
            }
            else if(page==2){     
                submenu = encoder_menu;
                counter=3;
                pushed = 0;
                Ready=0; 
                calib = 0b00000100 | calib; //encoder 1
                sensorStat.id2 = '+';
            }
            else if(page==3){
                submenu = encoder_menu;
                counter=6;
                pushed = 0;
                Ready=0;       
                calib = 0b00001000 | calib;        //encoder 2
                sensorStat.id3 = '+';
            }
            else if(page==4){
                submenu = encoder_menu;
                counter=9;
                pushed = 0;
                Ready=0;       
                calib = 0b00010000 | calib;         //encoder 3
                sensorStat.id4 = '+';
            }
            else if(page== 5){
                submenu = encoder_menu;
                counter=12;
                pushed = 0;
                Ready=0;       
                calib = 0b00100000 | calib;        //encoder 4
                sensorStat.id5 = '+'; 
            }
            else if(page==6){
                submenu = encoder_menu;
                counter=15;
                pushed = 0;
                Ready=0;       
                calib = 0b01000000 | calib;        //encoder 5
                sensorStat.id6 = '+';
            }
            else if(page==7){
                submenu = encoder_menu;
                counter=18;
                pushed = 0;
                Ready=0;       
                calib = 0b10000000 | calib;        //encoder 6
                sensorStat.id7 = '+';
            }
            else if(page==8){
                //all encoder
                submenu = encoder_menu;
                counter=21;
                pushed = 0;
                Ready=0;
                calib = 0b11111110;
                sensorStat.id1 = '+';
                sensorStat.id2 = '+';
                sensorStat.id3 = '+';
                sensorStat.id4 = '+';
                sensorStat.id5 = '+';
                sensorStat.id6 = '+';
                sensorStat.id7 = '+';
            }
            else if(page==9){
                //publish calib for right arm
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                calib = calib | 0b00000001;
                negate_selection();
                i = 0;
                lcd.clear();
                while(i<3){
                    lcd.clear();
                    lcd.setCursor(2,0);
                    lcd.print("Calibrating");
                    lcd.setCursor(2,2);  
                    lcd.print("Sensor ID : "); 
                    lcd.print(calib);
                    UI_msg.calibID = calib;
                    UI_msg.calibArm = 0;
                    UI_msg.sys_flag   = (rst<<3)| (flag.repo<<2) | (power<<1) | start_stop;  
                    pub_ui.publish(&UI_msg);
                    i++;
                }
                delay(calib_time);
                calib = 0;
                lcd.clear();
            }
            else if(page==10){
                //publish calib for right arm
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                calib = calib | 1;
                negate_selection(); 
                lcd.clear();
                i = 0;
                while(i<3){
                    lcd.setCursor(2,0);
                    lcd.print("Calibrating");
                    lcd.setCursor(2,2);  
                    lcd.print("Sensor ID : "); 
                    lcd.print(calib);
                    UI_msg.calibID = calib;
                    UI_msg.calibArm = 1;
                    UI_msg.sys_flag   = (rst<<3)| (flag.repo<<2) | (power<<1) | start_stop;  
                    pub_ui.publish(&UI_msg);
                    i++;
                }
                delay(calib_time);
                lcd.clear();
                calib = 0;
            }
            //finished
            else if(page == 11){
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;       
                calib = 0;        //stop calibrating
                lcd.clear();
                lcd.setCursor(5,1);
                lcd.print(" Exiting");
                negate_selection();         
                delay(1000);
                lcd.clear();
            }   
        }
        else if(submenu == gripper_menu){
            if(gripCalib_menu == 0 ){
                submenu = gripper_menu;
                counter = 0;
                pushed = 0;
                Ready = 0;
                lcd.clear();
                if(arm == 0){
                    lcd.setCursor(0,0);
                    lcd.print("  Calibrating");
                    lcd.setCursor(0,2);
                    lcd.print("  Right Gripper");
                    delay(1000);
                }
                else if (arm == 1){
                    lcd.setCursor(0,0);
                    lcd.print("  Calibrating");
                    lcd.setCursor(0,2);
                    lcd.print("  Left Gripper");
                    delay(1000);
                }
                gripCalib_menu = 1;
            }
            if(gripCalib_menu == 3){
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                lcd.clear();
                lcd.setCursor(0,1);
                lcd.print("  Calibration Done");
                calibGrip = 0;
                UI_msg.calibGrip = calibGrip;
                UI_msg.calibArm = arm;
                UI_msg.sys_flag   = (rst<<3)| (flag.repo<<2) | (power<<1) | start_stop;  
                pub_ui.publish( &UI_msg );
                gripCalib_menu = 0;
                delay(2000);
                lcd.clear();  
            }
        }
        else if(submenu == reset_menu){
            if(page==4){
                submenu = HOME;
                counter=0;
                pushed = 0;
                Ready=0;
                lcd.clear();
                lcd.setCursor(5,1);
                lcd.print("Resetting");
                rst = 1;
                UI_msg.sys_flag = (rst<<3) | (flag.repo<<2) | (power<<1) | start_stop;  
                pub_ui.publish( &UI_msg );
                delay(2000);
                lcd.clear();  
                rst = 0;
            }
        }
        else if(submenu == power_menu){
            if(page == 7){
                //stays in this menu after powering off
                counter=0;
                pushed = 0;
                Ready=0;
                lcd.clear();
                lcd.setCursor(5,1);
                lcd.print(" Good-Bye");
                power = 1;
                UI_msg.sys_flag = (power<<1) | start_stop;
                pub_ui.publish( &UI_msg );
                delay(2000);
                lcd.clear(); 
            }
        }
            //Main menu button
        if(submenu == MAIN && Ready==1){    
            if(page==1){
                if(flag.repo == 1)
                    submenu=load_menu;
                counter=0;
                pushed=0;
                delay(500); 
            }   
            if(page==2) {
                if(flag.repo == 1)
                    submenu = save_menu ;
                counter=0;
                pushed=0;
                delay(500);
            }
            if(page==3){
                if(flag.repo == 1)
                    submenu = multiplier_menu;
                counter = 0;
                pushed = 0;
                delay(500);
            }
            if(page == 4){
                if(flag.repo == 1)
                    submenu= reset_menu;
                counter = 0;
                pushed = 0;
                delay(500);
            }
            if(page ==5){
                if(flag.repo == 1)
                    submenu=encoder_menu;
                counter = 0;
                pushed = 0;
                delay(500);
            }
            if(page ==6){
                if(flag.repo == 1)
                    submenu=gripper_menu;
                counter = 0;
                pushed = 0;
                delay(500);
            }
            if (page == 7){
                if(flag.repo == 1)
                    submenu = power_menu;
                counter = 0;
                pushed = 0;
                delay(500);
            }
        }//end of submenu 0
        else if(submenu == HOME && Ready == 1){
            if(page == 1){
                if(flag.repo == 1)
                    submenu = MAIN;
                pushed = 0;
                counter = 0;
                Ready = 0;
                delay(500);
            }
        }
        else if (submenu == TITLE && Ready == 1){
            if(flag.repo == 1){
                submenu = HOME;}
            pushed = 0;
            counter = 0;
            Ready = 0;
            delay(500);
        }
    }
        //Publishing the Outputs Outside start stop toggle
    power =0;
        //assigning message with process variables
    UI_msg.Multi   = mul*flag.repo; 
    UI_msg.calibID = calib;
    UI_msg.calibGrip = calibGrip;
    UI_msg.calibArm      = 0;
    UI_msg.sys_flag   = (rst<<3) | (flag.repo<<2) | (power<<1) | start_stop;    
        //publishing the message to the topic
    pub_ui.publish( &UI_msg);
    n.spinOnce();  
    delay(1000/LOOP_FREQ);
}
    //Interruption vector
ISR(PCINT0_vect){    
    clk_State =   (PINB & B00000001); //pin 8 state, clock pin? 
    dt_State  =   (PINB & B00000010); 
    if (clk_State != Last_State){     
        // If the data state is different to the clock state, that means the encoder is rotating clockwise
        if (dt_State != clk_State) { //THIS IS CW
            if(counter != 40){
                counter++;
            }
            if (mul > 1){
                mul = 1;
            }
            if(flag.scale == 1){
                flag.scale = 0;
                if(sub_mul_menu == 0){
                    mul = 1;
                }
                else if (sub_mul_menu == 1){
                    mul +=0.1;
                }
                else if (sub_mul_menu == 2){
                    mul += 0.01;
                }
            }
        }
        else { // this is CCW
            if (counter != 0)
            {counter--;}
            if (mul < 0){
                mul = 0;
            }
            if(flag.scale == 1){
                if(sub_mul_menu == 0){
                    mul = 0;
                }
                else if (sub_mul_menu == 1){
                    mul -=0.1;
                }
                else if (sub_mul_menu == 2){
                    mul -= 0.01;
                }
            }
        } 
    } 
    Last_State = clk_State; // Updates the previous state of the data with the current state
    last_counter = counter;
}
    //Functions
void repos_function(){
    // Limit Switch Repositioning
    if (submenu == HOME){
        if ((millis()-lastLimit >= 200) && repoReading == 1 && enLim==1){
            flag.repo = 0;    //LimitSwitch Enganged (Repo = ON)
            lastLimit = millis();
            if(enLim == 1){
                lcd.setCursor(12,3);
                lcd.print("        ");
            }   
            enLim = 0;          
        }
        else if ((millis()-lastLimit >= 200) && repoReading == 0 && enLim ==0){
            flag.repo = 1;     //LimitSwitch disenganged (Repo = OFF)
            lastLimit = millis();
            if (enLim == 0){
                home_display();
            }   
            enLim =1;
        }
    }
}
void home_display(){
    lcd.clear();
    flag.scale = 0;
    lcd.setCursor(0,0);
    lcd.print("   MASTER CONSOLE");
    lcd.setCursor(0,1);   
    lcd.print("Multiplier: ");
    lcd.print(mul*flag.repo);  
    lcd.print("  ");    
    lcd.setCursor(0,2);
    lcd.print("Status    : ");
    lcd.print(stats); 
    lcd.setCursor(0,3);
    lcd.print("Repos     : ");
    if(flag.repo == 1){
        lcd.print("Inactive");
    }   
    else
    {
        lcd.print("Active");
    }
    lcdrefresh = millis();
    page=1;
    if(pushed)                     
    {  
        pushed = 1;
        lcd.clear();
    }
}
void title_display(){
    lcd.setCursor(1,0);   //first row
    lcd.print("TA004 ROBOSURGERY");
    lcd.setCursor(2,2); // second row
    lcd.print("  Please Push");
    lcd.setCursor(2,3);      //3rd row
    lcd.print("Button to start"); 
}
void negate_selection(){
    sensorStat.id1 = '-';
    sensorStat.id2 = '-';
    sensorStat.id3 = '-';
    sensorStat.id4 = '-';
    sensorStat.id5 = '-';
    sensorStat.id6 = '-';
    sensorStat.id7 = '-';
}
