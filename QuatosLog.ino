/*
•Connect the 5V pin to the 5V pin on the Arduino
•Connect the GND pin to the GND pin on the Arduino
•Connect CLK to pin 13 or 52
•Connect DO to pin 12 or 50
•Connect DI to pin 11 or 51
•Connect CS to pin 10 or 53


*/
#include <stdio.h>
#include <Servo.h> 
#include <SD.h>
#include <SPI.h>
#include "ToString.h"
#include <DueTimer.h>


/*

*/





//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------



#define USE_SD_READER 1
#define USE_ESC32 1


#define DELAY   (5)
#define BAUD    (115200)
#define ESC32BAUD (115200)
#define MAX_BUF (128)

//#define LOGPWM //comment to skip PWM logging
#define PWM_PIN_OUT  12// PWM
#define PWM_RESOLUTION 12// Due 12Bit
#define PWM_MAXVAL 4096

#define TIME_STEP1 10  //100Hz LOG
#define TIME_STEP2 100 //10Hz RPM
#define TIME_STEP3 4000 // RPM - Thrust Logging
#define RPM_SAMPLING  1 //every n Timestep
#define ANALOG_RESOLUTION 4096

#define VOLTAGE_PIN  1
#define VOLTAGE_SCALER  47.22 //53.5 //139.416 //mV/V

#define CURRENT_PIN  2
#define CURRENT_SCALER  25.91   // 32.31 //73.20 //mV/A

#define LOGRPM //comment to skip RPM logging
#define RPM_PIN  22  //digital pin

#define ENCODER_MODE 0 //1024 FALLING, Mode =1 for 2048 CHANGE
#define ENCODER_MODE_MAXRPM 10000

//#define RPM_SCALER_1024 565.375775
//#define RPM_SCALER_2048 1130.75155

//1024 -> scaler = 566.3637 (2048 -> scale = 1130.75155 /  46 * 25.39774 ) //Motor Poles, Flanken pro U * Übersetzungsverhältnis gemessen 46.18 * 25.39
// 1024 FALLING, 2048 CHANGE. 72 Zähne, 40 Zähne.
//#define RPM_PULSE_READS 30 //not used yet

#define MOTOR_POLES 2 // 2 for 2 Blade Prop for Propsensor. 
#define RPM_SCALER_1024 14 //For phase measuring
#define RPM_SCALER_2048 28 //

#define TORQUE_PIN_LEFT  4
#define TORQUE_SCALER_LEFT 3300 //3000 * 3 / 100 // max Voltage (3300mV) @ 3000Gr --> 3000 * Dist Axle / 100 --> Sensor (3cm) = grams*cm --> /100 =grams*m

#define TORQUE_PIN_RIGHT  5
#define TORQUE_SCALER_RIGHT 3300 // 3000 * 3 / 100

#define THRUST_PIN  6
#define THRUST_SCALER 3300 //max Voltage (3300mV) @ 3000grams

#define LOAD_CELL_MAX_LOAD 3000 //grams == 3.3V

#define T 0  //Time
#define U 1  //Voltage
#define I 2  //Current
#define R 3  //RPM measured
#define TL 4 //Torque Left
#define TR 5 //Torque Right
#define TH 6 //Thrust
#define C 7  //RPM comCount
#define CT 8 //Time comCount


//Log Functoions defines 
#define MOT_SCAL 0      //run SetMaxRpm() first or enter valid Max RPM for this Mot / prop / Batt combo
#define MOT_A1 0  //use the right value
#define MOT_A2 0  //use the right value

#define RPMBASE 3000
#define RPMMIN 1000
#define RPMSTEP 100
#define RPMAMP 500

#define PWM_DISARM 700
#define PWM_ARM 950
#define PWM_OFFSET 1000
#define PWM_MIN 1150
#define PWM_MAX 1950

#define EXFREQ 5 //w: 0-5


//------------------------------------------------------------------------------
// global variables
//------------------------------------------------------------------------------

char TestedUnit[MAX_BUF] = "Unknown Motor & unknown prop @ ?s with unknown ESC             "; //64 Zeichen
float Mot_A1 =0;
float Mot_A2 =0; 
float Mot_Scal = MOT_SCAL; //run SetMaxRpm() first
float Prop_K1 =0;
float Rpm_Scaler = 0 ;
float Motor_Poles = MOTOR_POLES;

char SerBuffer[MAX_BUF];
int sofar;

int MaxSystemRPM = 0;

char test_on=0;  // only used in processCommand as an example.
float pos_x;
float pos_y;
float pos_z;


float voltage_in = 0;         //0-4096
float voltage_out = 0;
float current_in = 0;         //0-4096
float current_out = 0;

float rpm_in = 0;             //Digital pulses
float rpm_out = 0;

float torque_in_left = 0;     //0-4096
float torque_out_left = 0;
float torque_tare_left = 0;

float torque_in_right = 0;    //0-4096
float torque_out_right = 0;
float torque_tare_right = 0;

float thrust_in = 0;          //0-4096
float thrust_out = 0;
float thrust_tare = 0;

boolean load_cell_status = true;
boolean motor_running = true;
boolean logRpm = true;

float LogFreq = 0;

int StartLog = 0;
int LastStartLog = 0; 
int LogRetries = 0;
int TimeStep = TIME_STEP1;
unsigned long counter = 0;

unsigned long timer_1 = 0;
unsigned long timer_2 = 0;
unsigned long timer_3 = 0;
unsigned long timer_4 = 0;
unsigned long timer_5 = 0;
unsigned long timer_6 = 0;
unsigned long timer_7 = 0;
unsigned long timer_8 = 0;
unsigned long timer_sys = 0;


volatile int comCount = 0;
int RPMSampleCount = 0 ; 
unsigned long RPMMedian = 0;
unsigned long ESC32Rpm = 0;
unsigned MaxRpmLogged = 0;
unsigned long start_log_rpm = 0;

int pwm = 0;
int PwmOffset = PWM_OFFSET; // 1000
int PwmMin = PWM_MIN; //1150
int PwmMax = PWM_MAX; //1950
int LogStep = RPMAMP; //500
boolean ESC32FirstRun = true; 

Servo ESC32;

File LogFile;

float measured[9][1100];
float avOut[9];
int AvLogCount = 0;

void HartBeat(){
  //char cAry[12];
  //sprintf(cAry, "SYS; %10.2f; %10.2f; %10.2f; %10.2f; %10.2f; %10.4E; %10.4E; %5.2f; %5.0f ; %2.0f; %5.0i; %5.0i "  , nU, nI, nTL, nTR, nTH, Mot_A1, Mot_A2, Prop_K1, Mot_Scal, Motor_Poles, PwmOffset, PwmMax );  
  //Serial.println(cAry);
    Serial.print("hBt;");
    Serial.println(StartLog);
   
  }

void setup() {
  // put your setup code here, to run once:

Timer1.attachInterrupt(HartBeat);
//Timer1.start(1000000);

Serial.begin(BAUD); 
Serial1.begin(ESC32BAUD);

analogReadResolution(12);
ESC32.attach(PWM_PIN_OUT); //,PWM_OFFSET,PWM_MAX);
ESC32.writeMicroseconds(PWM_DISARM);

if (USE_SD_READER == 1){
  SDStart();
  }

if (ENCODER_MODE == 0) {
Rpm_Scaler = Motor_Poles;
MaxSystemRPM = 0;
}
if (ENCODER_MODE == 1) {
Rpm_Scaler = Motor_Poles * 2;
MaxSystemRPM = ENCODER_MODE_MAXRPM;
}

  
CliStart();
Timer1.start(1000000);
StartReadRpm();
}

void loop() {

  SetTimers();
  //counter++ ;
  
  ListenSerial();
  
  UpdateInterface();
  
     if (StartLog == 1){
       SDOpenFile("QuatLog.txt");
       LogQuatos();
       StartLog = 0;
     }  
     
     if (StartLog == 2){
        SDOpenFile("Rpm2Thr.txt");
        LogRpm2Thrust();
        StartLog = 0;
        }
     if (StartLog == 3){
        SDOpenFile("SinRpm.txt");
        LogSinRPM();
        StartLog = 0;       
       } 

     if (StartLog == 4){
        SDOpenFile("OLRpm.txt");
        LogOLRPM100Hz() ;
        StartLog = 0;        
     }

     if (StartLog == 5){
        SDOpenFile("CLRpmA.txt");
        LogCLRPM100Hz() ;
        StartLog = 0;        
     }
     
     if (StartLog == 6){
        SDOpenFile("CLRpmB.txt");
        LogCLRPM1000Hz() ;
        StartLog = 0;        
     }
     
     if (StartLog == 7){
        SDOpenFile("CLRpmC.txt");
        LogCLTrust100Hz() ;
        StartLog = 0;        
     }
     
     if (StartLog == 10){
        TareEncoder();
        StartLog = 0;        
     }
    
     if (StartLog == 20){
        SetMaxRpm();
        StartLog = 0;        
     }    
     
     LogFile.close();
     
     if ( (LastStartLog > 0) && (LogRetries < 2) ){
       StartLog = LastStartLog;
       LogRetries ++;
       }
     
     else  {
       
       if ( LogRetries == 2) {
         Serial.println("Giving up, sequence terminated" );  
         SDPrint("Giving up, Sequence terminated");
       }   
       
       StartLog = 0;
       LogRetries =0;
       LastStartLog =0;     
     }
          
}




