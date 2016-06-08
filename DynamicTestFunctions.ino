void LogQuatos(){
  
int ni = 0;
unsigned long sTime = 0;
unsigned long nTime = 0;
unsigned long lTime = 0;
AvLogCount = 0;

//taring sensors
float TareTh = 0;
float TareTl = 0;
float TareTr = 0;
int samples = 600;
int i = 0;
  for (int i=0; i<samples;i++){
    TareTh += float( analogRead(THRUST_PIN) * THRUST_SCALER / ANALOG_RESOLUTION );
    TareTl += float( analogRead(TORQUE_PIN_LEFT) * TORQUE_SCALER_LEFT / ANALOG_RESOLUTION );
    TareTr += float( analogRead(TORQUE_PIN_RIGHT) * TORQUE_SCALER_RIGHT / ANALOG_RESOLUTION );
    delay(5); 
    }
thrust_tare = (TareTh/samples) ;
torque_tare_left  = (TareTl/samples) ;
torque_tare_right = (TareTr/samples) ;

//init
load_cell_status = true;
motor_running = true;
AvLogCount = 0;


ScrPrintHead(0,0);

Serial.println("Logging Data, no Motor commands sent");
Serial.println("Reading sensors @ max speed, Log @ 100Hz");
Serial.println("Process running @ max speed .....");
Serial.println("");
Serial.println("Micros; U(V); I(A); Commutations; RpmMeasured; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  
  
  //Begin TestData
SDPrintHead(0,0);
SDPrintln("Logging Data, no Motor commands sent");
SDPrintln("Reading sensors @ max speed, log @ 100Hz ");
SDPrintln("");
SDPrintln("Micros; U(V); I(A); Commutations; RpmMeasured; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
SDPrintln("");

//StartReadRpm();
SetTimers();

nTime = micros();
  while (StartLog == 1){
     
  ReadSensors(); 
  if (Timer100Hz()) { 
     sTime =  micros() - nTime;
     lTime = sTime - lTime;
     DoLogDirect(1,lTime,0,0,0,0,0);
     lTime = sTime;
     }

  if (Timer1Hz()) { 
    ListenSerial(); 
    UpdateInterface();
    DoLogDirect(101,lTime,0,0,0,0,0);
  }

  }    
    
}



void LogOLRPMold() {
/*
1) Put Esc32 in openLoop mode
a) Give it a 'base' rpm (1000, 2000,...etc)
b) Do a step up and step down test at each of these base rpms by 500 rpm. Log the following Data
time cmd_baseRPM cmd_stepRPM measuredRPM
--> PWM rechnen mit AM1 und AM2
*/
int rpmStep = RPMSTEP;
int r = RPMMIN + 1000;
int rs = 0;
int i = 0;
int n = 0;
unsigned long sTime = 0;
unsigned long nTime = 0;
int pwm = 0;
boolean nblink = false;


if (Mot_Scal == 0) {
   SetMaxRpm();
   }
      
Serial.println("ESC32 armed, Mode set to Open Loop, starting step up and down test.");
Serial.println("Reading sensors @ 200Hz, Log @ 200Hz");
Serial.println("Process running @ 200Hz .....");
Serial.println("");
//Serial.println("Micros; PWM; Rpm; RpmStep; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  

SetTimers(); 
StartReadRpm();
StartReadSensors();
  
StartESC32( 1, getPWMval(r , Mot_Scal ) );
  
  //Begin TestData
SDPrintln("ESC32 armed, Mode set to Open Loop, starting step up and down test.");
SDPrintln("Reading sensors @ 200Hz, Log @ 200Hz");
SDPrintln("");
SDPrintln("Micros; PWM; Rpm; RpmStep; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
SDPrintln("");
  

nTime = micros();
while (( r <= Mot_Scal ) && (r <= MaxSystemRPM) && (StartLog == 4) && SystemHealth()){
  
 // LOG r 
 // if (!Timer05Hz()) {
    

  /*     if(Timer10Hz()){
         ListenSerial();
         UpdateInterface();
     }
  */  
     
     
     ReadSensors(); //Full Speed
     if (Timer200Hz())  {
       sTime =  micros() - nTime;
       //nTime = micros();
       DoLogDirect(4,sTime,pwm,r,rs,0, 0);
     }
  
 // } //!t05Hz
  
 if (Timer05Hz()) { 
    n++;
    
 if (n == 1){
   r -= RPMAMP;
   rs = -RPMAMP;
   }
 else if (n == 2){
   r += 2*RPMAMP;
   rs = 2*RPMAMP;
   }
 else if (n == 3) {
   r += RPMAMP;
   rs = RPMAMP;
   n = 0;
   }
   
   //SystemHealth
      /*counter++ ;
      if ( (counter >10) && (rpm_out < 800)) {
        motor_running = false;
      }
      */
   ListenSerial();
   pwm = getPWMval( r, Mot_Scal)  ; 
   SetMotorPwm( pwm ) ;
    
  }
    
 
 }


//Serial1.println("stop" );
//delay(4000);   
StopESC32();

Serial.println(""); 
Serial.println("Step up Log finishd.");
SDPrintln("");
SDPrintln("Step up Log finishd");

}



void LogOLRPM100Hz() {
/*
1) Put Esc32 in OpenLoop mode
a) Give it a 'base' rpm (1000, 2000,...etc)
b) Do a step up and step down test at each of these base rpms by 500 rpm. Log the following Data
time cmd_baseRPM cmd_stepRPM measuredRPM
--> PWM rechnen mit AM1 und AM2
*/
int rpmStep = RPMAMP; //500
int r = RPMMIN; //1000
int n = 0;
int rPWM = 0;
unsigned long sTime = 0;
unsigned long nTime = 0;
unsigned long lTime = 0;
  char  aAry[128];
  char *cAry;

if (Mot_Scal == 0) {
   SetMaxRpm();
   }
      
ScrPrintHead(r,rpmStep);

Serial.println("ESC32 armed, Mode set to Open Loop, starting step up test.");
Serial.println("Reading sensors @ max speed, Log @ 100Hz");
Serial.println("Process running @ max speed .....");
Serial.println("");
//Serial.println("Micros; PWM; Rpm; RpmStep; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  
StartReadSensors(); 
rPWM = getPWMval(r , Mot_Scal );
StartESC32( 1, rPWM );

  //Begin TestData
SDPrintHead(r,rpmStep);
SDPrintln("ESC32 armed, Mode set to Open Loop, starting step up test.");
SDPrintln("Reading sensors @ max speed, log @ 100Hz ");
SDPrintln("");
SDPrintln("Micros; PWM; RPM; ToL(gr*m); ToR(gr*m); Thrust (gr);" );
SDPrintln("");

//StartReadRpm();
SetTimers();
nTime = micros();
//while (( r <= Mot_Scal ) && (r <= MaxSystemRPM) && (StartLog == 5) && SystemHealth()){
while (( r <= Mot_Scal ) && (StartLog == 4) ){
  ReadSensors();
  if (Timer100Hz()) {
     sTime =  micros() - nTime;
     lTime = sTime - lTime;
     DoLogDirect(4,lTime,rPWM,r,0,0,0);
     lTime = sTime;
     }

  if (Timer1Hz()) { 
    ListenSerial();
    r += rpmStep ; 
    rPWM = getPWMval(r , Mot_Scal );
    SetMotorPwm( rPWM ) ;
    //DoLogDirect(5,0,r,0,0,0,0);
    //nTime = micros(); //reset
    //timer_6 = micros(); //reset
    }
 
 }

StopESC32();

Serial.println(""); 
Serial.println("Step up Log finishd.");
SDPrintln("");
SDPrintln("Step up Log finishd");

}


void LogCLRPM100Hz() { //100Hz
/*
1) Put Esc32 in closedLoop mode
a) Give it a 'base' rpm (1000, 2000,...etc)
b) Do a step up and step down test at each of these base rpms by 500 rpm. Log the following Data
time cmd_baseRPM cmd_stepRPM measuredRPM
--> PWM rechnen mit AM1 und AM2
*/
int rpmStep = LogStep; //500
int r = RPMMIN; //1000
int n = 0;
unsigned long sTime = 0;
unsigned long nTime = 0;
unsigned long lTime = 0;
  char  aAry[128];
  char *cAry;

if (Mot_Scal == 0) {
   SetMaxRpm();
   }
   
ScrPrintHead(r,rpmStep);

Serial.println("ESC32 armed, Mode set to Closed Loop, starting step test.");
Serial.println("Reading sensors @ max speed, Log @ 100Hz");
Serial.println("Process running @ max speed .....");
Serial.println("");
//Serial.println("Micros; PWM; Rpm; RpmStep; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  
StartReadSensors();  
StartESC32( 2, r );
  
  //Begin TestData
SDPrintHead(r,rpmStep);
SDPrintln("ESC32 armed, Mode set to Closed Loop, starting step test.");
SDPrintln("Reading sensors @ max speed, log @ 100Hz ");
SDPrintln("");
SDPrintln("Micros; RPM; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
SDPrintln("");
  
SetTimers(); 
nTime = micros();
//while (( r <= Mot_Scal ) && (r <= MaxSystemRPM) && (StartLog == 5) && SystemHealth()){
  
while (( r <= Mot_Scal ) && (r >= RPMMIN) && (StartLog == 5) ){  
  ReadSensors(); 
  if (Timer100Hz()) { 
     sTime =  micros() - nTime;
     lTime = sTime - lTime;
     DoLogDirect(5,lTime,r,0,0,0,0);
     lTime = sTime;
     }

  if (Timer1Hz()) { 
    ListenSerial();
    
    if ( n == 0 ){   
      r += rpmStep ; 
      }
    
    if (r >= Mot_Scal){
      r -= rpmStep ; 
      n=1;
      }
      
    if ( n == 1 ){   
      r -= rpmStep ; 
      }    
      
//    Serial.print("RPM set to ");   
//    Serial.println(r);   
//    delay(10);
    SetMotorRpm( r ) ;
    //DoLogDirect(5,0,r,0,0,0,0);
    //nTime = micros(); //reset
    //timer_6 = micros(); //reset
    }
 
 }

StopESC32();

Serial.println(""); 
Serial.println("Step Log finishd.");
SDPrintln("");
SDPrintln("Step Log finishd");
}



void LogCLRPM1000Hz() { //1000Hz
/*
1) Put Esc32 in closedLoop mode
a) Give it a 'base' rpm (1000, 2000,...etc)
b) Do a step up and step down test at each of these base rpms by 500 rpm. Log the following Data
time cmd_baseRPM cmd_stepRPM measuredRPM
--> PWM rechnen mit AM1 und AM2
*/
int rpmStep = RPMAMP; //500
int r = RPMMIN; //1000
int n = 0;
unsigned long sTime = 0;
unsigned long nTime = 0;
unsigned long lTime = 0;
  char  aAry[128];
  char *cAry;

if (Mot_Scal == 0) {
   SetMaxRpm();
   }
ScrPrintHead(r,rpmStep);

Serial.println("ESC32 armed, Mode set to Closed Loop, starting step up test.");
Serial.println("Reading sensors @ max speed, Log @ 1000Hz");
Serial.println("Process running @ max speed .....");
Serial.println("");
//Serial.println("Micros; PWM; Rpm; RpmStep; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  
StartReadSensors();  
StartESC32( 2, r );
 
SDPrintHead(r,rpmStep);

  //Begin TestData
SDPrintln("ESC32 armed, Mode set to Closed Loop, starting step up test.");
SDPrintln("Reading sensors @ max speed, log @ 1000Hz ");
SDPrintln("");
SDPrintln("Micros; RPM; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
SDPrintln("");
  
SetTimers(); 
nTime = micros();
//while (( r <= Mot_Scal ) && (r <= MaxSystemRPM) && (StartLog == 5) && SystemHealth()){
while (( r <= Mot_Scal ) && (StartLog == 6) ){  
  ReadSensors(); 
  if (Timer1000Hz()) { 
     sTime =  micros() - nTime;
     lTime = sTime - lTime;
     DoLogDirect(5,lTime,r,0,0,0,0);
     lTime = sTime;
     }

  if (Timer1Hz()) { 
    ListenSerial();
    r += rpmStep ; 
    SetMotorRpm( r ) ;
    //DoLogDirect(5,0,r,0,0,0,0);
    //nTime = micros(); //reset
    //timer_6 = micros(); //reset
    }
 
 }

StopESC32();

Serial.println(""); 
Serial.println("Step up Log finishd.");
SDPrintln("");
SDPrintln("Step up Log finishd");
}





void LogCLTrust100Hz() { //100Hz
/*
1) Put Esc32 in closedLoop mode
a) Give it a 'base' rpm (1000, 2000,...etc)
b) Do a step up and step down test at each of these base rpms by 500 rpm. Log the following Data
time cmd_baseRPM cmd_stepRPM measuredRPM
--> PWM rechnen mit AM1 und AM2
*/
int THStep = 20; //500
int r = 20; //1000
int n = 0;
unsigned long sTime = 0;
unsigned long nTime = 0;
unsigned long lTime = 0;
  char  aAry[128];
  char *cAry;

//if (Mot_Scal == 0) {
//   SetMaxRpm();
//   }
   
ScrPrintHead(r,THStep);

Serial.println("ESC32 armed, Mode set to Closed Loop Thrust Mode, starting step up test.");
Serial.println("Reading sensors @ max speed, Log @ 100Hz");
Serial.println("Process running @ max speed .....");
Serial.println("");
//Serial.println("Micros; PWM; Rpm; RpmStep; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  
StartReadSensors();  
StartESC32( 3, r );
  
  //Begin TestData
SDPrintHead(r,THStep);
SDPrintln("ESC32 armed, Mode set to Closed Loop Thrust Mode, starting step up test.");
SDPrintln("Reading sensors @ max speed, log @ 100Hz ");
SDPrintln("");
SDPrintln("Micros; Thrust; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
SDPrintln("");
  
SetTimers(); 
nTime = micros();
//while (( r <= Mot_Scal ) && (r <= MaxSystemRPM) && (StartLog == 5) && SystemHealth()){
while (( r <= Mot_Scal ) && (StartLog == 7) ){  
  ReadSensors(); 
  if (Timer100Hz()) { 
     sTime =  micros() - nTime;
     lTime = sTime - lTime;
     DoLogDirect(7,lTime,r,0,0,0,0);
     lTime = sTime;
     }

  if (Timer1Hz()) { 
    ListenSerial();
    r += THStep ; 
    SetMotorThrust( r ) ;
    //DoLogDirect(5,0,r,0,0,0,0);
    //nTime = micros(); //reset
    //timer_6 = micros(); //reset
    }
 
 }

StopESC32();

Serial.println(""); 
Serial.println("Step up Log finishd.");
SDPrintln("");
SDPrintln("Step up Log finishd");
}




void LogCLRPMDirectOld() { //1000Hz
/*
1) Put Esc32 in closedLoop mode
a) Give it a 'base' rpm (1000, 2000,...etc)
b) Do a step up and step down test at each of these base rpms by 500 rpm. Log the following Data
time cmd_baseRPM cmd_stepRPM measuredRPM
--> PWM rechnen mit AM1 und AM2
*/
int rpmStep = RPMAMP; //500
int r = RPMMIN; //1000
int n = 0;
unsigned long sTime = 0;
unsigned long nTime = 0;
  char  aAry[128];
  char *cAry;

if (Mot_Scal == 0) {
   SetMaxRpm();
   }
      
Serial.println("ESC32 armed, Mode set to Closed Loop, starting step up test.");
Serial.println("Reading sensors @ 1000Hz, Log @ 1000Hz");
Serial.println("Process running @ max speed .....");
Serial.println("");
//Serial.println("Micros; PWM; Rpm; RpmStep; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  
StartReadSensors();  
StartESC32( 2, r );
  
  //Begin TestData
SDPrintln("ESC32 armed, Mode set to Closed Loop, starting step up test.");
SDPrintln("Reading sensors @ 1000Hz, log @ 1000Hz ");
SDPrintln("");
SDPrintln("Micros; RPM; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
SDPrintln("");
  
StartReadRpm();
SetTimers(); 
nTime = micros();
//while (( r <= Mot_Scal ) && (r <= MaxSystemRPM) && (StartLog == 5) && SystemHealth()){
while (( r <= Mot_Scal ) && (StartLog == 6) ){
  
 if (Timer1000Hz()) { 
     sTime =  micros() - nTime;
     ReadSensorsDirect();
     DoLogDirectFast(5,sTime,r,0,0,0,0);
 }
  
 if (Timer1Hz()) { 
   ListenSerial();
   r += rpmStep ; 
   SetMotorRpm( r ) ;
   DoLogDirectFast(5,0,r,0,0,0,0);
   nTime = micros(); //reset
   timer_6 = micros(); //reset
  }
 
 }

StopESC32();

Serial.println(""); 
Serial.println("Step up Log finishd.");
SDPrintln("");
SDPrintln("Step up Log finishd");

}

void LogSinRPM() {
/*
c) Do a sinusoid based around the target rpm with an amplitude of 500 and varying input frequencies. Log the Data
time cmd_baseRPM cmd_Amplitude cmd_frequency measuredRPM
This will help identify the open loop characteristics of the Esc32+Motor+Prop combination and we could choose the 'right' constants for the Esc32 closed loop mode.
*/
float rpmstep = RPMSTEP;
float w = 0;
unsigned long nTime = 0;
unsigned long sTime = 0;
unsigned long lTime = 0;
int pwOut = 0;
int pwStep = 0;
int r = 3000;
float rpm = 0;
int exFreq = EXFREQ; //0-5
float i = 0;
boolean nblink = false;


if (Mot_Scal == 0) {
   SetMaxRpm();
   }
ScrPrintHead(r,RPMAMP);   
Serial.println("ESC32 armed, Mode set to Open Loop, starting Sinusoidal RPM log over PWM in 2 secs");
Serial.println("Reading sensors @ max speed, Log @ 100Hz");
Serial.println("Process running @ max speed .....");
Serial.println("");
//Serial.println("Millis; RpmAmpl; pwOut; Rpm; w; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  



StartReadSensors();
pwOut = getPWMval(r, Mot_Scal);  
StartESC32( 1, pwOut );
  
  //Begin TestData
SDPrintHead(r,RPMAMP);  
SDPrintln("ESC32 armed, Mode set to Open Loop, starting Sinusoidal RPM log over PWM");
SDPrintln("Reading sensors @ max speed, Log @ 100Hz");
SDPrintln("");
SDPrintln("Millis; RpmCmd; pwOut; RpmBase; w; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
SDPrintln("");

nTime = millis(); 
SetTimers(); 

while (( r <= Mot_Scal ) && (StartLog ==3) && SystemHealth()){
  
  nTime = millis();
  i = 0;
  while (i <= exFreq*5) {
      
      sTime = millis() - nTime;
      //RPM & Puls can only be measured @ 10Hz
      if (Timer10Hz())  {
        ListenSerial();
        rpm =  getSinusoidRPM(r, Mot_Scal, RPMAMP, i/5, sTime );       
        pwOut = getPWMval(rpm, Mot_Scal);
        SetMotorPwm( pwOut ) ;
        //SystemHealth
        // Raise i @ 10Hz
        i++;
        } //end 10 Hz
      
      ReadSensors(); //Full Speed
      if (Timer100Hz()) {
        DoLogDirect(3, sTime, rpm, pwOut, r, i/5, 0);
        }

     }

     if (Timer1Hz()) {
        r += rpmstep;
        }    


   }
//delay(100);
//Serial1.println("stop" );
//delay(4000);   
StopESC32();
     
Serial.println(""); 
Serial.println("Sinusoidal RPM log over PWM finishd.");
Serial.println(""); 

SDPrintln("");
SDPrintln("Sinusoidal RPM log over PWM finishd.");
SDPrintln("");

}// END LogSinRPM()








void LogRpm2Thrust(){
  /*
  3. put the Esc32 in CL mode using constants derived from step1 and run the static tests to determine
  MOT_A1, MOT_A2 and PROP_K1
  */
  char cAry[256];
  int IdleTime = 1500; // millis 
  int LogTime = 500;  // millis
  int StartRpm = RPMMIN;
  int EndRpm = Mot_Scal;
  int StepRpm = 200;
  int TargetRpm = StartRpm;
  int i = 1;
  int n = 0;
  int xy[8][200];
  float avQ[10][200];
  float a=0 ,b=0, c=0, d=0;
  float sumTL = 0;
  float sumTR = 0; 
  float sumTH = 0; 
  int iq = 0;
  float x1=0, y1=0, s1=0, s2=0, s3=0, s4=0, s5=0;
  int x = 0;
  boolean RunOnce = true;
  
  unsigned long  LocalTimer = 0; 
  unsigned long  LocalTimeStamp = 0;
  
  float measuredAv[9][LogTime/9];
  int mAvLoops = 0;
  
  if (Mot_Scal == 0) {
   SetMaxRpm();
   }
   
  if (Mot_Scal > 10000) {
   StepRpm = 500;
  }  
   
  ScrPrintHead(StartRpm, StepRpm);
 
  Serial.println("ESC armed, starting thrust log in 2 secs");
  Serial.println(""); 
  Serial.println("RPMin; PWM; RPMeff; ToL(gr*m); ToR(gr*m); Thrust (gr)");
  Serial.println("-----------------------------------------------------");
  
  SDPrintHead(StartRpm, StepRpm);
  
   //Print Header
    
  SetTimers(); 
  //StartReadRpm();
  StartReadSensors();
  StartESC32( 2, StartRpm);
  
  //Begin TestData
  SDPrintln("ESC armed, starting thrust log...");
  SDPrintln("");
  SDPrintln("RPMin; PWM; RPMeff; ToL(gr*m); ToR(gr*m); Thrust (gr)");
  SDPrintln("-----------------------------------------------------");
  LocalTimer = millis();
  
  while (( i <= 100 ) && (StartLog == 2) && SystemHealth()){
    

    if (Timer2Hz()){
      ListenSerial();
      }
      
    LocalTimeStamp = millis() - LocalTimer;
    if (LocalTimeStamp <= IdleTime ) {
      
    }
        
    if ( (LocalTimeStamp > IdleTime ) && (LocalTimeStamp < (IdleTime + LogTime) ) ){
       
      ReadSensors();  
      
      if (RunOnce == true){
        StartReadRpm();
        RunOnce = false;
        }

     if ( Timer100Hz() ) {
         //DoLog(2,TargetRpm,rpm_in,0,0,0,0);
         
         getSensorAv(); 
         //save average
         
         for (int ia=0;ia<9;ia++){  
           measuredAv[ia][mAvLoops] = avOut[ia]; 
         }
         //avOut[R] = ReadRPM();
         mAvLoops++;
/*         
         Serial.print("AvLoops "); 
         Serial.print(mAvLoops); 
         Serial.print(" SensoReads "); 
         Serial.print(x); 
         Serial.print("  timer_3 "); 
         Serial.println(millis()-timer_3);
        
*/
       }
         
    
      }
      
    if (LocalTimeStamp >= (IdleTime + LogTime) ){
      
       rpm_out = ReadRpm();
       RunOnce = true; 
       //write saved average to measured
       for (int na=0; na<mAvLoops ;na++){
          for (int ia=0; ia<9 ;ia++){  
            measured[ia][na] = measuredAv[ia][na];
            measuredAv[ia][na]= 0.0;
            }  
       }
       AvLogCount = mAvLoops;
       mAvLoops = 0;
       //end write
           
       DoLog(2,TargetRpm,0,0,0,0,0); 
          
       xy[0][i] = TargetRpm ;
       xy[1][i] = int(thrust_out) ; 
       xy[2][i] = int(torque_out_left*1000 );
       xy[3][i] = int(torque_out_right*1000);  
       xy[4][i] = rpm_out;    
       
       TargetRpm = StartRpm + i*StepRpm;
       if (TargetRpm > Mot_Scal) {
          break;
         }
            
       SetMotorRpm( TargetRpm ) ;
       ESC32Rpm = TargetRpm;
       i++;
       LocalTimer = millis();
      
      }
      
   }
     
     
     //Serial1.println("stop" );
     //delay(4000);   
     StopESC32();
          

          
     if (StartLog == 0){
        Serial.println("");
        Serial.println("Sequence stopped manually");
        SDPrintln("");
        SDPrintln("Sequence stopped manually");     
       return;
       }
       

     
     
     //TestData
     //xy[0] = {1000, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500};
     //xy[1] = {   8,   32,   52,   77,  107,  144,  186,  236,  290,  357,  423,  499,  580,  668,  763,  863,  970};
     //i=16;
     //End TestData
     
     //Scheitelpunkt
     //xy[0][0] = 0;
     //xy[1][0] = 0;
     
     //Koord 1
     
     Serial.println("");
     Serial.println("Calculating.....");
     SDPrintln("");
     SDPrintln("");
     
     //Least Squares Regression for Quadratic Curve Fitting
     iq = 0;
     for (int ni = 1; ni <= (i-1); ni++) { //skip first and last value
       x1 = float( xy[0][ni] ); //x1
       y1 = float( xy[1][ni] ); //y1
      
       //DoLog(21,x1,y1,0,0,0,0);
       
       avQ[0][iq] = x1*y1;
       avQ[1][iq] = x1*x1*x1*x1;       
       avQ[2][iq] = x1*x1*x1;        
       avQ[3][iq] = x1*x1*y1;
       avQ[4][iq] = x1*x1;
       
       if ((ni >= i/4) && (ni <= i/4*3)) { 
         sumTH += float(xy[1][ni]);
         sumTL += float(xy[2][ni])/1000;
         sumTR += float(xy[3][ni])/1000;
       }
       
       iq++;
     }
     iq--;
     //building sum of terms
     for (int ni = 0; ni <= iq; ni++) {
       s1 +=  avQ[0][ni];
       s2 +=  avQ[1][ni];
       s3 +=  avQ[2][ni];
       s4 +=  avQ[3][ni];
       s5 +=  avQ[4][ni];
       //DoLog(21,s1,s2,s3,s4,s5,0);
     }

     a = (s1*s2 - s3*s4)/(s5*s2 - s3*s3); //a*x
     b = (s5*s4 - s1*s3)/(s5*s2 - s3*s3); //b*x^2
     c = sumTH/sumTL; 
     d = sumTH/sumTR;
     Serial.println("");
     Serial.println("Least Squares Regression for Quadratic Curve Fitting");
     Serial.println("Values based on input RPM");
     Serial.println("MOT_VALUE2T_A1(x);   MOT_VALUE2T_A2(x^2);  PROP_K1 (CCW); PROP_K1 (CW) ");
     
     SDPrintln("");
     SDPrintln("Least Squares Regression for Quadratic Curve Fitting");
     SDPrintln("Values based on input RPM");
     SDPrintln("MOT_VALUE2T_A1(x);   MOT_VALUE2T_A2(x^2);   PROP_K1 (CCW); PROP_K1 (CW) ");
     
     DoLog(22,a,b,c,d,0,0); 
     //Serial.println("Thrust Log finishd.");
     Mot_A1 = a;
     Mot_A2 = b;
     Prop_K1 = c;
     
     //*** Gemessene RPM
          //Least Squares Regression for Quadratic Curve Fitting
     iq = 0;
     for (int ni = 1; ni <= (i-1); ni++) { //skip first and last value
       x1 = float( xy[4][ni] ); //x1 (gemessene RPM)
       y1 = float( xy[1][ni] ); //y1
      
       //DoLog(21,x1,y1,0,0,0,0);
       
       avQ[0][iq] = x1*y1;
       avQ[1][iq] = x1*x1*x1*x1;       
       avQ[2][iq] = x1*x1*x1;        
       avQ[3][iq] = x1*x1*y1;
       avQ[4][iq] = x1*x1;
       
       if ((ni >= i/4) && (ni <= i/4*3)) { 
         sumTH += float(xy[1][ni]);
         sumTL += float(xy[2][ni])/1000;
         sumTR += float(xy[3][ni])/1000;
       }
       
       iq++;
     }
     iq--;
     //building sum of terms
     for (int ni = 0; ni <= iq; ni++) {
       s1 +=  avQ[0][ni];
       s2 +=  avQ[1][ni];
       s3 +=  avQ[2][ni];
       s4 +=  avQ[3][ni];
       s5 +=  avQ[4][ni];
       //DoLog(21,s1,s2,s3,s4,s5,0);
     }

     a = (s1*s2 - s3*s4)/(s5*s2 - s3*s3); //a*x
     b = (s5*s4 - s1*s3)/(s5*s2 - s3*s3); //b*x^2
     c = sumTH/sumTL; 
     d = sumTH/sumTR;
     Serial.println("");
     Serial.println("Least Squares Regression for Quadratic Curve Fitting");
     Serial.println("Values based on measured RPM");
     Serial.println("MOT_VALUE2T_A1(x);   MOT_VALUE2T_A2(x^2);  PROP_K1 (CCW); PROP_K1 (CW) ");
     
     SDPrintln("");
     SDPrintln("Least Squares Regression for Quadratic Curve Fitting");
     SDPrintln("Values based on measured RPM");
     SDPrintln("MOT_VALUE2T_A1(x);   MOT_VALUE2T_A2(x^2);   PROP_K1 (CCW); PROP_K1 (CW) ");
     
     DoLog(22,a,b,c,d,0,0); 
     Serial.println("Thrust Log finishd.");
     Mot_A1 = a;
     Mot_A2 = b;
     Prop_K1 = c;
     
     
   }
  
  
int getSinusoidRPM(float rpmBase, float rpmMax, float Amp, float w, float ntime){
 float rpm;
 rpm = rpmBase + Amp*sin(w*ntime);
 constrain(rpm, rpmBase, rpmMax);
return rpm;
}


int getPWMval(float rpm, float rpmMax){
 float val;
 if (rpmMax < 2000){
   return PwmOffset;
   }
 
 val = rpm/rpmMax;
 return int ( (val* (PwmMax-PwmOffset) ) + PwmOffset );// I have assumed pwmMax is 950!
}

int getRPMSinusoidSetPoint(float rpmBase, float rpmMax, float Amp, float w, float ntime){
 float rpm;
 rpm = rpmBase + Amp*sin(w*ntime);
 constrain(rpm, rpmBase, rpmMax);
 return getPWMval(rpm, rpmMax);

}

int getRPMStepSetPoint(float rpmBase, float rpmMax, float nstep){

 float rpm;
 rpm = rpmBase + nstep;
 return getPWMval(rpm, rpmMax);

}

float thrust2RPM(float thrust, float A1, float A2){

 float rpm;
 rpm = (-A1 +sqrtf(A1*A1+4*A2*thrust))/(2*A2);
 return rpm;

}

int getThrustSinusoidSetPoint(float thrustBase, float rpmMax, float Amp, float w, float time){
 float rpm;
 float thrust;
 thrust = thrustBase + Amp*sin(w*time);
 rpm = thrust2RPM(thrust, MOT_A1, MOT_A2);
 if (rpm > rpmMax){
    rpm = rpmMax;
 }
 return getPWMval(rpm, rpmMax);

}

int getThrustStepSetPoint(float thrustBase, float rpmMax, float nstep){

 float thrust;
 thrust = thrustBase + nstep;
 float rpm = thrust2RPM(thrust, MOT_A1, MOT_A2);
 if (rpm > rpmMax){
 rpm = rpmMax;
 }
 return getPWMval(rpm, rpmMax);

}




