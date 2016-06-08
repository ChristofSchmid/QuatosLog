/*
const cliCommand_t cliCommandTable[] = {
    {"arm", "", cliFuncArm},
    {"beep", "<frequency> <duration>", cliFuncBeep},
    {"binary", "", cliFuncBinary},
    {"bootloader", "", cliFuncBoot},
    {"config", "[READ | WRITE | DEFAULT]", cliFuncConfig},
    {"disarm", "", cliFuncDisarm},
    {"duty", "<percent>", cliFuncDuty},
    {"help", "", cliFuncHelp},
    {"input", "[PWM | UART | I2C | CAN]", cliFuncInput},
    {"mode", "[OPEN_LOOP | RPM | THRUST | SERVO]", cliFuncMode},
    {"pos", "<degrees>", cliFuncPos},
    {"pwm", "<microseconds>", cliFuncPwm},
    {"rpm", "<target>", cliFuncRpm},
    {"set", "LIST | [<PARAMETER> <value>]", cliFuncSet},
    {"start", "", cliFuncStart},
    {"status", "", cliFuncStatus},
    {"stop", "", cliFuncStop},
    {"telemetry", "<Hz>", cliFuncTelemetry},
    {"version", "", cliFuncVer} 
    
    */
    
void SendToSerialStr( char *val ){
  Serial1.println( val );
  return;
  }
 
void SendToSerialStr2( char *val, int n ){
  //Serial1.print( val );
  Serial1.print( val );
  Serial1.println( n );
return;
}
    
void SendToSerialIntNL( int val ){
  //Serial1.println( val);
  Serial1.println( val );
  return;
  }
    
void SetTimers(){
timer_1 = 0;
timer_2 = 0;
timer_3 = 0;
timer_4 = 0;
timer_5 = 0;
timer_6 = 0;
timer_7 = 0;
timer_8 = 0;
counter = 0;
}

bool SysTimer5Hz(){
  // 0.5Hz
  int RetVal = 0;
  if ( (millis() - timer_sys) >= 200 ) {
    timer_sys = millis();
    return true;
  }
  return false;
}

bool Timer05Hz(){
  // 0.5Hz
  int RetVal = 0;
  if ( (millis() - timer_4) >= 2000 ) {
    timer_4 = millis();
    return true;
  }
  return false;
}




bool Timer1Hz(){
  // 1Hz
  int RetVal = 0;
  if ( (millis() - timer_1) >= 1000 ) {
    timer_1 = millis();
    return true;
  }
  return false;
}

bool Timer2Hz(){
  // 1Hz
  int RetVal = 0;
  if ( (millis() - timer_8) >= 500 ) {
    timer_8 = millis();
    return true;
  }
  return false;
}

bool Timer10Hz(){
  // 10Hz
  int RetVal = 0;
  if ( (millis() - timer_2) >= 100 ) {
    timer_2 = millis();
    return true;
  }
  return false;
}

bool Timer100Hz(){
  // 100Hz
  int RetVal = 0;
  if ( (millis() - timer_3) >= 10 ) {
    timer_3 = millis();
    return true;
  }
  return false;
}

bool Timer200Hz(){
  // 10Hz
  int RetVal = 0;
  if ( (millis() - timer_5) >= 5 ) {
    timer_5 = millis();
    return true;
  }
  return false;
}

bool Timer500Hz(){
  // 0.5Hz
  int RetVal = 0;
  if ( (micros() - timer_7) >= 2000 ) {
    timer_7 = micros();
    return true;
  }
  return false;
}

bool Timer1000Hz(){
  // 10Hz
  int RetVal = 0;
  if ( (micros() - timer_6) >= 1000 ) {
    timer_6 = micros();
    return true;
  }
  return false;
}

 
void StartESC32( int mode, int r ){
  //This happens in the ESC32 Cli already.
    if (mode == 1) {   //OL
    SendToSerialStr("startup_mode 0");
    SendToSerialStr("torque_mode 0");
    //SendToSerialStr("mode OPEN_LOOP");
    SendToSerialStr("config write");
    Serial.println("ESC32 set to Open Loop");
    }
    
    if (mode == 2) {  //CL
    SendToSerialStr("startup_mode 1");
    SendToSerialStr("torque_mode 0");
    //SendToSerialStr("mode RPM");
    SendToSerialStr("config write");
    Serial.println("ESC32 set to Closed Loop");

    }
    if (mode == 3) {   //TH
    SendToSerialStr("startup_mode 1");
    SendToSerialStr("torque_mode 1");
    SendToSerialStr("config write");
    Serial.println("ESC32 set to Thrust Mode");
    }
    
  //delay(500);  
  //ListenESC32();
  
  SendToSerialStr("arm" );
  //delay(200);
  SendToSerialStr("start" ); //Default 2000 RPM
  ESC32.writeMicroseconds(PWM_MIN);
  //ListenESC32();
  delay(500);
  
  if (mode == 1) {
  pwm = r;  
  SendToSerialStr2("pwm ", r);
  ESC32.writeMicroseconds(r);
  }

  if (mode == 2) {
  pwm = getPWMval(r , Mot_Scal );  
  SendToSerialStr2("rpm ", r);
  ESC32.writeMicroseconds(pwm); 
  }
  
  if (mode == 3) {
  //pwm = getPWMval(r , Mot_Scal );  
  SendToSerialStr2("thrust ", r);
  //ESC32.writeMicroseconds(pwm); 
  }
  

  //SendToSerialStr("status" ); 
  //ListenESC32();
 
 delay( 1500 );
   
  }
  
  
void StopESC32(){
  SendToSerialStr("stop" );
  ESC32.writeMicroseconds(PWM_ARM);
  delay(4000);
  SendToSerialStr("disarm" );
  detachInterrupt(RPM_PIN);
  //ESC32.writeMicroseconds(PWM_DISARM);
  }

void ESC32Arm(){
  //SendToSerialStr("telemetry 10");
  delay(10);
  SendToSerialStr("arm");
  ESC32.writeMicroseconds(PWM_ARM);  
  
  // ListenESC32();
  }
  
void ESC32Disarm(){
  //SendToSerialStr("telemetry 0");
  delay(10);
  SendToSerialStr("disarm");
  //ListenESC32();
  ESC32.writeMicroseconds(PWM_DISARM);
  }


void SetMotorRpm( int r){
    pwm = getPWMval(r , Mot_Scal );
    SendToSerialStr2("rpm ", r);
    ESC32.writeMicroseconds(pwm); 
  }
  
 void SetMotorThrust( int r){
    //pwm = getPWMval(r , Mot_Scal );
    SendToSerialStr2("thrust ", r);
    //ESC32.writeMicroseconds(pwm); 
  } 
   
void SetMotorPwm( int p){
    pwm = p;
    SendToSerialStr2("pwm ", p);
    ESC32.writeMicroseconds(p);
    
    //ESC32.writeMicroseconds(p); 
  }  

void RunMotor(int type, int mode, int value){
  if (type == 1) { //serial
    
    if (mode == 1) { //pwm -> open loop
    //SendToSerialStr2("pwm ", value);
    }
    
    else if (mode == 2) { //rpm closed loop
    //SendToSerialStr2("rpm ", value);
    } 

  }
  else if (type == 2) { //pwm line, Set OL or CL

  // send <value> PWM to pin

  }  
  
}

void SetTareThrust(){
  float TareTh = 0;
  float TareTl = 0;
  float TareTr = 0;
  int samples = 600;
  int i = 0;
  
  Serial.println("" );
  Serial.println("taring sensors ....." );
  
  StartESC32( 1, 1200 );
  delay(2000);
  StopESC32();
  
  for (int i=0; i<samples;i++){
    TareTh += float( analogRead(THRUST_PIN) * THRUST_SCALER / ANALOG_RESOLUTION );
    TareTl += float( analogRead(TORQUE_PIN_LEFT) * TORQUE_SCALER_LEFT / ANALOG_RESOLUTION );
    TareTr += float( analogRead(TORQUE_PIN_RIGHT) * TORQUE_SCALER_RIGHT / ANALOG_RESOLUTION );
    //Serial.print("Sensors tare sample " );  
    //Serial.print(i);  
    //Serial.println("" );  
    delay(5); 
    }
  thrust_tare = (TareTh/samples) ;
  torque_tare_left  = (TareTl/samples) ;
  torque_tare_right = (TareTr/samples) ;
  Serial.println("Sensors tared to 0." );  
  SDPrintln("Sensors tared to 0." ); 
}






void getSensorAv(){
  int pi = AvLogCount;
  AvLogCount = 0;
  
  for (int m=0;m<=8;m++) {
    avOut[m] = 0;
  }
  
  if (pi == 0){
    return;
    }

  for (int n=0;n<pi;n++){
    
    for (int m=0;m<=8;m++) {    
      avOut[m] += measured[m][n] ;
      measured[m][n] = 0.0;      
    } 
}
  
  for (int m=0;m<=8;m++) {
     avOut[m] = avOut[m] / float(pi) ;
    }
    

  
   
  //Serial.println(" U(V); I(mAh); ComCount; Rpm(Measured); ToL(grcm); ToL(grcm); Thrust (gr);" );
  //sprintf(cAry, "%3.0u; %7.0f; %6.2f; %6.0f; %6.0f; %6.0f; %6.0f; %6.0f; %6.0f "  , pi, avOut[T], avOut[U], avOut[I], avOut[C], avOut[R], avOut[TL], avOut[TR], avOut[TH]);
  //Serial.println(cAry);

}



void resetSensorAv(){
  int pi = AvLogCount;
  for (int n=0;n<=pi;n++){  
    for (int m=0;m<=8;m++) {    
      measured[m][n] = 0.0;      
    }
  }
  for (int m=0;m<=8;m++) {
    //avOut[m] = avOut[m] / float( (pi+1)-(2*skip) ) ;
    avOut[m] = 0;
  }
  AvLogCount = 0;
}


void StartReadSensors(){
SetTareThrust();
load_cell_status = true;
motor_running = true;
AvLogCount = 0;

}

void ReadSensors(){
 int i = AvLogCount;
// T  0  //Time
// U  1  //Voltage
// I  2  //Current
// R  3  //RPM measured
// TL 4 //Torque Left
// TR 5 //Torque Right
// TH 6 //Thrust
// C  7  //RPM comCount
// CT 8  //Micros 

  measured[T][i] = float( millis() );
  measured[U][i] = float( analogRead(VOLTAGE_PIN) * 1000 / VOLTAGE_SCALER / ANALOG_RESOLUTION );
  measured[I][i] = float( analogRead(CURRENT_PIN) * 1000 / CURRENT_SCALER / ANALOG_RESOLUTION );
  measured[TL][i] = float( analogRead(TORQUE_PIN_LEFT) * TORQUE_SCALER_LEFT / ANALOG_RESOLUTION );
  measured[TR][i] = float( analogRead(TORQUE_PIN_RIGHT) * TORQUE_SCALER_RIGHT / ANALOG_RESOLUTION );
  measured[TH][i] = float( analogRead(THRUST_PIN) * THRUST_SCALER / ANALOG_RESOLUTION );
  measured[C][i] =  float( comCount );
  measured[CT][i] = float( micros() - start_log_rpm );
  //measured[R][i] = ReadRpm();
  
  
  if (measured[TL][i] > LOAD_CELL_MAX_LOAD) {
     load_cell_status = false;
     }
  if (measured[TR][i] > LOAD_CELL_MAX_LOAD) {
     load_cell_status = false;
     }
  if (measured[TH][i] > LOAD_CELL_MAX_LOAD) {
     load_cell_status = false;
     }
  
     
  //Serial.print( "Thrust ");
  //Serial.println(measured[TH][i]);
  AvLogCount++; 
//i++;
}


void ReadSensorsDirect(){
  
  torque_out_left  = float( 3 * ( (analogRead(TORQUE_PIN_LEFT) * TORQUE_SCALER_LEFT / ANALOG_RESOLUTION) -  torque_tare_left) / 100 ) ;
  torque_out_left  = constrain( torque_out_left, 0, torque_out_left);
  torque_out_right = float( 3 * ( (analogRead(TORQUE_PIN_RIGHT) * TORQUE_SCALER_RIGHT / ANALOG_RESOLUTION) - torque_tare_right) / 100 );
  torque_out_right = constrain( torque_out_right, 0, torque_out_right) ;
  thrust_out       = float( (analogRead(THRUST_PIN) * THRUST_SCALER / ANALOG_RESOLUTION ) - thrust_tare );
  thrust_out       = constrain( thrust_out, 0, thrust_out) ;
  //rpm_out          = ReadRpm();
  
  if (torque_out_left > 3 * (LOAD_CELL_MAX_LOAD - torque_tare_left) / 100  ) {
     load_cell_status = false;
     }
  if (torque_out_right > 3 * (LOAD_CELL_MAX_LOAD - torque_tare_right) / 100 ) {
     load_cell_status = false;
     }
  if ( thrust_out  > LOAD_CELL_MAX_LOAD - thrust_tare) {
     load_cell_status = false;
     }

}

void SetMaxRpm(){
  char *ichar; 
  int i = 0;
  int p = PwmMin;
  int pStep = 50;
  int rpm = 0;
  unsigned long nTime = 0;
  unsigned long sTime = 0;
  boolean RunOnce = false;
  Mot_Scal = 0;
  MaxRpmLogged = 1000;
  //StartESC32( 1 , p );

  Serial.println("Testing Max RPM ...." );  
  Serial.println("sTime; Pwm; RpmMeasured; ToL(gr*m); ToR(gr*m); Thrust (gr);" );  
  if (StartLog != 20){
  SDPrintln("Testing Max RPM ...." ); 
  SDPrintln("sTime; Pwm; RpmMeasured; ToL(gr*m); ToR(gr*m); Thrust (gr);" ); 
  }
  SetTimers();
  StartReadSensors();
  StartESC32( 1 , p );
  //SetMotorPwm( p ) ; 
  
  
  //Begin TestData
  
  nTime = millis();
  
    while ( ( p <= PwmMax ) && (StartLog > 0) && SystemHealth() ){ 
      
      if (Timer200Hz()) {
        ListenSerial();
        ReadSensors();
        }
      if (RunOnce == false) {
        StartReadRpm();
        RunOnce = true;
        }
        
      if (Timer2Hz())  {
        sTime = millis() - nTime;
        rpm_out = ReadRpm();
        DoLog(9,sTime,p,rpm,0,0,0);
        if (rpm_out > MaxRpmLogged){
           MaxRpmLogged = rpm_out;
           }
        p += pStep ;
        SetMotorPwm( p ) ;  
        
        }
                    
      }
      

      Mot_Scal = MaxRpmLogged;
      MaxSystemRPM = MaxRpmLogged;
      
      if (ENCODER_MODE == 1) {
         MaxSystemRPM = constrain( MaxSystemRPM, MaxSystemRPM, ENCODER_MODE_MAXRPM);
      }

      SetMotorPwm( PwmMin ) ;  
      delay(1000);
      StopESC32();
      Serial.print("Max RPM set to " );
      Serial.println( MaxRpmLogged );  
      Serial.println("");
      Serial.println("RPM test finished.");   
      
      if (StartLog != 20){
      SDPrint("Max RPM set to " ); 
      SDPrintln( i2str(MaxRpmLogged, ichar) ); 
      Serial.println("Starting Log in 2 Sec, stand clear"); 
      
      }
      delay(2000);
      

}

  
  
void DoLog(int lstat, float f1, float f2, float f3,float f4, float f5, float f6){
  
  float LogFreq = 0;
  float LogTime = 0;
  float ComCnt = 0;
  
  char cAry[128];
  
  getSensorAv();
  
  counter++ ;

  LogTime =avOut[T];
  
  voltage_out = avOut[U];
  current_out = avOut[I];
  //rpm_out = avOut[R]; //ReadRPM
  torque_out_left = 3 * ( avOut[TL] - torque_tare_left) /100;  // gr*m
  torque_out_left = constrain( torque_out_left, 0, torque_out_left);
  
  torque_out_right = 3 * ( avOut[TR] - torque_tare_right) /100; // gr*m
  torque_out_right = constrain( torque_out_right, 0, torque_out_right);
  
  thrust_out = avOut[TH] - thrust_tare;
  thrust_out = constrain( thrust_out, 0, thrust_out);
  
  ComCnt = avOut[C];
  
   if ( (counter >10) && (rpm_out < 800)) {
     motor_running = false;
     }
  
  
  //LogFreq = 1000 / constrain( (millis() - timer_1),1, 1000);
  
//  if (rpm_out > MaxRpmLogged){
//    MaxRpmLogged = rpm_out;
//    }



if (lstat == 1){  
  
  //Serial.println(" U(V); I(mAh); ComCount; Rpm(Measured); ToL(grcm); ToL(grcm); Thrust (gr); Tare" );
  sprintf(cAry, "%6.2f; %6.2f; %6.0f; %6.0f; %8.4f; %8.4f; %6.0f  "  , voltage_out, current_out, ComCnt, rpm_out , torque_out_left, torque_out_right, thrust_out);
  //Serial.print(cAry);  
  }
  
  if (lstat == 2){ //RPM 2 Thrust 
  //Serial.println("RPMin; RPMeff; ToL(grcm); ToR(grcm); Thrust (gr)");
  //DoLog(2,TargetRpm,Rpm_in,0,0,0,0);
  sprintf(cAry, "%6.0f; %6.0i; %6.0f; %8.4f; %8.4f; %6.0f "  , f1, pwm, rpm_out, torque_out_left, torque_out_right, thrust_out);
  //Serial.print(cAry);  
  }
  
  if (lstat == 21){ //RPM 2 Thrust LastLine
  //Serial.println("RPMin; RPMeff; ToL(grcm); ToR(grcm); Thrust (gr)");
  //DoLog(2,TargetRpm,Rpm_in,0,0,0,0);
  sprintf(cAry, "%12.8E; %12.8E; %12.8E; %12.8E; %12.8E "  , f1, f2, f3, f4, f5);
  //Serial.print(cAry);  
  }
  
    if (lstat == 22){ //RPM 2 Thrust LastLine 2
  
  //DoLog(2,a,b,c,0,0,0);
  sprintf(cAry, "%12.8E; %12.8E; %6.2f; %6.2f " , f1, f2, f3, f4);
  //Serial.print(cAry);  
  }
  
if (lstat == 3){  //SinRPM
  //DoLog(3, sTime, ltTime, pwOut, RPMAMP, w, 0 );
  // DoLog(3, sTime, RPMAMP, pwOut, r, i/5, 0 );  
  sprintf(cAry, "%9.0f; %6.0f; %6.0f; %6.0f; %6.2f; %6.0f ; %8.4f; %8.4f; %6.0f"  , f1, f2 , f3, f4, f5, rpm_out, torque_out_left, torque_out_right, thrust_out);
  //Serial.print(cAry);  
  }

if (lstat == 4){  //RPM step
  // Serial.println("sTime; PWM; Rpm; RpmStep; ToL(gr*m); ToL(gr*m); Thrust (gr)" );
  //DoLog(4, sTime, GetPWVal( r, mot_Scal), r, rs ,0 , 0 ); 
  sprintf(cAry, "%9.0f; %6.0f; %6.0f; %6.0f; %8.4f; %8.4f; %6.0f"  , f1, f2 , f3, f4, torque_out_left, torque_out_right, thrust_out);
  //Serial.print(cAry);  
  }
  
  if (lstat == 9){  //Max RPM
  // Serial.println("sTime; Rpm; RpmMeasured; ToL(grcm); ToL(grcm); Thrust (gr);" );  
  sprintf(cAry, "%8.0f; %6.0f; %6.0f; %8.4f; %8.4f; %6.0f"  , f1, f2 , rpm_out, torque_out_left, torque_out_right, thrust_out);
  //Serial.print(cAry);  
  }
 

  Serial.println(cAry);
  SDPrintln(cAry);
  UpdateInterface();
  
  }
 
 
 void DoLogDirect(int lstat, float f1, float f2, float f3,float f4, float f5, float f6){
  char  aAry[128];
  char *cAry;
  float LogFreq = 0;
  float LogTime = 0;
  float ComCnt = 0;
  
  getSensorAv();
  
  counter++ ;

  LogTime =avOut[T];
  
  voltage_out = avOut[U];
  current_out = avOut[I];
  //rpm_out = avOut[R]; //ReadRPM
  torque_out_left = 3 * ( avOut[TL] - torque_tare_left) /100;  // gr*m
  torque_out_left = constrain( torque_out_left, 0, torque_out_left);
  
  torque_out_right = 3 * ( avOut[TR] - torque_tare_right) /100; // gr*m
  torque_out_right = constrain( torque_out_right, 0, torque_out_right);
  
  thrust_out = avOut[TH] - thrust_tare;
  thrust_out = constrain( thrust_out, 0, thrust_out);
  
  ComCnt = avOut[C];
  
//   if ( (counter >20) && (rpm_out < 800)) {
//     motor_running = false;
//     }
  
  
  //LogFreq = 1000 / constrain( (millis() - timer_1),1, 1000);
  
//  if (rpm_out > MaxRpmLogged){
//    MaxRpmLogged = rpm_out;
//    }
  
  
  
  if (lstat == 1) {
    
  //Serial.println("Micros; U(V); I(mAh); Commutations; RpmMeasured; ToL(gr*m); ToR(gr*m); Thrust (gr);" );
  //DoLogDirect(1,sTime,0,0,0,0,0);
 
    
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, voltage_out, 2);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, current_out, 2);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, ComCnt, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,rpm_out, 0); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
  }
  
  if (lstat == 101) {
    
  //Serial.println("Micros; U(V); I(mAh); Commutations; RpmMeasured; ToL(gr*m); ToR(gr*m); Thrust (gr);" );
  //DoLogDirect(1,sTime,0,0,0,0,0);
 
  cAry = floatToString(aAry, f1, 0);
  Serial.print(cAry);
  //SDPrint(cAry);
  
  cAry = floatToString(aAry, voltage_out, 2);
  Serial.print(cAry);
  //SDPrint(cAry);
  
  cAry = floatToString(aAry, current_out, 2);
  Serial.print(cAry);
  //SDPrint(cAry);
  
  cAry = floatToString(aAry, ComCnt, 0);
  Serial.print(cAry);
  //SDPrint(cAry);
  
  cAry = floatToString(aAry,rpm_out, 0); 
  Serial.print(cAry);
  //SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_left, 4); 
  Serial.print(cAry);
  //SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  Serial.print(cAry);
  //SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  Serial.println(cAry);
  //SDPrintln(cAry);
  }
  
  
  
  
  if (lstat == 2) {
    //
    
    
    
    }
    
    
  if (lstat == 3) {
  //DoLogDirect(3, sTime, RPMAMP, pwOut, r, i/5, 0, 0);
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, f2, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, f3, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, f4, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, f5, 2);
  //Serial.print(cAry);
  SDPrint(cAry);
  
//  cAry = floatToString(aAry,rpm_out, 0); 
  //Serial.print(cAry);
//  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
    
    
    
    }
    
 if (lstat == 4) {
    //"Micros; PWM; RPM; ToL(gr*m); ToR(gr*m); Thrust (gr);"
    
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, f2, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry, f3, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
  
  
  //cAry = floatToString(aAry, f4, 0);
  //Serial.print(cAry);
  //SDPrint(cAry);
  
  //cAry = floatToString(aAry,rpm_out, 0); 
  //Serial.print(cAry);
  //SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
  }
  
  
  if (lstat == 5) {
    //Micros; Rpm; ToL(gr*m); ToR(gr*m); Thrust (gr);
    
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
   
   cAry = floatToString(aAry, f2, 0);
  //Serial.print(cAry);
  SDPrint(cAry); 
   
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
  }
  
   if (lstat == 7) {
    //Micros; Thrust; ToL(gr*m); ToR(gr*m); Thrust (gr);
    
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
   
   cAry = floatToString(aAry, f2, 0);
  //Serial.print(cAry);
  SDPrint(cAry); 
   
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
  }
    
}
  
  
  

  void DoLogDirectFast(int lstat, float f1, float f2, float f3,float f4, float f5, float f6){
  char  aAry[128];
  char *cAry;
  
    //Micros; Rpm; ToL(gr*m); ToR(gr*m); Thrust (gr);
  if (lstat == 1){   
    
  counter++ ;
  
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
   
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
  return;
}
    
  if (lstat == 2){   
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
   
   cAry = floatToString(aAry, f2, 0);
  //Serial.print(cAry);
  SDPrintln(cAry); 
  return;
}    

  if (lstat == 5) {
    //Micros; Rpm; ToL(gr*m); ToR(gr*m); Thrust (gr);
    
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
   
   cAry = floatToString(aAry, f2, 0);
  //Serial.print(cAry);
  SDPrint(cAry); 
   
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
  }
    
    
  if (lstat == 7) {
    //Micros; Thrust; ToL(gr*m); ToR(gr*m); Thrust (gr);
    
  cAry = floatToString(aAry, f1, 0);
  //Serial.print(cAry);
  SDPrint(cAry);
   
   cAry = floatToString(aAry, f2, 0);
  //Serial.print(cAry);
  SDPrint(cAry); 
   
  cAry = floatToString(aAry,torque_out_left, 4); 
  //Serial.print(cAry);
  SDPrint(cAry);
  
  cAry = floatToString(aAry,torque_out_right, 4);
  //Serial.print(cAry);
  SDPrint(cAry);
 
  cAry = floatToString(aAry,thrust_out, 0);
  //Serial.println(cAry);
  SDPrintln(cAry);
  }    
    
    
}
  
  
  
  
 void StartReadRpm(){
   pinMode(RPM_PIN, INPUT); 
   //digitalWrite(RPM_PIN, HIGH);       // turn on pullup resistor
   //rpm_out = 1000;
   if (ENCODER_MODE == 0){
     attachInterrupt(RPM_PIN, ReadCommutations, FALLING ) ;
     }
     
   if (ENCODER_MODE == 1){
     attachInterrupt(RPM_PIN, ReadCommutations, CHANGE ) ;
     }
   
   comCount =0;
   start_log_rpm = micros();
    
    }
  
  float ReadRpm(){
    float nRPM;
    unsigned long TimeStep;
    unsigned long nTimeRead; 
    nTimeRead =  micros() - start_log_rpm;
    detachInterrupt(RPM_PIN);
    TimeStep = 1000000 / nTimeRead;
    nRPM = float( comCount * 60 * TimeStep / Rpm_Scaler);
    StartReadRpm();
    //comCount =0;
    //start_log_rpm = micros();
    return nRPM;
    }
  
  
  void ReadCommutations(){
    comCount++;
    }


void ListenESC32(){
  char cbuffer[256];
  int i = 0;
  // stop listening for now
 return;
  // listen for serial Messages frm ESC32
SendToSerialStr("status");

  while(Serial1.available() > 0) {
    cbuffer[i++]=Serial1.read();
  }
    
    if (i>0) {
      cbuffer[i]=0;
      //Serial.print("Msg from ESC32: ");
      Serial.println(cbuffer);
      //Serial.println("End ESC32 Msg");

      //SDPrintln("Msg from ESC32: " ); 
      //SDPrintln(cbuffer ); 
      //SDPrintln("End ESC32 Msg" ); 

      }
 
  }
  
  

boolean SystemHealth(){
  boolean rval = true;
  LastStartLog = 0;
  if (load_cell_status == false) {
    StartLog =0;
    StopESC32();
    rval = false;
    Serial.println("");
    Serial.println("Load cell overload, abort to protect the sensors." );  
    SDPrintln("");
    SDPrintln("Load cell overload, abort to protect the sensors." ); 
    }
  if (motor_running == false) {
    Serial.println("");
    Serial.println("Motor not running, try to restart sequence." );  
    Serial.print("Restart sequence :" );  
    Serial.println(LogRetries + 1 );  
    Serial.println("" );  
    
    SDPrintln("");
    SDPrintln("Motor not running, try to restart sequence." );     
    SDPrintln("");
    LastStartLog = StartLog;
    StartLog =0;
    Mot_Scal=0;
    rval = false;
    }
return rval;
}


int TareEncoder(){
  int OldComCnt = 0;
  SetTimers();
  StartReadRpm();
  
  while  (StartLog == 10)  {
    ListenSerial();
    
    if (Timer10Hz() ){
    
    if ( comCount > OldComCnt ){
      Serial.print( "ComCounts: ");
      Serial.println(comCount); 
      
      }
      
      OldComCnt = comCount;    
    }
 
 }
    detachInterrupt(RPM_PIN);
  
  }
  
void  UpdateInterface(){
  char cAry[256];
  float nU = 0;
  float nI = 0;
  float nTL = 0;
  float nTR = 0;
  float nTH = 0;
  float nRpm = 0;
  
  
  if ( SysTimer5Hz() ) {
    // SendToSerialStr("binary");
    // delay(10);
    //Test1();
    //ListenESC32(); //Check ESC32 for Messages
    
    rpm_out = ReadRpm();
    nU  = float( analogRead(VOLTAGE_PIN) * 1000 / VOLTAGE_SCALER / ANALOG_RESOLUTION );
    nI  = float( analogRead(CURRENT_PIN) * 1000 / CURRENT_SCALER / ANALOG_RESOLUTION * 1000 );
    nTL = float( analogRead(TORQUE_PIN_LEFT) * TORQUE_SCALER_LEFT / ANALOG_RESOLUTION );
    nTL -= torque_tare_left;
    nTR = float( analogRead(TORQUE_PIN_RIGHT) * TORQUE_SCALER_RIGHT / ANALOG_RESOLUTION );
    nTR -= torque_tare_right ;
    nTH = float( analogRead(THRUST_PIN) * THRUST_SCALER / ANALOG_RESOLUTION ) ;
    nTH -= thrust_tare;
    sprintf(cAry, "SYS; %10.2f; %10.2f; %10.2f; %10.2f; %10.2f; %10.4E; %10.4E; %5.2f; %5.0f ; %2.0f; %5.0i; %5.0i; %10.2f "  , nU, nI, nTL, nTR, nTH, Mot_A1, Mot_A2, Prop_K1, Mot_Scal, Motor_Poles, PwmOffset, PwmMax, rpm_out );  
    Serial.println(cAry);
    
   // sprintf(cAry, "SYS; %12.4f; %12.4f; %12.4f; %12.4f; %12.4f; %12.4f; %12.4f; %12.4f; %12.4f; %12.4f; %12.8f; "  , voltage_out, current_out, comCount, rpm_out , torque_out_left, torque_out_right, thrust_out, Mot_Scal, Mot_A1, Mot_A2, Rpm_Scaler );  
   // Serial.println(cAry);
   //HartBeat();
  }   
  }
  
  
void sort(float a[], int size) {
  // Usage
  // int sortValues[13] = { 2, 7, 4, 6, 5, 3, 8, 10, 9, 11, 14, 12, 13 }; 
  // sort(sortValues,13);

    for(int i=0; i<(size-1); i++) {
        for(int o=0; o<(size-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    float t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}


void quickSort(int arr[], int left, int right) {
      int i = left, j = right;
      int tmp;
      int pivot = arr[(left + right) / 2];
 
      /* partition */
      while (i <= j) {
            while (arr[i] < pivot)
                  i++;
            while (arr[j] > pivot)
                  j--;
            if (i <= j) {
                  tmp = arr[i];
                  arr[i] = arr[j];
                  arr[j] = tmp;
                  i++;
                  j--;
            }
      };
 
      /* recursion */
      if (left < j)
            quickSort(arr, left, j);
      if (i < right)
            quickSort(arr, i, right);
}


void ScrPrintHead(int StartRpm, int StepRpm){
  Serial.print("Tested Unit: ");
  Serial.println(TestedUnit);
  Serial.print("StartRPM: ");
  Serial.println(StartRpm);
  Serial.print("RPM Steps: ");
  Serial.println(StepRpm);
  Serial.print("Max RPM: ");
  Serial.println(Mot_Scal);
  Serial.print("Prop Blades: ");
  Serial.println(Motor_Poles);
  Serial.print("PWM Offset: ");
  Serial.println(PwmOffset);
  Serial.print("Max PWM: ");
  Serial.println(PwmMax);
  Serial.println(""); 
  return;
  }


void SDPrintHead(int StartRpm, int StepRpm){
  char cAry[256]; 
   //Print Header
  SDPrint("Tested Unit: "); 
  SDPrintln(TestedUnit);
  SDPrintln("");
  SDPrint("StartRPM:    ");
  sprintf(cAry, "%5.0i" , StartRpm);
  SDPrintln( cAry );
  SDPrint("RPM Steps:   ");
  sprintf(cAry, "%5.0i" , StepRpm);
  SDPrintln(cAry);
  SDPrint("Max RPM:     ");
  sprintf(cAry, "%5.0f" , Mot_Scal);
  SDPrintln(cAry);
  SDPrint("Prop Blades: ");
  sprintf(cAry, "%5.0f" , Motor_Poles);
  SDPrintln(cAry);
  SDPrint("PWM Offset:  ");
  sprintf(cAry, "%5.0i" , PwmOffset);
  SDPrintln(cAry);
  SDPrint("Max PWM:     ");
  sprintf(cAry, "%5.0i" , PwmMax);
  SDPrintln(cAry);
  SDPrintln("");
  return;
  }



