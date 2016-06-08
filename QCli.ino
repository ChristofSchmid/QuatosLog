//------------------------------------------------------------------------------
// Serial Monitor control example
// dan@marginallyclever.com 2011-10-17
// Enhanced by CS 15.3.2014
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// methods
//------------------------------------------------------------------------------

#define MOT_A1 -.0004//use the right value
#define MOT_A2 5e-5//use the right value

//------------------------------------------------------------------------------
void jog( float x, float y, float z) {
  // do some tests here to validate x, y, and z.
  Mot_A1 = x;
  Mot_A2 = y;
  Mot_Scal=z;
}

void setMotorPoles( float x) {
  float Old_Scaler = Rpm_Scaler / Motor_Poles; 
  Rpm_Scaler = Rpm_Scaler * x / Motor_Poles;
  Motor_Poles = x;
}

void setQuatos(float x, float y){
 
 } 

void setMaxRpm( float x){
  Mot_Scal=x;
 }

void setTestedUnit(String x){
     char cAry[MAX_BUF];
     x = x.substring(0, x.length() -1 );
     x.toCharArray(cAry, MAX_BUF );
     memmove(TestedUnit, cAry, 50 );    
     Serial.print("Tested Unit set to :   ");
     Serial.println(TestedUnit);
  }

void RunPwmMax( float x ){
  // do some tests here to validate x, y, and z.
  ESC32.writeMicroseconds(PwmMax);  
 }

void RunPwmMin( float x ){
  // do some tests here to validate x, y, and z.
  ESC32.writeMicroseconds(PwmOffset);  
 }

void SetLogStep(float x){
  LogStep = int(x);
  }

void setPwmOffset(float x){
  PwmOffset = int(x);
 }
 
 void setPwmMax(float x){
  PwmMax = int(x); 
 }
 

void ParseSerSerBuffer( String c){
  String p = c.substring(4);
  int x;
  
    x = p.toInt();
Serial.print("this is my int ");
Serial.println( x );

//string.indexOf(val, from) 

 }


//------------------------------------------------------------------------------
void processCommand() {
  char buff[64] ;
  
  if(!strncmp(SerBuffer,"start log",9)) {
    Serial.println("Starting LOG");
    counter = 0;
    StartLog = 1;
    LastStartLog = 0;
    return;
    
 
/*   } else if(!strncmp(SerBuffer,"reset",5)) {
    Serial.println("Reseting Board.....");  
    software_Reset();
    return;    
*/   
    
  } else if(!strncmp(SerBuffer,"stop log",8)) {
    StartLog = 0;
    LastStartLog = 0;
    Serial.println("Stopping LOG");
    return;    
    
    
  } else if(!strncmp(SerBuffer,"log rpm",7)) {
    Serial.println("Starting RPM 2 Thrust Log");
    counter = 0;
    StartLog = 2 ;
    return;

  } else if(!strncmp(SerBuffer,"log sinrpm",10)) {
    Serial.println("Starting sinusoidal RPM Log");
    counter = 0;
    StartLog = 3 ;
    return;

  } else if(!strncmp(SerBuffer,"olsteprpm",9)) {
    Serial.println("Starting step RPM Log");
    counter = 0;
    StartLog = 4 ;
    return;
  
 } else if(!strncmp(SerBuffer,"clsteprpm100Hz",14)) {
    Serial.println("Starting Closed Loop step RPM Log 100Hz");
    counter = 0;
    StartLog = 5 ;
    return;     
    
 } else if(!strncmp(SerBuffer,"clsteprpm1000Hz",15)) {
    Serial.println("Starting Closed Loop step RPM Log 1000Hz");
    counter = 0;
    StartLog = 6 ;
    return;   

 } else if(!strncmp(SerBuffer,"clstepth100Hz",13)) {
    Serial.println("Starting Closed Loop step RPM Log 1000Hz");
    counter = 0;
    StartLog = 7 ;
    return;       

  } else if(!strncmp(SerBuffer,"startesc32ol",12)) {
    Serial.println("Starting ESC32 PWM 1150 Open Loop");
    StartESC32( 1, 1150 );
    return;

 } else if(!strncmp(SerBuffer,"startesc32cl",12)) {
    Serial.println("Starting ESC32 RPM 2000 Closed Loop");
    StartESC32( 2, 2000 );
    return;
    
 } else if(!strncmp(SerBuffer,"setesc32ol",10)) {
    Serial.println("Setting ESC32 Open Loop");
    SendToSerialStr("set STARTUP_MODE 0");
    SendToSerialStr("config WRITE");
    return;
    
 } else if(!strncmp(SerBuffer,"setesc32cl",10)) {
    Serial.println("Setting ESC32 closed Loop");
    SendToSerialStr("set STARTUP_MODE 1");
    SendToSerialStr("config WRITE");
    return;    

 } else if(!strncmp(SerBuffer,"tareencoder",11)) {
    Serial.println("Starting Tare Encoder");
    StartLog = 10 ;
    return;
    
 } else if(!strncmp(SerBuffer,"runmaxrpmtest",13)) {
   Serial.println("Starting Max RPM Test...");
    counter = 0;
    StartLog = 20 ;
    return;


// ESC32



 } else if(!strncmp(SerBuffer,"arm",3)) {
    Serial.println("Arm ESC32");
    ESC32Arm();
    //SendToSerialStr("arm");
    //ESC32.writeMicroseconds(PWM_ARM);
    return;
    
     } else if(!strncmp(SerBuffer,"disarm",6)) {
    Serial.println("Disarm ESC32");
    ESC32Disarm();
    //SendToSerialStr("disarm");
    //ESC32.writeMicroseconds(PWM_DISARM);
    return;
    
     } else if(!strncmp(SerBuffer,"rpm 2000",8)) {
    Serial.println("Start Motor 2000 RPM");
    //SendToSerialStr("rpm 2000");
    SetMotorRpm(2000);
    
    return;
    
    } else if(!strncmp(SerBuffer,"stop",4)) {
    StopESC32();
    return;
    
    } else if(!strncmp(SerBuffer,"start",5)) {
    Serial.println("Start Motor");
    //SendToSerialStr("start");
    SetMotorRpm(1000);
    return;
    
//End ESC32
    
      
  } else if(!strncmp(SerBuffer,"help",4)) {
    
    Serial.println("Help");

    Serial.println("General Log for PWM use ");
    Serial.println(" ");
    Serial.println("start log; -> send");
    Serial.println("stop log; -> send");

    Serial.println(" ");
    Serial.println("Quatos functions. If you know Max RPM, set it with the following function.");
    Serial.println("Otherwise a max RPM test is performed first. The Motor will go to full RPM. ");
    Serial.println("If you have a SD reader with card inserted attached to SPI, the Logs are writen to SD.");


    Serial.println(" ");

    Serial.println("quatos [x(2T_A1)] [y(2T_A2)] [z(MaxRPM)] -> send");

    Serial.println(" ");

    Serial.println("log rpm; -> send (RPM 2 Thrust LOG, use to determine A1 and A2) ");
    Serial.println("log sinrpm; -> send");
    Serial.println("log steprpm; -> send");

    Serial.println(" ");
    Serial.println("Some basic Testfunctions for com with ESC32");
    Serial.println(" ");

    Serial.println("ESC32: stop; -> send");
    Serial.println("ESC32: start; -> send");
    Serial.println("ESC32: arm; -> send");
    Serial.println("ESC32: disarm; -> send");

    return;

        
  
  } else if(!strncmp(SerBuffer,"quatos",6)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx=Mot_A1;
    float yy=Mot_A2;
    float zz=Mot_Scal;


    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'x': case 'X': xx=atof(ptr+1);  Serial.print("A1:   "); Serial.println(xx); break;
      case 'y': case 'Y': yy=atof(ptr+1);  Serial.print("A2:   "); Serial.println(yy); break;
      case 'z': case 'Z': zz=atof(ptr+1);  Serial.print("SCAL: "); Serial.println(zz); break;
      default: ptr=0; break;
      }
    }

    jog(xx,yy,zz);
  
  
    } else if(!strncmp(SerBuffer,"setmotorpoles",13)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("Prop blades set to:   ");
      Serial.println(xx);
      break;
      }
      setMotorPoles(xx);
      
    } else if(!strncmp(SerBuffer,"setmaxrpm",9)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("Max RPM set to:   ");
      Serial.println(xx);
      break;
      }
      setMaxRpm(xx);
  
  
      } else if(!strncmp(SerBuffer,"setpwmoffset",12)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("Offset PWM set to:   ");
      Serial.println(xx);
      break;
      }
      setPwmOffset(xx);
  
  
      } else if(!strncmp(SerBuffer,"setpwmmax",9)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("Max PWM set to:   ");
      Serial.println(PwmMax);
      break;
      }
      setPwmMax(xx);
  
    } else if(!strncmp(SerBuffer,"runpwmmax",9)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("Max PWM output set in us:   ");
      Serial.println(PwmMax);
      break;
      }
      RunPwmMax(PwmMax);
      
            } else if(!strncmp(SerBuffer,"runpwmmin",9)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("Max PWM output set in us:   ");
      Serial.println(PwmOffset);
      break;
      }
      RunPwmMin(PwmOffset);

            } else if(!strncmp(SerBuffer,"setmotorpwm",11)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("PWM output set to :   ");
      Serial.println(ptr);
      break;
      }
      SetMotorPwm( int(xx)) ;
  
            } else if(!strncmp(SerBuffer,"setmotorrpm",11)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("RPM set to :   ");
      Serial.println(ptr);
      break;
      }
      SetMotorRpm( int(xx)) ;
  
  
              } else if(!strncmp(SerBuffer,"setmotorthrust",14)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("RPM set to :   ");
      Serial.println(ptr);
      break;
      }
      SetMotorThrust( int(xx)) ;
      
      
                    } else if(!strncmp(SerBuffer,"setrpmstep",10)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx = 0;
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      xx=atof(ptr);
      Serial.print("RPM step set to :   ");
      Serial.println(ptr);
      break;
      }
      SetLogStep( int(xx)) ;
      
      
    } else if(!strncmp(SerBuffer,"settestedunit",13)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    char *ptr=SerBuffer;
    while(ptr && ptr<SerBuffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      //Serial.print("Tested Unit set to :   ");
      //Serial.println(ptr);
      break;
      }
      setTestedUnit(ptr);
  }
  
}


//------------------------------------------------------------------------------
void CliStart() {
  Serial.println("Init...");
  Serial.println("Stretching...");

  sofar=0;
  Serial.println("** AWAKE **");
}


//------------------------------------------------------------------------------
void ListenSerial() {
  // listen for serial commands
  
  
  
  while(Serial.available() > 0) {
    SerBuffer[sofar++]=Serial.read();
    if(SerBuffer[sofar-1]==';') break;  // in case there are multiple instructions
  }

  // if we hit a semi-colon, assume end of instruction.
  if(sofar>0 && SerBuffer[sofar-1]==';') {
    // what if message fails/garbled?
    
    // echo confirmation
    SerBuffer[sofar]=0;
    //Serial.println(SerBuffer);

    // do something with the command
    processCommand();
    
    // reset the SerBuffer
    sofar=0;
    
    // echo completion
    //Serial.println("Done.");
  }
}





