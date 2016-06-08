
 
//File myFile;
 
void SDStart()
{
  
  if (USE_SD_READER == 0){
  return;
  }
  //Serial.begin(9600);
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
   pinMode(10, OUTPUT);
 
  if (!SD.begin(24)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
 
}
 

void SDOpenFile( char *FileName ){
  int fileCounter = 0;
  int DotPos = 0;
  int i=0;
  
  char *OrigFile = FileName;
  char a[13];
  char f[13];
  char cCnt[1];
  
  if (USE_SD_READER == 0){
  return;
  }
 
 
 while ( (SD.exists(FileName) && (fileCounter < 10) ) ){
    
//    while ( (fileCounter < 10)  ){
    

    for (i=0; i<13;i++){
      if (OrigFile[i] == '.') {
         DotPos = constrain(i, 1,7);
         }
      }
     for (i=DotPos; i < DotPos+4; i++) {
      a[i-DotPos] = OrigFile[i];
      //Serial.println(a[i - DotPos]);
      } 
      
     for (i= 0; i <= DotPos + 4 ; i++){
       //Serial.println(f);
       
       if (i < DotPos){         
         f[i] = OrigFile[i];
         
         }
       else if (i==DotPos){  
        
         sprintf( cCnt, "%1u" ,fileCounter); 
         f[i] = cCnt[0];
         
         }
         
       else if (i > DotPos){   
         f[i] = a[i-DotPos-1];
         
         
         }  
     }
       
    fileCounter ++;
//    Serial.println(f);
    FileName = f;
    }

  LogFile = SD.open( FileName, FILE_WRITE);
 
  // if the file opened okay, write to it:
  if (!LogFile) {
   
    Serial.print("error opening file ");
    Serial.println(FileName);
  }
 } 
  
  
void SDPrint( char* cText ){
    
    if (USE_SD_READER == 1){
      //Serial.print("SD ");
      //Serial.println(cText);
      LogFile.print(cText); 
      }
  }
  
  void SDPrintln( char* cText ){
    
    if (USE_SD_READER == 1){
      //Serial.print("SD ");
      //Serial.println(cText);
      LogFile.println(cText); 
      }
  }
