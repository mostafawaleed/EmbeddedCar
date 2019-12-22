#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <semphr.h> // add to be able to use semaphores
#include <LiquidCrystal.h>
#include <rdm630.h>
#include <SoftwareSerial.h>

LiquidCrystal lcd(30,32,40,38,36,34);
rdm630 rfid(12, 33);

//Define Tasks
void startEngine( void *pvParameters); 
void frontMotors( void *pvParameters); 
void Mirrors( void *pvParameters); 
void seatBelt( void *pvParameters);
void rainDetection( void *pvParameters);
void checkFuelLevel( void *pvParameters);
void keyLessEntry( void *pvParameters);
void laneDeparture( void *pvParameters);

boolean locked = true;
int counter =0;
const int startStopButton = 2;
const int onLed =46;
const int startMoving =48;
const int enC = 3;
const int enD = 4;
const int in5 = 47;
const int in6 = 49;
const int in7 = 51;
const int in8 = 53;
const int enA =5;
const int enB =6;
const int in1=7;
const int in2=8;
const int in3=9;
const int in4=10;
const int vrx =A0;
const int vry =A1;
const int seatbelt=26;
const int beltbuzzer =24;
const int digitalRD= 28;
const int analogRD = A2;
const int fuelLevel = A3;
const int linedetector = 42;
const int lineDetector2 = 31;
const int laneDepartureFlag = 44;

String tag="";
String intag="24870484849666550666948563";
int i=0;
int lastbeltbuttonstate=0;
int beltbuttonstate=0;

Servo myservo;

int pos=0;
boolean onFlag=false;
boolean moving =false;
boolean beltOn =false;
SemaphoreHandle_t LCDSem;
String tagstring ="";
int t = 0;

unsigned long realKeyNumber= 1833625;



void setup() {
lcd.begin(16,2);

pinMode(startStopButton,INPUT);
pinMode(seatbelt,INPUT);
pinMode(beltbuzzer,OUTPUT);
pinMode(onLed,OUTPUT);
pinMode(startMoving,INPUT);
pinMode(enA,OUTPUT);
pinMode(enB,OUTPUT);
pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
pinMode(enC,OUTPUT);
pinMode(enD,OUTPUT);
pinMode(in5,OUTPUT);
pinMode(in6,OUTPUT);
pinMode(in7,OUTPUT);
pinMode(in8,OUTPUT);
myservo.attach(11);
pinMode (vrx,INPUT);
pinMode (vry,INPUT);
pinMode (digitalRD,INPUT);
pinMode (analogRD,INPUT);
pinMode (fuelLevel,INPUT);
pinMode (linedetector,INPUT);
pinMode (lineDetector2, INPUT);
pinMode(laneDepartureFlag,OUTPUT);
LCDSem = xSemaphoreCreateMutex();
Serial.begin(9600);
rfid.begin();   
attachInterrupt(digitalPinToInterrupt(startStopButton), isr , FALLING);
//xTaskCreate (startEngine, "startEngine", 128, NULL, 6, NULL);
//xTaskCreate (frontMotors, "frontMotors", 128, NULL, 5, NULL);
//xTaskCreate (Mirrors, "Mirrors", 128, NULL, 5, NULL);
//xTaskCreate (seatBelt, "seatBelt", 128, NULL, 5, NULL);
//xTaskCreate (rainDetection, "rainDetection", 128, NULL, 5, NULL);
//xTaskCreate (checkFuelLevel, "checkFuelLevel", 128, NULL, 5, NULL);
//xTaskCreate (keyLessEntry, "keyLessEntry", 128, NULL, 7, NULL);
//xTaskCreate (laneDeparture, "laneDeparture", 128, NULL, 5, NULL);
xTaskCreate (startEngine, "startEngine", 128, NULL, 5, NULL);
xTaskCreate (frontMotors, "frontMotors", 128, NULL, 5, NULL);
xTaskCreate (Mirrors, "Mirrors", 128, NULL, 5, NULL);
xTaskCreate (seatBelt, "seatBelt", 128, NULL, 5, NULL);
xTaskCreate (rainDetection, "rainDetection", 128, NULL, 5, NULL);
xTaskCreate (checkFuelLevel, "checkFuelLevel", 128, NULL, 5, NULL);
xTaskCreate (keyLessEntry, "keyLessEntry", 128, NULL, 5, NULL);
xTaskCreate (laneDeparture, "laneDeparture", 128, NULL, 5, NULL);

}


void loop() {
// Empty. Things are done in Tasks.

}
void isr(){
  if(!locked)
   onFlag=!onFlag;
  if(!onFlag){
     locked = true;
  }

}


void startEngine (void *pvParameters) // This is a Task. 
{
TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();
  while (1) {
     
         if(onFlag==true){
          digitalWrite(onLed,HIGH);
          
         }
          else{
           digitalWrite(onLed,LOW);
           beltOn=false;
             
          }
       
       
    vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
    
   
  }
}
void frontMotors (void *pvParameters){
  TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();
  while(1){
    if(onFlag == true){
      if(digitalRead(startMoving)==HIGH){
        moving=true;
        for(int i=0;i<255;i+=15){
          digitalWrite(in1,HIGH);//front =right;
          digitalWrite(in2,LOW);
          digitalWrite(in3,HIGH);
          digitalWrite(in4,LOW);
          analogWrite(enA,i);
          analogWrite(enB,i);//rear
          digitalWrite(in5,LOW);
          digitalWrite(in6,HIGH);
          digitalWrite(in7,LOW);
          digitalWrite(in8,HIGH);
          analogWrite(enC,i);
          analogWrite(enD,i);
        }
      }
    }else{
      digitalWrite(laneDepartureFlag,LOW);
      digitalWrite(in1,LOW);
          digitalWrite(in2,LOW);
          digitalWrite(in3,LOW);
          digitalWrite(in4,LOW);
      digitalWrite(enA,LOW);
      digitalWrite(enB,LOW);
      digitalWrite(in5,LOW);
          digitalWrite(in6,LOW);
          digitalWrite(in7,LOW);
          digitalWrite(in8,LOW);
      digitalWrite(enC,LOW);
      digitalWrite(enD,LOW);
       moving=false;
    }
    vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(50));
  }
}




void Mirrors(void *pvParameters){
    TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();
  while(1){
    if(onFlag){
        xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(0,0);
           lcd.print("M:");
           if(myservo.read()<10){
            lcd.setCursor(3,0);
            lcd.print(" ");
               lcd.setCursor(2,0);
           lcd.print(myservo.read());
           }else{
            lcd.setCursor(2,0);
             lcd.print(myservo.read());
           }
           xSemaphoreGive(LCDSem);
    if(analogRead(vrx)>515 && analogRead(vrx)<=1023&&pos<=90 ){
    
       myservo.write(pos+=3);
        
      
    
    }else if(analogRead(vrx)<500 && analogRead(vrx) >=0 && pos>0){
         
          myservo.write(pos-=3);
      
       
      
    }
    }
 
    vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(250));
  }
}
void seatBelt(void *pvParameters){
   TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();

while(1){
   beltbuttonstate=digitalRead(seatbelt);
if(beltbuttonstate != lastbeltbuttonstate){
     if(beltbuttonstate == HIGH){
      beltOn = !beltOn;
     }
}
lastbeltbuttonstate= beltbuttonstate;
  if(moving){
    if(beltOn == false){
      digitalWrite(beltbuzzer,HIGH);
       xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(5,0);
         
           lcd.print("   belt:OFF ");
            vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
           xSemaphoreGive(LCDSem);
    }else{
      digitalWrite(beltbuzzer,LOW);
          xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(5,0);
          lcd.print("   belt:ON  ");
            vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
           xSemaphoreGive(LCDSem);
    }
    
  }else{
    digitalWrite(beltbuzzer,LOW);
  }
  vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(200));
}
}
void rainDetection(void *pvParameters){
     TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();
while(1){
  if(onFlag){
  if(analogRead(A2)<300){
    xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(0,1);
           lcd.print("Rain:H ");
            vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
           xSemaphoreGive(LCDSem);
  }
 else if(analogRead(analogRD)<500){
            xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(0,1);
           lcd.print("Rain:M ");
            vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
           xSemaphoreGive(LCDSem);
 }
 else{
 xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(0,1);
           lcd.print("NO RAIN");
            vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
           xSemaphoreGive(LCDSem);
 }
  }




  vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(250));
}
}

void checkFuelLevel(void *pvParameters){
   TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();
while(1){
  if(analogRead(fuelLevel)<500){
    xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(8,1);
           lcd.print("Fuel:L");
            vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
           xSemaphoreGive(LCDSem);
  }
 
  else{
        xSemaphoreTake (LCDSem,portMAX_DELAY);
          lcd.setCursor(8,1);
           lcd.print("Fuel:H");
            vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
           xSemaphoreGive(LCDSem);
  }


  
  
  vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(250));
  }
}
void keyLessEntry(void *pvParameters){

TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();
  while(1){
  byte data[6];
    byte length;
   

    
    if(rfid.available()){
        rfid.getData(data,length);
        unsigned long result = 
          ((unsigned long int)data[1]<<24) + 
          ((unsigned long int)data[2]<<16) + 
          ((unsigned long int)data[3]<<8) + 
          data[4];              
        Serial.print("decimal CardID: ");
        Serial.println(result);
        if(result == realKeyNumber && locked == true){
          Serial.println( "Car is unlocked, Welcome my lovely owner" );
          locked = false;
        }
         else if(locked == true){
          Serial.println( "yo Thief, What are you doin!, This car has maximum security");
        }
    }
  
   vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(50));
     
    
    
  }
}void laneDeparture(void *pvParameters){
   TickType_t xLastWakeTime;

xLastWakeTime = xTaskGetTickCount ();
int lastvalue=digitalRead(linedetector);
int lastvalue2=digitalRead(lineDetector2);
while(1){
  if(moving){
  int value =digitalRead(linedetector);
  int value2 =digitalRead(lineDetector2);
  if(value==0 and lastvalue==1 ){
    digitalWrite(laneDepartureFlag,HIGH);
    
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    
     
    xSemaphoreTake (LCDSem,portMAX_DELAY);
    lcd.setCursor(0,0);
    lcd.print("lane departure!");
  }
//--------------------------------------------------------------------
  
  else if(value2==0 and lastvalue2==1 ){
    digitalWrite(laneDepartureFlag,HIGH);
    
     digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    
    //vTaskDelay(pdMS_TO_TICKS(50));
     
    xSemaphoreTake (LCDSem,portMAX_DELAY);
    lcd.setCursor(0,0);
    lcd.print("lane departure!");
      
   
  }else if((value2==1 and lastvalue2==0) or (value==1 and lastvalue==0)){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
     digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    xSemaphoreGive(LCDSem);
     digitalWrite(laneDepartureFlag,LOW);
  }
  lastvalue =value;
  lastvalue2 =value2;
  }else{
    digitalWrite(laneDepartureFlag,LOW);
  }
  vTaskDelayUntil(&xLastWakeTime ,pdMS_TO_TICKS(100));
}

}
