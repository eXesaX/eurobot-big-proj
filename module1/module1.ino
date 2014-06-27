#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield motors;

//пины для чтения энкодеров
#define encoder0Pin  20
#define encoder1Pin  21

#define WAITING 10
#define MOVING 1
#define TURNING 2
#define STOP 3
#define FAST_MOVING 4

byte prevMode;
byte mode = WAITING;
int parameter = 0;

long sum0 = 0;
long diff0;
long prevDiff0 = 0;
long ddiff0;

long sum1 = 0;
long diff1;
long prevDiff1 = 0;
long ddiff1;

long encoder0Pos = 0; //счечик энкодера
long encoder1Pos = 0;

long ISPS0 = 1300;
long ISPS1 = 1291;
long ISPF0 = 400;
long ISPF1 = 397;
long ISP0 = 0;
long ISP1 = 0;

unsigned long prevTime0 = 0;
unsigned long prevTime1 = 0;
long interval0;
long interval1;
long interval00;
long interval11;

//устанавливаем нужную скорость
long velocitySetPoint0 = 0;
long velocitySetPoint1 = 0;

//переменные для подсчета скорости
long newposition;
long oldposition = 0;
//вычисление интервала срабатывания PID
unsigned long pidtime;
unsigned long oldpidtime = 0;  

unsigned long printtime;
unsigned long oldprinttime = 0;  


//счетчик
byte count0 = 0;
byte count1 = 0;

//сумма импульсов
long Qinterval0 = 2500;
long Qinterval1 = 2500;

int intervalSetPoint0 = 0;
int intervalSetPoint1 = 0;

bool resetVariables = true;

//PID0
float kp0 = 0.2;
float ki0 = 0.01;
float kd0 = 2;
//PID1
float kp1 = 0.2;
float ki1 = 0.01;
float kd1 = 2;

long enc0 = 0;
long enc1 = 0;

const float TICKS_PER_CM = 40; //74,27230678
const float TICKS_PER_DEG_R = 6.8; //12,96296296
const float TICKS_PER_DEG_L = 6.8; //12,96296296

//mouse

void setup() { 
  motors.init();

  pinMode(encoder0Pin, INPUT); 
  digitalWrite(encoder0Pin, HIGH);       // turn on pullup resistor
  pinMode(encoder1Pin, INPUT); 
  digitalWrite(encoder1Pin, HIGH);       // turn on pullup resistor

  attachInterrupt(3, doEncoder, RISING);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(2, doEncoder_, RISING);  // encoder pin on interrupt 0 - pin 2
  
  Serial.begin (115200);
  Serial2.begin (115200);
  Serial.setTimeout(5);
  Serial2.setTimeout(5);
  delay(50);
  Serial.println("start");
  delay(50);
  
  pinMode(13, OUTPUT);
  
  //mouse init
  
} 

void loop(){
 if (Serial2.available() > 0) {          
  byte tempMode = Serial2.parseInt();    
  if (tempMode != 0) {
	if (tempMode == 3) {
		prevMode = mode;
	}
	mode = tempMode;                     
  }                                     
  int tempParameter = Serial2.parseInt();
  if (tempParameter != 0) {             
   parameter = tempParameter;           
  }                                     
  resetVariables = true;
  
  digitalWrite(13, HIGH);
  
  Serial.print("received: ");
  Serial.print(tempMode);
  Serial.print(" ");
  Serial.println(tempParameter);
 } 
  
  
 setMode(parameter);
 
 
 
  
 //doOutput();
}// END OF MAIN LOOP
 
void setMode(int param) {
  
  switch(mode) { //global
    case WAITING: {
      //do nothing
      break;
    }
    case FAST_MOVING: {
      ISP0 = ISPF0;
      ISP1 = ISPF1;
      MoveOn(param);
    }
    case MOVING: {
      ISP0 = ISPS0;
      ISP1 = ISPS1;
      MoveOn(param);
      break;
    }
    case TURNING: {
      TurnOn(param);
      break; 
    }
    case STOP: {
	  float divider = 1;
      motors.setBrakes(400, 400);
	  switch (prevMode) {
		case 1: {divider = TICKS_PER_CM; break;}
		case 2: {divider = TICKS_PER_DEG_L;break;}
	  }
	  String toSend = String(prevMode) + String(" ") + String(round((encoder0Pos - enc0) / divider)) + String(" ") + String(round((encoder1Pos - enc1) / divider));
      Serial2.println(toSend);
      
     /* Serial.print("Sent(stop): ");
      Serial.println(toSend);
      Serial.print("Debug: ");
      String debug = String(encoder0Pos)+" - "+String(enc0)+" / "+String(round(divider))+" "+String(encoder1Pos)+" - "+String(enc1)+" / "+String(round(divider));
      Serial.println(debug);*/
      
      mode = WAITING;  
      resetVariables = true;
      
      break; 
    }
  }
}


  
void MoveOn(int dist) {
  
  float distance = dist * TICKS_PER_CM;
  
  
  int const accBefore = 500;
  int const decAfter = 500;

  if (resetVariables) {
    enc0 = encoder0Pos;
    enc1 = encoder1Pos;
    
    sum0 = 0;
    diff0 = 0;
    
    
    prevDiff0 = 0;
    ddiff0 = 0;

    sum1 = 0;
    diff1 = 0;
    prevDiff1 = 0;
    ddiff1 = 0;
    
    resetVariables = false;
  }
 
  int dir = 1;
  if (dist < 0)  {dir = -1;}
  if (dist >= 0) {dir =  1;}

  pidtime = micros();
  printtime = micros();
    
  
  motors.setSpeeds(dir * velocitySetPoint0, dir * velocitySetPoint1);
   
  if (encoder0Pos - enc0 >= 0 && encoder0Pos - enc0 < accBefore || 
      encoder1Pos - enc1 >= 0 && encoder1Pos - enc1 < accBefore) {
    intervalSetPoint0 = 1300;//1300
    intervalSetPoint1 = 1291;//1291
  }
  
  
  if (encoder0Pos - enc0 > accBefore && encoder0Pos - enc0 < dir * distance - decAfter || 
      encoder1Pos - enc1 > accBefore && encoder1Pos - enc1 < dir * distance - decAfter) {
    intervalSetPoint0 = ISP0;//400;//900 //500
    intervalSetPoint1 = ISP1;//397;//894 //497
  }
  
  
  if (encoder0Pos - enc0 > dir * distance - decAfter && encoder0Pos - enc0 < dir * distance || 
      encoder1Pos - enc1 > dir * distance - decAfter && encoder1Pos - enc1 < dir * distance) {
    intervalSetPoint0 = 1300;
    intervalSetPoint1 = 1291;  
  }
  
  
  if (encoder0Pos - enc0 >= dir * distance || encoder1Pos - enc1 >= dir * distance) {
    motors.setBrakes(400, 400);
    mode = WAITING;
    resetVariables = true;
    String toSend = String(1) + String(" ") + String(round((encoder0Pos - enc0) / TICKS_PER_CM)) + String(" ") + String(round((encoder1Pos - enc1) / TICKS_PER_CM));
    Serial2.println(toSend);
    
    Serial.print("Sent(move): ");
    Serial.println(toSend);
    Serial.print("Debug: ");
    String debug = String(encoder0Pos)+" - "+String(enc0)+" / "+String(round(TICKS_PER_CM))+" "+String(encoder1Pos)+" - "+String(enc1)+" / "+String(round(TICKS_PER_CM));
    Serial.println(debug);
    digitalWrite(13, LOW);
  }    
     
  if (pidtime - oldpidtime > 10000){
  
    diff0 = Qinterval0 - intervalSetPoint0;
    ddiff0 = diff0 - prevDiff0;
    sum0 += diff0;
    velocitySetPoint0 = kp0 * diff0 + ki0 * sum0 + kd0 * ddiff0;  //P + I + D;
    if (velocitySetPoint0 > 400) {
       velocitySetPoint0 = 400;
    }
    if (velocitySetPoint0 < 0) {
       velocitySetPoint0 = 0;
    }
    
    prevDiff0 = diff0; 
    
    
    diff1 = Qinterval1 - intervalSetPoint1;
    ddiff1 = diff1 - prevDiff1;
    sum1 += diff1;
    velocitySetPoint1 = kp1 * diff1 + ki1 * sum1 + kd1 * ddiff1;  //P + I + D;
    if (velocitySetPoint1 > 400) {
       velocitySetPoint1 = 400;
    }
    if (velocitySetPoint1 < 0) {
       velocitySetPoint1 = 0;
    }
    
    
    prevDiff1 = diff1;
    oldpidtime = pidtime; 
  } // end of if(pidtime...)
}//end of MoveOn

void TurnOn(int ang) {
 
  float angle = 0;
  
  if (resetVariables) {
   enc0 = encoder0Pos;
   enc1 = encoder1Pos;
   
   sum0 = 0;
   diff0 = 0;
   prevDiff0 = 0;
   ddiff0 = 0;

   sum1 = 0;
   diff1 = 0;
   prevDiff1 = 0;
   ddiff1 = 0;
  
   resetVariables = false; 
  }
 
  int dir = 1;
  if (ang <  0) {
    dir = -1; 
    angle = TICKS_PER_DEG_R * ang;
  }
  if (ang >= 0) {
    dir =  1;
    angle = TICKS_PER_DEG_L * ang;
  }
  
  pidtime = micros();
  printtime = micros();
  
  motors.setSpeeds(dir * -velocitySetPoint0, dir * velocitySetPoint1);
  
  if (encoder0Pos - enc0 < dir * angle || encoder1Pos - enc1 < dir * angle) { 
   intervalSetPoint0 = 1300;//900;
   intervalSetPoint1 = 1291;//894;
  }
 
  if (encoder0Pos - enc0 >= dir * angle || encoder1Pos - enc1 >= dir * angle) {
   motors.setBrakes(400, 400);
   resetVariables = true;
   mode = WAITING;
   String toSend = String(2) + String(" ") + String(round((encoder0Pos - enc0) / TICKS_PER_DEG_L)) + String(" ")  + String(round((encoder1Pos - enc1) / TICKS_PER_DEG_L));
   Serial2.println(toSend);
   
   Serial.print("Sent:(turn) ");
   Serial.println(toSend);
   Serial.print("Debug: ");
   String debug = String(encoder0Pos)+" - "+String(enc0)+" / "+String(round(TICKS_PER_DEG_L))+" "+String(encoder1Pos)+" - "+String(enc1)+" / "+String(round(TICKS_PER_DEG_L));
   Serial.println(debug);
   digitalWrite(13, LOW);
  }
 
  if (pidtime - oldpidtime > 10000){
  
    diff0 = Qinterval0 - intervalSetPoint0;
    ddiff0 = diff0 - prevDiff0;
    sum0 += diff0;
    velocitySetPoint0 = kp0 * diff0 + ki0 * sum0 + kd0 * ddiff0;  //P + I + D;
    if (velocitySetPoint0 > 400) {
       velocitySetPoint0 = 400;
    }
    if (velocitySetPoint0 < 0) {
       velocitySetPoint0 = 0;
    }
    
    prevDiff0 = diff0; 
    
    
    diff1 = Qinterval1 - intervalSetPoint1;
    ddiff1 = diff1 - prevDiff1;
    sum1 += diff1;
    velocitySetPoint1 = kp1 * diff1 + ki1 * sum1 + kd1 * ddiff1;  //P + I + D;
    if (velocitySetPoint1 > 400) {
       velocitySetPoint1 = 400;
    }
    if (velocitySetPoint1 < 0) {
       velocitySetPoint1 = 0;
    }
    
    prevDiff1 = diff1;
    oldpidtime = pidtime;
  } // end of if(pidtime...)
}

void doOutput(){
   printtime = micros();
   if (printtime - oldprinttime > 10000) {
   //Serial.print(",");
   //Serial.print(velocitySetPoint1);
   //Serial.print(",");
   Serial.print(encoder0Pos - encoder1Pos);
   Serial.print(",");
   //Serial.println(encoder1Pos);
   //Serial.print(",");
   //Serial.print(encoder0Pos - encoder1Pos);
   //Serial.print(",");
   //Serial.print(diff0);
   //Serial.print(",");
   //Serial.print(diff1);
   //Serial.print(",");
   Serial.print(Qinterval0);
   Serial.print(",");
   Serial.print(Qinterval1);
   Serial.print(",");
   Serial.println(intervalSetPoint0);
   
    // Serial.println(millis() - printtime);
   oldprinttime = printtime;  }
}// end of doOutput


void doEncoder() {
   
    encoder0Pos++;
    
    interval00 = micros() - prevTime0;
    if (interval00 > 2500) {
     interval00 = 2500; 
    }
    interval0 += interval00;
    prevTime0 = micros();
    count0++;
    if (count0 == 16){ 
     
    

    Qinterval0 = interval0 / 16;
    count0 = 0;
    interval0 = 0; 
    }
}

void doEncoder_() {
    encoder1Pos++;
     
    interval11 = micros() - prevTime1;
    if (interval11 > 2500) {
     interval11 = 2500; 
    }
    interval1 += interval11;
    prevTime1 = micros();
    count1++;
    if (count1 == 16){ 
     
   
     
     Qinterval1 = interval1 / 16;
     count1 = 0;
     interval1 = 0; 
    }  
}


