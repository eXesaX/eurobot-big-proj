#include <Ultra.h>
#include <Arduino.h>





 Ultra::Ultra(int pin1, int pin2)
{
  pinMode(pin1,OUTPUT);
  pinMode(pin2,INPUT);
  trig = pin1;
  echo = pin2;
  	
}

long Ultra::Time()
{
 digitalWrite(trig,HIGH);
 delayMicroseconds(1000);
 digitalWrite(trig,LOW);
  return(pulseIn(echo,HIGH));
  
}
long Ultra::Range()
{
 long x,y;
 digitalWrite(trig,HIGH);
 delayMicroseconds(1000);
 digitalWrite(trig,LOW);
 x = pulseIn(echo,HIGH);
 y= (x/2)/29.1;
 return(y);

}
