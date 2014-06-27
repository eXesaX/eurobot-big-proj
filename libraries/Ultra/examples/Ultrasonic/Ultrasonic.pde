#include <Ultra.h>
Ultra AKR(7,6);
void setup()
{
  Serial.begin(9600);
}
void loop()
{
  int distance;
  Serial.println(AKR.Time());
 Serial.print(AKR.Range());
 Serial.println("CM");
}

