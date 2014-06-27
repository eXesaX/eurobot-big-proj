#ifndef Ultra_h
#define Ultra_h


class Ultra
{
  public:
  Ultra(int pin1,int pin2);
  long Range();
  long Time();
  private:
  int trig,echo;
};
#endif