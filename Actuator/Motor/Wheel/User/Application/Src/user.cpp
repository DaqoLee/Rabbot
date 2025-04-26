#include "user.h"
#include "shell_port.h"
 #include "foc.h"

 DRV8317 driver;
 MT6835 encoder;
 Current current;

 FOCController motor(7, &encoder, &driver, &current);

void setup(void)
{
  motor.init();
  userShellInit();
}


void loop(void)
{
  motor.loop();
  //motor.getMechAngle();
  //printf("angle: %f\r\n", motor.getMechAngle());
  //shellTask(&shell);
  
}

void logLoop(void)
{
  motor.logLoop();
  //motor.getMechAngle();
  //printf("angle: %f\r\n", motor.getMechAngle());
  //shellTask(&shell);
}

void torqueLoop(void)
{
  motor.TorqueLoop();
}