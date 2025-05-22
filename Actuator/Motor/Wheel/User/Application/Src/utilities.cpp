#include "utilities.h"
#include "shell_cpp.h"
 #include "foc.h"
 #include "shell_port.h"
 #include "log.h"
// #include "user.h"
extern FOCController motor;
int tarang = 100;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_INT), 
  tarang, 
  &tarang, 
  test);


void setPosition(float pos)
{
  motor.setTargetPosition(pos);
  logPrintln("position: %f",pos);
}

SHELL_EXPORT_CMD_AGENCY(SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
  setPosition, 
  setPosition, 
  set Position,
  SHELL_PARAM_FLOAT(p1));


void setIq(float iq)
{
  motor.setTargetIq(iq);
  logPrintln("iq: %f",iq);
}

SHELL_EXPORT_CMD_AGENCY(SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
  setIq, 
  setIq, 
  set Iq,
  SHELL_PARAM_FLOAT(p1));

  void setVelocity(float vel)
  {
    motor.setTargetVelocity(vel);
    logPrintln("Velocity: %f",vel);
  }
  
  SHELL_EXPORT_CMD_AGENCY(SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
    setVelocity, 
    setVelocity, 
    set Velocity,
    SHELL_PARAM_FLOAT(p1));
  
 void getPosition(void)
 {
    logPrintln("position: %f",motor.getMechAngle());
 }

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
  getPosition, 
  getPosition, 
  get Position);


  void getVelocity(void)
  {
     logPrintln("Velocity: %f",motor.getVelocity());
  }
 
 SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
   getVelocity, 
   getVelocity, 
   get Velocity);
 
