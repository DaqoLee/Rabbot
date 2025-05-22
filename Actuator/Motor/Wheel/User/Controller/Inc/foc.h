#pragma once

#include "stdio.h"
#include "conf.h"
#include "arm_math.h"
#include <stdbool.h>
#include "stm32g4Board.h"
#include "PID.h"
//namespace foc
//{
    
typedef enum  
{
    POSITION,
    VELOCITY,
    TORQUE,
    POSITION_VELOCITY,
    OPEN

}Mode;


class FOCController
{
private:
    uint8_t _pp;
    float _zeroElecAngle = 0.0f;

    EncoderBase *_encoder;
    DriverBase *_driver;
    CurrentBase *_current;

    arm_pid_instance_f32 _pidPosition;
    arm_pid_instance_f32 _pidVelocity;

    // arm_pid_instance_f32 _pidId;
    // arm_pid_instance_f32 _pidIq;

    Mode _mode = VELOCITY;
    float _targetPosition = 100.0f;
    float _targetVelovity = 100.0f ;

    float _targetIq = 0.08f;
    float _targetId = 0.0f ;

    float _uq = 0.0f;
    float _ud = 0.0f;
    float _q = 0.0f;
    float _d = 0.0f;

    void clarkeTransformation(float Ia, float Ib, float* alpha, float* beta);
    void parkTransformation(float alpha, float beta, float theta, float* d, float* q);

    void invClarkeTransformation(float alpha, float beta, float* Ua, float* Ub, float* Uc);
    void invParkTransformation(float d, float q, float theta, float* alpha, float* beta);
    void setPhaseVoltage(float Uq, float Ud, float angle_el);
public:

    
   // PIDController<float> _pidPos;
    PIDController<float> _pidIq;
    PIDController<float> _pidId;

    FOCController(uint8_t pp, EncoderBase *encoder, DriverBase *driver, CurrentBase *current);
    FOCController(uint8_t pp);
   // ~FOCController();

    void init(void);
    void attachDriver(DriverBase *driver);
    void attachEncoder(EncoderBase *encoder);

    float getElecAngle(void);
    float getMechAngle(void);
    float getVelocity(void);

    void setTargetPosition(float position);
    void setTargetVelocity(float velocity);
    void setTargetIq(float iq);
    void loop(void);
    void logLoop(void);
    void TorqueLoop(void);

};


