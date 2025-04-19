#pragma once

#include "stdio.h"
#include "conf.h"
#include "arm_math.h"
#include <stdbool.h>
#include "stm32g4Board.h"

class FOCController
{
private:
    uint8_t _pp;
    float _zeroElecAngle = 0.0f;

    EncoderBase *_encoder;
    DriverBase *_driver;
    arm_pid_instance_f32 _pidPosition;
    arm_pid_instance_f32 _pidVelocity;

    float _targetPosition = 100.0f;
    float _targetVelovity = 50.0f ;

    void clarkeTransformation(float Ia, float Ib, float* alpha, float* beta);
    void parkTransformation(float alpha, float beta, float theta, float* d, float* q);

    void invClarkeTransformation(float alpha, float beta, float* Ua, float* Ub, float* Uc);
    void invParkTransformation(float d, float q, float theta, float* alpha, float* beta);
    void setPhaseVoltage(float Uq, float Ud, float angle_el);
public:

    FOCController(uint8_t pp, EncoderBase *encoder, DriverBase *driver);
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

    void loop(void);


};

// FocController::FocController(/* args */)
// {
// }

// FocController::~FocController()
// {
// }


// #define rotor_phy_angle (encoder_angle - rotor_zero_angle) // ת������Ƕ�
// #define rotor_logic_angle (rotor_phy_angle * POLE_PAIRS)   // ת���߼��Ƕȣ���Ƕȣ�

// typedef enum
// {
//     control_type_null,                  // �����п���
//     control_type_position,              // λ�ÿ���
//     control_type_speed,                 // �ٶȿ���
//     control_type_torque,                // ���ؿ���
//     control_type_speed_torque,          // �ٶ�-���ؿ���
//     control_type_position_speed_torque, // λ��-�ٶ�-���ؿ���
// } motor_control_type;

// typedef struct
// {
//     motor_control_type type;
//     float position;        // Ŀ��Ƕȣ���λrad
//     float speed;           // Ŀ���ٶȣ���λrad/s
//     float torque_norm_d;   // Ŀ��d��ǿ�ȣ�0~1
//     float torque_norm_q;   // Ŀ��q��ǿ�ȣ�0~1
//     float max_speed;       // ��������ʱ������ٶȣ���λrad/s
//     float max_torque_norm; // ��������ʱ�����q�����أ�0~1
// } motor_control_context_t;

// extern motor_control_context_t motor_control_context;

// float cycle_diff(float diff, float cycle);
// void foc_forward(float d, float q, float rotor_rad);

// void lib_position_control(float rad);                                                           // λ�ã��Ƕȣ�����
// void lib_speed_control(float speed);                                                            // �ٶȿ���
// void lib_torque_control(float torque_norm_d, float torque_norm_q);                              // ���أ�����������
// void lib_speed_torque_control(float speed, float max_torque_norm);                              // �ٶ�-���ؿ���
// void lib_position_speed_torque_control(float position, float max_speed, float max_torque_norm); // λ��-�ٶ�-���ؿ���

// void set_motor_pid(
//     float position_p, float position_i, float position_d,
//     float speed_p, float speed_i, float speed_d,
//     float torque_d_p, float torque_d_i, float torque_d_d,
//     float torque_q_p, float torque_q_i, float torque_q_d);