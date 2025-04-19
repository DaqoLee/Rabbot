#include "foc.h"
#include "log.h"
#include "shell.h"

FOCController :: FOCController(uint8_t pp, EncoderBase *encoder, DriverBase *driver)
{
    _pp = pp;
    _encoder = encoder;
    _driver = driver;


    // _encoder->init();
}

FOCController :: FOCController(uint8_t pp)
{
    _pp = pp;
    // _encoder->init();
}
#define _3PI_2 4.71238898038f
void FOCController :: init(void)
{
    _encoder->init();
    _driver->init();

    _pidPosition.Kp = 1.5f;
    _pidPosition.Ki = 0.0f;
    _pidPosition.Kd = 0.2;
    arm_pid_init_f32(&_pidPosition, false);

    _pidVelocity.Kp = 0.0015f;
    _pidVelocity.Ki = 0.0001f;
    _pidVelocity.Kd = 0;
    arm_pid_init_f32(&_pidVelocity, false);
    // setPhaseVoltage(3, 0,_3PI_2);
    // HAL_Delay(2000);
    // while(_zeroElecAngle < 1)
    // {
    //     _encoder->update();
    //     _zeroElecAngle = getElecAngle();
    //     HAL_Delay(30);
    // }
      
    setPhaseVoltage(0, 0, 0);
}


float FOCController :: getElecAngle(void)
{
    float theta = fmod(_encoder->getRadian() * _pp -_zeroElecAngle, 2*PI);
    return theta < 0 ? theta + 2 * PI : theta;
}


float FOCController :: getMechAngle(void)
{

    return _encoder->getAngle();
}


void FOCController :: setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float ua = 0, ub = 0, uc = 0;
    float Ualpha = 0, Ubeta = 0;

    invParkTransformation(Ud, Uq, angle_el, &Ualpha, &Ubeta);

    invClarkeTransformation(Ualpha, Ubeta, &ua, &ub, &uc);
    //_driver->setPWM(ua, ub, uc);
    //printf("ua: %f ub: %f uc: %f angle_el: %f\r\n ",ua + 0.3f, ub+ 0.3f, uc+ 0.3f, angle_el);
    ua = 0.5f + ua;  
    ub = 0.5f + ub;
    uc = 0.5f + uc; 
    _driver->setPWM(ua, ub, uc);
}

void FOCController :: setTargetPosition(float position)
{
    _targetPosition = position;
}
void FOCController :: setTargetVelocity(float velocity)
{
    _targetVelovity = velocity;
}


void FOCController :: clarkeTransformation(float Ia, float Ib, float* alpha, float* beta)
{
    arm_clarke_f32(Ia, Ib, alpha, beta);
}
void FOCController ::parkTransformation(float alpha, float beta, float theta, float* d, float* q)
{
    float sin_theta = arm_sin_f32(theta);
    float cos_theta = arm_cos_f32(theta);

    arm_park_f32(alpha, beta, d, q, sin_theta, cos_theta);
}

void FOCController :: invClarkeTransformation(float alpha, float beta, float* Ua, float* Ub, float* Uc)
{
    arm_inv_clarke_f32(alpha, beta, Ua, Ub);
    *Uc = -(*Ua + *Ub);
}
void FOCController :: invParkTransformation(float d, float q, float theta, float* alpha, float* beta)
{
    float sin_theta = arm_sin_f32(theta);
    float cos_theta = arm_cos_f32(theta);

    arm_inv_park_f32(d, q, alpha, beta, sin_theta, cos_theta);
}


float FOCController :: getVelocity(void)
{
    return _encoder->getVelocity();
}

void  FOCController :: loop()
{
    static float theta = 0;
    float ang = 0.0f;
    float vel = 0.0f;
    float tarVel = 0.0f;
    _encoder->update(1000);

    ang = _encoder->getAngle();
    vel = _encoder->getVelocity();
    tarVel =  arm_pid_f32(&_pidPosition,  _targetPosition - ang );

    tarVel = tarVel > _targetVelovity ? _targetVelovity : tarVel;
    tarVel = tarVel < -_targetVelovity ? -_targetVelovity : tarVel;

    float uq =  arm_pid_f32(&_pidVelocity,  tarVel - vel );

    uq = uq > 0.5f ? 0.5f : uq;
    uq = uq < -0.5f ? -0.5f : uq;

    //printf("uq: %f angle %f\r\n", uq, ang);
    setPhaseVoltage(uq, 0, getElecAngle());


    
    // logError("Data length error");
    // HAL_Delay(10);
    // uint8_t _mode = 1;
    
    // switch (_mode)
    // {
    // case 1:
    //     // float alpha, beta;
    //     // clarkeTransformation(0, 0, &alpha, &beta);
        
    //     // float d, q;
    //     // parkTransformation(alpha, beta, getElecAngle(), &d, &q);

    //     setPhaseVoltage(10, 0, getElecAngle());
    //     break;
    
    // default:
    //     break;
    // }
}
// #define deg2rad(a) (PI * (a) / 180)
// #define rad2deg(a) (180 * (a) / PI)
// #define max(a, b) ((a) > (b) ? (a) : (b))
// #define min(a, b) ((a) < (b) ? (a) : (b))
// #define rad60 deg2rad(60)
// #define SQRT3 1.73205080756887729353

// arm_pid_instance_f32 pid_position;
// arm_pid_instance_f32 pid_speed;
// arm_pid_instance_f32 pid_torque_d;
// arm_pid_instance_f32 pid_torque_q;
// motor_control_context_t motor_control_context;

// float motor_i_u;
// float motor_i_v;
// float motor_i_d;
// float motor_i_q;
// float motor_speed;
// float motor_logic_angle;
// float encoder_angle;
// float rotor_zero_angle;


// static void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
// {
//     d = min(d, 1);
//     d = max(d, -1);
//     q = min(q, 1);
//     q = max(q, -1);

//     const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
//     const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};

//     float sin_phi = arm_sin_f32(phi);
//     float cos_phi = arm_cos_f32(phi);

//     float alpha = 0;
//     float beta = 0;

//     arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

//     bool A = beta > 0;
//     bool B = fabs(beta) > SQRT3 * fabs(alpha);
//     bool C = alpha > 0;

//     int K = 4 * A + 2 * B + C;
//     int sector = K_to_sector[K];

//     float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
//     float t_n = beta * arm_cos_f32(sector * rad60 - rad60) - alpha * arm_sin_f32(sector * rad60 - rad60);
//     float t_0 = 1 - t_m - t_n;

//     *d_u = t_m * v[sector - 1][0] + t_n * v[(sector) % 6][0] + t_0 / 2;
//     *d_v = t_m * v[sector - 1][1] + t_n * v[(sector) % 6][1] + t_0 / 2;
//     *d_w = t_m * v[sector - 1][2] + t_n * v[(sector) % 6][2] + t_0 / 2;
// }

// // __attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w)
// // {
// //     while (1)
// //         ;
// // }

// void set_pwm_duty(float d_u, float d_v, float d_w)
// {
//   d_u = min(d_u, 0.9); // 留出10%的电流采样时间
//   d_v = min(d_v, 0.9);
//   d_w = min(d_w, 0.9);
//   __disable_irq();
//   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR);
//   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
//   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);
//   __enable_irq();
// }

// void foc_forward(float d, float q, float rotor_rad)
// {
//     float d_u = 0;
//     float d_v = 0;
//     float d_w = 0;
//     svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
//     set_pwm_duty(d_u, d_v, d_w);
// }

// static float position_loop(float rad)
// {
//     float diff = cycle_diff(rad - motor_logic_angle, position_cycle);
//     return arm_pid_f32(&pid_position, diff);
// }

// static float speed_loop(float speed_rad)
// {
//     float diff = speed_rad - motor_speed;
//     return arm_pid_f32(&pid_speed, diff);
// }

// static float torque_d_loop(float d)
// {
//     float diff = d - motor_i_d / MAX_CURRENT;
//     float out_unlimited = arm_pid_f32(&pid_torque_d, diff);
//     float out_limited = 0;
//     out_limited = min(out_unlimited, 1);
//     out_limited = max(out_limited, -1);

//     float error_integral_windup = out_limited - out_unlimited; //  积分饱和误差
//     float Kt = 0.7f;                                           //  后积分增益 (Anti-windup Gain)
//     pid_torque_d.state[0] -= (pid_torque_d.Ki * Kt * error_integral_windup);

//     return out_limited;
// }

// static float torque_q_loop(float q)
// {
//     float diff = q - motor_i_q / MAX_CURRENT;
//     float out_unlimited = arm_pid_f32(&pid_torque_q, diff);
//     float out_limited = 0;
//     out_limited = min(out_unlimited, 1);
//     out_limited = max(out_limited, -1);

//     float Kt = 0.7f;                                           //  后积分增益 (Anti-windup Gain)
//     float error_integral_windup = out_limited - out_unlimited; //  积分饱和误差
//     pid_torque_q.state[0] -= (pid_torque_q.Ki * Kt * error_integral_windup);

//     return out_unlimited;
// }

// void lib_position_control(float rad)
// {
//     float d = 0;
//     float q = position_loop(rad);
//     foc_forward(d, q, rotor_logic_angle);
// }

// void lib_speed_control(float speed)
// {
//     float d = 0;
//     float q = speed_loop(speed);
//     foc_forward(d, q, rotor_logic_angle);
// }

// void lib_torque_control(float torque_norm_d, float torque_norm_q)
// {
//     float d = torque_d_loop(torque_norm_d);
//     float q = torque_q_loop(torque_norm_q);
//     foc_forward(d, q, rotor_logic_angle);
// }

// void lib_speed_torque_control(float speed_rad, float max_torque_norm)
// {
//     float torque_norm = speed_loop(speed_rad);
//     torque_norm = min(fabs(torque_norm), max_torque_norm) * (torque_norm > 0 ? 1 : -1);
//     lib_torque_control(0, torque_norm);
// }

// void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
// {
//     float speed_rad = position_loop(position_rad);
//     speed_rad = min(fabs(speed_rad), max_speed_rad) * (speed_rad > 0 ? 1 : -1);
//     lib_speed_torque_control(speed_rad, max_torque_norm);
// }

// void set_motor_pid(
//     float position_p, float position_i, float position_d,
//     float speed_p, float speed_i, float speed_d,
//     float torque_d_p, float torque_d_i, float torque_d_d,
//     float torque_q_p, float torque_q_i, float torque_q_d)
// {
//     pid_position.Kp = position_p;
//     pid_position.Ki = position_i;
//     pid_position.Kd = position_d;

//     pid_speed.Kp = speed_p;
//     pid_speed.Ki = speed_i;
//     pid_speed.Kd = speed_d;

//     pid_torque_d.Kp = torque_d_p;
//     pid_torque_d.Ki = torque_d_i;
//     pid_torque_d.Kd = torque_d_d;

//     pid_torque_q.Kp = torque_q_p;
//     pid_torque_q.Ki = torque_q_i;
//     pid_torque_q.Kd = torque_q_d;
//     arm_pid_init_f32(&pid_position, false);
//     arm_pid_init_f32(&pid_speed, false);
//     arm_pid_init_f32(&pid_torque_d, false);
//     arm_pid_init_f32(&pid_torque_q, false);
// }

// float cycle_diff(float diff, float cycle)
// {
//     if (diff > (cycle / 2))
//         diff -= cycle;
//     else if (diff < (-cycle / 2))
//         diff += cycle;
//     return diff;
// }