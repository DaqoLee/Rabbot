#include "foc.h"
#include "log.h"
#include "shell.h"


FOCController :: FOCController(uint8_t pp, EncoderBase *encoder, DriverBase *driver, CurrentBase *current)
{
    _pp = pp;
    _encoder = encoder;
    _driver = driver;
    _current = current;

   // _pidPos = new(PIDController<float>(0, 0, 0));


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
    _current->init();

    _pidPosition.Kp = 1.5f;
    _pidPosition.Ki = 0.0f;
    _pidPosition.Kd = 0.2;
    arm_pid_init_f32(&_pidPosition, false);

    _pidVelocity.Kp = 0.0010f;
    _pidVelocity.Ki = 0.00002f;
    _pidVelocity.Kd = 0;
    arm_pid_init_f32(&_pidVelocity, false);

    // _pidIq.Kp = 0.01f;
    // _pidIq.Ki = 0.0001f;
    // _pidIq.Kd = 0;
    // arm_pid_init_f32(&_pidIq, false);

    // _pidId.Kp = 0.01f;
    // _pidId.Ki = 0.0001f;
    // _pidId.Kd = 0;
    // arm_pid_init_f32(&_pidId, false);


    _pidIq.setPID(0.5f, 0.015f, 0);
    _pidIq.setOutputBounds(-0.3f, 0.3f);
    _pidIq.setMaxIntegralCumulation(0.1f);

    _pidId.setPID(0.5f, 0.015f, 0);
    _pidId.setOutputBounds(-0.3f, 0.3f);
    _pidId.setMaxIntegralCumulation(0.1f);

     setPhaseVoltage(0, 0.1,0);
    HAL_Delay(1000);
    while(_zeroElecAngle < 0.1)
    {
        _encoder->update(1000);
        _zeroElecAngle = getElecAngle();
        HAL_Delay(1);
    }
      
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

    Uq = Uq > 0.5f ? 0.5f : Uq;
    Uq = Uq < -0.5f ? -0.5f : Uq;

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

void FOCController :: setTargetIq(float iq)
{
    _targetIq = iq;
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

typedef struct {
    float Kp;
    float Ki;
    float integral;
    float output_limit;
} PI_Controller;


PI_Controller id_pi = { .Kp =  0.8f, .Ki = 0.15f, .output_limit = 0.1f };
PI_Controller iq_pi = { .Kp =  0.8f, .Ki = 0.15f, .output_limit = 0.1f };

float PI_Update(PI_Controller* pi, float error, float dt) {
    pi->integral += error * dt;
    pi->integral = fmaxf(fminf(pi->integral, pi->output_limit), -pi->output_limit);
    return pi->Kp * error + pi->Ki * pi->integral;
}

void FOCController :: TorqueLoop(void)
{
    float alpha= 0.0f, beta= 0.0f;
    //_current->update(20000);
    // clarkeTransformation(_current->getIa(),_current->getIb(),&alpha, &beta);
    // parkTransformation(alpha, beta, getElecAngle(), &_d, &_q);
    // _ud = PI_Update(&id_pi, (_targetId - _d), 0.00005f);
    // _uq = PI_Update(&iq_pi, (_targetIq - _q), 0.00005f);

    // setPhaseVoltage(_uq, _ud, getElecAngle());

}



void  FOCController :: loop()
{
    float ang = 0.0f;
    float vel = 0.0f;
    float tarVel = 0.0f;
    // float uq = 0.0f;
    // float ud = 0.0f;
    float alpha= 0.0f, beta= 0.0f;
    // float q = 0.0f, d = 0.0f;
    static float tempIq = 0;

    _encoder->update(1000);
    _current->update(1000);
    
    ang = _encoder->getAngle();
    vel = _encoder->getVelocity();


    // if (temp++ > 10)
    // {
    //     printf("d=: %f, %f, %f \n",_current->getIa(),_current->getIb(),_current->getIc());
    //     temp = 0;
    // }
    



    switch (_mode)
    {
    case Mode::POSITION:

       _uq =  arm_pid_f32(&_pidPosition,  _targetPosition - ang );
        setPhaseVoltage(_uq, _ud, getElecAngle());
        break;
    case Mode::VELOCITY:

        _uq =  arm_pid_f32(&_pidVelocity,  _targetVelovity - vel );
        setPhaseVoltage(_uq, _ud, getElecAngle());
        break;   
    case Mode::TORQUE:
        clarkeTransformation(_current->getIa(),_current->getIb(),&alpha, &beta);
        parkTransformation(alpha, beta, getElecAngle(), &_d, &_q);

        // uq = arm_pid_f32(&_pidIq,  _targetIq - q);
        // ud = arm_pid_f32(&_pidId,  _targetId - d);

        _uq = _pidIq.tick(_q, _targetIq);
        _ud = _pidId.tick(_d, _targetId);

        // _ud = PI_Update(&id_pi, (_targetId - _d), 0.001);
        // _uq = PI_Update(&iq_pi, (_targetIq - _q), 0.001);

        setPhaseVoltage(_uq, _ud, getElecAngle());
        break;
    case Mode::POSITION_VELOCITY:

        tarVel =  arm_pid_f32(&_pidPosition,  _targetPosition - ang );
        tarVel = tarVel > _targetVelovity ? _targetVelovity : tarVel;
        tarVel = tarVel < -_targetVelovity ? -_targetVelovity : tarVel;
        tempIq =  arm_pid_f32(&_pidVelocity,  tarVel - vel );

        tempIq = tempIq > _targetIq ? _targetIq : tempIq;
        tempIq = tempIq < -_targetIq ? -_targetIq : tempIq;

        clarkeTransformation(_current->getIa(),_current->getIb(),&alpha, &beta);
        parkTransformation(alpha, beta, getElecAngle(), &_d, &_q);

        _ud = PI_Update(&id_pi, (_targetId - _d), 0.001);
        _uq = PI_Update(&iq_pi, (tempIq - _q), 0.001);

        setPhaseVoltage(_uq, _ud, getElecAngle());
        break;
    case Mode::OPEN:

        break;
    default:
        logError("Mode error: %d ",_mode);
        break;
    }
}

extern uint16_t ADCdata[3];

float temp_trans(uint16_t ADC_value)
{
   
   float temp = 0;
   float Rt=0;  
   float R=3300; //
   float T0=273.15+25;//
   float B=3380; //Bֵ
   float Ka=273.15; //Kֵ
   float VR=0;//
   VR=(float)(((float)ADC_value/4096)*3.0); //
   Rt=(3.0-VR)*R/VR;//
   temp=1/(1/T0+log(Rt/R)/B)-Ka+0.5; //
   return temp;
}

void  FOCController :: logLoop()
{
    float ia, ib, ic;

    ia = _current->getIa();
    ib = _current->getIb();
    ic = _current->getIc();

    //  printf("%f %d\r\n",((float)ADCdata[0]/4096)*3.0*16,ADCdata[2]);

    printf("d=: %f, %f, %f, %f, %f, %f \n",ia,ib,ic,temp_trans(ADCdata[1]),((float)ADCdata[0]/4096)*3.0*16, _encoder->getVelocity());
}

template class PIDController<float>;
