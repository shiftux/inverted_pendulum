#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>

// pin definitions
#define MOTOR_PWM_A 9
#define MOTOR_PWM_B 5
#define MOTOR_PWM_C 6
#define MOTOR_ENABLE 8
#define MOTOR_POLE_PAIRS 11
#define MOTOR_ENCODER_A 3
#define MOTOR_ENCODER_B 2
#define PENDULUM_ENCODER_A 14
#define PENDULUM_ENCODER_B 15
#define CONTROL_LOOP_ENABLE 13

// parameters
#define INPUT_VOLTAGE 11.1
#define CONTROL_LOOP_DURATION 25 // ms
#define SWING_UP_VOLT_FACTOR 0.4
#define MOTOR_VOLT_LIMIT_FACTOR 0.7
#define LQR_K1 35   // 40
#define LQR_K2 5    // 7
#define LQR_K3 0.3  // 0.3

// BLDC motor init
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
// define BLDC driver
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE);
// encoder init
Encoder m_encoder = Encoder(MOTOR_ENCODER_A, MOTOR_ENCODER_B, 1024);
Encoder p_encoder = Encoder(PENDULUM_ENCODER_A, PENDULUM_ENCODER_B, 1024);
// channel A and B callbacks
void doA(){m_encoder.handleA();}
void doB(){m_encoder.handleB();}
void doPA(){p_encoder.handleA();}
void doPB(){p_encoder.handleB();}
// pin change listeners (for second encoder not on dedicated encoder pins)
PciListenerImp listenerPA(p_encoder.pinA, doPA);
PciListenerImp listenerPB(p_encoder.pinB, doPB);

void setup() {
  pinMode(CONTROL_LOOP_ENABLE, INPUT);
  // Serial.begin(9600);
  // initialize motor encoder hardware
  m_encoder.init();
  p_encoder.init();
  m_encoder.enableInterrupts(doA,doB);
  PciManager.registerListener(&listenerPA);
  PciManager.registerListener(&listenerPB);

  // driver config
  driver.voltage_power_supply = INPUT_VOLTAGE;
  driver.init();

  // set control loop type to be used
  motor.controller = ControlType::voltage;

  // link the motor to the encoder
  motor.linkSensor(&m_encoder);
  // link the motor to the driver
  motor.linkDriver(&driver);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
}

// loop down-sampling counter
long loop_count = 0;

void loop() {
  // ~1ms
  motor.loopFOC();
  // motor.move(2);
  // Serial.println((String)"motor shaft v: "+motor.shaftVelocity()+"pendulum angle: "+constrainAngle(p_encoder.getAngle() + M_PI)+"pendulum velocity: "+p_encoder.getVelocity());

  // control loop
  if(digitalRead(CONTROL_LOOP_ENABLE) == HIGH){
    Serial.println(digitalRead(CONTROL_LOOP_ENABLE));
    if(loop_count++ > CONTROL_LOOP_DURATION){

      // calculate the pendulum angle
      float pendulum_angle = constrainAngle(p_encoder.getAngle() + M_PI);

      float target_voltage;
      if( abs(pendulum_angle) < 0.5 ) // if angle small enough stabilize
        target_voltage = controllerLQR(pendulum_angle, p_encoder.getVelocity(), motor.shaftVelocity());
      else // else do swing-up
        // sets 40% of the maximal voltage to the motor in order to swing up
        target_voltage = -_sign(p_encoder.getVelocity())*driver.voltage_power_supply*SWING_UP_VOLT_FACTOR;

      // set the target voltage to the motor
      motor.move(target_voltage);

      // restart the counter
      loop_count=0;
    }
  } else {
        motor.move(0);
  }


}

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel){
  // if angle controllable
  // calculate the control law
  // LQR controller u = k*x
  //  - k = [40, 7, 0.3]
  //  - x = [pendulum angle, pendulum velocity, motor velocity]'
  float u =  LQR_K1*p_angle + LQR_K2*p_vel + LQR_K3*m_vel;

  // limit the voltage set to the motor
  if(abs(u) > driver.voltage_power_supply*MOTOR_VOLT_LIMIT_FACTOR) {
    u = _sign(u)*driver.voltage_power_supply*MOTOR_VOLT_LIMIT_FACTOR;
  }

  return u;
}