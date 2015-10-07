#ifndef RC_H
#define RC_H

#include <Arduino.h>

enum rc_pwm_t{
      MAX=0,
      MIN=1,
      CUR=2
};

extern void send(char * str);
/**
 * RC_Controller: Class used to read and handle commands sent by an attached RC controller
 *                The class reads auxiliary, throttle and rudder channels and converts from
 *                PPM into boolean, float and float respectively. These values can be read
 *                using the appropriate accessor methods. The class assumes the following
 *                functionality:
 *                AUX Pin is low:  Ignore throttle and rudder input
 *                AUX Pin is high: Read throttle and rudder input. Throttle and rudder
 *                                 output have no effect until the controller is armed using
 *                                 the command: throttle down, rudder left. After 5 seconds
 *                                 the same sequence will disarm the controller.
 *                                 A throttle down and rudder right command will force the
 *                                 esc into calibration mode. This process will block for
 *                                 10 seconds.
 *
 *                                 Once armed the throttle and rudder commands will be mixed
 *                                 to give a velocity for each motor
 */
class RC_Controller{
  public:
  
    RC_Controller();
    RC_Controller(int aux_pin, int throttle_pin, int rudder_pin, int arming_pin );

    void setLeftRudder(int lr);
    void setRightRudder(int rr);
    void setMaxThrottle(int mt);
    void setMidThrottle(int mt);
    void setMinThrottle(int mt);
    void setAuxLow(int al);
    void setAuxHigh(int ah);
    void setArmingLow(int gl);
    void setArmingHigh(int gh);
    void setMotorUpdateBlocked(bool state);
    void setVelocityMode(int mode);

    int getPWMThrottle(rc_pwm_t type);
    int getPWMRudder(rc_pwm_t type);
    int getPWMAux(rc_pwm_t type);
    int getPWMArming(rc_pwm_t type);
    
    bool  isCalibrateEnabled();
    bool  isOverrideEnabled();
    bool  isArmed();
    bool  isMotorUpdateBlocked();
    
    float throttleVal();
    float rudderVal();
    float rightVelocity();
    float leftVelocity();
    float rightFan();
    float leftFan();
    void  update();
    void  configUpdate();

  private:
      //RC Controller Ouptut Values
    int min_throttle = 962;
    int max_throttle = 1952;
    int mid_throttle = 1438;
    int cur_throttle = 0;

    int left_rudder = 961;
    int right_rudder = 1952;
    int cur_rudder = 0;
    
    int aux_low = 960*0.95;
    int aux_high = 1952*1.05;
    int aux_threshold_l = 1500*0.9;
    int aux_threshold_h = 1500*1.1;
    int aux_cur = 0;
    
    int arming_low = 960*0.95;
    int arming_high = 1952*1.05;
    int arming_threshold_l = 1500*0.9;
    int arming_threshold_h = 1500*1.1;
    int arming_cur = 0;
    
    bool  calibrate = false;
    bool  overrideEnabled = false;
    bool  armed = false;
    bool  motorUpdateBlocked = false;
    int velocityMode = 0;
    
    float throttle_val = 0;
    float rudder_val = 0;
    int   aux_val;
    int arming_val;


};



#endif
