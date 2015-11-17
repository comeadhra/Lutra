#include "RC.h"

//Rx input pins
int THROTTLE_PIN = 3;
int RUDDER_PIN = 2;
int AUX_PIN = 20;
int ARMING_PIN = 21;

//Throttle and Rudder Values as sent by RC
volatile uint32_t throttle_pwm = 0;
volatile uint32_t rudder_pwm = 0;
volatile uint32_t aux_pwm = 0;
volatile uint32_t arming_pwm = 0;

volatile bool RCoverride = false;



//auxiliary pin listener
void auxInterrupt(  )
{
  static uint32_t auxStartTime;
  //Start timer on rising edge
  if(digitalRead(AUX_PIN))
  {
    auxStartTime = micros();
  }
  //Stop timer on falling edge
  else
  {
    //Compute pulse duration
    aux_pwm = (uint32_t)(micros() - auxStartTime);

  }
}

//Throttle pin listener
void throttleInterrupt(  )
{
  static uint32_t throttleStartTime;

  if(digitalRead(THROTTLE_PIN))
  {
    throttleStartTime = micros();
  }
  else
  {
    throttle_pwm = (uint32_t)(micros() - throttleStartTime);
  }

}

//Rudder pin listener
void rudderInterrupt(  )
{
  static uint32_t rudderStartTime;

  if(digitalRead(RUDDER_PIN))
  {
    rudderStartTime = micros();
  }
  else
  {
    rudder_pwm = (uint32_t)(micros() - rudderStartTime);
  }

}

//Rudder pin listener
void armedInterrupt(  )
{
  static uint32_t armedStartTime;

  if(digitalRead(ARMING_PIN))
  {
    armedStartTime = micros();
  }
  else
  {
    arming_pwm = (uint32_t)(micros() - armedStartTime);
  }

}

/**
 * Rc_Controller default constructor
 * Enable aux, throttle and rudder pins to be used for interrupts
 * Attach interrupt to auxiliary pin to listen for manual override
 *
 * Default pins are used as defined in header file unless custom
 * constructor was called
 */
RC_Controller::RC_Controller()
{
  Serial.println("Constructor Started");
  pinMode(THROTTLE_PIN, INPUT); digitalWrite(THROTTLE_PIN, LOW);
  pinMode(RUDDER_PIN, INPUT);   digitalWrite(RUDDER_PIN, LOW);
  pinMode(AUX_PIN, INPUT);    digitalWrite(AUX_PIN, LOW);
  pinMode(ARMING_PIN, INPUT); digitalWrite(ARMING_PIN, LOW);
  attachInterrupt(AUX_PIN, auxInterrupt, CHANGE);
  attachInterrupt(RUDDER_PIN, rudderInterrupt, CHANGE);
  attachInterrupt(THROTTLE_PIN, throttleInterrupt, CHANGE);
  attachInterrupt(ARMING_PIN, armedInterrupt, CHANGE);
  Serial.println("Constructor Completed");

}

/**
 * Custom Constructor to attach Rx module to given pins
 * @param aux_pin: Input pin for auxiliary channel
 * @param throttle_pin: Input pin for throttle channel
 * @param rudder_pin: Input pin for rudder channel
 */
RC_Controller::RC_Controller(int aux_pin, int throttle_pin, int rudder_pin, int arming_pin)
{
  //Assign pins
  AUX_PIN = aux_pin;
  THROTTLE_PIN = throttle_pin;
  RUDDER_PIN = rudder_pin;
  ARMING_PIN = arming_pin;
  //Call default constructor to initialise pins
  RC_Controller();
}

//Mutator methods used to set channel input limits
//Used for calibration routine.
void RC_Controller::setLeftRudder(int lr)  { left_rudder = lr; }
void RC_Controller::setRightRudder(int rr) { right_rudder = rr; }
void RC_Controller::setMaxThrottle(int mt) { max_throttle = mt; }
void RC_Controller::setMidThrottle(int mt) { mid_throttle = mt; }
void RC_Controller::setMinThrottle(int mt) { min_throttle = mt; }

void RC_Controller::setAuxLow(int al)
{
  aux_low = al;
  //Update threshold for auxiliary pin decision
 // aux_threshold = (aux_low + aux_high)/2;
}

void RC_Controller::setAuxHigh(int ah)
{
  aux_high = ah;
  //Update threshold for auxiliary pin decision
 // aux_threshold = (aux_low + aux_high)/2;
}

void RC_Controller::setArmingLow (int al) { arming_low = al;  }
void RC_Controller::setArmingHigh(int ah) { arming_high = ah; }

//Accessor Methods
bool  RC_Controller::isOverrideEnabled() {return overrideEnabled;}
bool  RC_Controller::isArmed(){ return armed;}
float RC_Controller::throttleVal() { return throttle_val; }
float RC_Controller::rudderVal() { return rudder_val; }
bool  RC_Controller::isCalibrateEnabled() {
  if (calibrate)
  {
    calibrate = false;
    return true;   
  }
  else
    return false;
}

bool RC_Controller::isMotorUpdateBlocked() { return motorUpdateBlocked; }

void RC_Controller::setMotorUpdateBlocked(bool state) { motorUpdateBlocked = state; }

void RC_Controller::setVelocityMode(int mode) { velocityMode = mode; }

int RC_Controller::getPWMThrottle(rc_pwm_t type)
{
  switch(type)
  {
    case MAX: return max_throttle;
    case MIN: return min_throttle;
    case CUR: return cur_throttle;
  };
}

int RC_Controller::getPWMRudder(rc_pwm_t type)
{
  switch(type)
  {
    case MAX: return left_rudder;
    case MIN: return right_rudder;
    case CUR: return cur_rudder;
  };
}

int RC_Controller::getPWMAux(rc_pwm_t type)
{
  switch(type)
  {
    case MAX: return aux_high;
    case MIN: return aux_low;
    case CUR: return aux_cur;
  };
}


int RC_Controller::getPWMArming(rc_pwm_t type)
{
  switch(type)
  {
    case MAX: return arming_high;
    case MIN: return arming_low;
    case CUR: return arming_cur;
  };
}


//Velocity mixers
//Implemented as simple linear mixers. Turn speed is controlled by
//the position of the throttle stick. Turning angle is linearly proportional
//to the position of the rudder

//Returns the velocity for the right motor
float RC_Controller::rightVelocity()
{
   if (velocityMode == 1)
  {
    
  }
  else
  {
      //Turning right -> reduce left motor speed
    if (rudder_val > 0 && throttle_val > 0)
    {
      return throttle_val;
    }
    else if(throttle_val > 0)
    {
      return throttle_val*(1+rudder_val);
    }
    else if(rudder_val < 0)
    {
      return throttle_val;
    }
    else
    {
      return throttle_val*(1-rudder_val);
    }
  }
}

//Returns the velocity for the right motor
float RC_Controller::leftVelocity()
{
  if (velocityMode == 1)
  {
    
  }
  else
  {
    //Turning right -> reduce left motor speed
    if (rudder_val > 0 && throttle_val > 0)
    {
      return throttle_val*(1-rudder_val);
    }
    else if(throttle_val > 0)
    {
      return throttle_val;
    }
    else if(rudder_val < 0)
    {
      return throttle_val*(1+rudder_val);
    }
    else
    {
      return throttle_val;
    }
  }
}

float RC_Controller::rightFan()
{
 
   // return 90.0 + rudder_val*30.0;
   return rudder_val;
}


float RC_Controller::leftFan()
{
  return rudder_val;
   // return 90.0 + rudder_val*30.0;
  
}

void RC_Controller::configUpdate()
{
  
      //Turn off interrupts to update local variables
      noInterrupts();
      aux_cur = aux_pwm;
      cur_throttle = throttle_pwm;
      cur_rudder = rudder_pwm;
      arming_cur = arming_pwm;
      interrupts();

    char output_str[200];
    snprintf(output_str, 160,
             "{"
             "\"type\":\"RC\","
             "\"signal\":\"throttle\":%d, %d, %d,"
             "\"signal\":\"rudder\":%d, %d, %d,"
             "\"signal\":\"auxiliary\":%d, %d, %d,"
             "\"signal\":\"arming\":%d, %d, %d,"
             "}",
              max_throttle, min_throttle, cur_throttle,
              left_rudder, right_rudder, cur_rudder,
              aux_high, aux_low, aux_cur,
              arming_high, arming_low, arming_cur
             
            );
    send(output_str);
}

//RC Update loop
//Reads channel inputs if available
//Sets Override flag and arming + calibration routine
void RC_Controller::update()
{
     

      //Turn off interrupts to update local variables
      noInterrupts();

      //Update local variables
      aux_val = aux_pwm;
      
      //AUX value is below threshold or outside of valid window
      //set override to false
      if(aux_val < aux_threshold_l || aux_val > aux_high)
      {
        //If override was previously enabled, then
        //stop listening to throttle and rudder pins
        if(overrideEnabled)
        {
          //detachInterrupt(THROTTLE_PIN);
          //detachInterrupt(RUDDER_PIN);
          //detachInterrupt(arming_PIN);
          arming_val = arming_low;
          throttle_val = 0;
          rudder_val = 0;

        }

        overrideEnabled = false;
        control_velocity = 0.1;
       // armed = false;

      }
      //AUX pin is above threshold
      else if(aux_val > aux_threshold_h)
      {
        //Attach throttle and rudder listeners
        //if override was previously off
        if(!overrideEnabled)
        {
         //attachInterrupt(RUDDER_PIN, rudderInterrupt, CHANGE);
         //attachInterrupt(THROTTLE_PIN, throttleInterrupt, CHANGE);
         //attachInterrupt(arming_PIN, armedInterrupt, CHANGE);
        }
        overrideEnabled = true;
        
        //Zero throttle and rudder values to prevent false readings
        throttle_val = 0;
        rudder_val = 0;
        arming_val = arming_low;

      }
      else
      {
              interrupts();
              return;
      }
      

      //Read throttle and rudder values if auxiliary pin was high
      if(overrideEnabled )
      {
        arming_val = arming_pwm;
        throttle_val = (float)map(throttle_pwm, min_throttle, max_throttle, -100, 100)/100.0;
        rudder_val   = (float)map(rudder_pwm,left_rudder, right_rudder, -100, 100)/100.0;
        if(abs(throttle_val) < 0.01) throttle_val = 0;
        if(abs(rudder_val) < 0.01) rudder_val = 0;
      }
      

      //Local update complete - turn on interrupts
       interrupts();


      //RC commands are being received - deal with calibration and arming
      if(overrideEnabled)
      {
          //Throttle down and rudder left: Change ESC arm state
          if(arming_val > arming_threshold_h && arming_val < arming_high)
          {
              control_state = true;
              //armed = true;
          }
          else if(arming_val < arming_threshold_l || arming_val > arming_high )
          {
              if(control_state)
              {
                control_state = false;
                control_velocity += 0.1;
                if(control_velocity > 1.05) control_velocity = 0.1;
              }
              //armed = false;
          }
          //Throttle down and rudder right: calibrate ESCs
          if(!armed && abs(throttle_val) < 0.05 &&  rudder_val > 0.95)
          {
            Serial.println("In calibrate");
            calibrate = false;
          }
          else if(!armed)
          {
            throttle_val = 0;
            rudder_val = 0;
          }

       }

}
