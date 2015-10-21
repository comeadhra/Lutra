#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Platypus.h"
#include "RoboClaw.h"

namespace platypus 
{
  const int DEFAULT_BUFFER_SIZE = 128;

  // ESCs //
  class VaporPro : public Motor 
  {
  public:
    VaporPro(int channel) : Motor(channel) {}
    void arm();
  };

  class HobbyKingBoat : public Motor 
  {
  public:
    HobbyKingBoat(int channel) : Motor(channel) {}
    void arm();
  };
  
  class Seaking : public Motor 
  {
  public:
    Seaking(int channel) : Motor(channel) {}
    void arm();
  };

  class Swordfish : public Motor 
  {
  public:
    Swordfish(int channel) : Motor(channel) {}
    void arm();
  };

    class Dynamite : public Motor 
  {
  public:
    Dynamite(int channel) : Motor(channel) {}
    void arm();
  };

  // Sensors //
  class AnalogSensor : public Sensor 
  {
  public:
    AnalogSensor(int channel);

    bool set(char* param, char* value);
    virtual char *name() = 0;
    
    void scale(float scale);
    float scale();
    
    void offset(float offset);
    float offset();
    
  private:
    float scale_;
    float offset_;
  };
  
  class ServoSensor : public Sensor 
  {
  public:
    ServoSensor(int channel);
    ~ServoSensor();

    bool set(char* param, char* value);
    char *name();
    
    void position(float velocity);
    float position();
    
  private:
    Servo servo_;
    float position_;
  };

  class PoweredSensor : virtual public Sensor 
  {
  public:
    PoweredSensor(int channel, bool poweredOn=true);
    virtual char *name() = 0;
    bool powerOn();
    bool powerOff();

  private:
    bool state_;
  };

  class SerialSensor : virtual public Sensor
  {
  public:
    SerialSensor(int channel, int baudRate, int dataStringLength = 0);
    virtual char * name() = 0;
    void onSerial();

  private:
    int baud_;
    int minDataStringLength_;
    char recv_buffer_[DEFAULT_BUFFER_SIZE];
    unsigned int recv_index_;
  };

  
  class ES2 : public PoweredSensor, public SerialSensor
  {
  public:
    ES2(int channel);
    char *name();
    void loop();
    //void onSerial();
  };
/*
  class AtlasSensor : public Sensor 
  {
  public:
    AtlasSensor(int channel);
    virtual char *name();
    void onSerial();
    void loop();
    
  private:
    char recv_buffer_[DEFAULT_BUFFER_SIZE];
    unsigned int recv_index_;
  };
*/
  class AtlasPH : public SerialSensor
  {
  public:
    AtlasPH(int channel);
    char * name();
    void setTemp(double temp);
    void calibrate();
  };

  class AtlasDO : public SerialSensor
  {
  public:
    AtlasDO(int channel);
    char * name();
    void setTemp(double temp);
    void setEC(double EC);
    void calibrate();
  };
  
  class HDS : public Sensor 
  {
  public:
    HDS(int channel);
    char *name();
    void onSerial();
    
  private:
    char recv_buffer_[DEFAULT_BUFFER_SIZE];
    unsigned int recv_index_;
  };
  
  class Winch : public Sensor 
  {
  public:
    Winch(int channel, uint8_t address);
    char *name();
    bool set(char* param, char* value);
    
    void reset();

    void velocity(int32_t pos);
    void position(uint32_t pos);
    uint32_t encoder(bool *valid = NULL);

  private:
    RoboClaw roboclaw_;
    uint8_t address_;
    uint32_t desired_position_;
    int32_t desired_velocity_;
    uint32_t desired_acceleration_;
  };
}

#endif //COMPONENTS_H
