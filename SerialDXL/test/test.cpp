#define DEBUG
#include <SerialDXL.h>
#include <Encoder.h>
#include <PID_v1.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

/** 
 * Memory Mapping
 * 
 * General settings
 */
#define MODEL_L   0
#define MODEL_H   1
#define FIRMWARE  2
#define ID        3
#define BAUDRATE  4
#define RET_DELAY 5
/* Motor Settings */
#define STATE         6
#define VEL_COMMAND   7
#define DIR_COMMAND   8
#define POS_B0        9
#define POS_B1        10
#define POS_B2        11
#define POS_B3        12
#define POS_COMMAND   13
//------------------------------------------------------------------------------
/**
 * @brief Motor velocity control using DXL communication protocol
 * @details Motor velocity control using Dynamixel communication protocol over RS485.
 * 
 */
class MotorDXL: public DeviceDXL
{
  public:
    MotorDXL(uint8_t id, uint8_t dir_pin):
    DeviceDXL(id, 14),
    dir_pin_(dir_pin),
    eeprom_null_(0),
    state_(0),
    vel_command_(0),
    dir_command_(0),
    pos_command_(128),
    encoder_(2,3),
    position_(0),
    pid_in_(0),
    pid_out_(0),
    pid_ref_(0),
    pid_(&pid_in_, &pid_out_, &pid_ref_,  3.0, 1.0, 0.1, DIRECT)
    {
      /*
      * MMAP Config
      * General settings
      */
      // Model version
      MMAP_ENTRY(mmap_field_[MODEL_L], eeprom_null_, MMAP_EEPROM | MMAP_R);
      MMAP_ENTRY(mmap_field_[MODEL_H], eeprom_null_, MMAP_EEPROM | MMAP_R);
      // Firmware
      MMAP_ENTRY(mmap_field_[FIRMWARE], eeprom_null_, MMAP_EEPROM | MMAP_R);
      // ID
      MMAP_ENTRY(mmap_field_[ID], id_, MMAP_EEPROM | MMAP_RW);
      // Baudrate
      MMAP_ENTRY(mmap_field_[BAUDRATE], baudrate_, MMAP_EEPROM | MMAP_RW);
      // Return delay
      MMAP_ENTRY(mmap_field_[RET_DELAY], returnDelay_, MMAP_EEPROM | MMAP_RW);    
      
      /*
      * MMAP Config
      * Motor settings
      */
      // State
      MMAP_ENTRY(mmap_field_[STATE], state_, MMAP_RAM | MMAP_R);
      // Command
      MMAP_ENTRY(mmap_field_[VEL_COMMAND], vel_command_, MMAP_RAM | MMAP_RW);
      MMAP_ENTRY(mmap_field_[DIR_COMMAND], dir_command_, MMAP_RAM | MMAP_RW);
      // Encoder position
      MMAP_ENTRY(mmap_field_[POS_B0], position_b0_, MMAP_RAM | MMAP_R);
      MMAP_ENTRY(mmap_field_[POS_B1], position_b1_, MMAP_RAM | MMAP_R);
      MMAP_ENTRY(mmap_field_[POS_B2], position_b2_, MMAP_RAM | MMAP_R);
      MMAP_ENTRY(mmap_field_[POS_B3], position_b3_, MMAP_RAM | MMAP_R);
      // Position command
      MMAP_ENTRY(mmap_field_[POS_COMMAND], pos_command_, MMAP_RAM | MMAP_RW);
      // Init MMAP
      mmap_.init(mmap_field_);

      // Config LED pin
      pinMode(dir_pin_, OUTPUT);

      // H Bridge config
      pinMode(7,OUTPUT);
      pinMode(8,OUTPUT);
      pinMode(9,OUTPUT); // PWM

      // PID Config
      pid_.SetMode(AUTOMATIC);
      pid_.SetSampleTime(20);
      pid_.SetOutputLimits(-255, 255);
    }

    void initRAM()
    {
      mmap_.setFromEEPROM(ID);
      mmap_.setFromEEPROM(MODEL_L);
      mmap_.setFromEEPROM(MODEL_H);
      mmap_.setFromEEPROM(FIRMWARE);
      mmap_.setFromEEPROM(BAUDRATE);
    }

    void initEEPROM()
    {
      // Set default ID
      mmap_.setEEPROM(ID, 5);
      // Set model
      mmap_.setEEPROM(MODEL_L, 1);
      mmap_.setEEPROM(MODEL_H, 0);
      // Set firmware
      mmap_.setEEPROM(FIRMWARE, 1);
      // Baudrate 9600 baud
      mmap_.setEEPROM(BAUDRATE, 207);
    }

    void reset()
    {
      initEEPROM();
    }

    void update()
    {
      position_ = encoder_.read();
      pid_in_ = position_;
      pid_.Compute();


      position_b0_ = (position_>>24) & 0xFF;
      position_b1_ = (position_>>16) & 0xFF;
      position_b2_ = (position_>>8) & 0xFF;
      position_b3_ = position_ & 0xFF;

      analogWrite(9, abs(pid_out_));

      if (pid_out_ > 0)
      {
        digitalWrite(7, LOW);
        digitalWrite(8, HIGH);
        state_ = 0;
      }
      else
      {
        digitalWrite(7, HIGH);
        digitalWrite(8, LOW);
        state_ = 1;
      }
      position_ = encoder_.read();
      pid_in_ = position_;

      position_b0_ = (position_>>24) & 0xFF;
      position_b1_ = (position_>>16) & 0xFF;
      position_b2_ = (position_>>8) & 0xFF;
      position_b3_ = position_ & 0xFF;

      
    }

    inline __attribute__((always_inline))
    void setTX()
    {
      digitalWrite(dir_pin_,HIGH);
    }

    inline __attribute__((always_inline))
    void setRX()
    {
      digitalWrite(dir_pin_,LOW);
    }

    int32_t getPos()
    {
      return position_;
    }

    void setRef(double ref)
    {
      pid_ref_ = ref;
    }

  private:
    const uint8_t dir_pin_; // Toggle communication direction pin

    uint8_t eeprom_null_;

    // Fields
    uint8_t state_;
    uint8_t vel_command_;
    uint8_t dir_command_;
    uint8_t pos_command_;

    // Memory map
    mmap_entry_t mmap_field_[14];

    Encoder encoder_;
    int32_t position_;
    uint8_t position_b0_;
    uint8_t position_b1_;
    uint8_t position_b2_;
    uint8_t position_b3_;

    double pid_in_, pid_out_, pid_ref_;
    PID pid_;
};

void position_cb( const std_msgs::Float64& command);

MotorDXL motor(5, 4);
//SerialDXL<32> serial;
ros::NodeHandle nh;
std_msgs::Int32 pos_msg;
ros::Publisher pos_pub("torso_pos", &pos_msg);
ros::Subscriber<std_msgs::Float64> sub("torso_cmd", &position_cb );


void position_cb( const std_msgs::Float64& command){
  motor.setRef(command.data);
}

void setup() {
  // Init serial communication using Dynamixel format
  //serial.init(207, &motor);
  motor.setRX();
  nh.initNode();
  nh.advertise(pos_pub);
  nh.subscribe(sub);

  if (digitalRead(5)==HIGH)
  {
    DEBUG_PRINTLN("RESET!");
    motor.reset();
    toggleLed(3);
  }
  motor.initRAM();
}

void loop() {
  nh.spinOnce();
  motor.update();

  pos_msg.data = motor.getPos();
  pos_pub.publish(&pos_msg);
  
  delay(20);
}