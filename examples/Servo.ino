#include <SerialDXL.h>
#include <Servo.h>

// SERVO DXL basic config
#define SERVO_MODEL 100
#define SERVO_FIRMWARE 100

/**
 * DEVICEDXL_MIN_BUFFER = 6
 * Is the overhead for dynamixel like devices
 * Has MODEL_L, MODEL_H, FIRMWARE, ID, BAUDRATE and RETURN DELAY
 * This variables are inside DeviceDXL class
 */ 

// We add 2 UInt8 (2 bytes)
#define SERVO_MMAP_SIZE DEVICEDXL_MIN_BUFFER+2

// MMap position for command
#define SERVO_SELECT_COMMAND DEVICEDXL_MIN_BUFFER
#define SERVO_POS_COMMAND DEVICEDXL_MIN_BUFFER+1

// Pin numbers on Arduino Mega to control the servo devise
#define pin_dir 6
#define pin_reset 7
#define pin_servo 4

/**
 * @brief SERVO control using DXL communication protocol
 * @details SERVO control using Dynamixel communication protocol over RS485.
 * This implementation uses 2 uint8_t variable to control state of 5 SERVOs in address 6 and 7
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 * @param servo_pin SERVO pin.
 */
class ServoDXL: public DeviceDXL
{
  public:
    ServoDXL(uint8_t dir_pin, uint8_t reset_pin, uint8_t servo_pin, Servo *servo):
    DeviceDXL(SERVO_MODEL, SERVO_FIRMWARE, SERVO_MMAP_SIZE), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
    servo_pin_(servo_pin),    // SERVO pin
	servo_(servo),			// Puntero a objeto Servo
    servo_select_command_(SERVO_SELECT_COMMAND, MMap::Access::RW, 0U, 1U, 0U), // Servo command 1
    servo_pos_command_(SERVO_POS_COMMAND, MMap::Access::RW, 0U, 180U, 0U) // Servo command 2
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, OUTPUT);
      pinMode(servo_pin_, OUTPUT);
    }

    void init()
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init(msgBuf_, varList_);
      mmap_.registerVariable(&servo_select_command_);
      mmap_.registerVariable(&servo_pos_command_);
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data1: ");DEBUG_PRINTLN(servo_select_command_.data);
      DEBUG_PRINT("data2: ");DEBUG_PRINTLN(servo_pos_command_.data);
      
      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */

	  /*
	   * Attach the Servo variable to the servo_pin
	   */
	  servo_->attach(servo_pin_);
    }
	void moveServoTo(Servo *servo, uint8_t pos)
	{
		servo->write(pos);
	}
    void update()
    {
		//DEBUG_PRINTLN("UPDATE");
		//DEBUG_PRINT("data1: ");DEBUG_PRINTLN(servo_select_command_.data);
		//DEBUG_PRINT("data2: ");DEBUG_PRINTLN(servo_pos_command_.data);
		if (servo_select_command_.data == 0) moveServoTo(servo_, servo_pos_command_.data);
		//else if (servo_select_command_.data == 1) moveServoTo(servo_, 180);
		//if (servo_select_command_.data == 0) moveServoTo(servo_, 0);
		//else moveServoTo(servo_, 180);
    }

    inline bool onReset()
    {
      DEBUG_PRINTLN("ON RESET");
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      digitalWrite(dir_pin_,HIGH);
    }

    inline void setRX()
    {
      digitalWrite(dir_pin_,LOW);
    }

  private:
    const uint8_t dir_pin_; // Toggle communication direction pin
    const uint8_t reset_pin_; // Reset pin
    const uint8_t servo_pin_; // SERVO pin
	Servo *servo_; //puntero a objeto Servo
    
    // SERVO variable
    MMap::UInt8 servo_select_command_;
    MMap::UInt8 servo_pos_command_;
    
    // Memory map
    uint8_t msgBuf_[SERVO_MMAP_SIZE];
    MMap::VariablePtr varList_[SERVO_MMAP_SIZE];
};

Servo servo;
ServoDXL servo_dxl(pin_dir, pin_reset, pin_servo, &servo);
SerialDXL serialDxl;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  delay(50);
  
  // Init serial communication using Dynamixel format
  serialDxl.init(115200, &Serial3 , &servo_dxl);

  servo_dxl.init();
  servo_dxl.reset();
  servo_dxl.mmap_.serialize();
}
uint8_t packet =0;
void loop() {
  // Update msg buffer
  while (Serial3.available())
  {
		//packet = Serial3.read();
		//Serial.print("packet: ");
		//Serial.println(packet, DEC);
		//serialDxl.process(packet);
		serialDxl.process(Serial3.read());
  }
  servo_dxl.mmap_.deserialize();
  servo_dxl.update();
  servo_dxl.mmap_.serialize();
}
