#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_NONE
#include <SerialDXL.h>

// LED DXL basic config
#define LED_MODEL 100
#define LED_FIRMWARE 100
#define LED_MMAP_SIZE 1 // Use 1 variable

/**
 * @brief LED control using DXL communication protocol
 * @details LED control using Dynamixel communication protocol over RS485.
 * This implementation uses a 1 uint8_t variable for LED state in address 6 
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 * @param led_pin LED pin.
 */
class LedDXL: public DeviceDXL<LED_MODEL, LED_FIRMWARE, LED_MMAP_SIZE>
{
  public:
    LedDXL(uint8_t dataControlPin, uint8_t reset_pin, uint8_t led_pin):
    DeviceDXL(), // Call parent constructor
    reset_pin_(reset_pin),    // Reset pin
    led_pin_(led_pin),        // LED pin
    command_(MMap::Access::RW, MMap::Storage::RAM) // Led command
    {
      // Config pins
      pinMode(dataControlPin, OUTPUT);
      pinMode(reset_pin_, INPUT);
      pinMode(led_pin_, OUTPUT);

      // Get mask and port for data control pin
      dataControlPinMask_ = digitalPinToBitMask(dataControlPin);
      dataControlPinReg_ = portOutputRegister(digitalPinToPort(dataControlPin));
    }

    void init()
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init();
      mmap_.registerVariable(&command_);
      mmap_.init();
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
      INFO_PRINT("id: ");INFO_PRINTLN_RAW(id_.data);
      
      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */
    }

    void update()
    {
      if (command_.data == 1) digitalWrite(led_pin_, HIGH);
      else digitalWrite(led_pin_, LOW);
    }

    inline bool onReset()
    {
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      *dataControlPinReg_ |= dataControlPinMask_;
    }

    inline void setRX()
    {
      *dataControlPinReg_ &= ~dataControlPinMask_;
    }

  private:
    // Communication direction pin
    uint8_t dataControlPinMask_;
    volatile uint8_t *dataControlPinReg_;

    const uint8_t reset_pin_; // Reset pin
    const uint8_t led_pin_; // LED pin
    float float_raw;
    
    // LED variable
    MMap::Integer<UInt8, 0, 1, 1>::type command_;
};


LedDXL led(4, 8, 13);
SerialDXL<LedDXL> serialDxl;

void setup() {
  Serial.begin(115200);
  DEBUG_PRINTLN("INIT SETUP");
  
  led.init();
  led.reset();
  led.mmap_.serialize();

  // Init serial communication using Dynamixel format
  serialDxl.init(&Serial1 ,&led);
}

void loop() {
  // Update msg buffer
  while (Serial1.available()){
    serialDxl.process(Serial1.read());
  }
  
  led.mmap_.deserialize();
  led.update();
  led.mmap_.serialize();
  
}
