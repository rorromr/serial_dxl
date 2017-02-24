#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_NONE
// #define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_DEBUG
// https://github.com/rorromr/serial_dxl/archive/v0.1.tar.gz
#include <SerialDXL.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// SERVO DXL basic config
#define SERVO_MODEL 100
#define SERVO_FIRMWARE 100

/**
 * DEVICEDXL_MIN_BUFFER = 6
 * Is the overhead for dynamixel like devices
 * Has MODEL_L, MODEL_H, FIRMWARE, ID, BAUDRATE and RETURN DELAY
 * This variables are inside DeviceDXL class
 */ 

// We add 13 UInt8 (13 bytes)
#define SERVO_MMAP_SIZE 11

// MMap position for commands (mem Addrs)

#define SERVO0_POS      6
#define SERVO1_POS      7 
#define SERVO2_POS      8 
#define SERVO3_POS      9 
#define SERVO4_POS      10
#define SERVO5_POS      11
#define SERVO_CMD       12
#define LED_SELECT      13
#define LED_COLOR_      14
#define LED_BRIGHTNESS  15
#define LED_CMD         16

//Servos commands (not addr)
#define SERVO1_SWAP 20
#define SERVOS_INACTIVE 21

//LEDs commands (not addr)
#define SHOW_R1 1
#define SHOW_R2 2
#define UPDATE_C 3
#define CHANGE_BRIGHT 4
#define LEDS_INACTIVE 21

// Number of servos in the array
#define num_servos 6

// Pin numbers on Arduino Mega to control the servo devices
#define pin_dir 3
#define pin_reset 2
#define pin_servo0 6    //left_ear
#define pin_servo1 7    //mouth
#define pin_servo2 8    //left_eyebrow
#define pin_servo3 9    //right_ear
#define pin_servo4 10   //right_eyebrow
#define pin_servo5 11   //neck

// Pin numbers on Arduino Mega to control the LED devices
#define data_led_pin1 5
#define data_led_pin2 4
#define pixelNumber 16

/**
 * @brief SERVO and NeoPixel LEDs control using DXL communication protocol
 * @details SERVO and NeoPixel LEDs control using Dynamixel communication protocol over RS485.
 * This implementation uses 6 uint8_t variables to control state of 6 SERVOs and 4 variables for NeoPixel LEDs
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 * @param servo_pin SERVO pin.
 */
class HeadDXL: public DeviceDXL<SERVO_MODEL, SERVO_FIRMWARE, SERVO_MMAP_SIZE>
{
  public:
    //HeadDXL(uint8_t dir_pin, uint8_t reset_pin, uint8_t numServos, uint8_t servos_pins[], Servo *servos[]):
    HeadDXL(uint8_t dir_pin, uint8_t reset_pin, uint8_t numServos, uint8_t servos_pins[]):
    DeviceDXL(), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
    numServos_(numServos), // numero de servos
    servo0_pos_(MMap::Access::RW, MMap::Storage::RAM),
    servo1_pos_(MMap::Access::RW, MMap::Storage::RAM),
    servo2_pos_(MMap::Access::RW, MMap::Storage::RAM),
    servo3_pos_(MMap::Access::RW, MMap::Storage::RAM),
    servo4_pos_(MMap::Access::RW, MMap::Storage::RAM),
    servo5_pos_(MMap::Access::RW, MMap::Storage::RAM),
    servo_cmd_(MMap::Access::RW, MMap::Storage::RAM),
    led_select_(MMap::Access::RW, MMap::Storage::RAM),  // select LED command
    led_color_(MMap::Access::RW, MMap::Storage::RAM), // 8 bit encode color, RRRGGGBB
    led_brightness_(MMap::Access::RW, MMap::Storage::RAM),
    led_cmd_(MMap::Access::RW, MMap::Storage::RAM)
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, OUTPUT);
      for(uint8_t i=0;i<numServos_;i++)
      {
        pinMode(servos_pins[i], OUTPUT);
      }
    }

    void init(uint8_t servos_pins[], Servo servos[], Adafruit_NeoPixel LEDs[])
    {
        DEBUG_PRINTLN("INIT");
        /*
        * Register variables
        */
        DeviceDXL::init();

        mmap_.registerVariable(&servo0_pos_);
        mmap_.registerVariable(&servo1_pos_);
        mmap_.registerVariable(&servo2_pos_);
        mmap_.registerVariable(&servo3_pos_);
        mmap_.registerVariable(&servo4_pos_);
        mmap_.registerVariable(&servo5_pos_);
        mmap_.registerVariable(&servo_cmd_);
        mmap_.registerVariable(&led_select_);
        mmap_.registerVariable(&led_color_);
        mmap_.registerVariable(&led_brightness_);
        mmap_.registerVariable(&led_cmd_);
        mmap_.init();

        /*
        * Load default values
        */
        DEBUG_PRINTLN("Load default");
        mmap_.load(); // Load values from EEPROM

        /*
        * Read sensor data
        * e.g. Use ADC, check buttons, etc.
        */

        /*
        * Attach the Servo's variables to the servos_pins
        */
        for(uint8_t i=0;i<numServos_;i++)
        {
        servos[i].attach(servos_pins[i]);
        }
        // Settings for LED arrays
        // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
        #if defined (__AVR_ATtiny85__)
        if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
        #endif
        // End of trinket special code
        for(uint8_t i=0;i<2;i++)
        {
            LEDs[i].begin();
            LEDs[i].setBrightness(15);
            LEDs[i].show(); // Initialize all pixels to 'off'
        }
    }

    void moveServoTo(Servo *servo, uint8_t pos)
    {
        servo->write(pos);
    }

    void setPixelsTo(Adafruit_NeoPixel *LEDs_ring1, Adafruit_NeoPixel *LEDs_ring2, uint8_t R_colors[], uint8_t G_colors[], uint8_t B_colors[], uint8_t size)
    {
        for(uint8_t i=0; i<size; i++)
        {
            LEDs_ring1->setPixelColor(i, R_colors[i], G_colors[i], B_colors[i]);
            LEDs_ring2->setPixelColor(i, R_colors[i+20], G_colors[i+20], B_colors[i+20]);
        }
        LEDs_ring1->show();
        delay(5);
        LEDs_ring2->show();
        delay(5);
        // DEBUG_PRINTLN("LEDs_rings->show()");
    }

    void showRing(uint8_t select, Adafruit_NeoPixel *ring, uint8_t R_colors[], uint8_t G_colors[], uint8_t B_colors[], uint8_t size)
    {
        if (select == 1){
            for(uint8_t i=0; i<size; i++)
            {
                ring->setPixelColor(i, R_colors[i], G_colors[i], B_colors[i]);
            }
        }
        else{
            for(uint8_t i=0; i<size; i++)
            {
                ring->setPixelColor(i, R_colors[i+20], G_colors[i+20], B_colors[i+20]);
            }
        }

        ring->show();
        //delay(5);
    }

    uint8_t decodeRColor(uint8_t rgb8bit_encode){ return 36U*((rgb8bit_encode & 0b11100000)>>5);}
    uint8_t decodeGColor(uint8_t rgb8bit_encode){ return 36U*((rgb8bit_encode & 0b00011100)>>2);}
    uint8_t decodeBColor(uint8_t rgb8bit_encode){ return 85U*((rgb8bit_encode & 0b00000011)>>0);}

    void swapServo(Servo *servo, uint8_t angle_min, uint8_t angle_max)
    {
        for(uint8_t i=angle_min;i<angle_max;i++)
        {
            servo->write(i);                  // sets the servo position according to the scaled value
            delay(15);                           // waits for the servo to get there
        }
        for(uint8_t i=angle_max;i>angle_min;i--)
        {
            servo->write(i);                  // sets the servo position according to the scaled value
            delay(15);                           // waits for the servo to get there
        }
    }

    void updateServos(Servo servos[])
    {
        if (servo_cmd_.data == SERVO0_POS){ //when required update servo 1
              servos[0].write(servo0_pos_.data);
              servo_cmd_.data = SERVOS_INACTIVE;
        }
        else if (servo_cmd_.data == SERVO1_POS){ //when required update servo 2
              servos[1].write(servo1_pos_.data);
              servo_cmd_.data = SERVOS_INACTIVE;
        }
        else if (servo_cmd_.data == SERVO2_POS){ //when required update servo 3
              servos[2].write(servo2_pos_.data);
              servo_cmd_.data = SERVOS_INACTIVE;
        }
        else if (servo_cmd_.data == SERVO3_POS){ //when required update servo 4
              servos[3].write(servo3_pos_.data);
              servo_cmd_.data = SERVOS_INACTIVE;
        }
        else if (servo_cmd_.data == SERVO4_POS){ //when required update servo 5
              servos[4].write(servo4_pos_.data);
              servo_cmd_.data = SERVOS_INACTIVE;
        }
        else if (servo_cmd_.data == SERVO5_POS){ //when required update servo 6
              servos[5].write(servo5_pos_.data);
              servo_cmd_.data = SERVOS_INACTIVE;
        }
        /*else if (servo_cmd_.data == SERVO1_SWAP)   //predefined behavior function
        {
            swapServo(&servos[1], 115, 135);
        }*/
        /*else    //Inactive
        {
            DEBUG_PRINT("No command for servos");
        }*/
    }

    void updateLEDs(Adafruit_NeoPixel LEDs[])
    {
        if (led_cmd_.data == SHOW_R1){  //Lower bytes array of colors updated, show left eye colors
            showRing(1, &LEDs[0], R_colors_, G_colors_, B_colors_, 20);
            led_cmd_.data = LEDS_INACTIVE;
            DEBUG_PRINTLN("EXEC: show command SHOW_R1");
        }
        else if (led_cmd_.data == SHOW_R2){  //Upper bytes of colors updated, show right eye colors
            showRing(2, &LEDs[1], R_colors_, G_colors_, B_colors_, 20);
            led_cmd_.data = LEDS_INACTIVE;
            DEBUG_PRINTLN("EXEC: show command SHOW_R2");
        }
        else if (led_cmd_.data == CHANGE_BRIGHT){  //Change brighness of both Rings
            LEDs[0].setBrightness(led_brightness_.data);
            LEDs[1].setBrightness(led_brightness_.data);
            showRing(1, &LEDs[0], R_colors_, G_colors_, B_colors_, 20);
            showRing(2, &LEDs[1], R_colors_, G_colors_, B_colors_, 20);
            led_cmd_.data = LEDS_INACTIVE;
            DEBUG_PRINTLN("EXEC: command CHANGE_BRIGHT");
        }
        else if (led_cmd_.data == UPDATE_C){ //updating array of colors
            R_colors_[led_select_.data] = decodeRColor(led_color_.data);
            G_colors_[led_select_.data] = decodeGColor(led_color_.data);
            B_colors_[led_select_.data] = decodeBColor(led_color_.data);
            DEBUG_PRINTLN("EXEC: change color command UPDATE_C");
        }
        /*else    //Inactive, Leds commands are not used in this case
            //DEBUG_PRINTLN("Led command non used");
        */
    }

    void update()
    {
      ;
    }

    void update(Servo servos[], Adafruit_NeoPixel LEDs[])
    {
        // DEBUG_PRINTLN("UPDATE");

        updateServos(servos);
        updateLEDs(LEDs);
    }

    void SetDefaulState(Servo servos[], Adafruit_NeoPixel LEDs[])
    {
        //Default colors
        uint8_t R_colors_default[40] = {
          153, 153, 153, 0, 0, 0, 0, 153, //1-8
          153, 153, 0, 0, 0, 0, 0, 0,     //9-16
          0, 0, 0, 0,               //17-20
     
          0, 0, 0, 0, 0, 0, 153, 153, //21-28
          153, 0, 0, 0, 0, 153, 153, 153,     //29-36
          0, 0, 0, 0};              //37-40
        uint8_t G_colors_default[40] = {
          153, 153, 153, 0, 0, 0, 0, 153,         //1-8
          153, 153, 0, 0, 0, 0, 0, 0, //9-16
          0, 0, 0, 0,                 //17-20
     
          0, 0, 0, 0, 0, 0, 153, 153,         //21-28
          153, 0, 0, 0, 0, 153, 153, 153, //29-36
          0, 0, 0, 0};                //37-40
        uint8_t B_colors_default[40] = {
          0, 0, 0, 0, 0, 0, 0, 0, //1-8
          0, 0, 0, 0, 0, 0, 0, 0,   //9-16
          0, 0, 0, 0,                 //17-20
     
          0, 0, 0, 0, 0, 0, 0, 0, //21-28
          0, 0, 0, 0, 0, 0, 0, 0,   //29-36
          0, 0, 0, 0};                //37-40
        
        moveServoTo(&servos[0], 50);
        moveServoTo(&servos[1], 120);
        moveServoTo(&servos[2], 100);
        moveServoTo(&servos[3], 50);
        moveServoTo(&servos[4], 100);
        moveServoTo(&servos[5], 100);
        setPixelsTo(&LEDs[0], &LEDs[1], R_colors_default, G_colors_default, B_colors_default, 20); //show colors in LEDs
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
    const uint8_t numServos_; // Numero de servos
    //const uint8_t servos_pins_[5]; // SERVOs pins
    //Servo *servos_[5]; //puntero a objeto Servo
    
        // SERVOs variables
    MMap::Integer<UInt8, 0U, 170U, 0U>::type servo0_pos_;
    MMap::Integer<UInt8, 0U, 170U, 0U>::type servo1_pos_;
    MMap::Integer<UInt8, 0U, 170U, 0U>::type servo2_pos_;
    MMap::Integer<UInt8, 0U, 170U, 0U>::type servo3_pos_;
    MMap::Integer<UInt8, 0U, 170U, 0U>::type servo4_pos_;
    MMap::Integer<UInt8, 0U, 170U, 0U>::type servo5_pos_;
    MMap::Integer<UInt8, 0U, 20U, 21U>::type servo_cmd_;

    // LEDs variables
    MMap::Integer<UInt8, 0U, 255U, 0U>::type led_select_;
    MMap::Integer<UInt8, 0U, 255U, 0U>::type led_color_;
    MMap::Integer<UInt8, 1U, 255U, 15U>::type led_brightness_;
    MMap::Integer<UInt8, 0U, 255U, 0U>::type led_cmd_;
    
    //LEDs colors
    uint8_t R_colors_[40] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t G_colors_[40] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t B_colors_[40] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
};

//Objetos para posicionar los servos (PWM)
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

Servo servos[]={servo0, servo1, servo2, servo3, servo4, servo5};
uint8_t servos_pins[] = {pin_servo0, pin_servo1, pin_servo2, pin_servo3, pin_servo4, pin_servo5};

//Objetos para manejar los LEDs
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(pixelNumber+4, data_led_pin1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(pixelNumber+4, data_led_pin2, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel LEDs[] = {strip1, strip2};

//General device
HeadDXL head_dxl(pin_dir, pin_reset, num_servos, servos_pins);
SerialDXL<HeadDXL> serialDxl;

void setup() {
    //Serial port for debug
    Serial.begin(115200);

    head_dxl.init(servos_pins, servos, LEDs);
    head_dxl.SetDefaulState(servos, LEDs);
    head_dxl.reset();
    head_dxl.mmap_.serialize();

    // Init serial communication using Dynamixel format
    serialDxl.init(&Serial3 , &head_dxl);
}

void loop() {
    // Update msg buffer
    while (Serial3.available()){
        serialDxl.process(Serial3.read());
    }

    head_dxl.mmap_.deserialize();
    head_dxl.update(servos, LEDs);
    head_dxl.mmap_.serialize();
}
