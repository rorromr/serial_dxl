/**
 * @file
 * @brief Dynamixel Device Library
 */
#ifndef SERIALDXL_H
#define SERIALDXL_H

#include "data_serialization.h"
#include "variable.h"
#include "mmap.h"
#include "logger.h"
#include <avr/sleep.h>
#include <Arduino.h>

//------------------------------------------------------------------------------
/**
 * @class DeviceDXL
 * @brief Base class for device with Dynamixel protocol.
 */
template <uint16_t modelT, uint8_t firmwareT, size_t mmap_size>
class DeviceDXL {
  public:
    DeviceDXL():
      model_(MMap::Access::R, MMap::Storage::EEPROM),
      firmware_(MMap::Access::R, MMap::Storage::EEPROM),
      id_(MMap::Access::RW, MMap::Storage::EEPROM),
      baudrate_(MMap::Access::RW, MMap::Storage::EEPROM),
      return_delay_(MMap::Access::RW, MMap::Storage::EEPROM),
      mmap_()
    {
    }

    virtual inline bool onReset() = 0;

    virtual inline void setTX() = 0;

    virtual inline void setRX() = 0;

    virtual inline void update() = 0;

    void init()
    {
      mmap_.registerVariable(&model_);
      mmap_.registerVariable(&firmware_);
      mmap_.registerVariable(&id_);
      mmap_.registerVariable(&baudrate_);
      mmap_.registerVariable(&return_delay_);
    }

    void reset()
    {
      if (onReset())
      {
        INFO_PRINTLN("DEVICE RESET");
        mmap_.reset(); // Set default values and save EEPROM
      }
    }

    // Model
    MMap::Variable<UInt16, ConstInt<UInt16::type, 0U>, ConstInt<UInt16::type, 255U>, ConstInt<UInt16::type, modelT> > model_;

    // Firmware version
    MMap::Variable<UInt8, ConstInt<UInt8::type, 0U>, ConstInt<UInt8::type, 255U>, ConstInt<UInt8::type, firmwareT> > firmware_;

    // ID
    MMap::Variable<UInt8, ConstInt<UInt8::type, 0U>, ConstInt<UInt8::type, 255U>, ConstInt<UInt8::type, 1U> > id_;

    // Baudrate
    MMap::Variable<UInt8, ConstInt<UInt8::type, 0U>, ConstInt<UInt8::type, 255U>, ConstInt<UInt8::type, 1U> > baudrate_;

    // Return dalay time
    MMap::Variable<UInt8, ConstInt<UInt8::type, 0U>, ConstInt<UInt8::type, 255U>, ConstInt<UInt8::type, 250U> > return_delay_;

    // Memory mapping
    MMap::MMap<5+mmap_size> mmap_;
};
//------------------------------------------------------------------------------
/**
 * @class SerialDXL
 * @brief Serial port wrapper for DXL communication protocol.
 */

// Max message length, used for buffers
#define SERIALDXL_MSG_LENGTH 64

template <typename DeviceT>
class SerialDXL
{
  public:
    SerialDXL():
      msgState_(0U),
      msgParamIdx_(2U),
      msgLen_(0U),
      msgFinish_(0U),
      msgChecksum_(0U),
      error_(0U),
      last_call_(0UL)
    {
    }

    /**
     * @brief Initialize SerialDXL. Set baudrate and target device.
     * @details 
     * 
     * @param baud Baudrate using Dynamixel format.
     * @param port Serial port (Stream class).
     * @param device Target device.
     */
    void init(HardwareSerial *port, DeviceT *device)
    {
      // Set DeviceDXL
      device_ = device;
      // Set serial port
      port_ = port;
      // Set baudrate using Dynamixel relation
      uint32_t baud = F_CPU/8/(device_->baudrate_.data+1);
      INFO_PRINT("Baudrate at: "); INFO_PRINTLN(baud);
      port_->begin(baud);
    }

    /**
     * @brief Process data from Serial 
     * @details Add data to buffers
     * 
     * @param data Data from Serial port.
     */
    void process(uint8_t data)
    {
      DEBUG_PRINTLN(data);
      // Check message
      uint32_t now = millis();
      if (now - last_call_ > 100)
      {
        // Reset states
        DEBUG_PRINTLN("RESET MSG");
        msgState_ = 0;
        msgParamIdx_ = 2;
        msgLen_ = 0;
        msgChecksum_ = 0;
        msgFinish_ = 0;
      }
      last_call_ = now;

      switch(msgState_)
      {
        case 0: // 0xFF
          msgState_ = data == 0xFF ? 1 : 0;
          break;
          
        case 1: // 0XFF
          msgState_ = data == 0xFF ? 2 : 1;
          break;
          
        case 2: // ID
          // Check error
          msgState_ = data == device_->id_.data ? 3 : 0;
          break;
          
        case 3: // Length
          DEBUG_PRINTLN("MSG RECEIVED");
          msgLen_ = data;
          // Checksum
          msgChecksum_ += device_->id_.data + data;
          // Save length in the RX message buffer
          rxMsgBuf_[0] = data;
          msgState_ = 4;
          break;

        case 4: // Instruction
          // Save instruction in the RX message buffer
          rxMsgBuf_[1] = data;
          // Checksum
          msgChecksum_ += data;
          // Check for short message
          msgState_ = msgLen_ <= 2 ? 6 : 5;
          break;
        
        case 5: // Parameters
          rxMsgBuf_[msgParamIdx_++] = data;
          // Checksum
          msgChecksum_ += data;
          // Check message length
          msgState_ = msgParamIdx_ >= msgLen_ ? 6 : msgState_;
          break;

        case 6: // Checksum
          rxMsgBuf_[msgParamIdx_] = data;
          // Check
          msgFinish_ = ((uint8_t)(~msgChecksum_))==data;
          // Reset states
          msgState_ = 0;
          msgParamIdx_ = 2;
          msgChecksum_ = 0;
          msgLen_ = 0;
          break;
      }

      // Process message
      if (msgFinish_)
      {
        DEBUG_PRINTLN("MSG RECEIVED SUCCESS");
        uint8_t i, data, checksum;
        switch(rxMsgBuf_[1])
        {
          case 1: // Ping
            txMsgBuf_[0] = 0xFF;
            txMsgBuf_[1] = 0xFF;
            txMsgBuf_[2] = device_->id_.data; //ID 
            txMsgBuf_[3] = 2; // Length
            txMsgBuf_[4] = 0; // Error
            txMsgBuf_[5] = ~(txMsgBuf_[2]+txMsgBuf_[3]);
            // Status return delay
            delayMicroseconds(2*device_->return_delay_.data);
            device_->setTX();
            // Send
            port_->write(txMsgBuf_,6);
            port_->flush(); // Wait to complete
            device_->setRX();
            msgFinish_ = 0;
            break;

          case 2: // Read data
            DEBUG_PRINTLN("READ DATA");
            txMsgBuf_[0] = 0xFF;
            txMsgBuf_[1] = 0xFF;
            txMsgBuf_[2] = device_->id_.data; //ID 
            txMsgBuf_[3] = 2U+rxMsgBuf_[3]; // Length
            txMsgBuf_[4] = 0U; // Error
            // Reset checksum
            checksum = 0U;
            // Fill buffer
            for (i = 0U; i < rxMsgBuf_[3]; ++i)
            {
              data = device_->mmap_.get(rxMsgBuf_[2]+i);
              checksum += data;
              txMsgBuf_[5+i] = data;
            }
            // Checksum
            txMsgBuf_[5+i] = ~(txMsgBuf_[2]+txMsgBuf_[3]+txMsgBuf_[4]+checksum);
            // Status return delay
            delayMicroseconds(2*device_->return_delay_.data);

            device_->setTX();
            // Send
            DEBUG_PRINTLN(6+i);
            port_->write(txMsgBuf_, 6+i);
            port_->flush(); // Wait to complete

            device_->setRX();
            msgFinish_ = 0;
            break;

          case 3: // Write data
            DEBUG_PRINTLN("WRITE DATA");
            txMsgBuf_[0] = 0xFF;
            txMsgBuf_[1] = 0xFF;
            txMsgBuf_[2] = device_->id_.data; //ID 
            txMsgBuf_[3] = 2; // Length
            txMsgBuf_[4] = 0; // Error

            data = rxMsgBuf_[0] - 3U; // Number of params
            for (i = 0U; i < data; ++i)
            {
              // Set values
              device_->mmap_.set(rxMsgBuf_[2]+i,rxMsgBuf_[3+i]);
            }
            
            // Calc checksum
            txMsgBuf_[5] = ~(txMsgBuf_[2]+txMsgBuf_[3]);
            // Status return delay
            delayMicroseconds(2*device_->return_delay_.data);
            
            device_->setTX();
            // Send
            port_->write(txMsgBuf_, 6);
            port_->flush(); // Wait to complete
            
            device_->setRX();
            // Reset
            msgFinish_ = 0;
            break;

        }
      }
    }

  private:
    // DeviceDXL
    DeviceT *device_;
    // Serial port
    HardwareSerial *port_;
    // Mesage reception state
    uint8_t msgState_;
    uint8_t msgParamIdx_;
    uint8_t msgLen_;
    uint8_t msgFinish_;
    uint16_t msgChecksum_;

    // Error state
    uint8_t error_;

    // Last call tick
    uint32_t last_call_;

    // Receive message buffer
    uint8_t rxMsgBuf_[SERIALDXL_MSG_LENGTH];
    // Send message buffer
    uint8_t txMsgBuf_[SERIALDXL_MSG_LENGTH];
};

#endif