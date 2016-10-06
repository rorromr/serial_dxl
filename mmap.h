/**
 * @file
 * @brief Memory mapping
 */
#ifndef MMAP_H
#define MMAP_H
//------------------------------------------------------------------------------
#include <avr/eeprom.h>
#include "data_serialization.h"
#include "variable.h"
#include "logger.h"
#include "Arduino.h"
//------------------------------------------------------------------------------
/** Memory map max size */
static const uint8_t MMAP_MAX_SIZE = 64U;
//------------------------------------------------------------------------------
/** Cause error message for bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badMMapLength(void)
  __attribute__((error("MMAP length too large")));
//------------------------------------------------------------------------------
namespace MMap
{
class MMapVar
{
public:
  MMapVar():
    var(NULL),
    ramAddr(0U),
    eepromAddr(0U),
    saveFlag(false) {}

  inline void set(VariableBasePtr variable, const uint8_t ramAddress = 0U, const uint8_t eepromAddress = 0U)
  {
    var = variable;
    ramAddr = ramAddress;
    eepromAddr = eepromAddress;
  }

public:
  VariableBasePtr var;
  uint8_t ramAddr;
  uint8_t eepromAddr;
  bool saveFlag;
};
/** MMapVar pointer typedef */
typedef MMapVar* MMapVarPtr;

//------------------------------------------------------------------------------
/**
 * @class MMap
 * @brief Memory mapping.
 */
template<size_t mmap_size>
class MMap
{
  public:
    MMap(uint8_t eepromOffset = 0U):
      msgBuffer_(NULL),
      bufN_(0U),
      varN_(mmap_size),
      varCount_(0U),
      eepromOffset_(0U),
      ramOffset_(0U)
    {
      // Check size
      if (varN_ > MMAP_MAX_SIZE) badMMapLength();
    }

    inline void registerVariable(VariableBasePtr var)
    {
      varList_[varCount_++].set(var, ramOffset_, var->storage_ == Storage::EEPROM ? eepromOffset_ : 0U);
      ramOffset_ += var->size();
      eepromOffset_ += var->storage_ == Storage::EEPROM ? var->size() : 0U;
    }

    void init()
    {
      bufN_ = ramOffset_;
      msgBuffer_ = new uint8_t[bufN_];
    }
    
    uint8_t serialize()
    {
      uint8_t *buffer = msgBuffer_;
      for (uint8_t i = 0; i < varN_; ++i)
      {
        buffer += varList_[i].var->serialize(buffer);
      }
      return buffer - msgBuffer_;
    }

    uint8_t deserialize()
    {
      uint8_t *buffer = msgBuffer_;
      for (uint8_t i = 0; i < varN_; ++i)
      {
        MMapVar& mvar = varList_[i];
        buffer += mvar.var->deserialize(buffer);
        // Save data on EEPROM
        if (mvar.saveFlag)
        {
          mvar.var->serialize(eepromBuffer_);
          for (uint8_t j = 0; j < mvar.var->size(); ++j)
          {
            eeprom_write_byte ( (uint8_t*)(uint16_t) j + mvar.eepromAddr, eepromBuffer_[j]);
            DEBUG_PRINT_RAW("Writting "); DEBUG_PRINT_RAW(eepromBuffer_[j]); DEBUG_PRINT_RAW(" at "); DEBUG_PRINTLN_RAW(j + mvar.eepromAddr);
          }
          mvar.saveFlag = false;
        }
      }
      return buffer - msgBuffer_;
    }

    
    void load()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        MMapVar& mvar = varList_[i];
        if (mvar.var->storage_ == Storage::RAM)
        {
          mvar.var->setDefault();
        }
        else
        {
          for (uint8_t j = 0; j < mvar.var->size(); ++j)
          {
            eepromBuffer_[j] = eeprom_read_byte( (uint8_t*)(uint16_t) j + mvar.eepromAddr);
            INFO_PRINT_RAW("Reading "); INFO_PRINT_RAW(eepromBuffer_[j]); INFO_PRINT_RAW(" at "); INFO_PRINTLN_RAW(j + mvar.eepromAddr);
          }
          mvar.var->deserialize(eepromBuffer_, true); // Call with override access
        } 
      }
    }

    void save()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        MMapVar& mvar = varList_[i];

        if (mvar.var->storage_ == Storage::EEPROM)
        {

          mvar.var->serialize(eepromBuffer_);
          for (uint8_t j = 0; j < mvar.var->size(); ++j)
          {
            eeprom_write_byte ( (uint8_t*)(uint16_t) j + mvar.eepromAddr, eepromBuffer_[j]);
            DEBUG_PRINT_RAW("Writting "); DEBUG_PRINT_RAW(eepromBuffer_[j]); DEBUG_PRINT_RAW(" at "); DEBUG_PRINTLN_RAW(j + mvar.eepromAddr);
          }
        }
      }
    }
    
    
    void setDefault()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        varList_[i].var->setDefault();
      }
    }

    void reset()
    {
      setDefault();
      save();
    }

    inline uint8_t get(uint8_t index)
    {
      return msgBuffer_[index];
    }

    inline uint8_t set(uint8_t index, uint8_t value)
    {
      
      for (uint8_t i = 0; i <= varCount_; ++i)
      {
        if (varList_[i].var->storage_ == Storage::EEPROM && varList_[i].ramAddr == index)
        {
          varList_[i].saveFlag = true;
        }
      }

      return msgBuffer_[index] = value;
    }

    void printBuffer()
    {
      for(uint8_t i = 0; i < bufN_; ++i)
      {
        Serial.print(msgBuffer_[i], HEX); Serial.print('|');
      }
      Serial.println();
    }


  public:
    // Message buffer
    uint8_t *msgBuffer_;
    uint8_t eepromBuffer_[4];
    // Message buffer length
    uint8_t bufN_;
    // Variable list
    MMapVar varList_[mmap_size];
    // Variable length
    uint8_t varN_;
    uint8_t varCount_;
    // EEPROM offset
    uint8_t eepromOffset_;
    // RAM offset
    uint8_t ramOffset_;

}; // End MMap class
//------------------------------------------------------------------------------
} // End MMap namespace

#endif
