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
    eepromAddr(0U) {}

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
};
/** MMapVar pointer typedef */
typedef MMapVar* MMapVarPtr;

//------------------------------------------------------------------------------
/**
 * @class MMap
 * @brief Memory mapping.
 */
class MMap
{
  public:
    MMap(uint8_t size, uint8_t eepromOffset = 0U):
      msgBuffer_(NULL),
      bufN_(0U),
      varList_(NULL),
      varN_(size),
      varCount_(0U),
      eepromOffset_(0U),
      ramOffset_(0U)
    {
      // Check size
      if (varN_ > MMAP_MAX_SIZE) badMMapLength();
      varList_ = new MMapVar[varN_];
    }

    inline void registerVariable(VariableBasePtr var)
    {
      ramOffset_ += var->size();
      eepromOffset_ += var->storage_ == Storage::EEPROM ? var->size() : 0U;
      varList_[varCount_++].set(var, ramOffset_, eepromOffset_);
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
        buffer += varList_[i].var->deserialize(buffer);
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
            eepromBuffer_[j] = eeprom_read_byte( (uint8_t*)(uint16_t) mvar.eepromAddr);
          mvar.var->deserialize(eepromBuffer_);
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
          Serial.println(eepromBuffer_[0]);
          for (uint8_t j = 0; j < mvar.var->size(); ++j)
          {
            eeprom_write_byte ( (uint8_t*)(uint16_t) j + mvar.eepromAddr, eepromBuffer_[j]);
            Serial.print("Writting "); Serial.print(eepromBuffer_[j]); Serial.print(" at "); Serial.println(j);
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
    MMapVarPtr varList_;
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
