/**
 * @file
 * @brief Memory mapping
 */
#ifndef MMap_h
#define MMap_h
//------------------------------------------------------------------------------
#include <avr/eeprom.h>
#include <stdint.h>
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
// Set MMAP entry macro
#define MMAP_ENTRY(mmap, var, parameter) {(mmap).value = &(var); (mmap).param = (parameter);}
// Saturation function
#define MMAP_SAT(x, min, max) ( ((x) > max) ? max : ( ((x) < min) ? min : (x) ) )

//------------------------------------------------------------------------------
namespace MMap
{
/**
 * @brief Access class for read and write permission check
 * Implementation try to emulate scoped enumerations behavior
 */
struct Access
{
  enum type
  {
    RW = 0U,
    R = 1U
  };
  Access(type t) : value_(t) {}
  operator type() const { return value_; }
  private:
    type value_;
};
//------------------------------------------------------------------------------
/**
 * @brief Storage class for RAM or EEPROM check
 * Implementation try to emulate scoped enumerations behavior
 */
struct Storage
{
  enum type
  {
    RAM = 0U,
    EEPROM = 1U
  };
  Storage(type t) : value_(t) {}
  operator type() const { return value_; }
  private:
    type value_;
};
//------------------------------------------------------------------------------
/**
 * @class Variable
 * @brief Virtual base class for MMap variables.
 */
class Variable
{
  public:
    Variable(Access access = Access::RW, Storage storage = Storage::RAM):
      access_(access),
      storage_(storage) {};

    virtual uint8_t serialize(uint8_t *outbuffer) const = 0;
    virtual uint8_t deserialize(uint8_t *inbuffer) = 0;
    virtual void setDefault();
    virtual uint8_t size() = 0;
  
  public:
    const Access access_; ///< Access type
    const Storage storage_; ///< Storage type
};

/** Variable pointer typedef */
typedef Variable* VariablePtr;

/**
 * @class UInt8
 * @brief MMap variable for uint8_t.
 */
class UInt8: public Variable
{
  public:
    
    uint8_t data;

    /**
     * @brief UInt8 contructor
     * 
     * @param access Variable access
     * @param min Min value
     * @param max Max value
     * @param def Default value
     */
    UInt8(Access access = Access::RW, Storage storage = Storage::RAM, uint8_t min = 0U, uint8_t max = 255U, uint8_t def = 0U):
      Variable(access, storage),
      min_(min), max_(max), def_(def), data(def) {}
    
    /**
     * @brief Serialize data into message buffer
     * 
     * @param outbuffer Message buffer
     * @return Variable size (in bytes)
     */
    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    /**
     * @brief Deserialize data from message buffer
     * 
     * @param inbuffer Message buffer
     * @return Variable size (in bytes)
     */
    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      // Check for read only
      if (access_ == Access::R)
        return sizeof(this->data);

      uint8_t offset = 0;
      // Get data from buffer
      this->data = MMAP_SAT((uint8_t) (*(inbuffer + offset)), this->min_, this->max_);
      offset += sizeof(this->data);
      return offset;
    }


    virtual void setDefault() { this->data = this->def_; }

    virtual uint8_t size()
    {
      return sizeof(data);
    }

  private:
    const uint8_t min_; ///< Min value of data
    const uint8_t max_; ///< Max value of data
    const uint8_t def_; ///< Default value of data
};
//------------------------------------------------------------------------------
class MMapVar
{
public:
  MMapVar():
    var(NULL),
    ramAddr(0U),
    eepromAddr(0U) {}

  inline void set(VariablePtr variable, const uint8_t ramAddress = 0U, const uint8_t eepromAddress = 0U)
  {
    var = variable;
    ramAddr = ramAddress;
    eepromAddr = eepromAddress;
  }

public:
  VariablePtr var;
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

    inline void registerVariable(VariablePtr var)
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
        VariablePtr& var = varList_[i].var;
        if (var->storage_ == Storage::RAM)
        {
          var->setDefault();
        }
        else
        {
          ;
        } 
      }
    }

    void save()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        if (varList_[i].var->storage_ == Storage::EEPROM)
        {
          ;
          //varList_[i].var->save();
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
      for (uint8_t i = 0; i < varN_; ++i)
      {
        varList_[i].var->setDefault();
        //varList_[i].var->save();
      }
    }

    inline uint8_t get(uint8_t index)
    {
      return msgBuffer_[index];
    }

    inline uint8_t set(uint8_t index, uint8_t value)
    {
      return msgBuffer_[index] = value;
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
