/**
 * @file
 * @brief Memory mapping
 */
#ifndef MMap_h
#define MMap_h
//------------------------------------------------------------------------------
#include <avr/eeprom.h>
//------------------------------------------------------------------------------
/** Memory map max size */
static const uint8_t MMAP_MAX_SIZE = 64U;
/** First bit LSB */
static const uint8_t MMAP_RW = 1U;
static const uint8_t MMAP_R = 0U;
/** Second bit */
static const uint8_t MMAP_RAM = 1U << 1U;
static const uint8_t MMAP_EEPROM = 0U;


//------------------------------------------------------------------------------
/** Cause error message for bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badMMapLength(void)
  __attribute__((error("MMAP length too large")));
//------------------------------------------------------------------------------
/**
 * @class mmap_entry_t
 * @brief Entry type for memory map.
 */
typedef struct
{
  uint8_t *value;
  uint8_t param;
} mmap_entry_t;
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
 */
 // Implementation try to emulate scoped enumerations behavior
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
 * @class Variable
 * @brief Virtual base class for MMap variables.
 */
class Variable
{
  public:
    Variable(uint8_t address, Access access = Access::RW):
      address_(address),
      access_(access) {};

    virtual uint8_t serialize(uint8_t *outbuffer) const = 0;
    virtual uint8_t deserialize(uint8_t *inbuffer) = 0;
    virtual void load() = 0;
    virtual void save() const = 0;
    virtual void setDefault() {};
    static uint8_t size() {return 0;}
  
  protected:
    /**
     * Address in message buffer
     */
    const uint8_t address_;
    /**
     * Access type
     */
    const Access access_;
    
};

typedef Variable* VariablePtr;

/**
 * @class Variable
 * @brief Virtual base class for MMap non-volatile variables.
 */
class VariableNV : public Variable
{
  public:
    VariableNV(uint8_t address, uint8_t nv_address, Access access = Access::RW):
      Variable(address, access),
      nv_address_(nv_address) {};
  
  protected:
    /**
     * Non volatile (EEPROM) memory address
     */
    const uint8_t nv_address_;
};

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
     * @param address Message buffer address
     * @param access Variable access
     * @param min Min value
     * @param max Max value
     * @param def Default value
     */
    UInt8(uint8_t address, Access access = Access::RW, uint8_t min = 0U, uint8_t max = 255U, uint8_t def = 0U):
      Variable(address, access),
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


    virtual void load() { this->data = this->def_; }
    virtual void save() const {}
    virtual void setDefault() { this->data = this->def_; }

    static uint8_t size()
    {
      return sizeof(data);
    }

  private:
    /**
     * Min value of data
     */
    const uint8_t min_;
    /**
     * Max value of data
     */
    const uint8_t max_;
    /**
     * Default value of data
     */
    const uint8_t def_;
};

/**
 * @class UInt8NV
 * @brief MMap variable for non-volatile uint8_t (saved on EEPROM).
 */
class UInt8NV: public VariableNV
{
  public:
    uint8_t data;
    UInt8NV(uint8_t address, uint8_t nv_address, Access access = Access::RW, uint8_t min = 0U, 
      uint8_t max = 255U, uint8_t def = 0U):
      VariableNV(address, nv_address, access),
      min_(min), max_(max), def_(def), data(def) {}
    
    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      // Check for read only
      if (access_ == Access::R)
        return sizeof(this->data);

      uint8_t offset = 0;
      // Get data from buffer
      uint8_t updated_data = ((uint8_t) (*(inbuffer + offset)));
      // Only update EEPROM when value has changed
      if (this->data != updated_data)
      {
        eeprom_write_byte ( (uint8_t*)(uint16_t) nv_address_, updated_data);
        this->data = updated_data;
      }
      offset += sizeof(this->data);
      return offset;
    }

    virtual void load()
    {
      // Get data from EEPROM
      this->data = eeprom_read_byte( (uint8_t*)(uint16_t) nv_address_);
    }

    virtual void save() const
    {
      eeprom_write_byte ( (uint8_t*)(uint16_t) nv_address_, this->data);
    }

    virtual void setDefault(){ this->data = this->def_; }
    

    static uint8_t size()
    {
      return sizeof(data);
    }

  private:
    const uint8_t min_, max_, def_;
};


//------------------------------------------------------------------------------
/**
 * @class MMap
 * @brief Memory mapping.
 */
class MMap
{
  public:
    MMap(uint8_t bufSize):
    bufN_(bufSize),
    varN_(0U)
    {
      // Check size
      if (bufN_ > MMAP_MAX_SIZE) badMMapLength();
    }

    void init(uint8_t *msgBuffer, VariablePtr *varList)
    {
      msgBuffer_ = msgBuffer;
      varList_ = varList;
    }


    inline __attribute__((always_inline))
    void registerVariable(VariablePtr var)
    {
      varList_[varN_++]=var;
    }
    
    uint8_t serialize()
    {
      uint8_t *buffer = msgBuffer_;
      for (uint8_t i = 0; i < varN_; ++i)
      {
        buffer += varList_[i]->serialize(buffer);
      }
      return buffer - msgBuffer_;
    }

    uint8_t deserialize()
    {
      uint8_t *buffer = msgBuffer_;
      for (uint8_t i = 0; i < varN_; ++i)
      {
        buffer += varList_[i]->deserialize(buffer);
      }
      return buffer - msgBuffer_;
    }

    void load()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        varList_[i]->load();
      }
    }

    void save()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        varList_[i]->save();
      }
    }

    void setDefault()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        varList_[i]->setDefault();
      }
    }

    void reset()
    {
      for (uint8_t i = 0; i < varN_; ++i)
      {
        varList_[i]->setDefault();
        varList_[i]->save();
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


  private:
    // Message buffer
    uint8_t *msgBuffer_;
    // Message buffer length
    uint8_t bufN_;
    // Variable list
    VariablePtr *varList_;
    // Variable length
    uint8_t varN_;

}; // End MMap class
//------------------------------------------------------------------------------
} // End MMap namespace




#endif
