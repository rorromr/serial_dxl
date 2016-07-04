/**
 * @file MMAP Variables
 */
#ifndef BASE_H
#define BASE_H

#include <stdint.h>

namespace MMap
{

/**
 * @brief Saturation function
 * 
 * @param a Variable
 * @tparam min Min value
 * @tparam max Max value
 */
template<typename T, T min, T max>
inline void saturation(T& a)
{
  a = a > max ? max : ( a < min ? min : a );
}
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
 * @class ContainerBase
 * @brief Virtual base class for MMap variables.
 */
class VariableBase
{
  public:
    VariableBase(Access access, Storage storage):
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

typedef VariableBase* VariableBasePtr;

template <typename MessageT, typename T, T min, T max, T def>
class Variable : public VariableBase
{
  public:
    Variable(Access access, Storage storage):
      VariableBase(access, storage),
      C(),
      data(C.data)
    {}

    uint8_t serialize(uint8_t *outbuffer) const
    {
      return C.serialize(outbuffer);
    }
    
    uint8_t deserialize(uint8_t *inbuffer)
    {
      // Check for read only
      if (access_ == Access::R)
        return sizeof(C.data);
      uint8_t size = C.deserialize(inbuffer);
      saturation<T, min, max>(C.data);
      return size;
    }

    void setDefault()
    {
      C.data = def;
    }
    
    inline uint8_t size()
    {
      return sizeof(C.data);
    }

  public:
    MessageT C;
    T& data;
};

}


#endif