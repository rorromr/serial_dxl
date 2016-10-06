/**
 * @file MMAP Variables
 */
#ifndef BASE_H
#define BASE_H

#include "data_serialization.h"


namespace MMap
{

/**
 * @brief Saturation function
 * 
 * @param a Variable
 * @tparam min Min value
 * @tparam max Max value
 */
template<typename T, typename min, typename max>
inline void saturation(T& a)
{
  a = a > max::value ? max::value : ( a < min::value ? min::value : a );
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
    virtual uint8_t deserialize(uint8_t *inbuffer, bool overrride = false) = 0;
    virtual void setDefault();
    virtual uint8_t size() = 0;
  
  public:
    const Access access_; ///< Access type
    const Storage storage_; ///< Storage type
};

typedef VariableBase* VariableBasePtr;

template <typename MessageT, typename min, typename max, typename def>
class Variable : public VariableBase, public MessageT
{
  public:
    Variable(Access access, Storage storage):
      VariableBase(access, storage)
    {}

    typedef typename MessageT::type DataType;

    uint8_t serialize(uint8_t *outbuffer) const
    {
      return MessageT::serialize(outbuffer);
    }
    
    uint8_t deserialize(uint8_t *inbuffer, bool overrride = false)
    {
      // Check for read only
      if (access_ == Access::R && !overrride)
        return sizeof(MessageT::data);
      uint8_t size = MessageT::deserialize(inbuffer);
      saturation<DataType, min, max>(MessageT::data);
      return size;
    }

    void setDefault()
    {
      MessageT::data = def::value;
    }
    
    inline uint8_t size()
    {
      return sizeof(MessageT::data);
    }
};

template <typename T, typename T::type min, typename T::type max, typename T::type def>
struct Integer
{
    typedef Variable<T, ConstInt<typename T::type, min>, ConstInt<typename T::type, max>, ConstInt<typename T::type, def> > type;
};

template <typename min, typename max, typename def>
struct Float
{
    typedef Variable<Float32, min, max, def> type;
};


}


#endif