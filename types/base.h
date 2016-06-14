/**
 * @file
 * @brief UIint8
 */
#ifndef BASE_H
#define BASE_H
 
// Set MMAP entry macro
#define MMAP_ENTRY(mmap, var, parameter) {(mmap).value = &(var); (mmap).param = (parameter);}
// Saturation function
#define MMAP_SAT(x, min, max) ( ((x) > max) ? max : ( ((x) < min) ? min : (x) ) )

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
}

#endif