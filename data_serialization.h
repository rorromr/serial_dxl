/**
 * @brief Data serialization
 * @author Rodrigo Munoz
 * @date 2016
 *
 * Data Serialization
 * ------------------
 *
 * Functions for data serialization.
 *
 */
#ifndef DATA_SERIALIZATION_H
#define DATA_SERIALIZATION_H

#define __STDC_LIMIT_MACROS
#include <stdint.h>

/**
 * @brief Base virtual class for messages
 */
class Message
{
public:
  virtual uint8_t serialize(uint8_t *outbuffer) const = 0;
  virtual uint8_t deserialize(uint8_t *inbuffer) = 0;
};

//-----------------------------------------------------------------
/**
 * UInt8
 */
class UInt8: public Message
{
  public:
    uint8_t data;
    
    UInt8(): data(0U) {}

    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0U;
      *(outbuffer + offset + 0U) = (this->data >> (8U * 0U)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      uint8_t offset = 0U;
      this->data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data);
      return offset;
    }

    typedef uint8_t type;
    static const uint8_t max = UINT8_MAX;
    static const uint8_t min = 0U;
};

//---------------------------------------------------------------------------
/**
 * Int8
 */
class Int8: public Message
{
  public:
    int8_t data;
    
    Int8(): data(0U) {}

    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0U;
      union {
        int8_t real;
        uint8_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      uint8_t offset = 0U;
      union {
        int8_t real;
        uint8_t base;
      } u_data;
      u_data.base = 0U;
      u_data.base |= ((uint8_t) (*(inbuffer + offset + 0U))) << (8U * 0U);
      this->data = u_data.real;
      offset += sizeof(this->data);
      return offset;
    }

    typedef int8_t type;

};

//---------------------------------------------------------------------------
/**
 * UInt16
 */
class UInt16 : public Message
{
  public:
    uint16_t data;

  UInt16(): data(0U) {}

  virtual uint8_t serialize(uint8_t *outbuffer) const
  {
    uint8_t offset = 0U;
    *(outbuffer + offset + 0U) = (this->data >> (8U * 0U)) & 0xFF;
    *(outbuffer + offset + 1U) = (this->data >> (8U * 1U)) & 0xFF;
    offset += sizeof(this->data);
    return offset;
  }

  virtual uint8_t deserialize(uint8_t *inbuffer)
  {
    uint8_t offset = 0;
    this->data =  ((uint16_t) (*(inbuffer + offset)));
    this->data |= ((uint16_t) (*(inbuffer + offset + 1U))) << (8U * 1U);
    offset += sizeof(this->data);
   return offset;
  }

  typedef uint16_t type;

};

//---------------------------------------------------------------------------
/**
 * Int16
 */
class Int16 : public Message
{
  public:
    int16_t data;

  Int16(): data(0) {}
  
  virtual uint8_t serialize(uint8_t *outbuffer) const
  {
    uint8_t offset = 0U;
    union {
      int16_t real;
      uint16_t base;
    } u_data;
    u_data.real = this->data;
    *(outbuffer + offset + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
    *(outbuffer + offset + 1U) = (u_data.base >> (8U * 1U)) & 0xFF;
    offset += sizeof(this->data);
    return offset;
  }

  virtual uint8_t deserialize(uint8_t *inbuffer)
  {
    uint8_t offset = 0;
    union {
      int16_t real;
      uint16_t base;
    } u_data;
    u_data.base = 0;
    u_data.base |= ((uint16_t) (*(inbuffer + offset + 0U))) << (8U * 0U);
    u_data.base |= ((uint16_t) (*(inbuffer + offset + 1U))) << (8U * 1U);
    this->data = u_data.real;
    offset += sizeof(this->data);
   return offset;
  }

  typedef int16_t type;

};

//---------------------------------------------------------------------------
/**
 * UInt32
 */
class UInt32: public Message
{
  public:
    uint32_t data;
  
    UInt32(): data(0UL) {}

    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0;
      *(outbuffer + offset + 0U) = (this->data >> (8U * 0U)) & 0xFF;
      *(outbuffer + offset + 1U) = (this->data >> (8U * 1U)) & 0xFF;
      *(outbuffer + offset + 2U) = (this->data >> (8U * 2U)) & 0xFF;
      *(outbuffer + offset + 3U) = (this->data >> (8U * 3U)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      uint8_t offset = 0U;
      this->data =  ((uint32_t) (*(inbuffer + offset + 0U))) << (8U * 0U);
      this->data |= ((uint32_t) (*(inbuffer + offset + 1U))) << (8U * 1U);
      this->data |= ((uint32_t) (*(inbuffer + offset + 2U))) << (8U * 2U);
      this->data |= ((uint32_t) (*(inbuffer + offset + 3U))) << (8U * 3U);
      offset += sizeof(this->data);
      return offset;
    }

    typedef uint32_t type;
};

//---------------------------------------------------------------------------
/**
 * Int32
 */
class Int32 : public Message
{
  public:
    int32_t data;

  Int32():
    data(0UL)
  {
  }

  virtual uint8_t serialize(uint8_t *outbuffer) const
  {
    uint8_t offset = 0U;
    union {
      int32_t real;
      uint32_t base;
    } u_data;
    u_data.real = this->data;
    *(outbuffer + offset + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
    *(outbuffer + offset + 1U) = (u_data.base >> (8U * 1U)) & 0xFF;
    *(outbuffer + offset + 2U) = (u_data.base >> (8U * 2U)) & 0xFF;
    *(outbuffer + offset + 3U) = (u_data.base >> (8U * 3U)) & 0xFF;
    offset += sizeof(this->data);
    return offset;
  }

  virtual uint8_t deserialize(uint8_t *inbuffer)
  {
    uint8_t offset = 0U;
    union {
      int32_t real;
      uint32_t base;
    } u_data;
    u_data.base = 0U;
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 0U))) << (8U * 0U);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 1U))) << (8U * 1U);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 2U))) << (8U * 2U);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 3U))) << (8U * 3U);
    this->data = u_data.real;
    offset += sizeof(this->data);
    return offset;
  }

  typedef int32_t type;

};

//---------------------------------------------------------------------------
/**
 * Float32
 */
class Float32 : public Message
{
  public:
    float data;

  Float32():
    data(0.0f)
  {
  }

  virtual uint8_t serialize(uint8_t *outbuffer) const
  {
    uint8_t offset = 0U;
    union {
      float real;
      uint32_t base;
    } u_data;
    u_data.real = this->data;
    *(outbuffer + offset + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
    *(outbuffer + offset + 1U) = (u_data.base >> (8U * 1U)) & 0xFF;
    *(outbuffer + offset + 2U) = (u_data.base >> (8U * 2U)) & 0xFF;
    *(outbuffer + offset + 3U) = (u_data.base >> (8U * 3U)) & 0xFF;
    offset += sizeof(this->data);
    return offset;
  }

  virtual uint8_t deserialize(uint8_t *inbuffer)
  {
    uint8_t offset = 0U;
    union {
      float real;
      uint32_t base;
    } u_data;
    u_data.base = 0;
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 0U))) << (8U * 0U);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 1U))) << (8U * 1U);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 2U))) << (8U * 2U);
    u_data.base |= ((uint32_t) (*(inbuffer + offset + 3U))) << (8U * 3U);
    this->data = u_data.real;
    offset += sizeof(this->data);
    return offset;
  }

  typedef float type;
};

//---------------------------------------------------------------------------
/**
 * Constants values
 */
template<int32_t Num, int32_t Den = 1UL>
struct ConstRatio{
  static const float value = 1.0f*Num/Den;
};

template<typename T, T val>
struct ConstInt{
  static const T value = val;
};


#endif
