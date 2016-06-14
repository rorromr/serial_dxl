/**
 * @file
 * @brief Int32
 */
#ifndef INT32_H
#define INT32_H

#define __STDC_LIMIT_MACROS
#include "base.h"
#include <stdint.h>
namespace MMap
{

  /**
   * @class Int32
   * @brief MMap variable for int32_t.
   */
  class Int32 : public Variable
  {
    public:
      int32_t data;

    /**
    * @brief Int32 contructor
    * 
    * @param access Variable access
    * @param storage Storage type
    * @param min Min value
    * @param max Max value
    * @param def Default value
    */
    Int32(Access access = Access::RW, Storage storage = Storage::RAM,
        uint32_t min = INT32_MIN, uint32_t max = INT32_MAX, uint32_t def = 0UL):
      Variable(access, storage),
      min_(min), max_(max), def_(def), data(def) {}

    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      // Check for read only
      if (access_ == Access::R)
        return sizeof(this->data);

      uint8_t offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data = MMAP_SAT(u_data.real, this->min_, this->max_);
      offset += sizeof(this->data);
      return offset;
    }

    virtual void setDefault()
    {
      this->data = this->def_;
    }

    virtual uint8_t size()
    {
      return sizeof(data);
    }

    private:
      const uint32_t min_; ///< Min value of data
      const uint32_t max_; ///< Max value of data
      const uint32_t def_; ///< Default value of data

  };

}
#endif