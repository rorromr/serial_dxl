/**
 * @file
 * @brief UIint8
 */
#ifndef UINT8_H
#define UINT8_H

#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include "base.h"

namespace MMap
{

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
    * @param storage Storage type
    * @param min Min value
    * @param max Max value
    * @param def Default value
    */
    UInt8(Access access = Access::RW, Storage storage = Storage::RAM,
        uint8_t min = 0U, uint8_t max = UINT8_MAX, uint8_t def = 0U):
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


    virtual void setDefault()
    {
      this->data = this->def_;
    }

    virtual uint8_t size()
    {
      return sizeof(data);
    }

  private:
    const uint8_t min_; ///< Min value of data
    const uint8_t max_; ///< Max value of data
    const uint8_t def_; ///< Default value of data
  };
}

#endif
