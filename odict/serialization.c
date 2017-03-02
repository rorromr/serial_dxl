/**
 * @brief Functions for data serialization.
 * @author Rodrigo Munoz
 * @date Jan, 2017
 *
 */

#include "serialization.h"

od_serialize od_serialize_table[] = {ob_serialize_uint8, ob_serialize_int8, ob_serialize_uint16,
                                     ob_serialize_int16, ob_serialize_uint32, ob_serialize_int32, ob_serialize_float};

od_deserialize od_deserialize_table[] = {ob_deserialize_uint8, ob_deserialize_int8, ob_deserialize_uint16,
                                         ob_deserialize_int16, ob_deserialize_uint32, ob_deserialize_int32, ob_deserialize_float};

uint8_t od_serialize_entry(od_entry *entry, uint8_t *outbuffer)
{
  return od_serialize_table[entry->info.type](entry->var, outbuffer);
}

uint8_t od_deserialize_entry(od_entry *entry, uint8_t *inbuffer)
{
  return od_deserialize_table[entry->info.type](entry->var, inbuffer);
}

uint8_t ob_serialize_uint8(void *var, uint8_t *outbuffer)
{
  *(outbuffer) = *(uint8_t*)(var);
  return sizeof(uint8_t);
}

uint8_t ob_deserialize_uint8(void *var, uint8_t *inbuffer)
{
  *(uint8_t*)(var) =  (uint8_t)(*inbuffer);
  return sizeof(uint8_t);
}


uint8_t ob_serialize_int8(void *var, uint8_t *outbuffer)
{
  union {
      int8_t real;
      uint8_t base;
  } u_data;
  u_data.real = *(int8_t*)(var);
  *(outbuffer) = u_data.base;
  return sizeof(int8_t);
}

uint8_t ob_deserialize_int8(void *var, uint8_t *inbuffer)
{
  union {
      int8_t real;
      uint8_t base;
  } u_data;
  u_data.base = 0U;
  u_data.base |= (uint8_t)(*inbuffer);
  *(int8_t*)(var) = u_data.real;
  return sizeof(int8_t);
}


uint8_t ob_serialize_uint16(void *var, uint8_t *outbuffer)
{
  *(outbuffer + 0U) = (*(uint16_t*)(var) >> (8U * 0U)) & 0xFF;
  *(outbuffer + 1U) = (*(uint16_t*)(var) >> (8U * 1U)) & 0xFF;
  return sizeof(uint16_t);
}

uint8_t ob_deserialize_uint16(void *var, uint8_t *inbuffer)
{
  *(uint16_t*)(var)  = ((uint16_t) (*(inbuffer)));
  *(uint16_t*)(var) |= ((uint16_t) (*(inbuffer + 1U))) << (8U * 1U);
  return sizeof(uint16_t);
}


uint8_t ob_serialize_int16(void *var, uint8_t *outbuffer)
{
  union {
      int16_t real;
      uint16_t base;
  } u_data;
  u_data.real = *(int16_t*)(var);
  *(outbuffer + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
  *(outbuffer + 1U) = (u_data.base >> (8U * 1U)) & 0xFF;
  return sizeof(int16_t);
}

uint8_t ob_deserialize_int16(void *var, uint8_t *inbuffer)
{
  union {
      int16_t real;
      uint16_t base;
  } u_data;
  u_data.base = 0;
  u_data.base |= ((uint16_t) (*(inbuffer + 0U))) << (8U * 0U);
  u_data.base |= ((uint16_t) (*(inbuffer + 1U))) << (8U * 1U);
  *(int16_t*)(var) = u_data.real;
  return sizeof(int16_t);
}


uint8_t ob_serialize_uint32(void *var, uint8_t *outbuffer)
{
  *(outbuffer + 0U) = (*(uint32_t*)(var) >> (8U * 0U)) & 0xFF;
  *(outbuffer + 1U) = (*(uint32_t*)(var) >> (8U * 1U)) & 0xFF;
  *(outbuffer + 2U) = (*(uint32_t*)(var) >> (8U * 2U)) & 0xFF;
  *(outbuffer + 3U) = (*(uint32_t*)(var) >> (8U * 3U)) & 0xFF;
  return sizeof(uint32_t);
}

uint8_t ob_deserialize_uint32(void *var, uint8_t *inbuffer)
{
  *(uint32_t*)(var) =  ((uint32_t) (*(inbuffer + 0U))) << (8U * 0U);
  *(uint32_t*)(var) |= ((uint32_t) (*(inbuffer + 1U))) << (8U * 1U);
  *(uint32_t*)(var) |= ((uint32_t) (*(inbuffer + 2U))) << (8U * 2U);
  *(uint32_t*)(var) |= ((uint32_t) (*(inbuffer + 3U))) << (8U * 3U);
  return sizeof(uint32_t);
}


uint8_t ob_serialize_int32(void *var, uint8_t *outbuffer)
{
  union {
      int32_t real;
      uint32_t base;
  } u_data;
  u_data.real = *(int32_t*)(var);
  *(outbuffer + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
  *(outbuffer + 1U) = (u_data.base >> (8U * 1U)) & 0xFF;
  *(outbuffer + 2U) = (u_data.base >> (8U * 2U)) & 0xFF;
  *(outbuffer + 3U) = (u_data.base >> (8U * 3U)) & 0xFF;
  return sizeof(int32_t);
}

uint8_t ob_deserialize_int32(void *var, uint8_t *inbuffer)
{
  union {
      int32_t real;
      uint32_t base;
  } u_data;
  u_data.base = 0U;
  u_data.base |= ((uint32_t) (*(inbuffer + 0U))) << (8U * 0U);
  u_data.base |= ((uint32_t) (*(inbuffer + 1U))) << (8U * 1U);
  u_data.base |= ((uint32_t) (*(inbuffer + 2U))) << (8U * 2U);
  u_data.base |= ((uint32_t) (*(inbuffer + 3U))) << (8U * 3U);
  *(int32_t*)(var) = u_data.real;
  return sizeof(int32_t);
}


uint8_t ob_serialize_float(void *var, uint8_t *outbuffer)
{
  union {
      float real;
      uint32_t base;
  } u_data;
  u_data.real = *(float*)(var);
  *(outbuffer + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
  *(outbuffer + 1U) = (u_data.base >> (8U * 1U)) & 0xFF;
  *(outbuffer + 2U) = (u_data.base >> (8U * 2U)) & 0xFF;
  *(outbuffer + 3U) = (u_data.base >> (8U * 3U)) & 0xFF;
  return sizeof(float);
}

uint8_t ob_deserialize_float(void *var, uint8_t *inbuffer)
{
  union {
      float real;
      uint32_t base;
  } u_data;
  u_data.base = 0;
  u_data.base |= ((uint32_t) (*(inbuffer + 0U))) << (8U * 0U);
  u_data.base |= ((uint32_t) (*(inbuffer + 1U))) << (8U * 1U);
  u_data.base |= ((uint32_t) (*(inbuffer + 2U))) << (8U * 2U);
  u_data.base |= ((uint32_t) (*(inbuffer + 3U))) << (8U * 3U);
  *(float*)(var) = u_data.real;
  return sizeof(float);
}
