/**
 * @file
 * @brief Functions for data serialization.
 * @author Rodrigo Munoz
 * @date Jan, 2017
 *
 */

#ifndef ODICT_SERIALIZATION_H
#define ODICT_SERIALIZATION_H

#include "types.h"

/**
 * Serialization of od_entry struct.
 * @param entry Input struct.
 * @param outbuffer Out buffer.
 * @return Data size.
 */
uint8_t od_serialize_entry(od_entry *entry, uint8_t *outbuffer);

/**
 * Deserialization of od_entry struct.
 * @param entry Input struct.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t od_deserialize_entry(od_entry *entry, uint8_t *inbuffer);

/**
 * @typedef Serialization function pointer.
 */
typedef uint8_t (*od_serialize)(void*, uint8_t*);

/**
 * @typedef Deserialization function pointer.
 */
typedef uint8_t (*od_deserialize)(void*, uint8_t*);

/**
 * Serialization of uint8_t data type.
 * @param var Data pointer.
 * @param outbuffer Out buffer.
 * @return Data size.
 */

uint8_t ob_serialize_uint8(void *var, uint8_t *outbuffer);
/**
 * Deserialization of uint8_t data type.
 * @param var Data pointer.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t ob_deserialize_uint8(void *var, uint8_t *inbuffer);

/**
 * Serialization of int8_t data type.
 * @param var Data pointer.
 * @param outbuffer Out buffer.
 * @return Data size.
 */
uint8_t ob_serialize_int8(void *var, uint8_t *outbuffer);

/**
 * Deserialization of int8_t data type.
 * @param var Data pointer.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t ob_deserialize_int8(void *var, uint8_t *inbuffer);

/**
 * Serialization of uint16_t data type.
 * @param var Data pointer.
 * @param outbuffer Out buffer.
 * @return Data size.
 */
uint8_t ob_serialize_uint16(void *var, uint8_t *outbuffer);

/**
 * Deserialization of uint16_t data type.
 * @param var Data pointer.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t ob_deserialize_uint16(void *var, uint8_t *inbuffer);

/**
 * Serialization of int16_t data type.
 * @param var Data pointer.
 * @param outbuffer Out buffer.
 * @return Data size.
 */
uint8_t ob_serialize_int16(void *var, uint8_t *outbuffer);

/**
 * Deserialization of int16_t data type.
 * @param var Data pointer.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t ob_deserialize_int16(void *var, uint8_t *inbuffer);

/**
 * Serialization of uint32_t data type.
 * @param var Data pointer.
 * @param outbuffer Out buffer.
 * @return Data size.
 */
uint8_t ob_serialize_uint32(void *var, uint8_t *outbuffer);

/**
 * Deserialization of uint32_t data type.
 * @param var Data pointer.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t ob_deserialize_uint32(void *var, uint8_t *inbuffer);

/**
 * Serialization of int32_t data type.
 * @param var Data pointer.
 * @param outbuffer Out buffer.
 * @return Data size.
 */
uint8_t ob_serialize_int32(void *var, uint8_t *outbuffer);

/**
 * Deserialization of int32_t data type.
 * @param var Data pointer.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t ob_deserialize_int32(void *var, uint8_t *inbuffer);

/**
 * Serialization of float data type.
 * @param var Data pointer.
 * @param outbuffer Out buffer.
 * @return Data size.
 */
uint8_t ob_serialize_float(void *var, uint8_t *outbuffer);

/**
 * Deserialization of float data type.
 * @param var Data pointer.
 * @param inbuffer Input buffer.
 * @return Data size.
 */
uint8_t ob_deserialize_float(void *var, uint8_t *inbuffer);


#endif //ODICT_SERIALIZATION_H
