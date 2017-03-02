/**
 * @file
 * @brief Basic data types for odict serialization.
 * @author Rodrigo Munoz
 * @date Jan, 2017
 *
 */

#ifndef ODICT_TYPES_H
#define ODICT_TYPES_H

#include <stdio.h>
#include <stdint.h>

/**
 * Data type status.
 */
typedef enum _od_status
{
    OD_LOCKED = 0U, /**< Data is lock for write */
    OD_FREE   = 1U  /**< Data is free for read and write */
}od_status;

/**
 * Storage type used.
 */
typedef enum _od_storage
{
    OD_RAM    = 0U, /**< Used for only RAM storage, data initialization on default value */
    OD_EEPROM = 1U  /**< Used for RAM ans non volatile storage, data initialization on last saved value */
}od_storage;

/**
 * Data access type.
 */
typedef enum _od_access
{
    OD_R  = 0U, /**< Master device can only read the value. */
    OD_RW = 1U  /**< Master device can read and write the value. */
}od_access;

/**
 * Data type.
 */
typedef enum _od_type
{
    OD_UINT8  = 0U, /**< uint8_t value. */
    OD_INT8   = 1U, /**< int8_t value. */
    OD_UINT16 = 2U, /**< uint16_t value. */
    OD_INT16  = 3U, /**< int16_t value. */
    OD_UINT32 = 4U, /**< uint32_t value. */
    OD_INT32  = 5U, /**< int32_t value. */
    OD_FLOAT  = 6U, /**< float value. */
}od_type;

/**
 * Information storage bit field.
 */
typedef union _od_info
{
    uint8_t val;
    struct
    {
        uint8_t type   : 3; /**< od_type */
        uint8_t status : 1; /**< od_status */
        uint8_t storage: 1; /**< od_storage */
        uint8_t access : 1; /**< od_access */
    };
}od_info;

/**
 * Entry struct.
 */
typedef struct _od_entry
{
    void *var;      /**< Variable pointer. */
    uint8_t offset; /**< Buffer offset. */
    od_info info;   /**< od_info */
}od_entry;

#endif //ODICT_TYPES_H
