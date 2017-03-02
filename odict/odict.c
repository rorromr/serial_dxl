#include "odict.h"

uint8_t buffer[10];

void print_buffer()
{
    int i;
    for (i = 0; i < 10; ++i)
    {
        printf("%d: %d\n", i, buffer[i]);
    }
}

int main()
{
    printf("od_info == %lu\n", sizeof(od_info));
    printf("od_entry == %lu\n", sizeof(od_entry));
    printf("void* == %lu\n", sizeof(void*));

    /* uint8_t */
    int8_t var = -5;
    od_entry entry;
    entry.var = (void*) &var;
    entry.offset = 0U;
    entry.info.type = OD_INT8;
    entry.info.status = OD_FREE;
    entry.info.storage = OD_RAM;
    entry.info.access = OD_RW;

    od_serialize_entry(&entry, buffer);
    print_buffer();
  od_deserialize_entry(&entry, buffer);
  printf("Value %d", *(int8_t*)entry.var);
    return 0;
}