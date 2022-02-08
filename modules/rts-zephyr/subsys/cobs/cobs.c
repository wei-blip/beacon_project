/* Copyright 2011, Jacques Fortier. All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted,
 * with or without modification.
 */

#include <cobs/cobs.h>

/*
 * Stuffs "length" bytes of data at the location pointed to by "src",
 * writing the output to the location pointed to by "dst".
 *
 * Returns the length of the encoded data
 * or 0 if any of argument pointers is NULL.
 */
size_t cobs_encode(const void * src, size_t length, void * dst)
{
    if (src == NULL || dst == NULL) {
        return 0;
    }

    const uint8_t *input = (const uint8_t *) src;
    uint8_t *output = (uint8_t *) dst;
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while(read_index < length)
    {
        if(input[read_index] == 0)
        {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        }
        else
        {
            output[write_index++] = input[read_index++];
            code++;
            if(code == 0xFF)
            {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }

    output[code_index] = code;

    return write_index;
}

/* Unstuffs "length" bytes of data at the location pointed to by "src",
 * writing the output to the location pointed to by "dst".
 *
 * Returns the number of bytes written to "dst"
 * if "src" was successfully unstuffed,
 * and 0 if there was an error unstuffing "src",
 * or 0 if any of argument pointers is NULL.
 */
size_t cobs_decode(const void * src, size_t length, void * dst)
{
    if (src == NULL || dst == NULL) {
        return 0;
    }

    const uint8_t *input = (const uint8_t *) src;
    uint8_t *output = (uint8_t *) dst;
    size_t read_index = 0;
    size_t write_index = 0;
    uint8_t code;
    uint8_t i;

    while(read_index < length)
    {
        code = input[read_index];

        if(read_index + code > length && code != 1)
        {
            return 0;
        }

        read_index++;

        for(i = 1; i < code; i++)
        {
            output[write_index++] = input[read_index++];
        }
        if(code != 0xFF && read_index != length)
        {
            output[write_index++] = '\0';
        }
    }

    return write_index;
}
