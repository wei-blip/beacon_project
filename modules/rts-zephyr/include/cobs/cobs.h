#ifndef ZEPHYR_INCLUDE_COBS_COBS_H_
#define ZEPHYR_INCLUDE_COBS_COBS_H_

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t cobs_encode(const void * src, size_t length, void * dst);

size_t cobs_decode(const void * src, size_t length, void * dst);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_COBS_COBS_H_ */
