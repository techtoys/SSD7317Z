/**
 * @brief	Header file for c array generated by lcd-image-converter.
 * 		Project's home page: http://www.riuson.com/lcd-image-converter
 */

#ifndef INC_SYSFONT_H_
#define INC_SYSFONT_H_

#if defined(__cplusplus)
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif

#include <stdint.h>

typedef struct {
     const uint8_t *data;
     uint16_t width;
     uint16_t height;
     uint8_t dataSize;
     } tImage;

typedef struct {
	long int code;
	const tImage *image;
	} tChar;

typedef struct {
    int length;
    const tChar *chars;
	} tFont;

#ifdef __cplusplus
}
#endif

#endif /* INC_SYSFONT_H_ */
