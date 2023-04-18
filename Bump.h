/*
 * Bump.h
 *
 *  Created on: Feb 22, 2023
 *      Author: jsb19
 */

#ifndef BUMP_H_
#define BUMP_H_

#include <stdint.h>

#define B0 0x01
#define B1 0x04
#define B2 0x08
#define B3 0x20
#define B4 0x40
#define B5 0x80

void initP4(void);
uint8_t readP4(void);

#endif /* BUMP_H_ */
