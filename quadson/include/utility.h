#ifndef INC_UTILITY_H
#define INC_UTILITY_H

#define LOW_BYTE(x) ((unsigned char)((x)&0xFF))
#define HIGH_BYTE(x) ((unsigned char)(((x) >> 8) & 0xFF))
#define MAKE_SHORT(l, h) (((uint16_t)((h)&0xFF) << 8) | (uint16_t)((l)&0xFF))

#define CALC_R(a, b) (sqrt((a) * (a) + (b) * (b)))

#endif /* INC_BINARY_TOOL_H_ */