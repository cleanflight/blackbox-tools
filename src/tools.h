#ifndef TOOLS_H_
#define TOOLS_H_

#include <stdint.h>
#include <stdbool.h>

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

// Convert a token into a quoted string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

typedef union floatConvert_t {
    float f;
    uint32_t u;
    int32_t i;
} floatConvert_t;

float intToFloat(int32_t i);
float uintToFloat(uint32_t u);
int32_t floatToInt(float f);
uint32_t floatToUint(float f);

int32_t signExtend24Bit(uint32_t u);
int32_t signExtend14Bit(uint16_t word);
int32_t signExtend6Bit(uint8_t byte);
int32_t signExtend4Bit(uint8_t nibble);
int32_t signExtend2Bit(uint8_t byte);

uint32_t zigzagEncode(int32_t value);
int32_t zigzagDecode(uint32_t value);

double doubleAbs(double a);
double doubleMin(double a, double b);
double doubleMax(double a, double b);

bool startsWith(const char *string, const char *checkStartsWith);
bool endsWith(const char *string, const char *checkEndsWith);

void* memmem(const void *haystack, size_t haystackLen, const void *needle, size_t needleLen);

#endif
