#include <string.h>
#include "tools.h"

int32_t signExtend24Bit(uint32_t u)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (u & 0x800000) ? (int32_t) (u | 0xFF000000) : (int32_t) u;
}

int32_t signExtend14Bit(uint16_t word)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (word & 0x2000) ? (int32_t) (int16_t) (word | 0xC000) : word;
}

int32_t signExtend6Bit(uint8_t byte)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (byte & 0x20) ? (int32_t) (int8_t) (byte | 0xC0) : byte;
}

int32_t signExtend4Bit(uint8_t nibble)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (nibble & 0x08) ? (int32_t) (int8_t) (nibble | 0xF0) : nibble;
}

int32_t signExtend2Bit(uint8_t byte)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (byte & 0x02) ? (int32_t) (int8_t) (byte | 0xFC) : byte;
}

bool startsWith(const char *string, const char *checkStartsWith)
{
    return strncmp(string, checkStartsWith, strlen(checkStartsWith)) == 0;
}

bool endsWith(const char *string, const char *checkEndsWith)
{
    int stringLen = strlen(string);
    int endsWithLen = strlen(checkEndsWith);

    return stringLen >= endsWithLen && strncmp(string + stringLen - endsWithLen, checkEndsWith, endsWithLen) == 0;
}

double doubleAbs(double a)
{
    if (a < 0)
        return -a;
    return a;
}

double doubleMin(double a, double b)
{
    if (a < b)
        return a;
    return b;
}

double doubleMax(double a, double b)
{
    if (a > b)
        return a;
    return b;
}

float intToFloat(int32_t i)
{
    floatConvert_t convert;

    convert.i = i;
    return convert.f;
}

float uintToFloat(uint32_t u)
{
    floatConvert_t convert;

    convert.u = u;
    return convert.f;
}

int32_t floatToInt(float f)
{
    floatConvert_t convert;

    convert.f = f;
    return convert.i;
}

uint32_t floatToUint(float f)
{
    floatConvert_t convert;

    convert.f = f;
    return convert.u;
}

/**
 * ZigZag encoding maps all values of a signed integer into those of an unsigned integer in such
 * a way that numbers of small absolute value correspond to small integers in the result.
 *
 * (Compared to just casting a signed to an unsigned which creates huge resulting numbers for
 * small negative integers).
 */
uint32_t zigzagEncode(int32_t value)
{
    return (uint32_t)((value << 1) ^ (value >> 31));
}

int32_t zigzagDecode(uint32_t value)
{
    return (value >> 1) ^ -(int32_t) (value & 1);
}

/**
 * Just like strstr, but for binary strings. Not available on all platforms, so reimplemented here.
 */
void* memmem(const void *haystack, size_t haystackLen, const void *needle, size_t needleLen)
{
    if (needleLen <= haystackLen) {
        const char* c_haystack = (char*)haystack;
        const char* c_needle = (char*)needle;

        for (const char *pos = c_haystack; pos <= c_haystack + haystackLen - needleLen; pos++) {
            if (*pos == *c_needle && memcmp(pos, c_needle, needleLen) == 0)
                return (void*)pos;
        }
    }

    return NULL;
}
