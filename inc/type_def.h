#ifndef TYPE_DEF_H
#define TYPE_DEF_H
#include "stdint.h"
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

#define MCU_PACK __attribute__((packed))
#define delay_until vTaskDelayUntil
typedef signed long long int s64;
typedef signed long int s32;
typedef signed short s16;
typedef signed char s8;
typedef signed char i8;

typedef volatile signed long long int vs64;
typedef volatile signed long int vs32;
typedef volatile signed short vs16;
typedef volatile signed char vs8;
typedef volatile signed char vi8;

typedef uint64_t u64;
typedef unsigned long int u32;
typedef unsigned short u16;
typedef unsigned char u8;

typedef const unsigned long long int uc64;
typedef const unsigned long int uc32;
typedef const unsigned short uc16;
typedef const unsigned char uc8;

typedef volatile unsigned long int vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char vu8;

typedef volatile unsigned long int const vuc32;
typedef volatile unsigned short const vuc16;
typedef volatile unsigned char const vuc8;

#ifndef TRUE
  #define TRUE 1
#endif
#ifndef FALSE
  #define FALSE 0
#endif

#define MCU_ROOT_CODE __attribute__((section(".priv_code")))
#define MCU_PRIV_CODE MCU_ROOT_CODE
#define asm __asm

#endif// TYPE_DEF_H
