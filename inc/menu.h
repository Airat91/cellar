#include "stdint.h"

#ifndef MENU_H
#define MENU_H 1

/*========== DEFINES ==========*/

#define MAKE_MENU(Name, Next, Previous, Parent, Child, Child_num, Page, Text) \
    extern menuItem Next;     \
    extern menuItem Previous; \
    extern menuItem Parent;   \
    extern menuItem Child;  \
    menuItem Name = {(void*)&Next, (void*)&Previous, (void*)&Parent, (void*)&Child, (uint16_t)Child_num, (uint16_t)Page, { Text }}

#define PREVIOUS   ((menuItem*)pgm_read_word(&selectedMenuItem->Previous))
#define NEXT       ((menuItem*)pgm_read_word(&selectedMenuItem->Next))
#define PARENT     ((menuItem*)pgm_read_word(&selectedMenuItem->Parent))
#define CHILD      ((menuItem*)pgm_read_word(&selectedMenuItem->Child))
#define SELECT		(pgm_read_byte(&selectedMenuItem->Select))

/*========== TYPEDEFS ==========*/

typedef enum {
    MAIN_PAGE = 0,
    MAIN_MENU,
    COMMON_INFO,
    INFO,
    MEAS_CHANNELS,
    MEAS_CH_0,
    MEAS_CH_1,
    MEAS_CH_2,
    MEAS_CH_3,
    MEAS_CH_4,
    MEAS_CH_5,
    MEAS_CH_6,
    MEAS_CH_7,
    MEAS_CH_8,
    MEAS_CH_9,
    MEAS_CH_10,
    MEAS_CH_11,
    MEAS_CH_12,
    LVL_CALIB,
    LVL_0,
    LVL_20,
    LVL_40,
    LVL_60,
    LVL_80,
    LVL_100,
    TMPR_CALIB,
    ADC_0,
    ADC_10,
    ADC_20,
    ADC_30,
    ADC_40,
    ADC_50,
    ADC_60,
    ADC_70,
    ADC_80,
    ADC_90,
    ADC_100,
    CONNECTION,
    MDB_ADDR,
    MDB_BITRATE,
    MDB_OVERRUN_ERR,
    MDB_PARITY_ERR,
    MDB_FRAME_ERR,
    MDB_NOISE_ERR,
    DISPLAY,
    LIGHT_LVL,
    AUTO_OFF,
    SAVE_CHANGES,
} menu_page_t;

typedef struct {
    void            *Next;
    void            *Previous;
    void            *Parent;
    void            *Child;
    uint16_t        Child_num;
    menu_page_t     Page;
    const char      Text[20];
} menuItem;



/*========= GLOBAL VARIABLES ==========*/

extern menuItem main_page;
extern menuItem save_changes;
extern menuItem* selectedMenuItem;

/*========== FUNCTION PROTOTYPES ==========*/

void menu_init (void);

void menuChange(menuItem* NewMenu);

#endif // MENU_H
