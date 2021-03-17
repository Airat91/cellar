#include "stdint.h"

#ifndef MENU_H
#define MENU_H 1

/*========== DEFINES ==========*/

#define MAKE_MENU(Name, Next, Previous, Parent, Child, Child_num, Page, Text) \
    extern const menuItem Next;     \
    extern const menuItem Previous; \
    extern const menuItem Parent;   \
    extern const menuItem Child;  \
    const menuItem Name = {(const void*)&Next, (const void*)&Previous, (const void*)&Parent, (const void*)&Child, (const uint16_t)Child_num, (const uint16_t)Page, { Text }}

/*#define PREVIOUS   ((menuItem*)pgm_read_word(&selectedMenuItem->Previous))
#define NEXT       ((menuItem*)pgm_read_word(&selectedMenuItem->Next))
#define PARENT     ((menuItem*)pgm_read_word(&selectedMenuItem->Parent))
#define CHILD      ((menuItem*)pgm_read_word(&selectedMenuItem->Child))
#define SELECT		(pgm_read_byte(&selectedMenuItem->Select))*/
#define NULL_ENTRY  Null_Menu
#define EDITED_VAL  edit_value

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
    MEAS_CH_13,
    MEAS_CH_14,
    MEAS_CH_15,
    MEAS_CH_16,
    MEAS_CH_17,
    MEAS_CH_18,
    MEAS_CH_19,
    ACT_CHANNELS,
    ACT_CH_0,
    ACT_EN_0,
    ACT_SET_0,
    ACT_HYST_0,
    ACT_CUR_0,
    ACT_CH_1,
    ACT_EN_1,
    ACT_SET_1,
    ACT_HYST_1,
    ACT_CUR_1,
    ACT_CH_2,
    ACT_EN_2,
    ACT_SET_2,
    ACT_HYST_2,
    ACT_CUR_2,
    ACT_CH_3,
    ACT_EN_3,
    ACT_SET_3,
    ACT_HYST_3,
    ACT_CUR_3,
    RELE_CHANNELS,
    RELE_CH_0,
    RELE_AUTO_MAN_0,
    RELE_CONTROL_0,
    RELE_CH_1,
    RELE_AUTO_MAN_1,
    RELE_CONTROL_1,
    RELE_CH_2,
    RELE_AUTO_MAN_2,
    RELE_CONTROL_2,
    RELE_CH_3,
    RELE_AUTO_MAN_3,
    RELE_CONTROL_3,
    RELE_CH_4,
    RELE_AUTO_MAN_4,
    RELE_CONTROL_4,
    RELE_CH_5,
    RELE_AUTO_MAN_5,
    RELE_CONTROL_5,
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
    TIME,
    TIME_HOUR,
    TIME_MIN,
    TIME_SEC,
    DATE,
    DATE_DAY,
    DATE_MONTH,
    DATE_YEAR,
    SAVE_CHANGES,
    EDIT,
} menu_page_t;

typedef struct {
    const void          *Next;
    const void          *Previous;
    const void          *Parent;
    const void          *Child;
    const uint16_t      Child_num;
    const menu_page_t   Page;
    const char          Text[20];
} menuItem;



/*========= GLOBAL VARIABLES ==========*/

extern const menuItem main_page;
extern const menuItem save_changes;
extern menuItem* selectedMenuItem;
extern const menuItem edit_value;

/*========== FUNCTION PROTOTYPES ==========*/

void menu_init (void);

void menuChange(menuItem* NewMenu);

#endif // MENU_H
