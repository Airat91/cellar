#include "menu.h"
#include "main.h"
#include "buttons.h"
#include "dcts.h"
#include "dcts_config.h"
#include "string.h"

/**
  * @defgroup menu
  * @brief work with menu
  */


#define NULL_ENTRY Null_Menu
static menuItem Null_Menu = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = 0,
    .Text = {0},
};

menuItem* selectedMenuItem;
static menuItem* menuStack[10];
static volatile uint8_t menuStackTop;

//                  NAME           NEXT            PREV            PARENT          CHILD        GHILD_NUM   PAGE                    TEXT
MAKE_MENU       (main_page,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     common_info,    4,          MAIN_PAGE,          "Главное меню");
  MAKE_MENU     (common_info,   meas_channels,  display,        main_page,      info,           0,          COMMON_INFO,        "Об устройстве");
    MAKE_MENU   (info,          NULL_ENTRY,     NULL_ENTRY,     common_info,    NULL_ENTRY,     0,          INFO,               "Об устройстве");
  MAKE_MENU     (meas_channels, lvl_calib,      common_info,    main_page,      meas_ch_0,      13,         MEAS_CHANNELS,      "Изм. каналы");
    MAKE_MENU   (meas_ch_0,     meas_ch_1,      meas_ch_12,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_0,          0x00);
    MAKE_MENU   (meas_ch_1,     meas_ch_2,      meas_ch_0,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_1,          0x00);
    MAKE_MENU   (meas_ch_2,     meas_ch_3,      meas_ch_1,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_2,          0x00);
    MAKE_MENU   (meas_ch_3,     meas_ch_4,      meas_ch_2,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_3,          0x00);
    MAKE_MENU   (meas_ch_4,     meas_ch_5,      meas_ch_3,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_4,          0x00);
    MAKE_MENU   (meas_ch_5,     meas_ch_6,      meas_ch_4,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_5,          0x00);
    MAKE_MENU   (meas_ch_6,     meas_ch_7,      meas_ch_5,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_6,          0x00);
    MAKE_MENU   (meas_ch_7,     meas_ch_8,      meas_ch_6,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_7,          0x00);
    MAKE_MENU   (meas_ch_8,     meas_ch_9,      meas_ch_7,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_8,          0x00);
    MAKE_MENU   (meas_ch_9,     meas_ch_10,     meas_ch_8,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_9,          0x00);
    MAKE_MENU   (meas_ch_10,    meas_ch_11,     meas_ch_9,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_10,         0x00);
    MAKE_MENU   (meas_ch_11,    meas_ch_12,     meas_ch_10,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_11,         0x00);
    MAKE_MENU   (meas_ch_12,    meas_ch_0,      meas_ch_11,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_12,         0x00);
  MAKE_MENU     (lvl_calib,     tmpr_calib,     meas_channels,  main_page,      lvl_0,          6,          LVL_CALIB,          "Калиб. уровня");
    MAKE_MENU   (lvl_0,         lvl_20,         lvl_100,        lvl_calib,      NULL_ENTRY,     0,          LVL_0,              "АЦП при 0л");
    MAKE_MENU   (lvl_20,        lvl_40,         lvl_0,          lvl_calib,      NULL_ENTRY,     0,          LVL_20,             "АЦП при 20л");
    MAKE_MENU   (lvl_40,        lvl_60,         lvl_20,         lvl_calib,      NULL_ENTRY,     0,          LVL_40,             "АЦП при 40л");
    MAKE_MENU   (lvl_60,        lvl_80,         lvl_40,         lvl_calib,      NULL_ENTRY,     0,          LVL_60,             "АЦП при 60л");
    MAKE_MENU   (lvl_80,        lvl_100,        lvl_60,         lvl_calib,      NULL_ENTRY,     0,          LVL_80,             "АЦП при 80л");
    MAKE_MENU   (lvl_100,       lvl_0,          lvl_80,         lvl_calib,      NULL_ENTRY,     0,          LVL_100,            "АЦП при 100л");
  MAKE_MENU     (tmpr_calib,    connection,    lvl_calib,       main_page,      adc_0,          11,         TMPR_CALIB,         "Калиб. темпер.");
    MAKE_MENU   (adc_0,         adc_10,         adc_100,        tmpr_calib,     NULL_ENTRY,     0,          ADC_0,              "АЦП при 0°C");
    MAKE_MENU   (adc_10,        adc_20,         adc_0,          tmpr_calib,     NULL_ENTRY,     0,          ADC_10,             "АЦП при 10°C");
    MAKE_MENU   (adc_20,        adc_30,         adc_10,         tmpr_calib,     NULL_ENTRY,     0,          ADC_20,             "АЦП при 20°C");
    MAKE_MENU   (adc_30,        adc_40,         adc_20,         tmpr_calib,     NULL_ENTRY,     0,          ADC_30,             "АЦП при 30°C");
    MAKE_MENU   (adc_40,        adc_50,         adc_30,         tmpr_calib,     NULL_ENTRY,     0,          ADC_40,             "АЦП при 40°C");
    MAKE_MENU   (adc_50,        adc_60,         adc_40,         tmpr_calib,     NULL_ENTRY,     0,          ADC_50,             "АЦП при 50°C");
    MAKE_MENU   (adc_60,        adc_70,         adc_50,         tmpr_calib,     NULL_ENTRY,     0,          ADC_60,             "АЦП при 60°C");
    MAKE_MENU   (adc_70,        adc_80,         adc_60,         tmpr_calib,     NULL_ENTRY,     0,          ADC_70,             "АЦП при 70°C");
    MAKE_MENU   (adc_80,        adc_90,         adc_70,         tmpr_calib,     NULL_ENTRY,     0,          ADC_80,             "АЦП при 80°C");
    MAKE_MENU   (adc_90,        adc_100,        adc_80,         tmpr_calib,     NULL_ENTRY,     0,          ADC_90,             "АЦП при 90°C");
    MAKE_MENU   (adc_100,       adc_0,          adc_90,         tmpr_calib,     NULL_ENTRY,     0,          ADC_100,            "АЦП при 100°C");
  MAKE_MENU     (connection,    display,        tmpr_calib,     main_page,      mdb_addr,       6,          CONNECTION,         "Связь");
    MAKE_MENU   (mdb_addr,      bitrate,        noise_err,      connection,     NULL_ENTRY,     0,          MDB_ADDR,           "Адрес ModBUS");
    MAKE_MENU   (bitrate,       overrun_err,    mdb_addr,       connection,     NULL_ENTRY,     0,          MDB_BITRATE,        "Битрейт");
    MAKE_MENU   (overrun_err,   parity_err,     bitrate,        connection,     NULL_ENTRY,     0,          MDB_OVERRUN_ERR,    "Ошибки чтения");
    MAKE_MENU   (parity_err,    frame_err,      overrun_err,    connection,     NULL_ENTRY,     0,          MDB_PARITY_ERR,     "Ошибки паритета");
    MAKE_MENU   (frame_err,     noise_err,      parity_err,     connection,     NULL_ENTRY,     0,          MDB_FRAME_ERR,      "Ошибки кадра");
    MAKE_MENU   (noise_err,     mdb_addr,       frame_err,      connection,     NULL_ENTRY,     0,          MDB_NOISE_ERR,      "Ошибки помехи");
  MAKE_MENU     (display,       common_info,    connection,     main_page,      light_lvl,      2,          DISPLAY,            "Дисплей");
    MAKE_MENU   (light_lvl,     auto_off,       auto_off,       display,        NULL_ENTRY,     0,          LIGHT_LVL,          "Яркость");
    MAKE_MENU   (auto_off,      light_lvl,      light_lvl,      display,        NULL_ENTRY,     0,          AUTO_OFF,           "Выкл. подсв.");

MAKE_MENU       (save_changes,  NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     0,          SAVE_CHANGES,       "Сохранить изм.");


/*========== FUNCTIONS ==========*/

void menu_init (void){
    strcpy(meas_ch_0.Text, dcts_meas[0].name);
    strcpy(meas_ch_1.Text, dcts_meas[1].name);
    strcpy(meas_ch_2.Text, dcts_meas[2].name);
    strcpy(meas_ch_3.Text, dcts_meas[3].name);
    strcpy(meas_ch_4.Text, dcts_meas[4].name);
    strcpy(meas_ch_5.Text, dcts_meas[5].name);
    strcpy(meas_ch_6.Text, dcts_meas[6].name);
    strcpy(meas_ch_7.Text, dcts_meas[7].name);
    strcpy(meas_ch_8.Text, dcts_meas[8].name);
    strcpy(meas_ch_9.Text, dcts_meas[9].name);
    strcpy(meas_ch_10.Text, dcts_meas[10].name);
    strcpy(meas_ch_11.Text, dcts_meas[11].name);
    strcpy(meas_ch_12.Text, dcts_meas[12].name);

    selectedMenuItem = &main_page;
}

void menuChange(menuItem* NewMenu){
    if (NewMenu != &NULL_ENTRY){
        selectedMenuItem = NewMenu;
    }
}
