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


static menuItem Null_Menu = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = 0,
    .Text = {0},
};
menuItem edit_value = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = EDIT,
    .Text = {0},
};

menuItem* selectedMenuItem;

//                  NAME           NEXT            PREV            PARENT          CHILD        GHILD_NUM   PAGE                    TEXT
MAKE_MENU       (main_page,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     common_info,    4,          MAIN_PAGE,          "Главное меню");
  MAKE_MENU     (common_info,   meas_channels,  date,           main_page,      info,           1,          COMMON_INFO,        "Об устройстве");
    MAKE_MENU   (info,          NULL_ENTRY,     NULL_ENTRY,     common_info,    NULL_ENTRY,     0,          INFO,               "Об устройстве");
  MAKE_MENU     (meas_channels, act_channels,   common_info,    main_page,      meas_ch_0,      18,         MEAS_CHANNELS,      "Изм. каналы");
    MAKE_MENU   (meas_ch_0,     meas_ch_1,      meas_ch_17,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_0,          0x00);
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
    MAKE_MENU   (meas_ch_12,    meas_ch_13,     meas_ch_11,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_12,         0x00);
    MAKE_MENU   (meas_ch_13,    meas_ch_14,     meas_ch_12,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_13,         0x00);
    MAKE_MENU   (meas_ch_14,    meas_ch_15,     meas_ch_13,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_14,         0x00);
    MAKE_MENU   (meas_ch_15,    meas_ch_16,     meas_ch_14,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_15,         0x00);
    MAKE_MENU   (meas_ch_16,    meas_ch_17,     meas_ch_15,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_16,         0x00);
    MAKE_MENU   (meas_ch_17,    meas_ch_0,      meas_ch_16,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_17,         0x00);
  MAKE_MENU     (act_channels,  rele_channels,  meas_channels,  main_page,      act_ch_0,       13,         ACT_CHANNELS,       "Упр. каналы");
    MAKE_MENU   (act_ch_0,      act_ch_1,       act_ch_3,       act_channels,   NULL_ENTRY,     0,          ACT_CH_0,           0x00);
    MAKE_MENU   (act_ch_1,      act_ch_2,       act_ch_0,       act_channels,   NULL_ENTRY,     0,          ACT_CH_1,           0x00);
    MAKE_MENU   (act_ch_2,      act_ch_3,       act_ch_1,       act_channels,   NULL_ENTRY,     0,          ACT_CH_2,           0x00);
    MAKE_MENU   (act_ch_3,      act_ch_0,       act_ch_2,       act_channels,   NULL_ENTRY,     0,          ACT_CH_3,           0x00);
  MAKE_MENU     (rele_channels, connection,     act_channels,   main_page,      rele_ch_0,      6,          RELE_CHANNELS,      "Релейные выходы");
    MAKE_MENU   (rele_ch_0,     rele_ch_1,      rele_ch_5,      rele_channels,  NULL_ENTRY,     0,          RELE_CH_0,          0x00);
    MAKE_MENU   (rele_ch_1,     rele_ch_2,      rele_ch_0,      rele_channels,  NULL_ENTRY,     0,          RELE_CH_1,          0x00);
    MAKE_MENU   (rele_ch_2,     rele_ch_3,      rele_ch_1,      rele_channels,  NULL_ENTRY,     0,          RELE_CH_2,          0x00);
    MAKE_MENU   (rele_ch_3,     rele_ch_4,      rele_ch_2,      rele_channels,  NULL_ENTRY,     0,          RELE_CH_3,          0x00);
    MAKE_MENU   (rele_ch_4,     rele_ch_5,      rele_ch_3,      rele_channels,  NULL_ENTRY,     0,          RELE_CH_4,          0x00);
    MAKE_MENU   (rele_ch_5,     rele_ch_0,      rele_ch_4,      rele_channels,  NULL_ENTRY,     0,          RELE_CH_5,          0x00);
  MAKE_MENU     (connection,    display,        rele_channels,  main_page,      mdb_addr,       6,          CONNECTION,         "Связь");
    MAKE_MENU   (mdb_addr,      bitrate,        noise_err,      connection,     EDITED_VAL,     0,          MDB_ADDR,           "Адрес ModBUS");
    MAKE_MENU   (bitrate,       overrun_err,    mdb_addr,       connection,     EDITED_VAL,     0,          MDB_BITRATE,        "Битрейт");
    MAKE_MENU   (overrun_err,   parity_err,     bitrate,        connection,     NULL_ENTRY,     0,          MDB_OVERRUN_ERR,    "Ошибки чтения");
    MAKE_MENU   (parity_err,    frame_err,      overrun_err,    connection,     NULL_ENTRY,     0,          MDB_PARITY_ERR,     "Ошибки паритета");
    MAKE_MENU   (frame_err,     noise_err,      parity_err,     connection,     NULL_ENTRY,     0,          MDB_FRAME_ERR,      "Ошибки кадра");
    MAKE_MENU   (noise_err,     mdb_addr,       frame_err,      connection,     NULL_ENTRY,     0,          MDB_NOISE_ERR,      "Ошибки помехи");
  MAKE_MENU     (display,       time,           connection,     main_page,      light_lvl,      2,          DISPLAY,            "Дисплей");
    MAKE_MENU   (light_lvl,     auto_off,       auto_off,       display,        EDITED_VAL,     0,          LIGHT_LVL,          "Яркость");
    MAKE_MENU   (auto_off,      light_lvl,      light_lvl,      display,        EDITED_VAL,     0,          AUTO_OFF,           "Автовыкл. подсветки");
  MAKE_MENU     (time,          date,           display,        main_page,      time_hour,      3,          TIME,               "Время");
    MAKE_MENU   (time_hour,     time_min,       time_sec,       time,           EDITED_VAL,     0,          TIME_HOUR,          "Часы");
    MAKE_MENU   (time_min,      time_sec,       time_hour,      time,           EDITED_VAL,     0,          TIME_MIN,           "Минуты");
    MAKE_MENU   (time_sec,      time_hour,      time_min,       time,           EDITED_VAL,     0,          TIME_SEC,           "Секунды");
  MAKE_MENU     (date,          common_info,    time,           main_page,      date_day,       3,          DATE,               "Дата");
    MAKE_MENU   (date_day,      date_month,     date_year,      date,           EDITED_VAL,     0,          DATE_DAY,           "День");
    MAKE_MENU   (date_month,    date_year,      date_day,       date,           EDITED_VAL,     0,          DATE_MONTH,         "Месяц");
    MAKE_MENU   (date_year,     date_day,       date_month,     date,           EDITED_VAL,     0,          DATE_YEAR,          "Год");

MAKE_MENU       (save_changes,  NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     0,          SAVE_CHANGES,       "Сохранить изм.");


/*========== FUNCTIONS ==========*/

void menu_init (void){
    strcpy(meas_ch_0.Text, dcts_meas[0].name_cyr);
    strcpy(meas_ch_1.Text, dcts_meas[1].name_cyr);
    strcpy(meas_ch_2.Text, dcts_meas[2].name_cyr);
    strcpy(meas_ch_3.Text, dcts_meas[3].name_cyr);
    strcpy(meas_ch_4.Text, dcts_meas[4].name_cyr);
    strcpy(meas_ch_5.Text, dcts_meas[5].name_cyr);
    strcpy(meas_ch_6.Text, dcts_meas[6].name_cyr);
    strcpy(meas_ch_7.Text, dcts_meas[7].name_cyr);
    strcpy(meas_ch_8.Text, dcts_meas[8].name_cyr);
    strcpy(meas_ch_9.Text, dcts_meas[9].name_cyr);
    strcpy(meas_ch_10.Text, dcts_meas[10].name_cyr);
    strcpy(meas_ch_11.Text, dcts_meas[11].name_cyr);
    strcpy(meas_ch_12.Text, dcts_meas[12].name_cyr);
    strcpy(meas_ch_13.Text, dcts_meas[13].name_cyr);
    strcpy(meas_ch_14.Text, dcts_meas[17].name_cyr);
    strcpy(meas_ch_15.Text, dcts_meas[15].name_cyr);
    strcpy(meas_ch_16.Text, dcts_meas[16].name_cyr);
    strcpy(meas_ch_17.Text, dcts_meas[17].name_cyr);

    strcpy(act_ch_0.Text, dcts_act[0].name_cyr);
    strcpy(act_ch_1.Text, dcts_act[1].name_cyr);
    strcpy(act_ch_2.Text, dcts_act[2].name_cyr);
    strcpy(act_ch_3.Text, dcts_act[3].name_cyr);

    strcpy(rele_ch_0.Text, dcts_rele[0].name_cyr);
    strcpy(rele_ch_1.Text, dcts_rele[1].name_cyr);
    strcpy(rele_ch_2.Text, dcts_rele[2].name_cyr);
    strcpy(rele_ch_3.Text, dcts_rele[3].name_cyr);
    strcpy(rele_ch_4.Text, dcts_rele[4].name_cyr);
    strcpy(rele_ch_5.Text, dcts_rele[5].name_cyr);

    selectedMenuItem = &main_page;
}

void menuChange(menuItem* NewMenu){
    if ((NewMenu != &NULL_ENTRY)&&(NewMenu != &EDITED_VAL)){
        selectedMenuItem = NewMenu;
    }
}
