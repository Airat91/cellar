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


static const menuItem Null_Menu = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = 0,
    .Text = {0},
};
const menuItem edit_value = {
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
  MAKE_MENU     (meas_channels, act_channels,   common_info,    main_page,      meas_ch_0,      20,         MEAS_CHANNELS,      "Изм. каналы");
    MAKE_MENU   (meas_ch_0,     meas_ch_1,      meas_ch_19,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_0,          "Температура 1");
    MAKE_MENU   (meas_ch_1,     meas_ch_2,      meas_ch_0,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_1,          "Температура 2");
    MAKE_MENU   (meas_ch_2,     meas_ch_3,      meas_ch_1,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_2,          "Температура усред.");
    MAKE_MENU   (meas_ch_3,     meas_ch_4,      meas_ch_2,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_3,          "Влажность 1");
    MAKE_MENU   (meas_ch_4,     meas_ch_5,      meas_ch_3,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_4,          "Влажнсть 2");
    MAKE_MENU   (meas_ch_5,     meas_ch_6,      meas_ch_4,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_5,          "Влажность усред.");
    MAKE_MENU   (meas_ch_6,     meas_ch_7,      meas_ch_5,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_6,          "Температура снаружи");
    MAKE_MENU   (meas_ch_7,     meas_ch_8,      meas_ch_6,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_7,          "Влажность снаружи");
    MAKE_MENU   (meas_ch_8,     meas_ch_9,      meas_ch_7,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_8,          "Дренаж верх АЦП");
    MAKE_MENU   (meas_ch_9,     meas_ch_10,     meas_ch_8,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_9,          "Дренаж верх В");
    MAKE_MENU   (meas_ch_10,    meas_ch_11,     meas_ch_9,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_10,         "Дренаж низ АЦП");
    MAKE_MENU   (meas_ch_11,    meas_ch_12,     meas_ch_10,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_11,         "Дренаж низ В");
    MAKE_MENU   (meas_ch_12,    meas_ch_13,     meas_ch_11,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_12,         "Мин уровень Ом");
    MAKE_MENU   (meas_ch_13,    meas_ch_14,     meas_ch_12,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_13,         "Мин. уровень АЦП");
    MAKE_MENU   (meas_ch_14,    meas_ch_15,     meas_ch_13,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_14,         "Мин. уровень В");
    MAKE_MENU   (meas_ch_15,    meas_ch_16,     meas_ch_14,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_15,         "Макс. уровень Ом");
    MAKE_MENU   (meas_ch_16,    meas_ch_17,     meas_ch_15,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_16,         "Макс. уровень АЦП");
    MAKE_MENU   (meas_ch_17,    meas_ch_18,     meas_ch_16,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_17,         "Макс. уровень В");
    MAKE_MENU   (meas_ch_18,    meas_ch_19,     meas_ch_17,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_18,         "Опорное напр. АЦП");
    MAKE_MENU   (meas_ch_19,    meas_ch_0,      meas_ch_18,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_19,         "Батарейка В");
  MAKE_MENU     (act_channels,  rele_channels,  meas_channels,  main_page,      act_ch_0,       5,          ACT_CHANNELS,       "Упр. каналы");
    MAKE_MENU   (act_ch_0,      act_ch_1,       act_ch_4,       act_channels,   act_en_0,       2,          ACT_CH_0,           "Клапан приточный");
      MAKE_MENU (act_en_0,      act_set_0,      act_set_0,      act_ch_0,       EDITED_VAL,     0,          ACT_EN_0,           "Управление");
      MAKE_MENU (act_set_0,     act_en_0,       act_en_0,       act_ch_0,       EDITED_VAL,     0,          ACT_SET_0,          "Задано");
      //MAKE_MENU (act_hyst_0,    act_cur_0,      act_set_0,      act_ch_0,       EDITED_VAL,     0,          ACT_HYST_0,         "Гистерезис");
      //MAKE_MENU (act_cur_0,     act_en_0,       act_hyst_0,     act_ch_0,       NULL_ENTRY,     0,          ACT_CUR_0,          "Текущее");
    MAKE_MENU   (act_ch_1,      act_ch_2,       act_ch_0,       act_channels,   act_en_1,       2,          ACT_CH_1,           "Клапан вытяжной");
      MAKE_MENU (act_en_1,      act_set_1,      act_set_1,      act_ch_1,       EDITED_VAL,     0,          ACT_EN_1,           "Управление");
      MAKE_MENU (act_set_1,     act_en_1,       act_en_1,       act_ch_1,       EDITED_VAL,     0,          ACT_SET_1,          "Задано");
      //MAKE_MENU (act_hyst_1,    act_cur_1,      act_set_1,      act_ch_1,       EDITED_VAL,     0,          ACT_HYST_1,         "Гистерезис");
      //MAKE_MENU (act_cur_1,     act_en_1,       act_hyst_1,     act_ch_1,       NULL_ENTRY,     0,          ACT_CUR_1,          "Текущее");
    MAKE_MENU   (act_ch_2,      act_ch_3,       act_ch_1,       act_channels,   act_en_2,       4,          ACT_CH_2,           "Температура");
      MAKE_MENU (act_en_2,      act_set_2,      act_cur_2,      act_ch_2,       EDITED_VAL,     0,          ACT_EN_2,           "Управление");
      MAKE_MENU (act_set_2,     act_hyst_2,     act_en_2,       act_ch_2,       EDITED_VAL,     0,          ACT_SET_2,          "Задано");
      MAKE_MENU (act_hyst_2,    act_cur_2,      act_set_2,      act_ch_2,       EDITED_VAL,     0,          ACT_HYST_2,         "Гистерезис");
      MAKE_MENU (act_cur_2,     act_en_2,       act_hyst_2,     act_ch_2,       NULL_ENTRY,     0,          ACT_CUR_2,          "Текущее");
    MAKE_MENU   (act_ch_3,      act_ch_4,       act_ch_2,       act_channels,   act_en_3,       4,          ACT_CH_3,           "Влажность");
      MAKE_MENU (act_en_3,      act_set_3,      act_cur_3,      act_ch_3,       EDITED_VAL,     0,          ACT_EN_3,           "Управление");
      MAKE_MENU (act_set_3,     act_hyst_3,     act_en_3,       act_ch_3,       EDITED_VAL,     0,          ACT_SET_3,          "Задано");
      MAKE_MENU (act_hyst_3,    act_cur_3,      act_set_3,      act_ch_3,       EDITED_VAL,     0,          ACT_HYST_3,         "Гистерезис");
      MAKE_MENU (act_cur_3,     act_en_4,       act_hyst_3,     act_ch_3,       NULL_ENTRY,     0,          ACT_CUR_3,          "Текущее");
    MAKE_MENU   (act_ch_4,      act_ch_0,       act_ch_3,       act_channels,   act_en_4,       5,          ACT_CH_4,           "Авто дренаж");
      MAKE_MENU (act_en_4,      wtr_min_ref,    wtr_max_cur,    act_ch_4,       EDITED_VAL,     0,          ACT_EN_4,           "Управление");
      MAKE_MENU (wtr_min_ref,   wtr_min_cur,    act_en_4,       act_ch_4,       EDITED_VAL,     0,          WTR_MIN_REF,        "Мин. порог");
      MAKE_MENU (wtr_min_cur,   wtr_max_ref,    wtr_min_ref,    act_ch_4,       NULL_ENTRY,     0,          MEAS_CH_12,         "Мин. уровень");
      MAKE_MENU (wtr_max_ref,   wtr_max_cur,    wtr_min_cur,    act_ch_4,       EDITED_VAL,     0,          WTR_MAX_REF,        "Макс. порог");
      MAKE_MENU (wtr_max_cur,   act_en_4,       wtr_max_ref,    act_ch_4,       NULL_ENTRY,     0,          MEAS_CH_15,         "Макс. уровень");
  MAKE_MENU     (rele_channels, connection,     act_channels,   main_page,      rele_ch_0,      6,          RELE_CHANNELS,      "Релейные выходы");
    MAKE_MENU   (rele_ch_0,     rele_ch_1,      rele_ch_5,      rele_channels,  rele_auto_0,    2,          RELE_CH_0,          "Приточка");
      MAKE_MENU (rele_auto_0,   rele_cntrl_0,   rele_cntrl_0,   rele_ch_0,      EDITED_VAL,     0,          RELE_AUTO_MAN_0,    "Управление");
      MAKE_MENU (rele_cntrl_0,  rele_auto_0,    rele_auto_0,    rele_ch_0,      EDITED_VAL,     0,          RELE_CONTROL_0,     "Состояние");
    MAKE_MENU   (rele_ch_1,     rele_ch_2,      rele_ch_0,      rele_channels,  rele_auto_1,    2,          RELE_CH_1,          "Нагреватель");
      MAKE_MENU (rele_auto_1,   rele_cntrl_1,   rele_cntrl_1,   rele_ch_1,      EDITED_VAL,     0,          RELE_AUTO_MAN_1,    "Управление");
      MAKE_MENU (rele_cntrl_1,  rele_auto_1,    rele_auto_1,    rele_ch_1,      EDITED_VAL,     0,          RELE_CONTROL_1,     "Состояние");
    MAKE_MENU   (rele_ch_2,     rele_ch_3,      rele_ch_1,      rele_channels,  rele_auto_2,    2,          RELE_CH_2,          "Охладитель");
      MAKE_MENU (rele_auto_2,   rele_cntrl_2,   rele_cntrl_2,   rele_ch_2,      EDITED_VAL,     0,          RELE_AUTO_MAN_2,    "Управление");
      MAKE_MENU (rele_cntrl_2,  rele_auto_2,    rele_auto_2,    rele_ch_2,      EDITED_VAL,     0,          RELE_CONTROL_2,     "Состояние");
    MAKE_MENU   (rele_ch_3,     rele_ch_4,      rele_ch_2,      rele_channels,  rele_auto_3,    2,          RELE_CH_3,          "Конвекция");
      MAKE_MENU (rele_auto_3,   rele_cntrl_3,   rele_cntrl_3,   rele_ch_3,      EDITED_VAL,     0,          RELE_AUTO_MAN_3,    "Управление");
      MAKE_MENU (rele_cntrl_3,  rele_auto_3,    rele_auto_3,    rele_ch_3,      EDITED_VAL,     0,          RELE_CONTROL_3,     "Состояние");
    MAKE_MENU   (rele_ch_4,     rele_ch_5,      rele_ch_3,      rele_channels,  rele_auto_4,    2,          RELE_CH_4,          "Дренаж");
      MAKE_MENU (rele_auto_4,   rele_cntrl_4,   rele_cntrl_4,   rele_ch_4,      EDITED_VAL,     0,          RELE_AUTO_MAN_4,    "Управление");
      MAKE_MENU (rele_cntrl_4,  rele_auto_4,    rele_auto_4,    rele_ch_4,      EDITED_VAL,     0,          RELE_CONTROL_4,     "Состояние");
    MAKE_MENU   (rele_ch_5,     rele_ch_0,      rele_ch_4,      rele_channels,  rele_auto_5,    2,          RELE_CH_5,          "Резерв");
      MAKE_MENU (rele_auto_5,   rele_cntrl_5,   rele_cntrl_5,   rele_ch_5,      EDITED_VAL,     0,          RELE_AUTO_MAN_5,    "Управление");
      MAKE_MENU (rele_cntrl_5,  rele_auto_5,    rele_auto_5,    rele_ch_5,      EDITED_VAL,     0,          RELE_CONTROL_5,     "Состояние");
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
    selectedMenuItem = &main_page;
}

void menuChange(menuItem* NewMenu){
    if ((NewMenu != &NULL_ENTRY)&&(NewMenu != &EDITED_VAL)){
        selectedMenuItem = NewMenu;
    }else if(NewMenu == &EDITED_VAL){
        navigation_style = DIGIT_EDIT;
    }
}
