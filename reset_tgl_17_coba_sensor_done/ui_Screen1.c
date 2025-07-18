// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: hmi screen 3 manual

#include "ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0xF6FFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel4 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel4, 750);
    lv_obj_set_height(ui_Panel4, 440);
    lv_obj_set_x(ui_Panel4, 1);
    lv_obj_set_y(ui_Panel4, -1);
    lv_obj_set_align(ui_Panel4, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel4, lv_color_hex(0xF6FFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button1 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Button1, 165);
    lv_obj_set_height(ui_Button1, 45);
    lv_obj_set_x(ui_Button1, -225);
    lv_obj_set_y(ui_Button1, 58);
    lv_obj_set_align(ui_Button1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0x0050AC), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_Button1);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "Lets Get Started");
    lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image5 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image5, &ui_img_pojok_kiri_atas_screen_1_png);
    lv_obj_set_width(ui_Image5, LV_SIZE_CONTENT);   /// 141
    lv_obj_set_height(ui_Image5, LV_SIZE_CONTENT);    /// 122
    lv_obj_set_x(ui_Image5, -303);
    lv_obj_set_y(ui_Image5, -159);
    lv_obj_set_align(ui_Image5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image5, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image7 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image7, &ui_img_pojok_kiri_bawah_png);
    lv_obj_set_width(ui_Image7, LV_SIZE_CONTENT);   /// 252
    lv_obj_set_height(ui_Image7, LV_SIZE_CONTENT);    /// 65
    lv_obj_set_x(ui_Image7, -248);
    lv_obj_set_y(ui_Image7, 185);
    lv_obj_set_align(ui_Image7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image7, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image8 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image8, &ui_img_pojok_kanan_bawah_screen_1_png);
    lv_obj_set_width(ui_Image8, LV_SIZE_CONTENT);   /// 63
    lv_obj_set_height(ui_Image8, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_x(ui_Image8, 343);
    lv_obj_set_y(ui_Image8, 186);
    lv_obj_set_align(ui_Image8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image8, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image9 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image9, &ui_img_pojok_kanan_atas_png);
    lv_obj_set_width(ui_Image9, LV_SIZE_CONTENT);   /// 293
    lv_obj_set_height(ui_Image9, LV_SIZE_CONTENT);    /// 55
    lv_obj_set_x(ui_Image9, 228);
    lv_obj_set_y(ui_Image9, -193);
    lv_obj_set_align(ui_Image9, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image9, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image9, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image10 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image10, &ui_img_gambar_orang_new_screen1_png);
    lv_obj_set_width(ui_Image10, LV_SIZE_CONTENT);   /// 313
    lv_obj_set_height(ui_Image10, LV_SIZE_CONTENT);    /// 166
    lv_obj_set_x(ui_Image10, 146);
    lv_obj_set_y(ui_Image10, 11);
    lv_obj_set_align(ui_Image10, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image10, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image10, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image11 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image11, &ui_img_font_smart_handglove_png);
    lv_obj_set_width(ui_Image11, LV_SIZE_CONTENT);   /// 315
    lv_obj_set_height(ui_Image11, LV_SIZE_CONTENT);    /// 45
    lv_obj_set_x(ui_Image11, -150);
    lv_obj_set_y(ui_Image11, -25);
    lv_obj_set_align(ui_Image11, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image11, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image11, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    lv_obj_add_event_cb(ui_Button1, ui_event_Button1, LV_EVENT_ALL, NULL);

}
