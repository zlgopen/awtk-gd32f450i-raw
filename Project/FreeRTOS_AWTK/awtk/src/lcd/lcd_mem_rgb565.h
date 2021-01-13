﻿/**
 * File:   lcd_mem_rgb565.h
 * Author: AWTK Develop Team
 * Brief:  rgb565 mem lcd.
 *
 * Copyright (c) 2018 - 2021  Guangzhou ZHIYUAN Electronics Co.,Ltd.
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * license file for more details.
 *
 */

/**
 * history:
 * ================================================================
 * 2018-10-09 Generated by gen.sh(DONT MODIFY IT)
 *
 */

#ifndef TK_LCD_MEM_RGB565_H
#define TK_LCD_MEM_RGB565_H

#include "lcd/lcd_mem.h"

BEGIN_C_DECLS

/**
 * @class lcd_mem_rgb565_t
 * @parent lcd_t
 * @annotation ["fake"]
 */

/**
 * @method lcd_mem_rgb565_create
 *
 * 创建lcd对象。
 *
 * @param {wh_t} w 宽度。
 * @param {wh_t} h 高度。
 * @param {bool_t} alloc 是否分配内存。
 *
 * @return {lcd_t*} 返回lcd对象。
 */
lcd_t* lcd_mem_rgb565_create(wh_t w, wh_t h, bool_t alloc);

/**
 * @method lcd_mem_rgb565_create_single_fb
 *
 * 创建single fb lcd对象。
 *
 * @param {wh_t} w 宽度。
 * @param {wh_t} h 高度。
 * @param {uint8_t*} fbuff 帧率缓冲区。
 *
 * @return {lcd_t*} 返回lcd对象。
 */
lcd_t* lcd_mem_rgb565_create_single_fb(wh_t w, wh_t h, uint8_t* fbuff);

/**
 * @method lcd_mem_rgb565_create_double_fb
 *
 * 创建double fb lcd对象。
 *
 * @param {wh_t} w 宽度。
 * @param {wh_t} h 高度。
 * @param {uint8_t*} online_fb 在线帧率缓冲区。
 * @param {uint8_t*} offline_fb 离线帧率缓冲区。
 *
 * @return {lcd_t*} 返回lcd对象。
 */
lcd_t* lcd_mem_rgb565_create_double_fb(wh_t w, wh_t h, uint8_t* online_fb, uint8_t* offline_fb);

/**
 * @method lcd_mem_rgb565_create_three_fb
 * 
 * 创建three fb lcd对象。
 *
 * @param {wh_t} w 宽度。
 * @param {wh_t} h 高度。
 * @param {uint8_t*} online_fb 在线帧率缓冲区。
 * @param {uint8_t*} offline_fb 离线帧率缓冲区。
 * @param {uint8_t*} next_fb 待显示的帧率缓冲区。
 *
 * @return {lcd_t*} 返回lcd对象。
 */
lcd_t* lcd_mem_rgb565_create_three_fb(wh_t w, wh_t h, uint8_t* online_fb, uint8_t* offline_fb,
                                      uint8_t* next_fb);

END_C_DECLS

#endif /*TK_LCD_MEM_RGB565_H*/
