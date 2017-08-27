//=======================================================
// File Name    : lcd_lib.h
//
// Title        : AVR LCD ライブラリィヘッダ
// Revision     : 0.1
// Notes        :
// Target MCU   : Atmel AVR series
// Tool Chain   :
//
// Revision History:
// When         Who         Description of change
// -----------  ----------- -----------------------
// 2012/02/19   ばんと      開発開始
// 2011/02/19   ばんと      Ver0.1完了
//=======================================================

#ifndef _LCD_LIB_H_
#define _LCD_LIB_H_

#include <inttypes.h>

/* ===[使用LCDの選択]================================== */
/* SC1602    : USE_SC1602                               */
/* ACM0802C  : USE_ACM0802C                             */
/* デフォルト: USE_SC1602                               */
/* ==================================================== */
/* ↓これを修正 */
//#define USE_ACM0802C
#define USE_SC1602

/* ===[接続ポートの上位下位ビット設定]================= */
/* 上位4ビットがデータポート : USE_HIGH_NIBBLE          */
/* 下位4ビットがデータポート : USE_LOW_NIBBLE           */
/* デフォルト  : USE_LOW_NIBBLE                         */
/* ==================================================== */
/* ↓これを修正 */
#define USE_LOW_NIBBLE
//#define USE_HIGH_NIBBLE

/* ===[使用ポートの宣言]=============================== */
/*  LCDのDB4?Db7をポートの上位4ビットまたは            */
/*  下位4ビットに接続すること                           */
/* ==================================================== */
/* ↓これを修正 */
#define DATA_DDR        DDRB
#define DATA_PORT       PORTB

#define CTRL_DDR        DDRB
#define CTRL_PORT       PORTB
#define LCD_RS          PB5
#define LCD_E           PB4

#define LINE2
//#define LINE4


/* ====[以降は変更する必要なし]======================== */

/* ==================================================== */
/*  型番宣言選択                                        */
/* ==================================================== */
#ifdef USE_SC1602
#undef USE_ACM0802C
#elif defined USE_ACM0802C
#undef USE_SC1602
#else
#define USE_SC1602
#endif

/* ==================================================== */
/*  上位4ビット下位4ビット宣言選択                      */
/* ==================================================== */
#ifdef USE_HIGH_NIBBLE
#undef USE_LOW_NIBBLE
#elif defined USE_LOW_NIBBLE
#undef USE_HIGH_NIBBLE
#else
#define USE_HIGH_NIBBLE
#endif

/* ==================================================== */
/*  変数定義                                            */
/* ==================================================== */
#define RS_OFF  0
#define RS_ON   1


/* ==================================================== */
/*  関数プロトタイプ                                    */
/* ==================================================== */
void lcd_init(void);
void lcd_out(uint8_t code, uint8_t rs);
void lcd_cmd(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_puts(const char *s);
void lcd_puts_p(const char *progmem_s);
void lcd_pos(uint8_t line, uint8_t col);

/* ==================================================== */
/*  マクロ関数                                          */
/* ==================================================== */
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_home()          lcd_cmd(0x02)
#define lcd_display_on()    lcd_cmd(0x0C)
#define lcd_display_off()   lcd_cmd(0x08)

#endif
