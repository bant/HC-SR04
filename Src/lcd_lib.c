//=======================================================
// File Name    : lcd_lib.c
//
// Title        : AVR LCD ライブラリィ
// Revision     : 0.1
// Notes        :
// Target MCU   : Atmel AVR series
// Tool Chain   :
//
// Revision History:
// When         Who         Description of change
// -----------  ----------- -----------------------
// 2012/02/19   ばんと      開発開始
// 2012/02/19   ばんと      Ver0.1完了
//=======================================================

/* Includes ------------------------------------------------------------------*/
#include "lcd_lib.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* macro ---------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/

/* functions -----------------------------------------------------------------*/

/*======================================*/
/*  LCD接続ポートの初期化               */
/*======================================*/
void lcd_init_port(void)
{
#ifdef USE_LOW_NIBBLE
    DATA_DDR |= 0x0F;
    DATA_PORT &= ~0x0F;
#else
    DATA_DDR |= 0xF0;
    DATA_PORT &= ~0xF0;
#endif
    CTRL_DDR |= (_BV(LCD_RS) | _BV(LCD_E));
    CTRL_PORT &= ~(_BV(LCD_RS) | _BV(LCD_E));
}

/*======================================*/
/*  LCDデータ出力関数                   */
/*======================================*/
void lcd_out(uint8_t code, uint8_t rs)
{
#ifdef USE_LOW_NIBBLE
    DATA_PORT = (code & 0x0F) | (DATA_PORT & 0xF0);
#else
    DATA_PORT = (code & 0xF0) | (DATA_PORT & 0x0F);
#endif
    if (rs == RS_ON)
    {
        CTRL_PORT |= _BV(LCD_RS);
    }
    else
    {
        CTRL_PORT &= ~_BV(LCD_RS);
    }

    _delay_ms(1);
    CTRL_PORT |= _BV(LCD_E);
    _delay_ms(1);
    CTRL_PORT &= ~_BV(LCD_E);
}

/*======================================*/
/*  4ビットコマンド出力関数             */
/*======================================*/
void lcd_cmd(uint8_t cmd)
{
#ifdef USE_LOW_NIBBLE
    lcd_out(cmd >> 4, RS_OFF);
    lcd_out(cmd, RS_OFF);
#else
    lcd_out(cmd, RS_OFF);
    lcd_out(cmd << 4, RS_OFF);
#endif

    if (cmd & 0xFC)
    {
        _delay_us(50);
    }
    else
    {
        _delay_ms(2);
    }
}

/*======================================*/
/*  4ビットデータ出力関数               */
/*======================================*/
void lcd_data(uint8_t data)
{
#ifdef USE_LOW_NIBBLE
    lcd_out(data >> 4, RS_ON);
    lcd_out(data, RS_ON);
#else
    lcd_out(data, RS_ON);
    lcd_out(data << 4, RS_ON);
#endif

    _delay_us(50);
}

/*======================================*/
/*  4ビットデータ出力関数               */
/*======================================*/
void lcd_init(void)
{
    /* ポートの初期化 */
    lcd_init_port();
#ifdef USE_LOW_NIBBLE
#ifdef USE_ACM0802C
    /* ACM0802C */

    _delay_ms(35); // 30mS以上のウェイト(実際は35ms)

    lcd_out(0x02, RS_OFF); // Function set
    lcd_out(0x02, RS_OFF); //
    lcd_out(0x08, RS_OFF); //

    _delay_us(50); // 39us以上のウェイト(実際は50us)

    lcd_out(0x00, RS_OFF); // Display ON/OFF Command
    lcd_out(0x0C, RS_OFF); //

    _delay_us(50); // 39us以上のウェイト(実際は50us)

    lcd_out(0x00, RS_OFF); // Display Clear
    lcd_out(0x01, RS_OFF); //

    _delay_ms(2); // 1.53ms以上のウェイト(実際は2ms)

    lcd_out(0x00, RS_OFF); // Entry Mode Set
    lcd_out(0x06, RS_OFF); //

    _delay_ms(50); //
#else
    /* 1602 */
    _delay_ms(15); // 15ms以上のウェイト

    lcd_out(0x03, RS_OFF); //
    _delay_ms(5);          // 4.1ms以上のウェイト

    lcd_out(0x03, RS_OFF); //
    _delay_us(150);        // 100us以上のウェイト

    lcd_out(0x03, RS_OFF); //
    _delay_us(50);         // 40us以上のウェイト

    lcd_out(0x02, 0); // 4ビットモード
    _delay_us(100);

    lcd_cmd(0x28);
    lcd_cmd(0x08);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x02);
    lcd_cmd(0x01);
#endif
#else
#ifdef USE_ACM0802C
    /* ACM0802C */

    _delay_ms(35); // 30mS以上のウェイト(実際は35ms)

    lcd_out(0x20, RS_OFF); // Function set
    lcd_out(0x20, RS_OFF); //
    lcd_out(0x80, RS_OFF); //

    _delay_us(50); // 39us以上のウェイト(実際は50us)

    lcd_out(0x00, RS_OFF); // Display ON/OFF Command
    lcd_out(0xC0, RS_OFF); //

    _delay_us(50); // 39us以上のウェイト(実際は50us)

    lcd_out(0x00, RS_OFF); // Display Clear
    lcd_out(0x10, RS_OFF); //

    _delay_ms(2); // 1.53ms以上のウェイト(実際は2ms)

    lcd_out(0x00, RS_OFF); // Entry Mode Set
    lcd_out(0x60, RS_OFF); //

    _delay_ms(50); //
#else
    /* 1602 */
    _delay_ms(15); // 15ms以上のウェイト

    lcd_out(0x30, RS_OFF); //
    _delay_ms(5);          // 4.1ms以上のウェイト

    lcd_out(0x30, RS_OFF); //
    _delay_us(150);        // 100us以上のウェイト

    lcd_out(0x30, RS_OFF); //
    _delay_us(50);         // 40us以上のウェイト

    lcd_out(0x20, 0); // 4ビットモード
    _delay_us(100);

    lcd_cmd(0x28);
    lcd_cmd(0x08);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x02);
    lcd_cmd(0x01);
#endif
#endif
}

/*======================================*/
/*  文字例出力関数                      */
/*======================================*/
void lcd_puts(const char *s)
{
    register char c;

    while ((c = *s++))
    {
        lcd_data(c);
    }
}

/*======================================*/
/*  文字例出力関数2                     */
/*======================================*/
void lcd_puts_p(const char *progmem_s)
/* print string from program memory on lcd (no auto linefeed) */
{
    register char c;

    while ((c = pgm_read_byte(progmem_s++)))
    {
        lcd_data(c);
    }

} /* lcd_puts_p */

/*======================================*/
/*  表示位置指定関数                    */
/*======================================*/
void lcd_pos(uint8_t line, uint8_t col)
{
    uint8_t pos;

    switch (line)
    {
        case 1:
            pos = 0x80;
            break;
        case 2:
            pos = 0xC0;
            break;
#ifdef LINE4
        case 3:
            pos = 0x94;
            break;
        case 4:
            pos = 0xD4;
            break;
#endif
        default:
            pos = 0x80;
            break;
    }
    lcd_cmd(pos + col - 1);
}
