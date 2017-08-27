//========================================================================
// File Name    : main.c
//
// Title        : HC-SR04を使った距離測定機器のメイン
// Revision     : 0.11
// Notes        :
// Target MCU   : AVR ATtiny 2313
// Tool Chain   :
//
// Revision History:
// When         Who         Description of change
// -----------  ----------- -----------------------
// 2014/11/01   ばんと      バックライトの点灯消灯の改良
// 2014/10/04   ばんと      製作開始
//------------------------------------------------------------------------
//【ピン配置】                    ___ ___
//                         RST#|  U  |Vcc
//                         TX  |     |PB7 ...
//                         RX  |     |PB6 ...
//                         Xtl2|  　 |PB5 ... LCD(RS)
//                         Xtl1|     |PB4 ... LCD(E)
//      HC-SR04(Echo)   .. INT0|     |PB3 ... LCD(DATA)
//              SW     ... INT1|     |PB2 ... "
//     HC-SR04(Trigger)... PD4 |     |PB1 ... "
//                     ... PD5 |     |PB0 ... "
//                         GND |     |PD6 ... LED
//                             +-----+
//                            ATTiny2313
//
// 【メモ】
//  ATtiny2313動作クロック速度は8Mhz。
//  内蔵のRC発振でもよいが、精度が良くないので、できれば水晶発振子か
//  セラロックを使用すること
//------------------------------------------------------------------------
// This code is distributed under Apache License 2.0 License
//        which can be found at http://www.apache.org/licenses/
//========================================================================

/* Includes -------------------------------------------------------------*/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "lcd_lib.h"
#include "xitoa.h"

/* local define ---------------------------------------------------------*/
#define BACKLIGHT_ON_TIMES 100

/* local typedef --------------------------------------------------------*/
/* local macro ----------------------------------------------------------*/
/* local variables ------------------------------------------------------*/

// 1/0 flag to check if echo is over
volatile uint8_t echoDone = 0;
// current timer0 count
volatile uint32_t countTimer0 = 0;

// バックライト遅延消灯タイミング(2014/11/01)
volatile uint8_t backlightTimes;

/* local function prototypes --------------------------------------------*/
void initHCSR04(void);
uint32_t getDistance(void);
void initBackLight(void);

/* [ここから割り込み関係] =============================================== */

/*======================================*/
/*  TIMER0 overflow interrupt           */
/*======================================*/
ISR(TIMER0_OVF_vect)
{
    // increment
    countTimer0 += 255;
}

/*======================================*/
/*  INT0 interrupt                      */
/*======================================*/
ISR(INT0_vect)
{
    // read INT0(PD2)
    if (PIND & (1 << PD2))
    {
        // rising edge:
        TCCR0B |= (1 << CS00); // 1分周でカウントスタート
        TIMSK |= (1 << TOIE0); // タイマ0のオーバーフロー割り込み許可
    }
    else
    {
        // falling edge
        TCCR0B &= ~(1 << CS00); // TIMER0割り込み ストップ
        countTimer0 += TCNT0;
        TCNT0 = 0;
        echoDone = 1;
    }
}

/*======================================*/
/*  INT1 interrupt                      */
/*======================================*/
ISR(INT1_vect)
{
    // バックライト遅延消灯タイミング更新(2014/11/01)
    if (!(PIND & (1 << PD3)))
    {
        backlightTimes = BACKLIGHT_ON_TIMES;
    }
}

/* =============================================== [ここまで割り込み関係] */

/* [ここからHC-SR04関係の関数] ========================================== */

/*======================================*/
/*  HC-SR04 関係PORT初期化              */
/*======================================*/
void initHCSR04(void)
{
    DDRD |= (1 << PD4);  // トリガー接続ポートを出力
    DDRD &= ~(1 << PD2); // ECHO受信接続ポートを入力
}

/*======================================*/
/*  距離取得関数                        */
/*======================================*/
uint32_t getDistance(void)
{
    uint32_t distance;

    cli();
    MCUCR |= (0 << ISC01) | (1 << ISC00); // INT0の論理変化
    GIMSK |= (1 << INT0);                 // INT0割り込み許可
    // set echo flag
    echoDone = 0;    // フラグクリア
    countTimer0 = 0; // カウンタの初期化
    sei();

    // send 10us trigger pulse
    //    _
    // __| |__
    PORTD &= ~(1 << PD4);
    _delay_us(20);
    PORTD |= (1 << PD4);
    _delay_us(12);
    PORTD &= ~(1 << PD4);
    _delay_us(20);

    // listen for echo and time it
    //       ____________
    // _____|            |___

    // loop till echo pin goes low
    while (!echoDone)
        ;

    // disable pin-change interrupt:
    // disable interrupt
    GIMSK &= ~(1 << INT0);

    // =========================================
    // 距離の演算方法
    //
    // 距離 = 掛った時間 * 音速 * 1/2
    //
    //
    // 掛った時間 = countTimer0/8000000.0;
    // 音速は340.26m　とすれば
    //   340.26 * 100 * 1 / 2 = 17013
    // なので、計算式は
    //    distance = (17013 * countTimer0)/8000000;  // 単位はcm

    distance = (17013 * countTimer0) / 800000; // 単位はmm

    return distance;
}
/* ===========================================[ここまでHC-SR04関係の関数] */

/* [ここからバックライト関係の関数] ====================================== */
/*======================================*/
/*  バックライト関係の初期化            */
/*======================================*/
void initBackLight(void)
{
    DDRD |= (1 << PD6);
    PORTD &= ~(1 << PD6); // バックライト点灯(2014/11/01)

    DDRD &= ~(1 << PD3);
    PORTD |= (1 << PD3);

    backlightTimes = BACKLIGHT_ON_TIMES;

    cli();
    MCUCR |= (0 << ISC11) | (1 << ISC10); // INT1の論理変化
    GIMSK |= (1 << INT1);                 // INT1割り込み許可
    sei();
}
/* =======================================[ここまでバックライト関係の関数] */

/* [ここからメイン関数] ================================================= */
/*======================================*/
/*  メイン                              */
/*======================================*/
int main(void)
{
    uint32_t distance;
    int temp1, temp2;

    // HC-SR04
    initHCSR04();

    // LCD INIT
    initBackLight();
    lcd_init();

    xdev_out(lcd_data);

    lcd_clear();
    lcd_pos(1, 1);
    xputs(PSTR("HC-SR04 "));
    lcd_pos(2, 1);
    xputs(PSTR("SENSOR! "));
    _delay_ms(1000);
    lcd_clear();

    while (1)
    {
        distance = getDistance();
        // センサの性能では計測距離は400cmまでらしい
        if (distance > 4000)
        {
            lcd_pos(1, 1);
            xputs(PSTR("Out of  "));
            lcd_pos(2, 1);
            xputs(PSTR(" Range!!"));
        }
        else
        {
            lcd_pos(1, 1);
            xputs(PSTR("Distance"));
            lcd_pos(2, 1);
            temp1 = distance / 10;
            temp2 = distance % 10;
            xprintf(PSTR("%3u.%u cm"), temp1, temp2);
        }

        // バックライト遅延消灯処理(2014/11/01)
        if (backlightTimes != 0)
        {
            PORTD &= ~(1 << PD6); // バックライト点灯
            backlightTimes--;
        }
        else
        {
            PORTD |= (1 << PD6); // バックライト消灯
        }

        _delay_ms(100);
    }
}
/* ================================================ [ここまでメイン関数]  */