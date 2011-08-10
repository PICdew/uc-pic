typedef unsigned char u8;
typedef signed char s8;

#define VERSION "LCD display v.2"

#if defined(_16F526)
#warning Building for 16F526

#define _XTAL_FREQ 8000000
#include <pic.h>

__CONFIG(WDTDIS & OSC_8MHZ & INTIO & UNPROTECT & MCLRDIS); /* Internal 8 Mhz osc */

#define LCD_REG_BIT     RC0
#define LCD_REG_CLK     RC1
#define LCD_REG_BUSY    RC0
#define LCD_REG_E       RC2

#define IN_BUSY         RC5
#define IN_0_5          PORTB
#define IN_5            RB5
#define IN_6            RC4
#define IN_CLR_N        RC3

#define TRISB_MASK      0b111111
#define TRISC_MASK      0b110000
#else
#error Unsupported chip type
#endif

#if 1
/* From http://www.microchipc.com/sourcecode/#delay16x for 8Mhz clk */
u8 __delay_us;
#define DelayDivisor 2
#define WaitFor1Us asm("nop")
#define Jumpback asm("goto $ - 2")
#define DelayUs(x) {                                      \
        __delay_us = (u8)(x/DelayDivisor);                \
        WaitFor1Us; }                                     \
        asm("decfsz ___delay_us, f");                     \
        Jumpback;

void DelayMs(u8 cnt)
{
	u8 i;
	do {
		i = 4;
		do {
			DelayUs(250);
			CLRWDT();
		} while (--i);
	} while (--cnt);
}

#define udelay DelayUs
#define mdelay DelayMs
#else
#define udelay __delay_us
#define mdelay __delay_ms
#endif

/*  */
#define lcd_start_operation() do {              \
        LCD_REG_E = 1;                          \
    } while (0)
/*  */
#define lcd_stop_operation() do {               \
        LCD_REG_E = 0;                          \
    } while (0)
/*  */
#define lcd_push_bit() do {                     \
        LCD_REG_CLK = 1;                        \
        LCD_REG_CLK = 0;                        \
    } while (0)
#define lcd_put_bit(val) do {                   \
        LCD_REG_BIT = (val);                    \
        lcd_push_bit();                         \
    } while (0)
/*  */
#define lcd_put_bits(rs, rw, b3, b2, b1, b0) do {       \
        lcd_put_bit(b3);                                \
        lcd_put_bit(b2);                                \
        lcd_put_bit(b1);                                \
        lcd_put_bit(b0);                                \
        lcd_put_bit(rw);                                \
        lcd_put_bit(rs);                                \
    } while (0)
/*  */

enum lcd_commands {
    LCD_CLEAR_CMD           = (1<<0),
    LCD_RETURN_HOME_CMD     = (1<<1),
    LCD_ENTRY_MODE_CMD      = (1<<2),
    LCD_DISPLAY_CURSOR_CMD  = (1<<3),
    LCD_SHIFT_CTRL_CMD      = (1<<4),
    LCD_FUNCTION_CTRL_CMD   = (1<<5),
    LCD_CGRAM_ADDRESS_CMD   = (1<<6),
    LCD_DDRAM_ADDRESS_CMD   = (1<<7),
};

#define lcd_clear_screen()                          lcd_set_instruction(LCD_CLEAR_CMD)
#define lcd_return_home()                           lcd_set_instruction(LCD_RETURN_HOME_CMD)
#define lcd_entry_mode(inc, shifted)                lcd_set_instruction(LCD_ENTRY_MODE_CMD|((inc)<<1)|((shifted)<<0))
#define lcd_display_cursor(on, cursor, blink)       lcd_set_instruction(LCD_DISPLAY_CURSOR_CMD|((on)<<2)|((cursor)<<1)|((blink)<<0))
#define lcd_shift_ctrl(disp_shift, right)           lcd_set_instruction(LCD_SHIFT_CTRL_CMD|((disp_shift)<<3)|((right)<<2))
#define lcd_function_ctrl(i_8_b, d_2_l, big)        lcd_set_instruction(LCD_FUNCTION_CTRL_CMD|((i_8_b)<<4)|((d_2_l)<<3)|((big)<<2))
#define lcd_set_address(a)                          lcd_set_instruction(LCD_DDRAM_ADDRESS_CMD|(a))

#if 1
void lcd_wait_busy(void)
{
    u8 bb;

    lcd_put_bits(0, 1, 0, 0, 0, 0);

    TRISC = TRISC_MASK | (1<<0);

    do {
        lcd_start_operation();
        NOP(); NOP(); NOP(); NOP();
        bb = LCD_REG_BUSY;
        lcd_stop_operation();

        lcd_start_operation();
        lcd_stop_operation();
    } while (bb);

    TRISC = TRISC_MASK & ~(1<<0);
}
#else
#define lcd_wait_busy() mdelay(8)
#endif

void lcd_set_instruction(u8 i)
{
    lcd_wait_busy();

    lcd_put_bits(0, 0, (i>>7)&0x1, (i>>6)&0x1, (i>>5)&0x1, (i>>4)&0x1);
    lcd_start_operation();
    lcd_stop_operation();

    lcd_put_bits(0, 0, (i>>3)&0x1, (i>>2)&0x1, (i>>1)&0x1, (i>>0)&0x1);
    lcd_start_operation();
    lcd_stop_operation();
}

void lcd_putc(u8 c)
{
    lcd_wait_busy();

    lcd_put_bits(1, 0, (c>>7)&0x1, (c>>6)&0x1, (c>>5)&0x1, (c>>4)&0x1);
    lcd_start_operation();
    lcd_stop_operation();

    lcd_put_bits(1, 0, (c>>3)&0x1, (c>>2)&0x1, (c>>1)&0x1, (c>>0)&0x1);
    lcd_start_operation();
    lcd_stop_operation();
}

#define lcd_puts(s) do {                        \
        const u8 *__s = (s);                    \
                                                \
        while (*__s) {                          \
            lcd_putc(*__s);                     \
            ++__s;                              \
        }                                       \
    } while (0)

#if defined(_16F526)
#define platform_setup() do {                                   \
        OSCCAL = 0b01111110; /* Max freq. */                    \
                                                                \
        OPTION = 0b11010111;                                    \
                                                                \
        ADCON0 = 0b00111100; /* Disable A/D */                  \
        STATUS = 0b00011111; /* Clear status */                 \
        CM1CON0 = 0b11110111; /* Disable comparator 1 */        \
        CM2CON0 = 0b11110111; /* Disable comparator 2 */        \
                                                                \
        TRISB = TRISB_MASK; /* RB{0,1,2,3,4,5} - IN */                  \
        TRISC = TRISC_MASK; /* RC{0,1,2,3} - OUT, RC{4,5} - IN */       \
                                                                        \
        PORTB = 0;                                                      \
        PORTC = 0;                                                      \
        IN_CLR_N = 1;                                                   \
    } while (0)

#if 0
#define get_opcode()    ((IN_6<<1)|IN_5)
#define get_qualifier() (IN_0_5&0x1F)
#else
#define get_opcode(b)    ((b)>>5)
#define get_qualifier(b) ((b)&0x1F)
#endif
#define get_bits()      (IN_0_5|(IN_6<<6))
#define toggle_busy()   do { IN_CLR_N = 0, IN_CLR_N = 1; } while (0)
#endif

#define lcd_setup(multiline, font) do {         \
        mdelay(50);                             \
                                                \
        /* set 4-bit interface */               \
        lcd_wait_busy();                        \
        lcd_put_bits(0, 0, 0, 0, 1, 0);         \
        lcd_start_operation();                  \
        lcd_stop_operation();                   \
                                                \
        lcd_function_ctrl(0, multiline, big);   \
                                                \
        lcd_display_cursor(1, 0, 0);            \
        lcd_entry_mode(1, 0);                   \
                                                \
        lcd_clear_screen();                     \
        lcd_return_home();                      \
    } while (0)                                 \

#define get_byte(b) do {                        \
        while (IN_BUSY == 0) NOP();             \
                                                \
        b = get_bits();                         \
                                                \
        toggle_busy();                          \
    } while (0)

/*
 * packet structure:
 *   [header:8b][payload:variable length]
 *
 * header structure:
 * [1, x, x, y, y, y, y, y]
 *  bit[7]   : must be always set to 1
 *  bit[6:5] : opcode
 *  bit[4:0] : qualifier
 *
 * opcodes:
 *  00b: string to be displayed
 *   qualifier:
 *    bit[4:0]: length of payload[up to 32]
 *    payload : string to be displayed
 *  01b: lcd config[no payload]
 *   qualifier:
 *    00000b: 1x80
 *    00001b: 2x40
 *    00010b: 4x20
 *    00011b: big font
 *  10b: goto command
 *   qualifier:
 *    bit[4:0]: row
 *    payload : col
 *  11b: generic commands[no payload]
 *   qualifier:
 *    00000b: show version
 *    00001b: clear screen
 */

u8 string0[16];
u8 string1[16];

/* hd44780 supports only 2 lines max by design[line 1 has range 0x0:0x3F, line 2 has range 0x40:7F]
 * in case of 4-lines display 3rd line is continuation of line 1, 4th line is a continuation of line 2
 * thus 4-lines display can be max of 20 chars wide
 */
enum lcdSize {
    LCD_1x80,
    LCD_2x40,
    LCD_4x20,
};

const u8 line_offsets[] = {0x00, 0x40, 0x14, 0x54};

void main(void)
{
    u8 size = LCD_4x20, big = 0;
    u8 opcode, qualifier;
    u8 b;

    platform_setup();
    lcd_setup(size!=LCD_1x80, big);

    for (;;) {
        get_byte(b);

        opcode = get_opcode(b);
        qualifier = get_qualifier(b);

        if (opcode == 0) {      /* string */
            s8 len = qualifier, i;
            for (i=0;i<len;++i) {
                get_byte(b);

                if (i < 16)
                    string0[i] = b;
                else if (i < 32)
                    string1[i-16] = b;
            }

            for (i=0;i<len;++i) {
                if (i < 16)
                    lcd_putc(string0[i]);
                else if (i < 32)
                    lcd_putc(string1[i-16]);
            }
        } else if (opcode == 1) {
            if (qualifier == 0 || qualifier == 1 || qualifier == 3) { /* size */
                size = qualifier;
                if (size != LCD_1x80)
                    big = 0;
                lcd_function_ctrl(1, size!=LCD_1x80, big);
            } else if (qualifier == 2) { /* big font */
                if (size != LCD_1x80)
                    big = 1;
                lcd_function_ctrl(1, size!=LCD_1x80, big);
            }
        } else if (opcode == 2) { /* goto */
            get_byte(b);
            lcd_set_address(line_offsets[qualifier] + b);
        } else if (opcode == 3) { /* commands */
            if (qualifier == 0) { /* show version */
                lcd_puts(VERSION);
            } else if (qualifier == 1) { /* clear screen */
                lcd_clear_screen();
            }
        }
    }
}
