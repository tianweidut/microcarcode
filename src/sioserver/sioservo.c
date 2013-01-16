/****************************************************************************/
/* Servo center adjustment from PC "sioservo.c"                             */
/*         May 2007, Renesas Technology Micom Car Rally Executive Committee */
/****************************************************************************/
/*
This is a program to check the servo adjustment of the micom car using keys.
The action described takes place when one of the following keys is pressed.

Z key: servo offset value +1
X key: servo offset value -1
A key: servo offset value +10
S key: servo offset value -10

Connections are as follows;
* Transfer cable <- -> PC (running Tera Term Pro or HyperTerminal communication software)
* PB <- motor drive board (Vol. 3) <- servo

*/

/*======================================*/
/* Include                              */
/*======================================*/
#include    <no_float.h>                /* Simplifies stdio, place at beginning */
#include    <stdio.h>
#include    <machine.h>
#include    "h8_3048.h"

/*======================================*/
/* Symbol definitions                   */
/*======================================*/

/*======================================*/
/* Prototype declarations               */
/*======================================*/
void init( void );

/*======================================*/
/* Global variable declarations         */
/*======================================*/
unsigned int    servo_offset;           /* Servo offset             */

/************************************************************************/
/* Main program                                                         */
/************************************************************************/
void main( void )
{
    int     i, ret;
    char    c;

    /* Initialize MCU functions */
    init();                             /* Initialize               */
    init_sci1( 0x00, 79 );              /* SCI1 initialize          */
    set_ccr( 0x00 );                    /* Enable all interrupts    */

    servo_offset = 5000;
    printf(
        "Servo Center Adjustment Soft\n"
        "'Z' key   : Center Value +1\n"
        "'X' key   : Center Value -1\n"
        "\n"
        "'A' key   : Center Value +10\n"
        "'S' key   : Center Value -10\n"
        "\n"
    );
    printf( "%5d\r", servo_offset );

    while( 1 ) {
        ITU4_BRB = servo_offset;

        i = get_sci( &c );
        if( i == 1 ) {
            switch( c ) {
            case 'Z':
            case 'z':
                servo_offset++;
                if( servo_offset > 10000 ) servo_offset = 10000;
                printf( "%5d\r", servo_offset );
                break;

            case 'A':
            case 'a':
                servo_offset += 10;
                if( servo_offset > 10000 ) servo_offset = 10000;
                printf( "%5d\r", servo_offset );
                break;

            case 'X':
            case 'x':
                servo_offset--;
                if( servo_offset < 1000 ) servo_offset = 1000;
                printf( "%5d\r", servo_offset );
                break;

            case 'S':
            case 's':
                servo_offset -= 10;
                if( servo_offset < 1000 ) servo_offset = 1000;
                printf( "%5d\r", servo_offset );
                break;

            default:
                break;
            }
        }
    }
}

/************************************************************************/
/* Initialize H8/3048F-ONE on-chip peripheral functions                 */
/************************************************************************/
void init( void )
{
    /* Port I/O settings */
    P1DDR = 0xff;
    P2DDR = 0xff;
    P3DDR = 0xff;
    P4DDR = 0xff;
    P5DDR = 0xff;
    P6DDR = 0xf0;                       /* DIP switches on CPU board    */
    P8DDR = 0xff;
    P9DDR = 0xf7;                       /* Communication ports          */
    PADDR = 0xff;
    PBDR  = 0xc0;
    PBDDR = 0xfe;                       /* Motor drive board, Vol. 3    */
    /* Pin 7 on the sensor board is input-only, so no I/O setting is needed. */

    /* ITU3 and ITU4 reset synchronized PWM modes for left and right motors and servo */
    ITU3_TCR = 0x23;                    /* Clear counter setting        */
    ITU_FCR  = 0x3e;                    /* Reset synchronized PWM mode  */
    ITU3_GRA = 49151;                   /* Period setting               */
    ITU3_GRB = ITU3_BRB = 0;            /* Left motor PWM setting       */
    ITU4_GRA = ITU4_BRA = 0;            /* Right motor PWM setting      */
    ITU4_GRB = ITU4_BRB = 5000;         /* Servo PWM setting            */
    ITU_TOER = 0x38;                    /* Output pin setting           */
    ITU_STR  = 0x08;                    /* Timer start                  */
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/