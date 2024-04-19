///////////////////////////////////////////////////////////////////////////////
//
//  Roman Piksaykin [piksaykin@gmail.com], R2BDY
//  https://www.qrz.com/db/r2bdy
//
///////////////////////////////////////////////////////////////////////////////
//
//
//  main.c - The project entry point.
//
//  DESCRIPTION
//      The pico-WSPR-tx project provides WSPR beacon function using only
//  Pi Pico board. *NO* additional hardware such as freq.synth required.
//  External GPS receiver is optional and serves a purpose of holding
//  WSPR time window order and accurate frequancy drift compensation.
//
//  HOWTOSTART
//      ./build.sh; cp ./build/*.uf2 /media/Pico_Board/
//
//  PLATFORM
//      Raspberry Pi pico.
//
//  REVISION HISTORY
//      Rev 0.1   18 Nov 2023
//      Rev 0.5   02 Dec 2023
//
//  PROJECT PAGE
//      https://github.com/RPiks/pico-WSPR-tx
//
//  SUBMODULE PAGE
//      https://github.com/RPiks/pico-hf-oscillator
//
//  LICENCE
//      MIT License (http://www.opensource.org/licenses/mit-license.php)
//
//  Copyright (c) 2023 by Roman Piksaykin
//
//  Permission is hereby granted, free of charge,to any person obtaining a copy
//  of this software and associated documentation files (the Software), to deal
//  in the Software without restriction,including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "pico/multicore.h"
#include "pico-hf-oscillator/lib/assert.h"
#include "pico-hf-oscillator/defines.h"
#include <defines.h>
#include <piodco.h>
#include <WSPRbeacon.h>
#include <logutils.h>
#include <protos.h>
#include "pico/time.h" // Include for time_us_64() function

#define CONFIG_GPS_SOLUTION_IS_MANDATORY NO
#define CONFIG_GPS_RELY_ON_PAST_SOLUTION NO
#define CONFIG_SCHEDULE_SKIP_SLOT_COUNT 5
#define CONFIG_WSPR_DIAL_FREQUENCY 18106000UL //24926000UL // 28126000UL //7040000UL
#define CONFIG_CALLSIGN "KE0QHN"
#define CONFIG_LOCATOR4 "DM79"

WSPRbeaconContext *pWSPR;

int main()
{
    StampPrintf("\n");
    sleep_ms(5000);
    StampPrintf("R2BDY Pico-WSPR-tx start.");

    InitPicoHW();

    PioDco DCO = {0};

    StampPrintf("WSPR beacon init...");

    WSPRbeaconContext *pWB = WSPRbeaconInit(
        CONFIG_CALLSIGN,/* the Callsign. */
        CONFIG_LOCATOR4,/* the default QTH locator if GPS isn't used. */
        12,             /* Tx power, dbm. */
        &DCO,           /* the PioDCO object. */
        CONFIG_WSPR_DIAL_FREQUENCY,
        55UL,           /* the carrier freq. shift relative to dial freq. */
        RFOUT_PIN       /* RF output GPIO pin. */
        );
    assert_(pWB);
    pWSPR = pWB;

    pWB->_txSched._u8_tx_GPS_mandatory  = CONFIG_GPS_SOLUTION_IS_MANDATORY;
    pWB->_txSched._u8_tx_GPS_past_time  = CONFIG_GPS_RELY_ON_PAST_SOLUTION;
    pWB->_txSched._u8_tx_slot_skip      = CONFIG_SCHEDULE_SKIP_SLOT_COUNT;

    multicore_launch_core1(Core1Entry);
    StampPrintf("RF oscillator started.");

    DCO._pGPStime = GPStimeInit(0, 9600, GPS_PPS_PIN);
    assert_(DCO._pGPStime);

    int tick = 0;
    bool transmitting = false;
    uint64_t next_tx_time = 0;

    for(;;)
    {
        if (transmitting) {
            // Transmitting, so flash the LED
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);

            // Check if transmission is complete
            if (!TxChannelPending(pWB->_pTX)) {
                transmitting = false;
                PioDCOStop(pWB->_pTX->_p_oscillator);
                StampPrintf("Transmission complete.");

                // Set the next transmission time to 2 minutes from now
                next_tx_time = time_us_64() + 120000000;
            }
        } else {
            // Not transmitting, check if it's time to transmit again
            if (time_us_64() >= next_tx_time) {
                transmitting = true;
                StampPrintf("Transmitting...");
                PioDCOStart(pWB->_pTX->_p_oscillator);
                WSPRbeaconCreatePacket(pWB);
                sleep_ms(100);
                WSPRbeaconSendPacket(pWB);
            }
        }

#ifdef DEBUG
        if (0 == ++tick % 60)
            WSPRbeaconDumpContext(pWB);
#endif
        sleep_ms(100); // Adjust this delay as needed
    }
}
