/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "../../opentx.h"

extern uint16_t ppmStream[NUM_MODULES][20];
volatile uint32_t ppmStreamIndex[NUM_MODULES] = { MODULES_INIT(0) };  // Modified in interrupt routine
extern uint8_t pxxStream[NUM_MODULES][64]; // TODO not here, duplicated
extern uint8_t *pxxStreamPtr[NUM_MODULES];  // TODO not here, duplicated
extern uint8_t dsm2Stream[64];  // Likely more than we need
extern uint8_t *dsm2StreamPtr;

void module_output_active()
{
  register Pio *pioptr = PIOA ;
  pioptr->PIO_ABCDSR[0] &= ~PIO_PA17 ;            // Peripheral C
  pioptr->PIO_ABCDSR[1] |= PIO_PA17 ;             // Peripheral C
  pioptr->PIO_PDR = PIO_PA17 ;                    // Disable bit A17 Assign to peripheral
#if defined(REVX)
  pioptr->PIO_MDER = PIO_PA17 ;                   // Open Drain O/p in A17
  pioptr->PIO_PUER = PIO_PA17 ;                   // With pull up
#endif
}


void init_main_ppm(uint32_t period, uint32_t out_enable)
{
  register Pwm *pwmptr ;

  setupPulsesPPM(EXTERNAL_MODULE) ;

  if (out_enable) {
    module_output_active();
  }

  pwmptr = PWM ;
  // PWM3 for PPM output
  pwmptr->PWM_CH_NUM[3].PWM_CMR = 0x0000000B ;                  // CLKA
  if (!g_model.moduleData[EXTERNAL_MODULE].ppmPulsePol)
    pwmptr->PWM_CH_NUM[3].PWM_CMR |= 0x00000200 ;               // CPOL
  pwmptr->PWM_CH_NUM[3].PWM_CPDR = period ;                     // Period in half uS
  pwmptr->PWM_CH_NUM[3].PWM_CPDRUPD = period ;                  // Period in half uS
  pwmptr->PWM_CH_NUM[3].PWM_CDTY = g_model.moduleData[EXTERNAL_MODULE].ppmDelay*100+600;    // Duty in half uS
  pwmptr->PWM_CH_NUM[3].PWM_CDTYUPD = g_model.moduleData[EXTERNAL_MODULE].ppmDelay*100+600; // Duty in half uS
  pwmptr->PWM_ENA = PWM_ENA_CHID3 ;                             // Enable channel 3
  pwmptr->PWM_IER1 = PWM_IER1_CHID3 ;

#if !defined(REVA) && !defined(REVX)
  // PWM1 for PPM2
  configure_pins(PIO_PC15, PIN_PERIPHERAL | PIN_INPUT | PIN_PER_B | PIN_PORTC | PIN_NO_PULLUP ) ;
  pwmptr->PWM_CH_NUM[1].PWM_CMR = 0x0000000B ;    // CLKB
  if (!g_model.moduleData[EXTRA_MODULE].ppmPulsePol)
    pwmptr->PWM_CH_NUM[1].PWM_CMR |= 0x00000200 ;   // CPOL
  pwmptr->PWM_CH_NUM[1].PWM_CPDR = period ;                       // Period
  pwmptr->PWM_CH_NUM[1].PWM_CPDRUPD = period ;            // Period
  pwmptr->PWM_CH_NUM[1].PWM_CDTY = g_model.moduleData[EXTRA_MODULE].ppmDelay*100+600 ;                             // Duty
  pwmptr->PWM_CH_NUM[1].PWM_CDTYUPD = g_model.moduleData[EXTRA_MODULE].ppmDelay*100+600 ;          // Duty
  pwmptr->PWM_ENA = PWM_ENA_CHID1 ;                                               // Enable channel 1
#endif

  pwmptr->PWM_IER1 = PWM_IER1_CHID1 ;
  NVIC_EnableIRQ(PWM_IRQn) ;
}

void init_no_pulses(uint32_t port)
{
  if (port == EXTERNAL_MODULE) {
    init_main_ppm(3000, 0);
  }
  else {
    // TODO
  }
}

void disable_no_pulses(uint32_t port)
{
  if (port == EXTERNAL_MODULE) {
    disable_ppm(0);
  }
  else {
    // TODO
  }
}

void init_ppm(uint32_t port)
{
  if (port == EXTERNAL_MODULE) {
    init_main_ppm(3000, 1);
  }
  else {
    // TODO
  }
}

void disable_ppm(uint32_t port)
{
  register Pio *pioptr ;

  if (port == EXTERNAL_MODULE) {
    pioptr = PIOA ;
    pioptr->PIO_PER = PIO_PA17 ;                                            // Assign A17 to PIO
    PWM->PWM_IDR1 = PWM_IDR1_CHID3 ;
  }
  else {
#if !defined(REVA) && !defined(REVX)
    pioptr = PIOC ;
    pioptr->PIO_PER = PIO_PC17 ;                                            // Assign A17 to PIO
    PWM->PWM_IDR1 = PWM_IDR1_CHID1 ;
    NVIC_DisableIRQ(PWM_IRQn) ;
#endif
  }
}

// Initialise the SSC to allow PXX output.
// TD is on PA17, peripheral A
void init_ssc()
{
  register Ssc *sscptr ;

  PMC->PMC_PCER0 |= 0x00400000L ;               // Enable peripheral clock to SSC

  configure_pins( PIO_PA17, PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTA | PIN_NO_PULLUP ) ;

  sscptr = SSC ;
  sscptr->SSC_CMR = Master_frequency / (125000*2) ;               // 8uS per bit
  sscptr->SSC_TCMR = 0 ;          //  0000 0000 0000 0000 0000 0000 0000 0000
  sscptr->SSC_TFMR = 0x00000027 ;         //  0000 0000 0000 0000 0000 0000 1010 0111 (8 bit data, lsb)
  sscptr->SSC_CR = SSC_CR_TXEN ;
}

void disable_ssc()
{
  register Pio *pioptr ;
  register Ssc *sscptr ;

  // Revert back to pwm output
  pioptr = PIOA ;
  pioptr->PIO_PER = PIO_PA17 ;                                         // Assign A17 to PIO

  sscptr = SSC ;
  sscptr->SSC_CR = SSC_CR_TXDIS ;
}

void init_pxx(uint32_t port)
{
  if (port == EXTERNAL_MODULE) {
    init_main_ppm(5000, 0);
    init_ssc();
  }
  else {
    // TODO
  }
}

void disable_pxx(uint32_t port)
{
  if (port == EXTERNAL_MODULE) {
    disable_ssc();
  }
  else {
    // TODO
  }
}

void init_dsm2(uint32_t port)
{
  if (port == EXTERNAL_MODULE) {
    init_main_ppm(5000, 0);
    init_ssc();
  }
  else {
    // TODO
  }
}

void disable_dsm2(uint32_t port)
{
  if (port == EXTERNAL_MODULE) {
    disable_ssc();
  }
  else {
    // TODO
  }
}

#if !defined(SIMU)
extern "C" void PWM_IRQHandler(void)
{
  register Pwm *pwmptr;
  register Ssc *sscptr;
  uint32_t period;
  uint32_t reason;

  pwmptr = PWM;
  reason = pwmptr->PWM_ISR1 ;
  if (reason & PWM_ISR1_CHID3) {
    // Use the current protocol, don't switch until set_up_pulses
    switch (s_current_protocol[EXTERNAL_MODULE]) {
      case PROTO_PXX:
        // Alternate periods of 15.5mS and 2.5 mS
        period = pwmptr->PWM_CH_NUM[3].PWM_CPDR;
        if (period == 5000) { // 2.5 mS
          period = 15500 * 2;
        }
        else {
          period = 5000;
        }
        pwmptr->PWM_CH_NUM[3].PWM_CPDRUPD = period; // Period in half uS
        if (period != 5000) { // 2.5 mS
          setupPulses(EXTERNAL_MODULE);
        }
        else {
          // Kick off serial output here
          sscptr = SSC;
          sscptr->SSC_TPR = CONVERT_PTR_UINT(pxxStream[EXTERNAL_MODULE]);
          sscptr->SSC_TCR = (uint8_t *)pxxStreamPtr[EXTERNAL_MODULE] - (uint8_t *)pxxStream[EXTERNAL_MODULE];
          sscptr->SSC_PTCR = SSC_PTCR_TXTEN; // Start transfers
        }
        break;

      case PROTO_DSM2_LP45:
      case PROTO_DSM2_DSM2:
      case PROTO_DSM2_DSMX:
        // Alternate periods of 19.5mS and 2.5 mS
        period = pwmptr->PWM_CH_NUM[3].PWM_CPDR;
        if (period == 5000) { // 2.5 mS
          period = 19500 * 2;
        }
        else {
          period = 5000;
        }
        pwmptr->PWM_CH_NUM[3].PWM_CPDRUPD = period; // Period in half uS
        if (period != 5000) { // 2.5 mS
          setupPulses(0);
        }
        else {
          // Kick off serial output here
          sscptr = SSC;
          sscptr->SSC_TPR = CONVERT_PTR_UINT(dsm2Stream);
          sscptr->SSC_TCR = (uint8_t *)dsm2StreamPtr - (uint8_t *)dsm2Stream;
          sscptr->SSC_PTCR = SSC_PTCR_TXTEN; // Start transfers
        }
        break;

      default:
        pwmptr->PWM_CH_NUM[3].PWM_CPDRUPD = ppmStream[EXTERNAL_MODULE][ppmStreamIndex[EXTERNAL_MODULE]++]; // Period in half uS
        if (ppmStream[EXTERNAL_MODULE][ppmStreamIndex[EXTERNAL_MODULE]] == 0) {
          ppmStreamIndex[EXTERNAL_MODULE] = 0;
          setupPulses(EXTERNAL_MODULE);
        }
        break;

    }
  }

  if (reason & PWM_ISR1_CHID1) {
    pwmptr->PWM_CH_NUM[1].PWM_CPDRUPD = ppmStream[EXTRA_MODULE][ppmStreamIndex[EXTRA_MODULE]++] ;  // Period in half uS
    if (ppmStream[EXTRA_MODULE][ppmStreamIndex[EXTRA_MODULE]] == 0) {
      ppmStreamIndex[EXTRA_MODULE] = 0;
      setupPulsesPPM(EXTRA_MODULE);
    }
  }
}
#endif
