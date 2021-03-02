/*
 * This file is part of the W1209 firmware replacement project
 * (https://github.com/mister-grumbler/w1209-firmware).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Control functions for analog-to-digital converter (ADC).
 * The ADC1 interrupt (22) is used to get signal on end of convertion event.
 * The port D6 (pin 3) is used as analog input (AIN6).
 */

#include "adc.h"
#include "stm8s003/adc.h"
#include "params.h"

#define NUMSAMPLES 256
static unsigned char waitAdc = 1;
static unsigned int results[NUMSAMPLES];
static int index;
static int valid;
static unsigned long averaged;
static unsigned int result;


/* ADC voltages in mV */
#define pressureZero 500
#define pressureMax 4500
/* max pressure in PSI/10 */
#define pressureSensorMax 1000

static int mvPerTenthPSI = ((pressureMax-pressureZero) / pressureSensorMax) ;


/**
 * @brief Initialize ADC's configuration registers.
 */
void initADC()
{
    ADC_CR1 |= 0x70;    // Prescaler f/18 (SPSEL)
    ADC_CSR |= 0x06;    // select AIN6
    ADC_CSR |= 0x20;    // Interrupt enable (EOCIE)
    ADC_CR1 |= 0x01;    // Power up ADC
    averaged = 0;
    valid = 0;
    index = 0;
    result = 0;
}

/**
 * @brief Sets bit in ADC control register to start data convertion.
 */
void startADC()
{
    ADC_CR1 |= 0x01;
}

/**
 * @brief Gets averaged over 2^ADC_AVERAGING_BITS times result of data
 *  convertion.
 * @return averaged result.
 */
unsigned int getAdcAveraged()
{
    if (!valid)
        return 0;
    return result;
    //return (unsigned int) (averaged >> 8);
}

/**
 * @brief Calculation of real temperature using averaged result of
 *  AnalogToDigital conversion and the lookup table.
 * @return temperature in tenth of degrees of Celsius.
 */
int getPressure()
{
    int voltage = getAdcAveraged() * 4.887;
    return (int)((voltage-pressureZero) / mvPerTenthPSI);
}

/**
 * @brief This function is ADC's interrupt request handler
 *  so keep it extremely small and fast.
 */
void ADC1_EOC_handler() __interrupt (22)
{
    results[index] = ADC_DRH << 2;
    results[index] |= ADC_DRL;
    ADC_CSR &= ~0x80;   // reset EOC

    if(waitAdc) {
      waitAdc--;
      return ;
    }

    averaged += results[index];

    index++;
    if (index == NUMSAMPLES) {
        index = 0;
        valid = 1;
    }

    averaged -= results[index];

    if (index == 0) {
        result = averaged >> 8;
    }
}
