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
 * Control functions for relay.
 */

#include "relay.h"
#include "stm8s003/gpio.h"
#include "adc.h"
#include "params.h"

#define RELAY_PORT              PA_ODR
#define RELAY_BIT               0x04
#define RELAY_TIMER_MULTIPLIER  7

static unsigned int timer;
static bool state;

static unsigned char force ;

/**
 * @brief Configure appropriate bits for GPIO port A, reset local timer
 *  and reset state.
 */
void initRelay()
{
    PA_DDR |= RELAY_BIT;
    PA_CR1 |= RELAY_BIT;
    timer = 0;
    state = false;
    force = RELAY_FORCE_NA;
}

/**
 * @brief Sets state of the relay to force on, off or n/a.
 * @param rf - RELAY_FORCE_ON:ON, RELAY_FORCE_OFF:OFF, RELAY_FORCE_NA:N/A
 */
void setRelayForce (unsigned char rf)
{
  force = rf;
}

/**
 * @brief Sets state of the relay.
 * @param on - true, off - false
 */
void setRelay (bool on)
{
    if (on) {
        RELAY_PORT |= RELAY_BIT;
    } else {
        RELAY_PORT &= ~RELAY_BIT;
    }

}

/**
 * @brief This function is being called during timer's interrupt
 *  request so keep it extremely small and fast.
 */
void refreshRelay()
{
    bool mode = getParamById (PARAM_RELAY_MODE);

    int temp = getPressure();
    int hold = getParamById (PARAM_THRESHOLD);
    int hyst = getParamById (PARAM_RELAY_HYSTERESIS);

    // pressure readings while operating are highter by this formula
    // Found by taking readings and plotting. 
    // TODO: Make this configurable OR auto-calibrate somehow.
    int on_hold = hold * 0.65 + 200;

    // overheat protection
    if (getParamById (PARAM_OVERHEAT_INDICATION) ) {
        if ( temp < getParamById (PARAM_MIN_TEMPERATURE) *10 /*LLL*/ ||
             temp > getParamById (PARAM_MAX_TEMPERATURE) *10 /*HHH*/ ) {
            setRelay (false);
            timer = 0 ;
            return; // overheat or too cold
        }
    }

    if(mode) { // Hot
      // inverse
      temp = - temp ;
      hold = - hold ;
      on_hold = - on_hold;
    }
    if(state) { // Relay state is enabled
      if(temp <= on_hold) {
          if ( (getParamById (PARAM_RELAY_DELAY) << RELAY_TIMER_MULTIPLIER) < timer) {
              state = false; timer = 0 ;
          }
      }
    }else { // Relay state is disabled
      if(temp >= hold+hyst) {
          if ( (getParamById (PARAM_RELAY_DELAY) << RELAY_TIMER_MULTIPLIER) < timer) {
              state = true; timer = 0 ;
          }
      }
    }
    switch(force) {
    case RELAY_FORCE_ON:
        setRelay(true);
        break;
    case RELAY_FORCE_OFF:
        setRelay(false);
        break;
    case RELAY_FORCE_NA:
    default:
        setRelay(state);
    }
    timer++;
}
