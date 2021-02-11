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
 * Implementation of application menu.
 */

#include "menu.h"
#include "buttons.h"
#include "display.h"
#include "params.h"
#include "timer.h"
#include "relay.h"

#define MENU_1_SEC_PASSED   32
#define MENU_3_SEC_PASSED   MENU_1_SEC_PASSED * 3
#define MENU_5_SEC_PASSED   MENU_1_SEC_PASSED * 5
#define MENU_AUTOINC_DELAY  MENU_1_SEC_PASSED / 8
#define MENU_AUTOINC_FAST_DELAY  MENU_1_SEC_PASSED / 32
#define MENU_FAST_WAIT      30

static unsigned char menuDisplay;
static unsigned char menuState;
static unsigned char fast_wait;
static unsigned int timer;
static bool hold,hold2,timer_reset;

#define DEBOUNCE_MAX 10
static int btnDebounce[2] ;
static bool btnPressed[2] ;

/**
 * @brief Initialization of local variables.
 */
void initMenu()
{
    timer = 0; fast_wait = MENU_FAST_WAIT ;
    hold = hold2 = false ;
    timer_reset = true ;
    menuState = menuDisplay = MENU_ROOT;
    btnDebounce[0]=btnDebounce[1]=-DEBOUNCE_MAX ;
    btnPressed[0]=btnPressed[1]=false ;
}

void resetMenuTimer()
{
    timer_reset = true ;
    timer = 0;
}

/**
 * @brief Gets menu state for displaying appropriate value on the SSD.
 * @return
 */
unsigned char getMenuDisplay()
{
    return menuDisplay;
}

/**
 * @brief Changing buttons' status
 * @param event is one of:
 *  MENU_EVENT_PUSH_BUTTON1
 *  MENU_EVENT_PUSH_BUTTON2
 *  MENU_EVENT_RELEASE_BUTTON1
 *  MENU_EVENT_RELEASE_BUTTON2
 */
void clickMenu(unsigned char event)
{
  int i ;
  switch(event) {
    case MENU_EVENT_PUSH_BUTTON1: btnPressed[i=0]=true ; break ;
    case MENU_EVENT_PUSH_BUTTON2: btnPressed[i=1]=true ; break ;
    case MENU_EVENT_RELEASE_BUTTON1: btnPressed[i=0]=false ; break ;
    case MENU_EVENT_RELEASE_BUTTON2: btnPressed[i=1]=false ; break ;
    default: return ;
  }
  if(btnDebounce[i]==(btnPressed[i]?-DEBOUNCE_MAX:DEBOUNCE_MAX))
    btnDebounce[i]=0 ;
}

/**
 * @brief Updating buttons' transition
**/
void transitMenu()
{
  int i;
  for(i=0;i<2;i++) {
    if(btnPressed[i]) {
      if(btnDebounce[i]<DEBOUNCE_MAX) {
        if(++btnDebounce[i]==DEBOUNCE_MAX) {
          switch(i) {
            case 0: feedMenu(MENU_EVENT_PUSH_BUTTON1) ; break;
            case 1: feedMenu(MENU_EVENT_PUSH_BUTTON2) ; break;
          }
        }
      }
    }else {
      if(btnDebounce[i]>-DEBOUNCE_MAX) {
        if(--btnDebounce[i]==-DEBOUNCE_MAX) {
          switch(i) {
            case 0: feedMenu(MENU_EVENT_RELEASE_BUTTON1) ; break;
            case 1: feedMenu(MENU_EVENT_RELEASE_BUTTON2) ; break;
          }
        }
      }
    }
  }
}

/**
 * @brief Updating state of application's menu and displaying info when new
 *  event is received. Possible states of menu and displaying are:
 *  MENU_ROOT
 *  MENU_SELECT_PARAM
 *  MENU_CHANGE_PARAM
 *  MENU_SET_THRESHOLD
 *
 * @param event is one of:
 *  MENU_EVENT_PUSH_BUTTON1
 *  MENU_EVENT_PUSH_BUTTON2
 *  MENU_EVENT_RELEASE_BUTTON1
 *  MENU_EVENT_RELEASE_BUTTON2
 *  MENU_EVENT_CHECK_TIMER
 */
void feedMenu(unsigned char event)
{
    bool blink;

    if (menuState == MENU_ROOT) {
        switch (event) {
        case MENU_EVENT_PUSH_BUTTON1:
        case MENU_EVENT_PUSH_BUTTON2:
            if(!hold) {
              menuDisplay = MENU_SET_THRESHOLD;
              hold = true ;
            }
            break;

        case MENU_EVENT_RELEASE_BUTTON1:
        case MENU_EVENT_RELEASE_BUTTON2:
            if (timer < MENU_5_SEC_PASSED) {
                menuState = MENU_SET_THRESHOLD;
            }
            hold = false ;
            break;

        case MENU_EVENT_CHECK_TIMER:
            if (getButton1() ) {
                if (timer > MENU_3_SEC_PASSED) {
                    setParamId (0);
                    timer = 0;
                    menuState = menuDisplay = MENU_SELECT_PARAM;
                }
            }else if(getButton2()) {
                if (timer > MENU_3_SEC_PASSED) {
                    timer = 0;
                    menuState = menuDisplay = MENU_RELAY_FORCE_ON ;
                }
            }

            break;

        default:
            break;
        }
    } else if (menuState == MENU_SELECT_PARAM) {
        switch (event) {
        case MENU_EVENT_PUSH_BUTTON1:
            if(!hold) {
              menuState = menuDisplay = MENU_CHANGE_PARAM;
              hold=true ;
            }
            break ;

        case MENU_EVENT_RELEASE_BUTTON1:
            hold = false ;
            break;

        case MENU_EVENT_PUSH_BUTTON2:
            if(!hold2) {
              hold2=true ;
              incParamId();
            }
            break ;

        case MENU_EVENT_RELEASE_BUTTON2:
            hold2=false ;
            break;

        case MENU_EVENT_CHECK_TIMER:
            #if 0
            if (hold2&&timer > MENU_1_SEC_PASSED + MENU_AUTOINC_DELAY) {
                if (getButton2() ) {
                    incParamId();
                    timer = MENU_1_SEC_PASSED;
                }
            }
            #endif

            if (timer > MENU_5_SEC_PASSED) {
                timer = 0;
                setParamId (0);
                storeParams();
                menuState = menuDisplay = MENU_ROOT;
            }

            break;

        default:
            break;
        }
    } else if (menuState == MENU_CHANGE_PARAM) {
        switch (event) {
        case MENU_EVENT_PUSH_BUTTON1:
            if(!hold2) {
              incParam();
              hold2=true ;
            }
            break ;

        case MENU_EVENT_RELEASE_BUTTON1:
            hold2=false ; fast_wait = MENU_FAST_WAIT ;
            break;

        case MENU_EVENT_PUSH_BUTTON2:
            if(!hold2) {
              decParam();
              hold2=true ;
            }
            break ;

        case MENU_EVENT_RELEASE_BUTTON2:
            hold2=false ; fast_wait = MENU_FAST_WAIT ;
            break;

        case MENU_EVENT_CHECK_TIMER:
            if (getButton1() || getButton2()) {
                blink = false;
            } else {
                blink = (bool) ( (unsigned char) getUptimeTicks() & 0x80);
            }

            if (hold2&&timer > MENU_1_SEC_PASSED + (!fast_wait?MENU_AUTOINC_FAST_DELAY:MENU_AUTOINC_DELAY) ) {

                if (getButton1() ) {
                    incParam();
                    timer = MENU_1_SEC_PASSED;
                } else if (getButton2()) {
                    decParam();
                    timer = MENU_1_SEC_PASSED;
                }

                if(fast_wait) fast_wait-- ;
            }

            setDisplayOff (blink);

            if (timer > MENU_5_SEC_PASSED) {
                timer = 0;

                storeParams();
                menuState = menuDisplay = MENU_ROOT;
                setDisplayOff (false);
            }

            break;

        default:
            break;
        }
    } else if ( menuState == MENU_RELAY_FORCE_ON ||
                menuState == MENU_RELAY_FORCE_OFF ) {
        switch (event) {
        case MENU_EVENT_PUSH_BUTTON2:
            if(!hold) {
              menuState = menuDisplay = MENU_ROOT;
              setRelayForce(RELAY_FORCE_NA) ;
              hold=true ;
            }
            break;

        case MENU_EVENT_RELEASE_BUTTON2:
            setRelayForce(RELAY_FORCE_ON) ;
            hold=false ;
            break;
        }
    } else if (menuState == MENU_SET_THRESHOLD) {
        switch (event) {
        case MENU_EVENT_PUSH_BUTTON1:
            if(!hold2) {
              setParamId (PARAM_THRESHOLD);
              incParam();
              hold2=true ;
            }
            break ;

        case MENU_EVENT_RELEASE_BUTTON1:
            hold2=false ; fast_wait = MENU_FAST_WAIT ;
            break;

        case MENU_EVENT_PUSH_BUTTON2:
            if(!hold2) {
              setParamId (PARAM_THRESHOLD);
              decParam();
              hold2=true ;
            }
            break ;

        case MENU_EVENT_RELEASE_BUTTON2:
            hold2=false ; fast_wait = MENU_FAST_WAIT ;
            break;

        case MENU_EVENT_CHECK_TIMER:
            if (getButton1() || getButton2()) {
                blink = false;
            } else {
                blink = (bool) ( (unsigned char) getUptimeTicks() & 0x80);
            }

            if (hold2&&timer > MENU_1_SEC_PASSED + (!fast_wait?MENU_AUTOINC_FAST_DELAY:MENU_AUTOINC_DELAY) ) {
                setParamId (PARAM_THRESHOLD);

                if (getButton1() ) {
                    incParam();
                    timer = MENU_1_SEC_PASSED;
                } else if (getButton2()) {
                    decParam();
                    timer = MENU_1_SEC_PASSED;
                }

                if(fast_wait) fast_wait-- ;
            }

            setDisplayOff (blink);

            if (timer > MENU_5_SEC_PASSED) {
                timer = 0;

                storeParams();
                menuState = menuDisplay = MENU_ROOT;
                setDisplayOff (false);
            }

            break;

        default:
            break;
        }
    }
}


/**
 * @brief This function is being called during timer's interrupt
 *  request so keep it extremely small and fast.
 *  During this call all time-related functionality of application
 *  menu is handled. For example: fast value change while holding
 *  a button, return to root menu when no action is received from
 *  user within a given time.
 */
void refreshMenu()
{
    if(timer_reset) timer_reset=false,timer=0 ;
    else timer++;
    feedMenu (MENU_EVENT_CHECK_TIMER);
}
