/*
 *  timing.cpp
 *
 *  Copyright (C) 2021, Arm Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif
#include "mbedtls/timing.h"
#include "drivers/Timeout.h"
#include "drivers/Timer.h"
#include "drivers/LowPowerTimer.h"
#include <chrono>

extern "C" {
    volatile int mbedtls_timing_alarmed = 0;
};

static void handle_alarm(void)
{
    mbedtls_timing_alarmed = 1;
}

extern "C" void mbedtls_set_alarm(int seconds)
{
    static mbed::Timeout t;
    mbedtls_timing_alarmed = 0;

    t.attach(handle_alarm, std::chrono::seconds(seconds));
}

#if !defined(HAVE_HARDCLOCK)
#define HAVE_HARDCLOCK
static int timer_init = 0;

extern "C" unsigned long mbedtls_timing_hardclock(void)
{
#if DEVICE_LPTICKER
    static mbed::LowPowerTimer timer;
#elif DEVICE_USTICKER
    static mbed::Timer timer;
#else
#error "MBEDTLS_TIMING_C requires either LPTICKER or USTICKER"
#endif

    if (timer_init == 0) {
        timer.reset();
        timer.start();
        timer_init = 1;
    }

    return timer.elapsed_time().count();
}
#endif /* !HAVE_HARDCLOCK */
