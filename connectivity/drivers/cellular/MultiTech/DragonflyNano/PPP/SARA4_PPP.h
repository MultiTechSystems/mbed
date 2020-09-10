/*
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SARA4_PPP_H_
#define SARA4_PPP_H_

#ifdef TARGET_FF_ARDUINO
#ifndef MBED_CONF_SARA4_PPP_TX
#define MBED_CONF_SARA4_PPP_TX D1
#endif
#ifndef MBED_CONF_SARA4_PPP_RX
#define MBED_CONF_SARA4_PPP_RX D0
#endif
#endif /* TARGET_FF_ARDUINO */

#include "AT_CellularDevice.h"

namespace mbed {

class SARA4_PPP : public AT_CellularDevice {

public:
    SARA4_PPP(FileHandle *fh);

public: // CellularDevice
    virtual AT_CellularNetwork *open_network_impl(ATHandler &at);

/** Enable or disable the 3GPP PSM.
*
*  Note: Application should reboot the module after enabling PSM in order to enter PSM state. (reboot_modem())
*  Note: Modem can be woken up by toggling the power-on signal. (wakeup_modem())
*  Note: When device enters PSM, all connections(PPP, sockets) and settings that are not saved in NV memory(ATE0, CREG etc) are lost.
*        host application should be prepared to re-initialize the modem and re-establish the connections.
*  Note: PSM is disabled if both periodic_time and active_time are 0.
*  Note: Not all variants/firmware versions support PSM URCs and in that case function will return false.
*
*  PSM string encoding code is borrowed from AT_CellularPower.cpp
*
* @param periodic_time    requested periodic TAU in seconds.
* @param active_time      requested active time in seconds.
* @param func             callback function to execute when modem goes to sleep
* @param ptr              parameter to callback function
* @return         True if successful, otherwise false.
*/
nsapi_error_t set_power_save_mode(int periodic_tau, int active_time);

};

} // namespace mbed

#endif // SARA4_PPP_H_
