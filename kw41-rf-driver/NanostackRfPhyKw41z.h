/*
 * Copyright (c) 2014-2015 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NANOSTACK_RF_PHY_ATMEL_H_
#define NANOSTACK_RF_PHY_ATMEL_H_

#include "ns_types.h"
#include "NanostackRfPhy.h"

class NanostackRfPhyKw41z : public NanostackRfPhy {
public:
    NanostackRfPhyKw41z();
    ~NanostackRfPhyKw41z();
    int8_t rf_register();
    void rf_unregister();
    void get_mac_address(uint8_t *mac);
    void set_mac_address(uint8_t *mac);

private:
};

#endif /* NANOSTACK_RF_PHY_ATMEL_H_ */
