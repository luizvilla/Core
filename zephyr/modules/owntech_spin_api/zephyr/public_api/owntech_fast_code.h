/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/*
 * OWNTECH_FAST_CODE: places a function in the CCM SRAM code area (the "ITCM" of Zephyr's Cortex-M
 * linker script), which the core fetches on the I-bus at 0 wait states instead of from flash at 4.
 * Zephyr copies the area from flash at boot (z_data_copy). Without a zephyr,itcm chosen node the
 * macro is empty and the function stays in flash.
 *
 * Use it only on code that runs every control period: the area is 31 KB. Calls between flash and
 * this area go through linker veneers (the two are 128 MB apart), so tag whole call chains.
 */

#ifndef OWNTECH_FAST_CODE_H_
#define OWNTECH_FAST_CODE_H_

#include <zephyr/devicetree.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_itcm))
#include <zephyr/linker/section_tags.h>
#define OWNTECH_FAST_CODE __itcm_section
#else
#define OWNTECH_FAST_CODE
#endif

#endif /* OWNTECH_FAST_CODE_H_ */
