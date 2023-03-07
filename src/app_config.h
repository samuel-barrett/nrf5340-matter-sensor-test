/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

/* ---- Template App Config ---- */

#include "board_util.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LIGHTING_BUTTON DK_BTN1
#define LIGHTING_BUTTON_MASK DK_BTN1_MSK

#define SYSTEM_STATE_LED DK_LED2
//#define FACTORY_RESET_SIGNAL_LED DK_LED2

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)


static constexpr struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

