/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "app_event.h"
#include "app_driver.h"

#include <platform/CHIPDeviceLayer.h>


#ifndef CHECK
#define CHECK(x, ...) ({ \
    int __ = x; \
    if (__) { \
        LOG_ERR("ERR #%d: %s", x, __VA_ARGS__); \
        return __; \
    } \
})
#endif

#ifndef CHECK_RET_VOID
#define CHECK_RET_VOID(x, ...) ({ \
    int __ = x; \
    if (__) { \
        LOG_ERR(__VA_ARGS__" ERROR #%d", x); \
        return; \
    } \
})
#endif

#ifndef LOGGING_MODULE_DECLARATION
#define LOGGING_MODULE_DECLARATION
LOG_MODULE_DECLARE(app, CONFIG_MATTER_LOG_LEVEL);
#endif

class AppTask {
public:
	static AppTask &Instance()
	{
		static AppTask sAppTask;
		return sAppTask;
	};

	CHIP_ERROR StartApp();
	static void PostEvent(const AppEvent &event);
private:
	CHIP_ERROR Init();
	static void DispatchEvent(const AppEvent &event);
	static void FunctionSCD30FetchEventHandler(const AppEvent &event);
	static void FunctionBH1750EventHandler(const AppEvent &event);
	static void SCD30MeasurementTimeoutCallback(k_timer *timer);
	static void BH1750MeasurementTimeoutCallback(k_timer * timer);
	static void ChipEventHandler(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg);
};
