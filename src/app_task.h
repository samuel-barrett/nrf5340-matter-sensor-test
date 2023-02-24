/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "app_event.h"
#include "app_driver.h"

#include <platform/CHIPDeviceLayer.h>


struct k_timer;

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

	void CancelTimer();
	void StartTimer(uint32_t timeoutInMs);

	static void DispatchEvent(const AppEvent &event);
	static void UpdateLedStateEventHandler(const AppEvent &event);
	static void FunctionHandler(const AppEvent &event);
	static void FunctionTimerEventHandler(const AppEvent &event);
	static void FunctionSCD30FetchEventHandler(const AppEvent &event);
	static void FunctionBH1750EventHandler(const AppEvent &event);

	static void ChipEventHandler(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg);
	static void ButtonEventHandler(uint32_t buttonState, uint32_t hasChanged);
	static void LEDStateUpdateHandler(LEDWidget &ledWidget);
	static void FunctionTimerTimeoutCallback(k_timer *timer);

	static void SCD30MeasurementTimeoutCallback(k_timer *timer);
	static void BH1750MeasurementTimeoutCallback(k_timer * timer);

	static void UpdateStatusLED();


	FunctionEvent mFunction = FunctionEvent::NoneSelected;
	//I2CDriver bh1750_driver("bh1750", 0x10);
	bool mFunctionTimerActive = false;
};
