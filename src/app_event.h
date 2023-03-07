/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <cstdint>

#include "event_types.h"

class LEDWidget;

enum class AppEventType : uint8_t { 
	None = 0, 
	Button, 
	ButtonPushed, 
	ButtonReleased, 
	SensorFetch,
	Lighting
};

enum class AppEventEndpointID: uint8_t {
	None = 0,
	Temperature = 1,
	RelativeHumidity = 2,
	Illuminance = 3,
	Light = 4
};

struct AppEvent {
	union {
		struct {
			uint8_t PinNo;
			uint8_t Action;
		} ButtonEvent;
		struct {
			uint8_t Action;
			int32_t Actor;
		} LightingEvent;
		struct {
			void *Context;
		} TimerEvent;
	};

	AppEventType Type{ AppEventType::None };
	AppEventEndpointID Cluster{ AppEventEndpointID::None };
	EventHandler Handler;
};
