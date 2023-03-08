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
			void *Context;
		} TimerEvent;
	};

	AppEventType Type{ AppEventType::None };
	AppEventEndpointID Cluster{ AppEventEndpointID::None };
	EventHandler Handler;
};
