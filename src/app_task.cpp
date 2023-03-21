/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"
#include "app_config.h"
#include "led_util.h"
#include "app_driver.h"

#include "thread_util.h"

#include <platform/CHIPDeviceLayer.h>
#include <zephyr/drivers/gpio.h>

#include "board_util.h"
#include <app-common/zap-generated/attributes/Accessors.h>
#include <app/server/OnboardingCodesUtil.h>
#include <app/server/Server.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <lib/support/CHIPMem.h>
#include <lib/support/CodeUtils.h>
#include <system/SystemError.h>

#include "ota_util.h"

#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceLayer;

namespace
{
	constexpr size_t kAppEventQueueSize = 10;

	K_MSGQ_DEFINE(sAppEventQueue, sizeof(AppEvent), kAppEventQueueSize, alignof(AppEvent));

	bool sIsNetworkProvisioned = false;
	bool sIsNetworkEnabled = false;
	bool sHaveBLEConnections = false;

	BH1750Driver bh1750_driver("bh1750");
	SCD30Driver scd30_driver("scd30");

} /* namespace */

namespace Timers 
{
	k_timer sSCD30Sensor;
	k_timer sBH1750Sensor;

	namespace FetchPeriodSeconds 
	{
		constexpr uint32_t sSCD30Sensor{ 20 };
		constexpr uint32_t sBH1750Sensor{ 2 };
	} /* FetchPeriodSeconds */
} /* Timers */

/**
 * @brief Initialize chip stack
 * 
 * @return CHIP_ERROR 
 */
CHIP_ERROR AppTask::Init()
{
	/* Initialize CHIP stack */
	LOG_INF("Init CHIP stack");

	CHIP_ERROR err = chip::Platform::MemoryInit();

	if (err != CHIP_NO_ERROR) 
	{
		LOG_ERR("Platform::MemoryInit() failed");
		return err;
	}

	err = PlatformMgr().InitChipStack();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("PlatformMgr().InitChipStack() failed");
		return err;
	}

	err = ThreadStackMgr().InitThreadStack();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("ThreadStackMgr().InitThreadStack() failed: %s", ErrorStr(err));
		return err;
	}

	err = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_MinimalEndDevice);

	if (err != CHIP_NO_ERROR) {
		LOG_ERR("ConnectivityMgr().SetThreadDeviceType() failed: %s", ErrorStr(err));
		return err;
	}

	if(bh1750_driver.init()) {
		LOG_ERR("Error in BH1750 init");
		return CHIP_ERROR_NOT_CONNECTED;
	}

	if(scd30_driver.init()) {
		LOG_ERR("Error in SCD30 init");
		return CHIP_ERROR_NOT_CONNECTED;
	}


	/* Initialize sensor timers */
	k_timer_init(&Timers::sSCD30Sensor, &AppTask::SCD30MeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::sSCD30Sensor, this);
	k_timer_start(&Timers::sSCD30Sensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sSCD30Sensor));

	k_timer_init(&Timers::sBH1750Sensor, &AppTask::BH1750MeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::sBH1750Sensor, this);
	k_timer_start(&Timers::sBH1750Sensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sBH1750Sensor));

	SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());

	static chip::CommonCaseDeviceServerInitParams initParams;
	(void)initParams.InitializeStaticResourcesBeforeServerInit();

	ReturnErrorOnFailure(chip::Server::GetInstance().Init(initParams));
	ConfigurationMgr().LogDeviceConfig();

	//QR code is currently wrong it appears, so avoid printing for now
	//TODO: Figure out how to print proper QR Code
	//PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

	/*
	 * Add CHIP event handler and start CHIP thread.
	 * Note that all the initialization code should happen prior to this point to avoid data races
	 * between the main and the CHIP threads.
	 */
	PlatformMgr().AddEventHandler(ChipEventHandler, 0);

	err = PlatformMgr().StartEventLoopTask();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("PlatformMgr().StartEventLoopTask() failed");
		return err;
	}

	return CHIP_NO_ERROR;
}

/**
 * @brief Call init and then get and dispatch message events in a loop
 * 
 * @return (CHIP_ERROR) Return any error value upon initialization. Otherwise it should not return.
 */
CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	AppEvent event = {};

	while (true)
	{
		k_msgq_get(&sAppEventQueue, &event, K_FOREVER);
		DispatchEvent(event);
	}

	return CHIP_NO_ERROR;
}

/**
 * @brief Callback function for SCD30 timer timeout. Adds SCD30 sensor reading function to event queue
 * 
 * @param timer (k_timer *) Pointer to timer from which callback is called.
 */
void AppTask::SCD30MeasurementTimeoutCallback(k_timer * timer) 
{
	if (!timer) return;

	AppEvent event;
	event.Type = AppEventType::SensorFetch;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionSCD30FetchEventHandler;
	PostEvent(event);
}

/**
 * @brief Callback function for BH1750 timer timeout. Adds BH1750 sensor reading function to event queue
 * 
 * @param timer (k_timer *) Pointer to timer from which callback is called.
 */
void AppTask::BH1750MeasurementTimeoutCallback(k_timer * timer)
{
	if (!timer) return;

	AppEvent event;
	event.Type = AppEventType::SensorFetch;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionBH1750EventHandler;
	PostEvent(event);
}



/**
 * @brief Reads SCD30 co2, temperature, humidity, and update the corrresponding matter attribute. If the on/off led is on,
 * then the SCD30 co2 reading is placed in the illuminance attribute (matter does not currently support CO2 readings). 
 * 
 * @param event (const AppEvent &event) event type
 */
void AppTask::FunctionSCD30FetchEventHandler(const AppEvent &event)
{
	bool data_ready = false;
	float co2 = 0.0, temperature = 0.0, humidity = 0.0;

	CHECK_RET_VOID(scd30_driver.get_data_ready_status(&data_ready), "Could not check data ready status");

	if(data_ready)
	{
		CHECK_RET_VOID(scd30_driver.read_measurement(&co2, &temperature, &humidity), "Error reading sensor data");
        CHECK_RET_VOID(co2 == 0, "Invalid co2 sample detected, skipping attribute updates");

		LOG_INF("Setting temp to: %d and humidity to %d",static_cast<uint16_t>(temperature*100), static_cast<uint16_t>(humidity*100));

		/* Simulate sensor for now */
		chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
			static_cast<chip::EndpointId>(AppEventEndpointID::Temperature), static_cast<uint16_t>(temperature*100.0));

		chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
			static_cast<chip::EndpointId>(AppEventEndpointID::RelativeHumidity), static_cast<uint16_t>(humidity*100.0));

		//Put co2 values in illuminance attribute if on/off light is on, otherwise lux will be used
		if(gpio_pin_get_dt(&led))
		{
			chip::app::Clusters::IlluminanceMeasurement::Attributes::MeasuredValue::Set(
				static_cast<chip::EndpointId>(AppEventEndpointID::Illuminance), co2);
		}
	}
}

/**
 * @brief Reads BH1750 lux value, and updates the corrresponding matter attribute. If the on/off led is on,
 * then the SCD30 co2 reading is placed in the illuminance attribute (matter does not currently support CO2 readings). 
 * 
 * @param event (const AppEvent &event) event type
 */
void AppTask::FunctionBH1750EventHandler(const AppEvent &event)
{
	uint16_t lux;

	//Putting SCD30 CO2 values in illuminance matter attribute instead if onoff light is on
	if(gpio_pin_get_dt(&led))
	{
		return;
	}

	CHECK_RET_VOID(bh1750_driver.read(&lux), "Error in i2c_read transfer");

	chip::app::Clusters::IlluminanceMeasurement::Attributes::MeasuredValue::Set(
		static_cast<chip::EndpointId>(AppEventEndpointID::Illuminance), lux);
}

/**
 * @brief Handle chip events such as a bluetooth adveritising change, or a thread state change.
 * 
 * @param event (const ChipDeviceEvent *event) Chip event type. Currently kCHIPoBLEAdvertisingChange, kDnssdPlatformInitialized,
 * kThreadStateChange are handled.
 */
void AppTask::ChipEventHandler(const ChipDeviceEvent *event, intptr_t /* arg */)
{
	switch (event->Type)
	{
		case DeviceEventType::kCHIPoBLEAdvertisingChange:
			sHaveBLEConnections = ConnectivityMgr().NumBLEConnections() != 0;
			break;

		case DeviceEventType::kDnssdPlatformInitialized:
			InitBasicOTARequestor();
			break;

		case DeviceEventType::kThreadStateChange:
			sIsNetworkProvisioned = ConnectivityMgr().IsThreadProvisioned();
			sIsNetworkEnabled = ConnectivityMgr().IsThreadEnabled();
			break;
			
		default:
			break;
	}
}

/**
 * @brief Post event in the message queue
 * 
 * @param event (const AppEvent &) Event handler
 */
void AppTask::PostEvent(const AppEvent & event)
{
	CHECK_RET_VOID(k_msgq_put(&sAppEventQueue, &event, K_NO_WAIT) != 0, 
		"Failed to post event to app task event queue");
}

/**
 * @brief Dispatch the event by passing the event to the event
 * 
 * @param event (const AppEvent &) Event handler
 */
void AppTask::DispatchEvent(const AppEvent & event)
{
	CHECK_RET_VOID(!(event.Handler), "Event received with no handler. Dropping event.");
	event.Handler(event);
}
