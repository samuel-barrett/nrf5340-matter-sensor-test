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

LOG_MODULE_DECLARE(app, CONFIG_MATTER_LOG_LEVEL);

namespace
{
	constexpr size_t kAppEventQueueSize = 10;


	K_MSGQ_DEFINE(sAppEventQueue, sizeof(AppEvent), kAppEventQueueSize, alignof(AppEvent));

	bool sIsNetworkProvisioned = false;
	bool sIsNetworkEnabled = false;
	bool sHaveBLEConnections = false;

	BH1750Driver bh1750_driver = BH1750Driver("bh1750");
	SCD30Driver scd30_driver = SCD30Driver("scd30");

} /* namespace */

namespace Timers 
{
	namespace Timers 
	{
		k_timer sSCD30Sensor;
		k_timer sBH1750Sensor;
	}

	namespace FetchPeriodSeconds 
	{
		constexpr uint32_t sSCD30Sensor{ 20 };
		constexpr uint32_t sBH1750Sensor{ 2 };
	}
}

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

	err = static_cast<CHIP_ERROR>(bh1750_driver.init());
	if(err != CHIP_NO_ERROR) {
		LOG_ERR("Error in BH1750 init");
		return err;
	}

	err = static_cast<CHIP_ERROR>(scd30_driver.init());
	if(err != CHIP_NO_ERROR) {
		LOG_ERR("Error in SCD30 init");
		return err;
	}

	/* Initialize sensor timers */
	k_timer_init(&Timers::Timers::sSCD30Sensor, &AppTask::SCD30MeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::Timers::sSCD30Sensor, this);
	k_timer_start(&Timers::Timers::sSCD30Sensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sSCD30Sensor));

	k_timer_init(&Timers::Timers::sBH1750Sensor, &AppTask::BH1750MeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::Timers::sBH1750Sensor, this);
	k_timer_start(&Timers::Timers::sBH1750Sensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sBH1750Sensor));

	SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());

	static chip::CommonCaseDeviceServerInitParams initParams;
	(void)initParams.InitializeStaticResourcesBeforeServerInit();

	ReturnErrorOnFailure(chip::Server::GetInstance().Init(initParams));
	ConfigurationMgr().LogDeviceConfig();
	PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

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

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	AppEvent event = {};

	while (true) {
		k_msgq_get(&sAppEventQueue, &event, K_FOREVER);
		DispatchEvent(event);
	}

	return CHIP_NO_ERROR;
}

void AppTask::FunctionTimerTimeoutCallback(k_timer *timer)
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::Timer;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionTimerEventHandler;
	PostEvent(event);
}

void AppTask::SCD30MeasurementTimeoutCallback(k_timer * timer) 
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::SensorFetch;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionSCD30FetchEventHandler;
	PostEvent(event);
}

void AppTask::BH1750MeasurementTimeoutCallback(k_timer * timer)
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::SensorFetch;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionBH1750EventHandler;
	PostEvent(event);
}

void AppTask::FunctionSCD30FetchEventHandler(const AppEvent &event)
{
	bool data_ready = false;
	float co2 = 0.0, temperature = 0.0, humidity = 0.0;
	int ret;

	ret = scd30_driver.get_data_ready_status(&data_ready);
    if(ret) {
        LOG_ERR("Could not check data ready status");
        return;
    }

	if(data_ready)
	{
		ret = scd30_driver.read_measurement(&co2, &temperature, &humidity);
		if(ret) {
			LOG_ERR("Error reading sensor data");
			return;
		}

        if (co2 == 0)
        {
            LOG_ERR("Invalid co2 sample detected, skipping");
			return;
        }

		LOG_INF("Setting temp to: %d and humidity to %d",static_cast<uint16_t>(temperature*100), static_cast<uint16_t>(humidity*100));

		/* Simulate sensor for now */
		chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
			static_cast<chip::EndpointId>(AppEventEndpointID::Temperature), static_cast<uint16_t>(temperature*100.0));

		chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
			static_cast<chip::EndpointId>(AppEventEndpointID::RelativeHumidity), static_cast<uint16_t>(humidity*100.0));

	}
}


void AppTask::FunctionBH1750EventHandler(const AppEvent &event)
{
	int error;
	uint16_t lux;

	error = bh1750_driver.read(&lux);
    if (error < 0) {
        LOG_ERR("I2C: Error in i2c_read transfer: %d", error);
		return;
    }

	chip::app::Clusters::IlluminanceMeasurement::Attributes::MeasuredValue::Set(
		static_cast<chip::EndpointId>(AppEventEndpointID::Illuminance), lux);
}

void AppTask::ChipEventHandler(const ChipDeviceEvent *event, intptr_t /* arg */)
{
	switch (event->Type) {
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


void AppTask::PostEvent(const AppEvent &event)
{
	if (k_msgq_put(&sAppEventQueue, &event, K_NO_WAIT) != 0) {
		LOG_INF("Failed to post event to app task event queue");
	}
}

void AppTask::DispatchEvent(const AppEvent &event)
{
	if (event.Handler) {
		event.Handler(event);
	} else {
		LOG_INF("Event received with no handler. Dropping event.");
	}
}
