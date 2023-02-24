/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"
#include "app_config.h"
#include "led_util.h"
#include "app_driver.h"

#ifdef CONFIG_NET_L2_OPENTHREAD
#include "thread_util.h"
#endif

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

#ifdef CONFIG_CHIP_WIFI
#include <app/clusters/network-commissioning/network-commissioning.h>
#include <platform/nrfconnect/wifi/NrfWiFiDriver.h>
#endif

#ifdef CONFIG_CHIP_OTA_REQUESTOR
#include "ota_util.h"
#endif

#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_DECLARE(app, CONFIG_MATTER_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceLayer;

namespace
{
	constexpr size_t kAppEventQueueSize = 10;
	constexpr uint32_t kFactoryResetTriggerTimeout = 6000;

	constexpr uint8_t kDefaultMinLevel = 0;
	constexpr uint8_t kDefaultMaxLevel = 254;

	K_MSGQ_DEFINE(sAppEventQueue, sizeof(AppEvent), kAppEventQueueSize, alignof(AppEvent));
	k_timer sFunctionTimer;


	LEDWidget sStatusLED;

	bool sIsNetworkProvisioned = false;
	bool sIsNetworkEnabled = false;
	bool sHaveBLEConnections = false;

	BH1750Driver bh1750_driver = BH1750Driver("bh1750");
	SCD30Driver scd30_driver = SCD30Driver("scd30");

} /* namespace */

namespace LedConsts {
	namespace StatusLed {
		namespace Unprovisioned {
			constexpr uint32_t kOn_ms{ 100 };
			constexpr uint32_t kOff_ms{ kOn_ms };
		} /* namespace Unprovisioned */

		namespace Provisioned {
			constexpr uint32_t kOn_ms{ 50 };
			constexpr uint32_t kOff_ms{ 950 };
		} /* namespace Provisioned */ 
	} /* namespace StatusLed */
} /* namespace LedConsts */

namespace Timers {
	namespace Timers {
		k_timer sSCD30Sensor;
		k_timer sBH1750Sensor;
	}

	namespace FetchPeriodSeconds {
		constexpr uint32_t sSCD30Sensor{ 20 };
		constexpr uint32_t sBH1750Sensor{ 2 };
	}
}

CHIP_ERROR AppTask::Init()
{
	/* Initialize CHIP stack */
	LOG_INF("Init CHIP stack");

	CHIP_ERROR err = chip::Platform::MemoryInit();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("Platform::MemoryInit() failed");
		return err;
	}

	err = PlatformMgr().InitChipStack();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("PlatformMgr().InitChipStack() failed");
		return err;
	}

#if defined(CONFIG_NET_L2_OPENTHREAD)
	err = ThreadStackMgr().InitThreadStack();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("ThreadStackMgr().InitThreadStack() failed: %s", ErrorStr(err));
		return err;
	}

#ifdef CONFIG_OPENTHREAD_MTD_SED
	err = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_SleepyEndDevice);
#elif CONFIG_OPENTHREAD_MTD
	err = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_MinimalEndDevice);
#else
	err = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_Router);
#endif /* CONFIG_OPENTHREAD_MTD_SED */
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("ConnectivityMgr().SetThreadDeviceType() failed: %s", ErrorStr(err));
		return err;
	}

#ifdef CONFIG_OPENTHREAD_DEFAULT_TX_POWER
	err = SetDefaultThreadOutputPower();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("Cannot set default Thread output power");
		return err;
	}
#endif /* CONFIG_OPENTHREAD_DEFAULT_TX_POWER */
#elif defined(CONFIG_CHIP_WIFI)
	sWiFiCommissioningInstance.Init();
#else
	return CHIP_ERROR_INTERNAL;
#endif /* CONFIG_NET_L2_OPENTHREAD */

	/* Initialize LEDs */
	LEDWidget::InitGpio();
	LEDWidget::SetStateUpdateCallback(LEDStateUpdateHandler);

	sStatusLED.Init(SYSTEM_STATE_LED);
	//sIdentifyLED.Init(LIGHTING_STATE_LED);

	UpdateStatusLED();

	/* Initialize buttons */
	int ret = dk_buttons_init(ButtonEventHandler);
	if (ret) {
		LOG_ERR("dk_buttons_init() failed");
		return chip::System::MapErrorZephyr(ret);
	}

	// Command to power up the sensor and set the measurement mode
	/*uint8_t bh1750_init_command[1] = {0x10};
	
	err = static_cast<CHIP_ERROR>(bh1750_driver.init(bh1750_init_command, 1));
	if(err != CHIP_NO_ERROR) {
		LOG_ERR("Error in BH1750 init");
		return err;
	}*/

	err = static_cast<CHIP_ERROR>(scd30_driver.init());
	if(err != CHIP_NO_ERROR) {
		LOG_ERR("Error in SCD30 init");
		return err;
	}

	/* Initialize function timer */
	k_timer_init(&sFunctionTimer, &AppTask::FunctionTimerTimeoutCallback, nullptr);
	k_timer_user_data_set(&sFunctionTimer, this);

	/* Initialize sensor timers */
	k_timer_init(&Timers::Timers::sSCD30Sensor, &AppTask::SCD30MeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::Timers::sSCD30Sensor, this);
	k_timer_start(&Timers::Timers::sSCD30Sensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sSCD30Sensor));

	k_timer_init(&Timers::Timers::sBH1750Sensor, &AppTask::BH1750MeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::Timers::sBH1750Sensor, this);
	k_timer_start(&Timers::Timers::sBH1750Sensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sBH1750Sensor));

	/* Initialize CHIP server */
#if CONFIG_CHIP_FACTORY_DATA
	ReturnErrorOnFailure(mFactoryDataProvider.Init());
	SetDeviceInstanceInfoProvider(&mFactoryDataProvider);
	SetDeviceAttestationCredentialsProvider(&mFactoryDataProvider);
	SetCommissionableDataProvider(&mFactoryDataProvider);
#else
	SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());
#endif

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

void AppTask::ButtonEventHandler(uint32_t buttonState, uint32_t hasChanged)
{
	AppEvent button_event;
	button_event.Type = AppEventType::Button;

	if (FUNCTION_BUTTON_MASK & hasChanged) {
		button_event.ButtonEvent.PinNo = FUNCTION_BUTTON;
		button_event.ButtonEvent.Action =
			static_cast<uint8_t>((FUNCTION_BUTTON_MASK & buttonState) ? AppEventType::ButtonPushed :
										    AppEventType::ButtonReleased);
		button_event.Handler = FunctionHandler;
		PostEvent(button_event);
	}
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

void AppTask::FunctionTimerEventHandler(const AppEvent &)
{
	if (Instance().mFunction == FunctionEvent::FactoryReset) {
		Instance().mFunction = FunctionEvent::NoneSelected;
		LOG_INF("Factory Reset triggered");

		sStatusLED.Set(true);

		chip::Server::GetInstance().ScheduleFactoryReset();
	}
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
        printk("Could not check data ready status\n");
        return;
    }

	if(data_ready)
	{
		ret = scd30_driver.read_measurement(&co2, &temperature, &humidity);
		if(ret) {
			printk("Error reading sensor data\n");
			return;
		}

        if (co2 == 0)
        {
            printk("Invalid co2 sample detected, skipping\n");
			return;
        }

		/* Simulate sensor for now */
		chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
			static_cast<chip::EndpointId>(AppEventEndpointID::Temperature), int16_t(temperature*100));

		chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
			static_cast<chip::EndpointId>(AppEventEndpointID::RelativeHumidity), int16_t(humidity*100));

	}
}


void AppTask::FunctionBH1750EventHandler(const AppEvent &event)
{
	int error;
	uint8_t data_buffer[2] = {0,0};

	// Read the actual sensor value using the BH1750 driver
	/*error = bh1750_driver.read(data_buffer, 2);
    if (error < 0) {
        printk("I2C: Error in i2c_read transfer: %d\n", error);
    }*/

    // The received data is in lux, represented as a 16-bit value in big-endian format
	int lux =  ((data_buffer[0] << 8) | data_buffer[1]) / 1.2;

	chip::app::Clusters::IlluminanceMeasurement::Attributes::MeasuredValue::Set(
		static_cast<chip::EndpointId>(AppEventEndpointID::Illuminance), lux);
}

void AppTask::FunctionHandler(const AppEvent &event)
{
	if (event.ButtonEvent.PinNo != FUNCTION_BUTTON)
		return;

	if (event.ButtonEvent.Action == static_cast<uint8_t>(AppEventType::ButtonPushed)) {
		Instance().StartTimer(kFactoryResetTriggerTimeout);
		Instance().mFunction = FunctionEvent::FactoryReset;
	} else if (event.ButtonEvent.Action == static_cast<uint8_t>(AppEventType::ButtonReleased)) {
		if (Instance().mFunction == FunctionEvent::FactoryReset) {
			//sFactoryResetLEDs.Set(false);
			UpdateStatusLED();
			Instance().CancelTimer();
			Instance().mFunction = FunctionEvent::NoneSelected;
			LOG_INF("Factory Reset has been Canceled");
		}
	}
}

void AppTask::LEDStateUpdateHandler(LEDWidget &ledWidget)
{
	AppEvent event;
	event.Type = AppEventType::UpdateLedState;
	event.Handler = UpdateLedStateEventHandler;
	event.UpdateLedStateEvent.LedWidget = &ledWidget;
	PostEvent(event);
}

void AppTask::UpdateLedStateEventHandler(const AppEvent &event)
{
	if (event.Type == AppEventType::UpdateLedState) {
		event.UpdateLedStateEvent.LedWidget->UpdateState();
	}
}

void AppTask::UpdateStatusLED()
{
	/* Update the status LED.
	 *
	 * If IPv6 networking and service provisioned, keep the LED On constantly.
	 *
	 * If the system has BLE connection(s) uptill the stage above, THEN blink the LED at an even
	 * rate of 100ms.
	 *
	 * Otherwise, blink the LED for a very short time. */
	if (sIsNetworkProvisioned && sIsNetworkEnabled) {
		sStatusLED.Set(true);
	} else if (sHaveBLEConnections) {
		sStatusLED.Blink(LedConsts::StatusLed::Unprovisioned::kOn_ms,
				 LedConsts::StatusLed::Unprovisioned::kOff_ms);
	} else {
		sStatusLED.Blink(LedConsts::StatusLed::Provisioned::kOn_ms, LedConsts::StatusLed::Provisioned::kOff_ms);
	}
}

void AppTask::ChipEventHandler(const ChipDeviceEvent *event, intptr_t /* arg */)
{
	switch (event->Type) {
	case DeviceEventType::kCHIPoBLEAdvertisingChange:
		sHaveBLEConnections = ConnectivityMgr().NumBLEConnections() != 0;
		UpdateStatusLED();
		break;
#if defined(CONFIG_NET_L2_OPENTHREAD)
	case DeviceEventType::kDnssdPlatformInitialized:
#if CONFIG_CHIP_OTA_REQUESTOR
		InitBasicOTARequestor();
#endif /* CONFIG_CHIP_OTA_REQUESTOR */
		break;
	case DeviceEventType::kThreadStateChange:
		sIsNetworkProvisioned = ConnectivityMgr().IsThreadProvisioned();
		sIsNetworkEnabled = ConnectivityMgr().IsThreadEnabled();
#elif defined(CONFIG_CHIP_WIFI)
	case DeviceEventType::kWiFiConnectivityChange:
		sIsNetworkProvisioned = ConnectivityMgr().IsWiFiStationProvisioned();
		sIsNetworkEnabled = ConnectivityMgr().IsWiFiStationEnabled();
#if CONFIG_CHIP_OTA_REQUESTOR
		if (event->WiFiConnectivityChange.Result == kConnectivity_Established) {
			InitBasicOTARequestor();
		}
#endif /* CONFIG_CHIP_OTA_REQUESTOR */
#endif
		UpdateStatusLED();
		break;
	default:
		break;
	}
}

void AppTask::CancelTimer()
{
	k_timer_stop(&sFunctionTimer);
}

void AppTask::StartTimer(uint32_t timeoutInMs)
{
	k_timer_start(&sFunctionTimer, K_MSEC(timeoutInMs), K_NO_WAIT);
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
