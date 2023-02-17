/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"
#include "app_config.h"
#include "led_util.h"

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

LOG_MODULE_DECLARE(app, CONFIG_MATTER_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceLayer;

namespace
{
	constexpr size_t kAppEventQueueSize = 10;
	constexpr EndpointId kLightEndpointId = 4;
	constexpr uint32_t kFactoryResetTriggerTimeout = 6000;

	constexpr uint8_t kDefaultMinLevel = 0;
	constexpr uint8_t kDefaultMaxLevel = 254;

	K_MSGQ_DEFINE(sAppEventQueue, sizeof(AppEvent), kAppEventQueueSize, alignof(AppEvent));
	k_timer sFunctionTimer;


	LEDWidget sStatusLED;
	FactoryResetLEDsWrapper<3> sFactoryResetLEDs{ { FACTORY_RESET_SIGNAL_LED, FACTORY_RESET_SIGNAL_LED1,
							FACTORY_RESET_SIGNAL_LED2 } };

	bool sIsNetworkProvisioned = false;
	bool sIsNetworkEnabled = false;
	bool sHaveBLEConnections = false;


	//const struct pwm_dt_spec sLightPwmDevice = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
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
		k_timer sTemperatureSensor;
		k_timer sRelativeHumiditySensor;
		k_timer sIlluminanceSensor;
	}

	namespace FetchPeriodSeconds {
		constexpr uint32_t sTemperatureSensor{ 10 };
		constexpr uint32_t sRelativeHumiditySensor{ 10 };
		constexpr uint32_t sIlluminanceSensor{ 1 };
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

	/* Initialize function timer */
	k_timer_init(&sFunctionTimer, &AppTask::FunctionTimerTimeoutCallback, nullptr);
	k_timer_user_data_set(&sFunctionTimer, this);

	/* Initialize sensor timers */
	k_timer_init(&Timers::Timers::sTemperatureSensor, &AppTask::TemperatureMeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::Timers::sTemperatureSensor, this);
	k_timer_start(&Timers::Timers::sTemperatureSensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sTemperatureSensor));

	k_timer_init(&Timers::Timers::sRelativeHumiditySensor, &AppTask::RelativeHumidityMeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::Timers::sRelativeHumiditySensor, this);
	k_timer_start(&Timers::Timers::sRelativeHumiditySensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sRelativeHumiditySensor));

	k_timer_init(&Timers::Timers::sIlluminanceSensor, &AppTask::IlluminanceMeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&Timers::Timers::sIlluminanceSensor, this);
	k_timer_start(&Timers::Timers::sIlluminanceSensor, K_NO_WAIT, K_SECONDS(Timers::FetchPeriodSeconds::sIlluminanceSensor));


	/* Initialize lighting device (PWM) */
	/*ret = mPWMDevice.Init(&sLightPwmDevice, kDefaultMinLevel, kDefaultMaxLevel, maxLightLevel);
	if (ret != 0) {
		return chip::System::MapErrorZephyr(ret);
	}
	mPWMDevice.SetCallbacks(ActionInitiated, ActionCompleted);*/

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

/*void AppTask::IdentifyStartHandler(Identify *)
{
	AppEvent event;
	event.Type = AppEventType::IdentifyStart;
	event.Handler = [](const AppEvent &) {
		Instance().mPWMDevice.SuppressOutput();
		sIdentifyLED.Blink(LedConsts::kIdentifyBlinkRate_ms);
	};
	PostEvent(event);
}

void AppTask::IdentifyStopHandler(Identify *)
{
	AppEvent event;
	event.Type = AppEventType::IdentifyStop;
	event.Handler = [](const AppEvent &) {
		sIdentifyLED.Set(false);
		Instance().mPWMDevice.ApplyLevel();
	};
	PostEvent(event);
}*/

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
		sFactoryResetLEDs.Set(true);

		chip::Server::GetInstance().ScheduleFactoryReset();
	}
}

void AppTask::TemperatureMeasurementTimeoutCallback(k_timer * timer) 
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::SensorFetch;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionTemperatureFetchEventHandler;
	PostEvent(event);
}

void AppTask::RelativeHumidityMeasurementTimeoutCallback(k_timer * timer)
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::SensorFetch;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionRelativeHumidityEventHandler;
	PostEvent(event);
}

void AppTask::IlluminanceMeasurementTimeoutCallback(k_timer * timer)
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::SensorFetch;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionIlluminanceEventHandler;
	PostEvent(event);
}

void AppTask::FunctionTemperatureFetchEventHandler(const AppEvent &event)
{
	/* Simulate sensor for now */
	chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
		static_cast<chip::EndpointId>(AppEventClusterID::Temperature), int16_t(rand() % 5000));
}

void AppTask::FunctionRelativeHumidityEventHandler(const AppEvent &event)
{
	/* Simulate sensor for now */
	chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
		static_cast<chip::EndpointId>(AppEventClusterID::RelativeHumidity), int16_t(rand() % 10000));
}

void AppTask::FunctionIlluminanceEventHandler(const AppEvent &event)
{
	/* Simulate sensor for now */
	chip::app::Clusters::IlluminanceMeasurement::Attributes::MeasuredValue::Set(
		static_cast<chip::EndpointId>(AppEventClusterID::Illuminance), int16_t(rand() % 1000));
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
			sFactoryResetLEDs.Set(false);
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

/*void AppTask::ActionInitiated(PWMDevice::Action_t action, int32_t actor)
{
	if (action == PWMDevice::ON_ACTION) {
		LOG_INF("Turn On Action has been initiated");
	} else if (action == PWMDevice::OFF_ACTION) {
		LOG_INF("Turn Off Action has been initiated");
	}
}

void AppTask::ActionCompleted(PWMDevice::Action_t action, int32_t actor)
{
	if (action == PWMDevice::ON_ACTION) {
		LOG_INF("Turn On Action has been completed");
	} else if (action == PWMDevice::OFF_ACTION) {
		LOG_INF("Turn Off Action has been completed");
	}
}*/

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

/*void AppTask::UpdateClusterState()
{
	SystemLayer().ScheduleLambda([this] {
		/ write the new on/off value /
		EmberAfStatus status =
			Clusters::OnOff::Attributes::OnOff::Set(kLightEndpointId, mPWMDevice.IsTurnedOn());

		if (status != EMBER_ZCL_STATUS_SUCCESS) {
			LOG_ERR("Updating on/off cluster failed: %x", status);
		}

		/ write the current level /
		status = Clusters::LevelControl::Attributes::CurrentLevel::Set(kLightEndpointId, mPWMDevice.GetLevel());

		if (status != EMBER_ZCL_STATUS_SUCCESS) {
			LOG_ERR("Updating level cluster failed: %x", status);
		}
	});
}*/