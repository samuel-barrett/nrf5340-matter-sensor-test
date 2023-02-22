#include "app_task.h"
#include "app_driver.h"
#include "app_config.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/ConcreteAttributePath.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000


using namespace ::chip;
using namespace ::chip::app::Clusters;
using namespace ::chip::app::Clusters::OnOff;

//extern __cpp_constexpr struct gpio_dt_spec led;


void MatterPostAttributeChangeCallback(const chip::app::ConcreteAttributePath & attributePath, uint8_t type,
                                       uint16_t size, uint8_t * value)
{
    ClusterId clusterId = attributePath.mClusterId;
	AttributeId attributeId = attributePath.mAttributeId;

    if (clusterId == OnOff::Id && attributeId == OnOff::Attributes::OnOff::Id) {
        if(*value == 0) {
            gpio_pin_set_dt(&led, GPIO_OUTPUT_INACTIVE); // Turn off LED1
            printf("MatterPostAttributechangeCallback GPIO setting value to: %d\n", GPIO_OUTPUT_INACTIVE);
        } else{
            gpio_pin_set_dt(&led, GPIO_OUTPUT_ACTIVE); // Turn on LED1
            printf("MatterPostAttributechangeCallback GPIO setting value to: %d\n", GPIO_OUTPUT_ACTIVE);
        }
    }
}


/** @brief OnOff Cluster Init
 *
 * This function is called when a specific cluster is initialized. It gives the
 * application an opportunity to take care of cluster initialization procedures.
 * It is called exactly once for each endpoint where cluster is present.
 *
 * @param endpoint   Ver.: always
 *
 * TODO Issue #3841
 * emberAfOnOffClusterInitCallback happens before the stack initialize the cluster
 * attributes to the default value.
 * The logic here expects something similar to the deprecated Plugins callback
 * emberAfPluginOnOffClusterServerPostInitCallback.
 *
 */
void emberAfOnOffClusterInitCallback(EndpointId endpoint)
{
	EmberAfStatus status;
	bool storedValue;

	/* Read storedValue on/off value */
	status = Attributes::OnOff::Get(endpoint, &storedValue);
	if (status == EMBER_ZCL_STATUS_SUCCESS) {
        int ret;

        if (!device_is_ready(led.port)) {
            return;
        }

        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE | GPIO_INPUT);
        if (ret < 0) {
            return;
        }

	}
    printf("emberAFOnOffClusterInitCallback GPIO setting value to: %u\n", GPIO_ACTIVE_HIGH);
	//AppTask::Instance().UpdateClusterState();
}