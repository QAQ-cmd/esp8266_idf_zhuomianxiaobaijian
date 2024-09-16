#ifndef SMARTCONFIG_MAIN_H
#define SMARTCONFIG_MAIN_H
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
#include "smartconfig_ack.h"

/* The examples use smartconfig type that you can set via project configuration menu.

   If you'd rather not, just change the below entries to enum with
   the config you want - ie #define EXAMPLE_ESP_SMARTCOFNIG_TYPE SC_TYPE_ESPTOUCH
*/
#define EXAMPLE_ESP_SMARTCOFNIG_TYPE      CONFIG_ESP_SMARTCONFIG_TYPE

/* FreeRTOS event group to signal when we are connected & ready to make a request */
EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
extern const int CONNECTED_BIT;
extern const int ESPTOUCH_DONE_BIT;
extern const char* TAG;

void smartconfig_example_task(void* parm);

void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void initialise_wifi(void); 

#endif // SMARTCONFIG_MAIN_H