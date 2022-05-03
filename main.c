#include <stdio.h>  
#include "string.h"
#include "esp_system.h"
#include "esp_log.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "math.h"
#include "time.h"

static const int RX_BUF_SIZE = 1024;

#define BUILTIN_LED 2                                       //++ Built-in LED on Devkit v1
#define TXD_PIN (GPIO_NUM_17)                               //++ UART TX and RX Pins for Communication
#define RXD_PIN (GPIO_NUM_16)
#define EN_GPIO 32                                          //++ Set any GPIO to connect to Simcomm ENABLE Pin

#define MAX_LENGTH 100                                      //++ Max Buffer length for array to store AT Commamds
#define INDEX 14                                            //++ Total Number of AT Commands

uint64_t count = 0;                                         //++ Count to post on MQTT
int signal_strength = 0;                                    //++ Varible to Store Signal Strength
bool mqtt_connect_flag = 0;                                 //++ Flag for MQTT Connection
uint8_t esp_restart_count = 0;                              //++ Counter Condition to restart ESP32

bool next_command = true;                                   //++ Flag to check condition for executing next AT Command
bool disconnect_mqtt = false;
bool correct_time_from_sntp = false;                        //++ Flag to check for SNTP Time Response

char longitude_string[50];                                  //++ String to Store Longitutde Co-ordinates
char latitude_string[50];                                   //++ String to Store Latitude Co-ordinates

uint8_t esp_chip_id[6];                                     //++ Array to get Chip ID
char mac_address[20];                                       //++ Array to store Mac Address
char mqtt_publish_topic[100];                               //++ String to store MQTT Publishing Topic


char AT_COMMANDS[INDEX][MAX_LENGTH] = {{"AT+CPIN?\r\n"},                                                //++ All the Necessary AT Commands
                                       {"AT\r\n"},
                                       {"AT+CSQ\r\n"},
                                       {"AT+CGDCONT=1,\"IP\",\"M2MISAFE\"\r\n"},
                                       {"AT+CGACT=1,1\r\n"},
                                       {"AT+CNTP=\"asia.pool.ntp.org\",0\r\n"},                         //++ SNTP for Asia
                                       {"AT+CNTP\r\n"},
                                       {"AT+CCLK?\r\n"},
                                       {"AT+CLBS=1\r\n"},                                               //++ AT Command for Lat-Long Co-ordinates
                                       {"AT+CMQTTSTART\r\n"},
                                       {"AT+CMQTTACCQ=0,\"CLIENT 4G\"\r\n"},
                                       {"AT+CMQTTCFG=\"argtopic\",0,1,1\r\n"},
                                       {"AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,1\r\n"},    //++ Enter your MQTT Broker here
                                       {"AT+CMQTTSUB=0,\"4GBOARD_SIMCOMM/cmd\",2,1\r\n"}};                  //++ Topic to Subscribe

char AT_PUB_COMMAND[2][200] = {{"AT+CMQTTPUB=0,\"94:3c:c6:c2:d8:78\",1,5\r\n"},
                               {"hello"}};

char AT_DISCONNET_MQTT_COMMAND[2][50] = {{"AT+CMQTTDISC=0,120\r\n"},
                                         {"AT+CMQTTSTOP\r\n"}};

void simcomm_uart_init(void)                                                                                //++ Initializing UART
{
    const uart_config_t uart_config = 
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void simcomm_response_parser(const char *data)                      //++ Parser to parse AT Responses from Simcomm
{
    for (int i = 0; i <= strlen(data); i++)
    {
        if (data[i] == 'O' && data[i + 1] == 'K')                   //++ AT Response for OK
        {
            printf(" AT Successful\n");
        }

        else if (data[i] == 'E' && data[i + 1] == 'R' && data[i + 2] == 'R' && data[i + 3] == 'O' && data[i + 4] == 'R')        //++ AT Response for ERROR
        {
            printf("AT Unsuccessful\n");
            next_command = false;
            disconnect_mqtt = true;
        }

        else if (data[i] == '+' && data[i + 1] == 'C' && data[i + 2] == 'S' && data[i + 3] == 'Q' && data[i + 4] == ':')        //++ AT Response for Signal Strength
        {
            int signal_strength_unit = 0;
            int signal_strength_ten = 0;
            int signal_strength_number = 0;

            *((char *)&signal_strength_ten + 0) = data[i + 6];
            signal_strength_ten = (signal_strength_ten - 48) * 10;
            *((char *)&signal_strength_unit + 0) = data[i + 7];

            signal_strength_number = signal_strength_ten + (signal_strength_unit - 48);
            signal_strength = signal_strength_number;
            printf("Signal Strength is %d\n", signal_strength);
        }

        else if (data[i] == 'C' && data[i + 1] == 'L' && data[i + 2] == 'B' && data[i + 3] == 'S' && data[i + 4] == ':')        //++ AT Response for Lat-Long Co-ordinates
        {
            memset(latitude_string, 0, sizeof(latitude_string));
            int data_index = i + 8;
            int latitude_index = 0;
            do
            {
                latitude_string[latitude_index] = data[data_index];
                latitude_index++;
                data_index++;
            } while (data[data_index] != ',');

            printf("latitude is %s\n", latitude_string);
            data_index = data_index + 1;

            memset(longitude_string, 0, sizeof(longitude_string));
            int longitude_index = 0;
            do
            {
                longitude_string[longitude_index] = data[data_index];
                longitude_index++;
                data_index++;
            } while (data[data_index] != ',');

            printf("longitude is %s\n", longitude_string);
        }

        else if (data[i] == 'C' && data[i + 1] == 'M' && data[i + 2] == 'Q' && data[i + 3] == 'T' && data[i + 4] == 'T' && data[i + 5] == 'C' && data[i + 6] == 'O' && data[i + 7] == 'N' && data[i + 8] == 'N' && data[i + 9] == 'E' && data[i + 10] == 'C' && data[i + 11] == 'T' && data[i + 12] == ':')
        {
            printf("client id : %c and MQTT connection status: %c\n", data[i + 14], data[i + 16]);          //++ AT Response for Successful MQTT Connection

            // when both are zero mean mqtt connected
            if (data[i + 14] == '0' && data[i + 16] == '0')
            {
                ESP_LOGI("MQTT","MQTT CONNECTED\n");
                mqtt_connect_flag = true;
                esp_restart_count = 0;
            }
        }

        else if (data[i] == 'C' && data[i + 1] == 'N' && data[i + 2] == 'T' && data[i + 3] == 'P' && data[i + 6] == '0')        //++ AT Response to connect with SNTP
        {
            correct_time_from_sntp = true;
        }
        else if (data[i] == 'C' && data[i + 1] == 'N' && data[i + 2] == 'T' && data[i + 3] == 'P' && data[i + 6] != '0')
        {
            correct_time_from_sntp = false;
        }

        else if(data[i] == 'C' && data[i + 1] == 'M' && data[i + 2] == 'Q' && data[i + 3] == 'T' && data[i + 4] == 'T' && data[i + 5] == 'R' && data[i + 6] == 'E' && data[i + 7] == 'C' && data[i + 8] == 'V')
        {
            char received_string[500] = "0";                //++ AT Response for payload published to subscribed Topic
            int count = 0;

            int index = strcspn(data,"{") + 2;
            
            while(data[index] != '\"')
            {
                received_string[count] = data[index];
                index++;
                count++;

            }

            printf("JSON RECIEVED : %s\n", received_string);
        }
    }

        if (correct_time_from_sntp == true)
        {
            for (int i = 0; i <= strlen(data); i++)
            {
                if (data[i] == 'C' && data[i + 1] == 'C' && data[i + 2] == 'L' && data[i + 3] == 'K' && data[i + 4] == ':')
                {
                    int index_of_data = i + 7;
                    int index_of_time_str = 0;
                    char sntp_time_string[100];
                    memset(sntp_time_string, 0, sizeof(sntp_time_string));

                    do
                    {
                        sntp_time_string[index_of_time_str] = data[index_of_data];
                        index_of_time_str++;
                        index_of_data++;
                    } while (data[index_of_data] != '+');

                    printf("Time Received from SNTP Server is : %s\n", sntp_time_string);
                }
            }
        }
}

int send_cmd_to_simcomm(const char *logName, const char *data)                  //++ Sending AT Commands to Simcomm via UART
{   
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "SIM_TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    while (1)
    {

    start:
        if (disconnect_mqtt == true && mqtt_connect_flag == true)                   //++ Condition for Disconnection in MQTT
        {
            for (int i = 0; i < 2; i++)
            {
                send_cmd_to_simcomm(TX_TASK_TAG, AT_DISCONNET_MQTT_COMMAND[i]);
            }
            disconnect_mqtt = false;
            mqtt_connect_flag = false;
        }

        gpio_set_level(BUILTIN_LED, 0);

        gpio_set_level(EN_GPIO, 1);                                                 //++ Restarting Simcomm via ENABLE pin
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        gpio_set_level(EN_GPIO, 0);

        vTaskDelay(20000 / portTICK_PERIOD_MS);
        next_command = true;

        for (int i = 0; i < 14; i++)                                                //++ Sending one-by-one every AT Command 
        {
            send_cmd_to_simcomm(TX_TASK_TAG, AT_COMMANDS[i]);
            if (next_command == false)
                goto start;

            // delay for PDP Configure or CSQ.
            if (i == 2 || i == 3 || i == 4)
            {
                vTaskDelay(1500 / portTICK_PERIOD_MS);
            }

            // delay to received time from sntp server or MQTT_CONNECT or Location.
            if (i == 6 || i == 7 || i == 8 || i == 12)
            {
                vTaskDelay(4000 / portTICK_PERIOD_MS);
            }
        }

        while (1)
        {
            send_cmd_to_simcomm(TX_TASK_TAG, AT_COMMANDS[2]);               //++ Check for Signal Strength in Loop
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            
            char payload[200];

            count = count + 1;
            ESP_LOGW("COUNT","Count is %lld", count);

            sprintf(payload, "{\"device\":\"%s\",\"rssi\":\"%d\",\"data\":{\"COUNT\":\"%lld\",\"LATITUDE\":\"%s\",\"LONGITUDE\":\"%s\"}}", mac_address, signal_strength, count, latitude_string, longitude_string);
            strcpy(AT_PUB_COMMAND[1], payload);                             //++ Passing above Payload 


            char pub_cmd_json[200];

            sprintf(pub_cmd_json, "AT+CMQTTPUB=0,\"%s\",1,%d\r\n", mqtt_publish_topic, strlen(AT_PUB_COMMAND[1])); 
            strcpy(AT_PUB_COMMAND[0], pub_cmd_json);

            if (mqtt_connect_flag == true)                                  //++ If MQTT is Connected, Publish the JSON
            {   
                gpio_set_level(BUILTIN_LED, 1);
                send_cmd_to_simcomm(TX_TASK_TAG, AT_PUB_COMMAND[0]);
                if (next_command == false)
                    goto start;

                send_cmd_to_simcomm(TX_TASK_TAG, AT_PUB_COMMAND[1]);
                if (next_command == false)
                    goto start;

                vTaskDelay(1500 / portTICK_PERIOD_MS);
            }

            else
            {   
                gpio_set_level(BUILTIN_LED, 0);
                ESP_LOGE("MQTT", "MQTT NOT CONNECTED\n");
                esp_restart_count++;

                if(esp_restart_count >= 3)                                  //++ If MQTT is Disconnected after 3 restart attempts of Simcomm, then restart ESP
                {
                    esp_restart();
                }
                goto start;
            }
        }
    }
}

static void rx_task(void *arg)                                              //++ UART Receive Task 
{
    static const char *RX_TASK_TAG = "SIM_RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 300 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            simcomm_response_parser((char *)data);                          //++ Call the AT Response Parser Function
        }
    }
    free(data);
}

void app_main(void)                                                         //++ Initialize Main Function
{
    simcomm_uart_init();                                                    //++ Call UART Initializing Function
    gpio_set_direction(EN_GPIO, GPIO_MODE_OUTPUT);                          //++ Set GPIO Pin Directions
    gpio_set_direction(BUILTIN_LED, GPIO_MODE_OUTPUT);

    esp_efuse_mac_get_default(esp_chip_id);                                 //++ Get the ESP Chip ID
    sprintf(mac_address, "%02x:%02x:%02x:%02x:%02x:%02x", esp_chip_id[0], esp_chip_id[1], esp_chip_id[2], esp_chip_id[3], esp_chip_id[4], esp_chip_id[5]);
    printf("MAC Address is %s\n", mac_address);

    memset(mqtt_publish_topic, 0, sizeof(mqtt_publish_topic));
    sprintf(mqtt_publish_topic, "4GBOARD_SIMCOMM");                         //++ Topic to Publish on MQTT 

    xTaskCreate(rx_task, "uart_rx_task", 2048 * 2, NULL, configMAX_PRIORITIES, NULL);               //++ Create FreeRtos Tasks
    xTaskCreate(tx_task, "uart_tx_task", 2048 * 2, NULL, configMAX_PRIORITIES - 1, NULL);

}
