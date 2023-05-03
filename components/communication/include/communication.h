#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"     // FreeRTOS
#include "freertos/task.h"         // Task
#include "freertos/event_groups.h" // Event Groups

#include "esp_system.h" // ESP32
#include "esp_wifi.h"   // Biblioteca do Wifi
#include "esp_wpa2.h"
#include "esp_event.h" // Biblioteca de Eventos
#include "esp_log.h"   // Biblioteca de Log
#include "esp_netif.h"

#include "lwip/err.h" // Biblioteca de Erros
#include "lwip/sys.h" // Biblioteca de Sistema

#include "mqtt_client.h" // Biblioteca MQTT

#define WIFI_CONNECTED_BIT BIT0 // Bit 0 - Wifi conectado
#define WIFI_FAIL_BIT BIT1      // Bit 1 - Falha na conexão
#define maximum_retry 50        // Quantidade máxima de tentativas de conexão

#define AP_MODE 0           // Modo AP
#define STATION_MODE 1      // Modo Station
#define STATION_EAP_MODE 2  // Modo Enterprise Station


typedef struct {
    char uuid[9];
    int8_t wifi_mode;          // Modo do Wi-Fi
    char ap_ssid[32];          // SSID do AP
    char ap_password[64];      // Senha do AP
    char ssid[32];             // SSID da rede
    char password[64];         // Senha da rede
    char eap_username[32];     // Usuário EAP
    char eap_password[64];     // Senha EAP
    char eap_identity[64];     // Identidade EAP
} _wifi_config_t;

typedef struct {
    char mqtt_server_uri[128]; // Endereço do servidor MQTT
    char mqtt_server_ip[18];   // Endereço IP do servidor MQTT
    char mqtt_user[32];        // Usuário do servidor MQTT
    char mqtt_password[64];    // Senha do servidor MQTT
    int16_t mqtt_server_port;  // Porta do servidor MQTT
    char mqtt_topic[128];      // Tópico do servidor MQTT
    bool use_uri;
} mqtt_config_t;

static const char *CommunicationTAG = "communication"; // Tag usada para Logs

static bool MQTT_CONNECTED = false; // Estado da conexão MQTT

void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
esp_err_t config_wifi();
esp_err_t init_wifi();

char *get_ip_address(void);
char *get_mac_address(void);

esp_err_t config_mqtt();
void setcallback(void (*_callback)(char *, char *, unsigned int, unsigned int));
void log_error_if_nonzero(const char *message, int error_code);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
esp_err_t mqtt_start(void);
int send_message(char *topic, char message[]);

#ifdef __cplusplus
}
#endif

#endif