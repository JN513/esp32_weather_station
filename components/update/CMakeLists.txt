idf_component_register(SRCS "update.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_https_ota esp_http_client app_update esp-tls json spiffs)
