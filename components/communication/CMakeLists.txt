idf_component_register(SRCS "communication.c"
                    INCLUDE_DIRS "include"
                    REQUIRES mqtt wpa_supplicant storage_manager)
