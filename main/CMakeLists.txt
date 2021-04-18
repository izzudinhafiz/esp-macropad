idf_component_register(SRCS "main.c"
                            "hid_dev.c"
                            "ble_profile.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
