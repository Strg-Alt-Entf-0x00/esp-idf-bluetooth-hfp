set(BLUEDROID_DIR "${IDF_PATH}/components/bt/host/bluedroid")
if(CONFIG_BT_HFP_USE_EXTERNAL_CODEC)
    file(GLOB_RECURSE SBC_SOURCES "third_party/sbc/*.c")
endif()

idf_component_register(
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS
                      "${BLUEDROID_DIR}/stack/include"
                      "${BLUEDROID_DIR}/api/include/api"
                      "${IDF_PATH}/components/bt/common/include"
                      "${BLUEDROID_DIR}/common/include"
    SRCS "src/bluetooth_hfp.cpp"
         ${SBC_SOURCES}
    REQUIRES esp_system freertos nvs_flash esp_ringbuf
    PRIV_REQUIRES bt
)

if(CONFIG_BT_HFP_USE_EXTERNAL_CODEC)
    target_include_directories(${COMPONENT_LIB} PRIVATE
        "third_party/sbc/decoder/include"
        "third_party/sbc/encoder/include"
        "third_party/sbc/plc/include"
    )
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        SBC_ENC_INCLUDED=TRUE
        SBC_DEC_INCLUDED=TRUE
    )
else()
    target_include_directories(${COMPONENT_LIB} PRIVATE
        "${BLUEDROID_DIR}/external/sbc/decoder/include"
        "${BLUEDROID_DIR}/external/sbc/encoder/include"
        "${BLUEDROID_DIR}/external/sbc/plc/include"
    )
endif()
