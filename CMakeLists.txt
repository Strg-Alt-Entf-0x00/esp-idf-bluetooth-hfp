set(BLUEDROID_DIR "${IDF_PATH}/components/bt/host/bluedroid")
idf_component_register(
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "${BLUEDROID_DIR}/external/sbc/decoder/include"
                      "${BLUEDROID_DIR}/external/sbc/encoder/include"
                      "${BLUEDROID_DIR}/stack/include"
                      "${BLUEDROID_DIR}/api/include/api"
                      "${IDF_PATH}/components/bt/common/include"
                      "${BLUEDROID_DIR}/common/include"
    SRCS "src/bluetooth_hfp.cpp"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/alloc.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/bitalloc-sbc.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/bitalloc.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/bitstream-decode.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/decoder-oina.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/decoder-private.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/decoder-sbc.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/dequant.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/framing-sbc.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/framing.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/oi_codec_version.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/synthesis-8-generated.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/synthesis-dct8.c"
         "${BLUEDROID_DIR}/external/sbc/decoder/srce/synthesis-sbc.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_analysis.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_dct.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_dct_coeffs.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_enc_bit_alloc_mono.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_enc_bit_alloc_ste.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_enc_coeffs.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_encoder.c"
         "${BLUEDROID_DIR}/external/sbc/encoder/srce/sbc_packing.c"
    REQUIRES esp_driver_dac esp_system freertos nvs_flash esp_ringbuf
    PRIV_REQUIRES bt
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    SBC_ENC_INCLUDED=TRUE
    SBC_DEC_INCLUDED=TRUE
)
