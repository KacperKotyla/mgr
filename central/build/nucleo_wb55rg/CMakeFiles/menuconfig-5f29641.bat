@echo off
cd /D C:\Users\Mis\test123\central\build\nucleo_wb55rg\zephyr\kconfig || (set FAIL_LINE=2& goto :ABORT)
C:\Users\Mis\.zephyrtools\cmake\cmake-3.22.0-windows-x86_64\bin\cmake.exe -E env ZEPHYR_BASE=C:/Users/Mis/test123/external/zephyr PYTHON_EXECUTABLE=C:/Users/Mis/.zephyrtools/env/Scripts/python.exe srctree=C:/Users/Mis/test123/external/zephyr KERNELVERSION=0x3060000 APPVERSION= APP_VERSION_EXTENDED_STRING= APP_VERSION_TWEAK_STRING= CONFIG_=CONFIG_ KCONFIG_CONFIG=C:/Users/Mis/test123/central/build/nucleo_wb55rg/zephyr/.config ARCH=arm ARCH_DIR=C:/Users/Mis/test123/external/zephyr/arch BOARD_DIR=C:/Users/Mis/test123/external/zephyr/boards/arm/nucleo_wb55rg BOARD_REVISION= KCONFIG_BINARY_DIR=C:/Users/Mis/test123/central/build/nucleo_wb55rg/Kconfig APPLICATION_SOURCE_DIR=C:/Users/Mis/test123/central ZEPHYR_TOOLCHAIN_VARIANT=zephyr TOOLCHAIN_KCONFIG_DIR=C:/Users/Mis/.zephyr_ide/toolchains/zephyr-sdk-0.16.6/cmake/zephyr TOOLCHAIN_HAS_NEWLIB=y TOOLCHAIN_HAS_PICOLIBC=y EDT_PICKLE=C:/Users/Mis/test123/central/build/nucleo_wb55rg/zephyr/edt.pickle ZEPHYR_ACPICA_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/acpica ZEPHYR_ACPICA_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/acpica/Kconfig ZEPHYR_CMSIS_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/cmsis ZEPHYR_CMSIS_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/cmsis/Kconfig ZEPHYR_CMSIS_DSP_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/cmsis-dsp ZEPHYR_CMSIS_DSP_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/cmsis-dsp/Kconfig ZEPHYR_CMSIS_NN_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/cmsis-nn ZEPHYR_CMSIS_NN_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/cmsis-nn/Kconfig ZEPHYR_FATFS_MODULE_DIR=C:/Users/Mis/test123/external/modules/fs/fatfs ZEPHYR_FATFS_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/fatfs/Kconfig ZEPHYR_ALTERA_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/altera ZEPHYR_HAL_AMBIQ_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/ambiq ZEPHYR_HAL_AMBIQ_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_ambiq/Kconfig ZEPHYR_ATMEL_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/atmel ZEPHYR_HAL_ESPRESSIF_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/espressif ZEPHYR_HAL_ETHOS_U_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/ethos_u ZEPHYR_HAL_ETHOS_U_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_ethos_u/Kconfig ZEPHYR_HAL_GIGADEVICE_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/gigadevice ZEPHYR_HAL_GIGADEVICE_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_gigadevice/Kconfig ZEPHYR_HAL_INFINEON_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/infineon ZEPHYR_HAL_INFINEON_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_infineon/Kconfig ZEPHYR_HAL_INTEL_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/intel ZEPHYR_MICROCHIP_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/microchip ZEPHYR_HAL_NORDIC_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/nordic ZEPHYR_HAL_NORDIC_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_nordic/Kconfig ZEPHYR_NUVOTON_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/nuvoton ZEPHYR_HAL_NXP_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/nxp ZEPHYR_HAL_NXP_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_nxp/Kconfig ZEPHYR_OPENISA_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/openisa ZEPHYR_QUICKLOGIC_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/quicklogic ZEPHYR_HAL_RENESAS_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/renesas ZEPHYR_HAL_RPI_PICO_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/rpi_pico ZEPHYR_HAL_RPI_PICO_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_rpi_pico/Kconfig ZEPHYR_SILABS_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/silabs ZEPHYR_HAL_ST_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/st ZEPHYR_HAL_ST_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hal_st/Kconfig ZEPHYR_STM32_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/stm32 ZEPHYR_HAL_TELINK_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/telink ZEPHYR_TI_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/ti ZEPHYR_HAL_WURTHELEKTRONIK_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/wurthelektronik ZEPHYR_XTENSA_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/xtensa ZEPHYR_HOSTAP_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/hostap ZEPHYR_HOSTAP_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/hostap/Kconfig ZEPHYR_LIBMETAL_MODULE_DIR=C:/Users/Mis/test123/external/modules/hal/libmetal ZEPHYR_LIBLC3_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/liblc3 ZEPHYR_LIBLC3_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/liblc3/Kconfig ZEPHYR_LITTLEFS_MODULE_DIR=C:/Users/Mis/test123/external/modules/fs/littlefs ZEPHYR_LITTLEFS_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/littlefs/Kconfig ZEPHYR_LORAMAC_NODE_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/loramac-node ZEPHYR_LORAMAC_NODE_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/loramac-node/Kconfig ZEPHYR_LVGL_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/gui/lvgl ZEPHYR_LVGL_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/lvgl/Kconfig ZEPHYR_MBEDTLS_MODULE_DIR=C:/Users/Mis/test123/external/modules/crypto/mbedtls ZEPHYR_MBEDTLS_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/mbedtls/Kconfig ZEPHYR_MCUBOOT_MODULE_DIR=C:/Users/Mis/test123/external/bootloader/mcuboot ZEPHYR_MIPI_SYS_T_MODULE_DIR=C:/Users/Mis/test123/external/modules/debug/mipi-sys-t ZEPHYR_NRF_HW_MODELS_MODULE_DIR=C:/Users/Mis/test123/external/modules/bsim_hw_models/nrf_hw_models ZEPHYR_OPEN_AMP_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/open-amp ZEPHYR_OPENTHREAD_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/openthread ZEPHYR_OPENTHREAD_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/openthread/Kconfig ZEPHYR_PERCEPIO_MODULE_DIR=C:/Users/Mis/test123/external/modules/debug/percepio ZEPHYR_PERCEPIO_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/percepio/Kconfig ZEPHYR_PICOLIBC_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/picolibc ZEPHYR_SEGGER_MODULE_DIR=C:/Users/Mis/test123/external/modules/debug/segger ZEPHYR_SEGGER_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/segger/Kconfig ZEPHYR_TINYCRYPT_MODULE_DIR=C:/Users/Mis/test123/external/modules/crypto/tinycrypt ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR=C:/Users/Mis/test123/external/modules/tee/tf-m/trusted-firmware-m ZEPHYR_TRUSTED_FIRMWARE_M_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/trusted-firmware-m/Kconfig ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR=C:/Users/Mis/test123/external/modules/tee/tf-a/trusted-firmware-a ZEPHYR_TRUSTED_FIRMWARE_A_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/trusted-firmware-a/Kconfig ZEPHYR_UOSCORE_UEDHOC_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/uoscore-uedhoc ZEPHYR_UOSCORE_UEDHOC_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/uoscore-uedhoc/Kconfig ZEPHYR_ZCBOR_MODULE_DIR=C:/Users/Mis/test123/external/modules/lib/zcbor ZEPHYR_ZCBOR_KCONFIG=C:/Users/Mis/test123/external/zephyr/modules/zcbor/Kconfig SHIELD_AS_LIST= DTS_POST_CPP=C:/Users/Mis/test123/central/build/nucleo_wb55rg/zephyr/zephyr.dts.pre DTS_ROOT_BINDINGS=C:/Users/Mis/test123/external/zephyr/dts/bindings C:/Users/Mis/.zephyrtools/env/Scripts/python.exe C:/Users/Mis/test123/external/zephyr/scripts/kconfig/menuconfig.py C:/Users/Mis/test123/external/zephyr/Kconfig || (set FAIL_LINE=3& goto :ABORT)
goto :EOF

:ABORT
set ERROR_CODE=%ERRORLEVEL%
echo Batch file failed at line %FAIL_LINE% with errorcode %ERRORLEVEL%
exit /b %ERROR_CODE%