# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

zephyr/drivers/spi/CMakeFiles/drivers__spi.dir/spi_ll_stm32.c.obj: \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cachel1_armv7.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cmsis_armcc.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cmsis_armclang.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cmsis_armclang_ltm.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cmsis_compiler.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cmsis_iccarm.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/cmsis_version.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm0.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm0plus.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm1.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm23.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm3.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm33.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm55.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/core_cm7.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/mpu_armv7.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/mpu_armv8.h \
 /home/rts/zephyr-2.7.0/modules/hal/cmsis/CMSIS/Core/Include/pmu_armv8.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/common_ll/include/stm32_ll_spi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/Legacy/stm32_hal_legacy.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/Legacy/stm32l4xx_hal_can_legacy.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_adc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_adc_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_can.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_comp.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_conf.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_cortex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_crc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_crc_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_cryp.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_cryp_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dac.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dac_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dcmi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_def.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dfsdm.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dfsdm_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dma.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dma2d.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dma_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_dsi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_exti.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_firewall.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_flash.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_flash_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_flash_ramfunc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_gfxmmu.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_gpio.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_gpio_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_hash.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_hash_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_hcd.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_i2c.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_i2c_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_irda.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_irda_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_iwdg.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_lcd.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_lptim.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_ltdc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_ltdc_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_mmc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_mmc_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_nand.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_nor.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_opamp.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_opamp_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_ospi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_pcd.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_pcd_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_pka.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_pssi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_pwr.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_pwr_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_qspi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_rcc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_rcc_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_rng.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_rng_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_rtc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_rtc_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_sai.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_sai_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_sd.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_sd_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_smartcard.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_smartcard_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_smbus.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_smbus_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_spi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_spi_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_sram.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_swpmi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_tim.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_tim_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_tsc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_uart.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_uart_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_usart.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_usart_ex.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_hal_wwdg.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_ll_adc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_ll_exti.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_ll_fmc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_ll_sdmmc.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_ll_spi.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/drivers/include/stm32l4xx_ll_usb.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l412xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l422xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l431xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l432xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l433xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l442xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l443xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l451xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l452xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l462xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l471xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l475xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l476xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l485xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l486xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l496xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4a6xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4p5xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4q5xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4r5xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4r7xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4r9xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4s5xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4s7xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4s9xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/stm32l4xx.h \
 /home/rts/zephyr-2.7.0/modules/hal/stm32/stm32cube/stm32l4xx/soc/system_stm32l4xx.h \
 /home/rts/zephyr-2.7.0/zephyr/drivers/pinmux/pinmux_stm32.h \
 /home/rts/zephyr-2.7.0/zephyr/drivers/spi/spi_context.h \
 /home/rts/zephyr-2.7.0/zephyr/drivers/spi/spi_ll_stm32.c \
 /home/rts/zephyr-2.7.0/zephyr/drivers/spi/spi_ll_stm32.h \
 /home/rts/zephyr-2.7.0/zephyr/include/app_memory/mem_domain.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/arc_addr_types.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/arch_inlines.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/sys-io-common.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/arc_connect.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/arcv2_irq_unit.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/asm_inline.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/asm_inline_gcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/aux_regs.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/error.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/exc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/irq.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/misc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/mpu/arc_mpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/secureshield/arc_secure.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arc/v2/sys_io.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arch_inlines.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/asm_inline.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/asm_inline_gcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_a_r/cmsis.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_a_r/cmsis_ext.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_a_r/cpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_a_r/mpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_a_r/sys_io.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_a_r/timer.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_m/cmsis.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_m/cpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_m/memory_map.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/cortex_m/nvic.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/error.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/exc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/irq.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/misc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/mpu/arm_mpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/mpu/arm_mpu_v7m.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/mpu/arm_mpu_v8m.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/mpu/nxp_mpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/nmi.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm/aarch32/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/arch_inlines.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/arm_mmu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/asm_inline.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/asm_inline_gcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/cpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/error.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/exc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/irq.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/lib_helpers.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/macro.inc \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/misc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/structs.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/sys_io.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/thread_stack.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/timer.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/arm64/tpidrro_el0.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/common/addr_types.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/common/ffs.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/common/sys_bitops.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/common/sys_io.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/cpu.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/nios2/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/nios2/asm_inline.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/nios2/asm_inline_gcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/nios2/nios2.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/nios2/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/posix/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/posix/asm_inline.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/posix/asm_inline_gcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/posix/posix_soc_if.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/posix/posix_trace.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/posix/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/csr.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/error.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/exp.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/riscv-privilege/asm_inline.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/riscv-privilege/asm_inline_gcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/riscv/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/sparc/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/sparc/sparc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/sparc/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/structs.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/arch_inlines.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/ia32/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/ia32/gdbstub.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/ia32/segmentation.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/ia32/sys_io.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/ia32/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/ia32/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/intel64/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/intel64/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/intel64/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/mmustructs.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/msr.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/x86/thread_stack.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/xtensa/arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/xtensa/arch_inlines.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/xtensa/atomic_xtensa.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/xtensa/exc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/xtensa/irq.h \
 /home/rts/zephyr-2.7.0/zephyr/include/arch/xtensa/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/device.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/clocks.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/dma.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/fixed-partitions.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/gpio.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/io-channels.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/ordinals.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/pinctrl.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/pwms.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/spi.h \
 /home/rts/zephyr-2.7.0/zephyr/include/devicetree/zephyr.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/clock_control.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/clock_control/stm32_clock_control.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/dma.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/dma/dma_stm32.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/gpio.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/interrupt_controller/loapic.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/interrupt_controller/sysapic.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/spi.h \
 /home/rts/zephyr-2.7.0/zephyr/include/drivers/timer/arm_arch_timer.h \
 /home/rts/zephyr-2.7.0/zephyr/include/dt-bindings/clock/stm32_clock.h \
 /home/rts/zephyr-2.7.0/zephyr/include/dt-bindings/gpio/gpio.h \
 /home/rts/zephyr-2.7.0/zephyr/include/dt-bindings/interrupt-controller/arm-gic.h \
 /home/rts/zephyr-2.7.0/zephyr/include/dt-bindings/pinctrl/stm32-pinctrl-common.h \
 /home/rts/zephyr-2.7.0/zephyr/include/dt-bindings/pinctrl/stm32-pinctrl.h \
 /home/rts/zephyr-2.7.0/zephyr/include/dt-bindings/pinctrl/stm32f1-pinctrl.h \
 /home/rts/zephyr-2.7.0/zephyr/include/fatal.h \
 /home/rts/zephyr-2.7.0/zephyr/include/init.h \
 /home/rts/zephyr-2.7.0/zephyr/include/irq.h \
 /home/rts/zephyr-2.7.0/zephyr/include/irq_offload.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel/mempool_heap.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel/sched_priq.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel/thread.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel/thread_stack.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel_includes.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel_structs.h \
 /home/rts/zephyr-2.7.0/zephyr/include/kernel_version.h \
 /home/rts/zephyr-2.7.0/zephyr/include/linker/section_tags.h \
 /home/rts/zephyr-2.7.0/zephyr/include/linker/sections.h \
 /home/rts/zephyr-2.7.0/zephyr/include/logging/log.h \
 /home/rts/zephyr-2.7.0/zephyr/include/logging/log_core.h \
 /home/rts/zephyr-2.7.0/zephyr/include/logging/log_core2.h \
 /home/rts/zephyr-2.7.0/zephyr/include/logging/log_instance.h \
 /home/rts/zephyr-2.7.0/zephyr/include/logging/log_msg.h \
 /home/rts/zephyr-2.7.0/zephyr/include/logging/log_msg2.h \
 /home/rts/zephyr-2.7.0/zephyr/include/pm/device.h \
 /home/rts/zephyr-2.7.0/zephyr/include/pm/pm.h \
 /home/rts/zephyr-2.7.0/zephyr/include/pm/state.h \
 /home/rts/zephyr-2.7.0/zephyr/include/spinlock.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sw_isr_table.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/__assert.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/arch_interface.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/atomic.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/atomic_arch.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/atomic_builtin.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/atomic_c.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/cbprintf.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/cbprintf_cxx.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/cbprintf_internal.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/device_mmio.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/dlist.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/kobject.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/list_gen.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/mem_manage.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/mpsc_packet.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/printk.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/rb.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/sflist.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/slist.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/sys_heap.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/sys_io.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/time_units.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/util.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/util_internal.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/util_loops.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys/util_macro.h \
 /home/rts/zephyr-2.7.0/zephyr/include/sys_clock.h \
 /home/rts/zephyr-2.7.0/zephyr/include/syscall.h \
 /home/rts/zephyr-2.7.0/zephyr/include/timing/timing.h \
 /home/rts/zephyr-2.7.0/zephyr/include/timing/types.h \
 /home/rts/zephyr-2.7.0/zephyr/include/toolchain.h \
 /home/rts/zephyr-2.7.0/zephyr/include/toolchain/armclang.h \
 /home/rts/zephyr-2.7.0/zephyr/include/toolchain/common.h \
 /home/rts/zephyr-2.7.0/zephyr/include/toolchain/gcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/toolchain/llvm.h \
 /home/rts/zephyr-2.7.0/zephyr/include/toolchain/mwdt.h \
 /home/rts/zephyr-2.7.0/zephyr/include/toolchain/xcc.h \
 /home/rts/zephyr-2.7.0/zephyr/include/tracing/tracing.h \
 /home/rts/zephyr-2.7.0/zephyr/include/tracing/tracing_macros.h \
 /home/rts/zephyr-2.7.0/zephyr/include/zephyr/types.h \
 /home/rts/zephyr-2.7.0/zephyr/lib/libc/newlib/include/stdint.h \
 /home/rts/zephyr-2.7.0/zephyr/soc/arm/st_stm32/common/st_stm32_dt.h \
 /home/rts/zephyr-2.7.0/zephyr/soc/arm/st_stm32/stm32l4/soc.h \
 zephyr/include/generated/autoconf.h \
 zephyr/include/generated/device_extern.h \
 zephyr/include/generated/devicetree_fixups.h \
 zephyr/include/generated/devicetree_unfixed.h \
 zephyr/include/generated/kobj-types-enum.h \
 zephyr/include/generated/syscall_list.h \
 zephyr/include/generated/syscalls/atomic_c.h \
 zephyr/include/generated/syscalls/device.h \
 zephyr/include/generated/syscalls/dma.h \
 zephyr/include/generated/syscalls/error.h \
 zephyr/include/generated/syscalls/gpio.h \
 zephyr/include/generated/syscalls/kernel.h \
 zephyr/include/generated/syscalls/kobject.h \
 zephyr/include/generated/syscalls/log_core.h \
 zephyr/include/generated/syscalls/log_msg2.h \
 zephyr/include/generated/syscalls/mem_manage.h \
 zephyr/include/generated/syscalls/spi.h \
 zephyr/include/generated/syscalls/time_units.h
