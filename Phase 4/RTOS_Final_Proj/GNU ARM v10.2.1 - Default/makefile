################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/Source_Files/subdir.mk
-include src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/system/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/device_init/src/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/ports/source/gnu/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/ports/source/generic/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/kernel/source/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/cpu/source/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/common/source/rtos/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/common/source/ring_buf/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/common/source/logging/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/common/source/lib/subdir.mk
-include gecko_sdk_3.2.1/platform/micrium_os/common/source/kal/subdir.mk
-include gecko_sdk_3.2.1/platform/emlib/src/subdir.mk
-include gecko_sdk_3.2.1/platform/driver/leddrv/src/subdir.mk
-include gecko_sdk_3.2.1/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_3.2.1/platform/common/src/subdir.mk
-include gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFM32PG12B/Source/GCC/subdir.mk
-include gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFM32PG12B/Source/subdir.mk
-include gecko_sdk_3.2.1/platform/CMSIS/RTOS2/Source/subdir.mk
-include gecko_sdk_3.2.1/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include SystemView/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: RTOS_Final_Proj.axf

# Tool invocations
RTOS_Final_Proj.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\zande\SimplicityStudio\v5_workspace\RTOS_Final_Proj/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="RTOS_Final_Proj.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o RTOS_Final_Proj.axf -Wl,--start-group "./SystemView/SEGGER_RTT.o" "./SystemView/SEGGER_RTT_ASM_ARMv7M.o" "./SystemView/SEGGER_SYSVIEW.o" "./SystemView/SEGGER_SYSVIEW_Config_MicriumOSKernel.o" "./SystemView/SEGGER_SYSVIEW_MicriumOSKernel.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_simple_led_instances.o" "./gecko_sdk_3.2.1/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_3.2.1/hardware/board/src/sl_board_init.o" "./gecko_sdk_3.2.1/platform/CMSIS/RTOS2/Source/os_systick.o" "./gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFM32PG12B/Source/GCC/startup_efm32pg12b.o" "./gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFM32PG12B/Source/system_efm32pg12b.o" "./gecko_sdk_3.2.1/platform/common/src/sl_slist.o" "./gecko_sdk_3.2.1/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_3.2.1/platform/driver/leddrv/src/sl_led.o" "./gecko_sdk_3.2.1/platform/driver/leddrv/src/sl_simple_led.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_acmp.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_assert.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_cmu.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_core.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_emu.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_gpio.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_rtcc.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_system.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/kal/kal_kernel.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/lib/lib_ascii.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/lib/lib_mem.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/lib/lib_str.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/logging/logging.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/ring_buf/ring_buf.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/rtos/rtos_err_str.o" "./gecko_sdk_3.2.1/platform/micrium_os/common/source/rtos/rtos_utils.o" "./gecko_sdk_3.2.1/platform/micrium_os/cpu/source/cpu_core.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/cmsis_os2.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_cfg_app.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_core.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_dbg.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_flag.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_mon.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_msg.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_mutex.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_prio.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_q.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_sem.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_stat.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_task.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_time.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_tmr.o" "./gecko_sdk_3.2.1/platform/micrium_os/kernel/source/os_var.o" "./gecko_sdk_3.2.1/platform/micrium_os/ports/source/generic/arm_cpu_dwt_ts.o" "./gecko_sdk_3.2.1/platform/micrium_os/ports/source/gnu/armv7m_cpu_a.o" "./gecko_sdk_3.2.1/platform/micrium_os/ports/source/gnu/armv7m_os_cpu_a.o" "./gecko_sdk_3.2.1/platform/micrium_os/ports/source/gnu/armv7m_os_cpu_c.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_dcdc_s1.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_emu_s1.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_hfxo_s1.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_lfxo_s1.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_3.2.1/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_3.2.1/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_3.2.1/platform/service/system/src/sl_system_init.o" "./gecko_sdk_3.2.1/platform/service/system/src/sl_system_kernel.o" "./src/Source_Files/app.o" "./src/Source_Files/bsp_os.o" "./src/Source_Files/capsense.o" "./src/Source_Files/cmu.o" "./src/Source_Files/gpio.o" "./src/Source_Files/pushbutton.o" "./src/Source_Files/queue.o" "./src/Source_Files/resources.o" "./src/Source_Files/task.o" "./src/main.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: RTOS_Final_Proj.hex'
	arm-none-eabi-objcopy -O ihex "RTOS_Final_Proj.axf" "RTOS_Final_Proj.hex"
	@echo ' '

	@echo 'Building bin file: RTOS_Final_Proj.bin'
	arm-none-eabi-objcopy -O binary "RTOS_Final_Proj.axf" "RTOS_Final_Proj.bin"
	@echo ' '

	@echo 'Building s37 file: RTOS_Final_Proj.s37'
	arm-none-eabi-objcopy -O srec "RTOS_Final_Proj.axf" "RTOS_Final_Proj.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "RTOS_Final_Proj.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) RTOS_Final_Proj.axf
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
