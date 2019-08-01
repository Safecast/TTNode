## Copyright 2017 Inca Roads LLC.  All rights reserved.
## Use of this source code is governed by licenses granted by the
## copyright holder including that found in the LICENSE file.

##	TT Bootloader
##
##	Prerequisites when installing a new SDK
##
##	Unlike our main app, there are a several things that you need to do to the SDK
##	itself to prepare for compilation and linking of the bootloader, which has
##	external dependencies.
##
##	1. Make sure that sdk/12.2/components/toolchain/gcc/Makefile.posix is updated
##		with the correct GNU_INSTALL_ROOT path
##	2. Make sure that you download and unzip this github repo:
##		https://github.com/kmackay/micro-ecc
##		...into...
##		external/micro-ecc/micro-ecc
##	3. Go into this folder and do a Make, to build the micro-ecc-lib_nrf52.a library:
##		external/micro-ecc/nrf52_armgcc/armgcc
##
##	There are two modes of operation of this bootloader.
##
##	BLE Bootloader
##		When compiling with DFUFONA=no, and if you hook a physical switch between
##		SPARE_PIN (custom_board.h) and GND, you can use that switch to trigger a
##		BLE DFU just to make sure everything is working well.  In this mode, you
##		can also watch debug output on the serial port if you enable DFUDEBUGSERIAL=yes
##		Note that in this mode, you first transfer the ZIP file containing the app plus
##		init packet over to nRF Toolbox.  Then you reboot the nRF.  If the switch is
##		sensed to be OFF by nrf_dfu.c, it just boots the app.  If the switch is ON, it
##		enters the bootloader.  In nRF Toolbox you select the file, and you select
##		"DfuTarg" as the target devie.
##
##		While the BLE Bootloader is fully functional and is very educational because
##		you can watch all the logging that explains the DFU process, it is ONLY used
##		for debugging because we do not have a "boot button" built into our unit.  
##
##	FONA Bootloader
##		When compiling with DFUFONA=yes, you must have a Fona installed in our standard
##		Pin/UART configuration and standard baud rate.  Furthermore, you MUST have set
##		DFUDEBUGSERIAL=no, because the UART is being used to talk to the Fona.  How
##		then do you debug?  Well, limited debugging is available if you do two things:
##		First, get a https://www.adafruit.com/product/954 USB to serial cable.  Second,
##		hook up GND and take the white (RCV) wire and connect it to the RX pin of the
##		FONA.  (Do NOT connect the cable's green wire to anything.)  This will allow
##		you to passively monitor what is being transmitted TO the Fona chip.  (Of course
##		if you connect that white wire to the TX pin of the FONA you can passively
##		monitor what is being transmitted FROM the Fona chip.)  Then, if you compile
##		this bootloader with DFUDEBUG, you will see that we have found ways to harmlessly
##		emit certain debug information in ways that is ignored by the FONA.
##
##		The way that the Fona bootloader works is to check for the presence of DFU.DAT
##		in the Fona's file system flash.  If it exists - because it was downloaded by
##		higher level software during a previous session - then we will enter the
##		bootloader, which will use DFU.DAT (the init packet) and DFU.BIN (the image)
##		to perform the DFU.
##

APPNAME			 := 

DFUFONA			 := yes
DFUSD			 := no
DFUDEBUG		 := yes
DFUDEBUGSERIAL	 := no
PROJ_DIR		 := .
OUTPUT_DIRECTORY := bin
TTNODE_DIR		 := ..
CONFIG_DIR		 := $(TTNODE_DIR)/board/config
SDKROOT := /Users/rob/Documents/sdk
NSDK			 := $(SDKROOT)/nRF5_SDK_12.2.0_f012efa
ifeq ($(APPNAME),breadboard)
BOARD			 := scv0
endif
ifeq ($(APPNAME),solarcast)
BOARD			 := scv1
endif
ifeq ($(APPNAME),scnano)
BOARD			 := scv2
endif
TARGETS			 := ttboot-$(BOARD)

## Just safety checks to prevent cockpit errors in configuring things above
ifneq ("$(DFUFONA)","yes")
ifeq ("$(DFUDEBUG)","yes")
DFUDEBUGSERIAL	 := yes
endif
ifneq ("$(DFUSD)","yes")
DFUSD			 := yes
endif
else
DFUDEBUGSERIAL	 := no
endif

ifeq ("$(DFUDEBUGSERIAL)","yes")
$(OUTPUT_DIRECTORY)/$(TARGETS).hex: $(OUTPUT_DIRECTORY)/$(TARGETS).bin
  LINKER_SCRIPT  := $(PROJ_DIR)/secure_dfu_gcc_nrf52.ld
else
## Ultimately for mem size it might be good to figure out what the non-debug size should be, update the .ld, and eliminate the _debug below
$(OUTPUT_DIRECTORY)/$(TARGETS).hex: $(OUTPUT_DIRECTORY)/$(TARGETS).bin
  LINKER_SCRIPT  := $(PROJ_DIR)/secure_dfu_gcc_nrf52.ld
endif

# Source files common to all targets
ifeq ("$(DFUDEBUG)","yes")
SRC_FILES += \
  $(NSDK)/components/libraries/log/src/nrf_log_frontend.c
endif

ifeq ("$(DFUDEBUGSERIAL)","yes")
SRC_FILES += \
  $(NSDK)/components/libraries/log/src/nrf_log_backend_serial.c
endif

SRC_FILES += \
  $(TTNODE_DIR)/src/debug.c \
  $(TTNODE_DIR)/src/gpio.c \
  $(TTNODE_DIR)/src/serial.c

SRC_FILES += \
  $(NSDK)/components/libraries/uart/app_uart_fifo.c \
  $(NSDK)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(NSDK)/components/libraries/util/app_error_weak.c \
  $(NSDK)/components/libraries/fifo/app_fifo.c \
  $(NSDK)/components/libraries/timer/app_timer.c \
  $(NSDK)/components/libraries/timer/app_timer_appsh.c \
  $(NSDK)/components/libraries/util/app_util_platform.c \
  $(NSDK)/components/libraries/hardfault/nrf52/handler/hardfault_handler_gcc.c \
  $(NSDK)/components/libraries/hardfault/hardfault_implementation.c \
  $(NSDK)/components/libraries/crc32/crc32.c \
  $(NSDK)/components/libraries/ecc/ecc.c \
  $(NSDK)/components/libraries/fstorage/fstorage.c \
  $(NSDK)/components/libraries/hci/hci_mem_pool.c \
  $(NSDK)/components/libraries/util/nrf_assert.c \
  $(NSDK)/components/libraries/crypto/nrf_crypto.c \
  $(NSDK)/components/libraries/queue/nrf_queue.c \
  $(NSDK)/components/libraries/sha256/sha256.c \
  $(NSDK)/components/drivers_nrf/common/nrf_drv_common.c \
  $(NSDK)/components/drivers_nrf/rng/nrf_drv_rng.c \
  $(NSDK)/components/drivers_nrf/hal/nrf_nvmc.c \
  $(PROJ_DIR)/dfu-cc/dfu-cc.pb.c \
  $(PROJ_DIR)/dfu_public_key.c \
  $(PROJ_DIR)/dfu_req_handling.c \
  $(PROJ_DIR)/main.c \
  $(NSDK)/external/nano-pb/pb_common.c \
  $(NSDK)/external/nano-pb/pb_decode.c \
  $(NSDK)/components/toolchain/gcc/gcc_startup_nrf52.S \
  $(NSDK)/components/toolchain/system_nrf52.c \
  $(NSDK)/components/libraries/bootloader/nrf_bootloader.c \
  $(NSDK)/components/libraries/bootloader/nrf_bootloader_info.c \
  $(NSDK)/components/libraries/bootloader/dfu/nrf_dfu.c \
  $(NSDK)/components/libraries/bootloader/dfu/nrf_dfu_flash.c \
  $(NSDK)/components/libraries/bootloader/dfu/nrf_dfu_mbr.c \
  $(NSDK)/components/libraries/bootloader/dfu/nrf_dfu_settings.c \
  $(NSDK)/components/libraries/bootloader/dfu/nrf_dfu_transport.c \
  $(NSDK)/components/libraries/bootloader/dfu/nrf_dfu_utils.c

## SDK modifications - must be checked manually each time we change SDKs
# $(NSDK)/components/libraries/bootloader/nrf_bootloader_app_start.c 
# $(NSDK)/components/libraries/scheduler/app_scheduler.c
SRC_FILES += \
  sdk-mods/nrf_bootloader_app_start.c \
  sdk-mods/app_scheduler.c

ifeq ("$(DFUSD)","yes")
SRC_FILES += \
  $(NSDK)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(NSDK)/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c
endif

ifneq ("$(DFUFONA)","yes")
SRC_FILES += \
  $(NSDK)/components/libraries/bootloader/ble_dfu/nrf_ble_dfu.c \
  $(NSDK)/components/ble/common/ble_advdata.c \
  $(NSDK)/components/ble/common/ble_conn_params.c \
  $(NSDK)/components/ble/common/ble_srv_common.c
else
SRC_FILES += \
  $(PROJ_DIR)/fona.c
endif

##  $(PROJ_DIR)/../../bsp

# Include folders common to all targets
INC_FOLDERS += \
  sdk-mods/ \
  $(CONFIG_DIR)/ \
  $(TTNODE_DIR)/src \
  $(NSDK)/components/libraries/hardfault \
  $(NSDK)/components/libraries/hardfault/nrf52 \
  $(NSDK)/components/toolchain/cmsis/include \
  $(NSDK)/components/drivers_nrf/rng \
  $(NSDK)/components/toolchain \
  $(NSDK)/components/device \
  $(NSDK)/components/drivers_nrf/uart \
  $(NSDK)/components/drivers_nrf/hal \
  $(NSDK)/components/libraries/sha256 \
  $(NSDK)/components/libraries/crc32 \
  $(NSDK)/components/libraries/uart \
  $(NSDK)/components/libraries/experimental_section_vars \
  $(NSDK)/components/softdevice/common/softdevice_handler \
  $(PROJ_DIR) \
  $(PROJ_DIR)/dfu-cc \
  $(NSDK)/components/libraries/fstorage \
  $(NSDK)/components/libraries/util \
  $(NSDK)/components \
  $(NSDK)/components/libraries/timer \
  $(NSDK)/components/drivers_nrf/clock \
  $(NSDK)/components/softdevice/s132/headers/nrf52 \
  $(NSDK)/components/libraries/log/src \
  $(NSDK)/components/libraries/bootloader/dfu \
  $(NSDK)/components/drivers_nrf/delay \
  $(NSDK)/components/libraries/fifo \
  $(NSDK)/components/libraries/svc \
  $(NSDK)/components/libraries/log \
  $(NSDK)/components/libraries/hci \
  $(NSDK)/components/libraries/scheduler \
  $(NSDK)/components/libraries/bootloader \
  $(NSDK)/components/libraries/crypto \
  $(NSDK)/components/libraries/queue \
  $(NSDK)/components/softdevice/s132/headers \
  $(NSDK)/components/drivers_nrf/common \
  $(NSDK)/external/micro-ecc/micro-ecc \
  $(NSDK)/external/nano-pb \
  $(NSDK)/components/libraries/ecc \
  $(NSDK)/components/toolchain/gcc

ifneq ("$(DFUFONA)","yes")
INC_FOLDERS += \
  $(NSDK)/components/ble/common \
  $(NSDK)/components/libraries/bootloader/ble_dfu
endif

# Libraries common to all targets
LIB_FILES += \
  $(NSDK)/external/micro-ecc/nrf52_armgcc/armgcc/micro_ecc_lib_nrf52.a \

# C flags common to all targets
CFLAGS += -D$(BOARD)
ifeq ("$(DFUDEBUG)","yes")
CFLAGS += -DDFUDEBUG
endif
ifeq ("$(DFUFONA)","yes")
CFLAGS += -DCELLX -DFONA -DDFUFONA -DHWFC=false
ifneq ($(BOARD),scv2)
CFLAGS += -DUSX -DUSLORA=USab -DUSFONA=USAb -DUSPMS=USaB -DUSGPS=USAB
endif
endif
## Ideally BLE_BOOTLOADER shouldn't be needed if not BLE, however if
## missing it generates a compile error in ble_dfu.c because of
## a requirement for the boot button definition even in the case
## where that _WEAK method isn't even included into the final
## executable bootloader!  So much for purity.
CFLAGS += -DBLE_BOOTLOADER
CFLAGS += -DCLOCK_ENABLED=0
CFLAGS += -DBOOTLOADERX
CFLAGS += -DNRF52
CFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF52832
ifeq ("$(DFUDEBUG)","yes")
CFLAGS += -DNRF_LOG_ENABLED=1
endif
ifeq ("$(DFUDEBUGSERIAL)","yes")
CFLAGS += -DDEBUG_USES_UART
CFLAGS += -DNRF_DFU_DEBUG_VERSION
endif
CFLAGS += -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_30
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DNRF52_PAN_53
CFLAGS += -D__HEAP_SIZE=0
#CFLAGS += -D__HEAP_SIZE=1024
#CFLAGS += -D__STACK_SIZE=2048
CFLAGS += -DS132
## Holy crap this was difficult. On 2017-02-15 Vidar Berg helped out.
## The bug was that Pin 21 in my app (UART_SELECT0) was not working,
## and was pinned high, but only when run via bootloader.  Apparently,
## this flag steals Pin 21 without any indictation within custom_board.h
## or sdk_config.h. This is implemented in system_nrf52.c.  Unbelievable.
#CFLAGS += -DCONFIG_GPIO_AS_PINRESET
ifeq ("$(DFUSD)","yes")
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF_SD_BLE_API_VERSION=3
CFLAGS += -DSOFTDEVICE_PRESENT
endif
CFLAGS += -DSERIAL_DFU_APP_REQUIRES_SD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
#CFLAGS += -fno-builtin --short-enums -flto
CFLAGS += -fno-builtin --short-enums
# generate dependency output file
CFLAGS += -MP -MD

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
ifeq ("$(DFUDEBUGSERIAL)","yes")
ASMFLAGS += -DNRF_DFU_DEBUG_VERSION
endif
ASMFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF52832
ASMFLAGS += -DNRF52_PAN_12
ASMFLAGS += -DNRF52_PAN_15
ASMFLAGS += -DNRF52_PAN_58
ASMFLAGS += -DNRF52_PAN_55
ASMFLAGS += -DNRF52_PAN_54
ASMFLAGS += -DNRF52_PAN_31
ASMFLAGS += -DNRF52_PAN_30
ASMFLAGS += -DNRF52_PAN_51
ASMFLAGS += -DNRF52_PAN_36
ASMFLAGS += -DNRF52_PAN_53
ASMFLAGS += -D__HEAP_SIZE=0
#ASMFLAGS += -D__HEAP_SIZE=1024
#ASMFLAGS += -D__STACK_SIZE=2048
ASMFLAGS += -DS132
ifeq ("$(DFUSD)","yes")
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=3
ASMFLAGS += -DSOFTDEVICE_PRESENT
endif
ASMFLAGS += -DSERIAL_DFU_APP_REQUIRES_SD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DNRF52_PAN_62
ASMFLAGS += -DNRF52_PAN_63

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

.PHONY: $(TARGETS) default all clean help flash 
# Default target - first one defined
default: $(NSDK)/external/micro-ecc/nrf52_armgcc/armgcc/micro_ecc_lib_nrf52.a $(TARGETS)

##### Micro ECC SDK
# Reason for needing is here:
# http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v12.0.0%2Flib_crypto.html&cp=4_0_0_3_3_1_2&anchor=lib_crypto_installing
# Unzip the SDK into $(SDKROOT)/external/micro-ecc/micro-ecc from here.  A library is then built below
# https://github.com/kmackay/micro-ecc
$(NSDK)/external/micro-ecc/nrf52_armgcc/armgcc/micro_ecc_lib_nrf52.a:
	@-$(shell cd $(NSDK)/external/micro-ecc/nrf52_armgcc/armgcc && make micro_ecc_lib)

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf52832_xxaa_s132

TEMPLATE_PATH := $(NSDK)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common
$(foreach target, $(TARGETS), $(call define_target, $(target)))
-include $(foreach target, $(TARGETS), $($(target)_dependencies))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/$(TARGETS).hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf52 --chiperase
	nrfjprog --reset -f nrf52

