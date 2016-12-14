# 2.1.7
# !!!! BETA Makefile !!!!
# !!!! Use at own risk !!!!
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make install = Install the most recently compiled hex file.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------

#----------- INSTALL OPTIONS ---------------------------

# Data directory we write hex files to.
DATA_DIR = /tmp

# Backup the old flash (YES, NO)
BACKUP_FLASH = YES

# Backup the old EEPROM (YES, NO)
BACKUP_EEPROM = YES

# Restore EEPROM to 9x (YES, NO) (will set BACKUP_EEPROM to YES).
RESTORE_EEPROM = NO

# Double write OK (YES, NO)
# Set this *if only if* your programmer can correctly write the EEPROM
# and FLASH in one command. If it doesn't, try setting the 9x fuse to
# not erase the eeprom when you erase the flash.
DOUBLE_WRITE_OK = NO

# Command to read/write FLASH/EEPROM images. Currently
# Must have avrdude interface
AVRDUDE = /usr/local/bin/avrdude

# Device options for AVRDUDE Command.
AVRDEVICE = -c usbasp -p m64

# Delay to start second AVRDUDE command (required by some).
DELAY = 5

#----------- BUILD OPTIONS ---------------------------

# PCB version
# Values: 9X, 9X128, 9X2561, 9XR, 9XR128, 9XR2561, GRUVIN9X, MEGA2560, SKY9X, 9XRPRO, AR9X, TARANIS
# Old values STD and STD128 are mapped to 9X and 9X128 for compatibility
PCB = TARANIS

# PCB revision
PCBREV = REVPLUS

# LCD
# Values: DEFAULT, ST7565P, ST7565R, ERC12864FSF, ST7920
LCD = DEFAULT

# Enable JETI-Telemetry or FrSky Telemetry reception on UART0
# For this option you need to modify your hardware!
# More information at [insertURLhere]
# Values = STD, FRSKY, FRSKY_SPORT, JETI, NMEA, ARDUPILOT, MAVLINK, TELEMETREZ
EXT = MAVLINK

# GUI enabled
# Values = YES, NO
GUI = YES

# FAI mode
# Values = YES, NO, CHOICE
FAI = NO

# the latests deliveries for Turnigy 9x transmitters (since 2 years now, 2014) do not stop sending if no pulses are generated.
# This fact totally breaks the nice throttle and switch warning concept, because even we prevent sending pulses the
# transmitter module ignores this, and continues to send the last value or 0 at beginning. And 0 means in the middle of throttle -> 50%.
# To cope with this situation, we need a new concept how to handle throttle and switch warnings.
# if you have a 0 value (50% ) for throttle, while throttle warning is active, you should activate this switch
# Values = YES, NO
TURNIGY_TRANSMITTER_FIX = NO

# Enable heli menu
# Values = YES, NO
HELI = NO

# Enable templates menu
# Values = YES, NO
TEMPLATES = $(GUI)

# Enable navigation with Pots / Sticks / Rotary encoders
# Values = NO
#          POT1 (only POT1 available for fields modification),
#          POT2 (menus scroll),
#          POT3 (cursor down/up),
#          POTS (POT1, POT2, POT3),
#          STICKS
#          ROTENC (Rotary encoder)
NAVIGATION = NO

# Switches automatic selection
# Values = YES, NO
AUTOSWITCH = NO

# Stock sticks replaced by FrSky ones
# Values = YES, NO
FRSKY_STICKS = NO

# Sources automatic selection
# Values = YES, NO
AUTOSOURCE = NO

# Double keys support (+/- means invert, +/up means max, -/dwn means min, up/down means reset to default)
# Values = YES, NO
DBLKEYS = NO

# AUDIO Mods
# Values = YES, NO
AUDIO = NO

# VOICE Mods
# Values = YES, NO
VOICE = NO

# corrects different rounding for negative values. So instead of -99.9 you will see again -100.0
# Values = YES, NO apple：may be we can use later, original:NO 
CORRECT_NEGATIVE_VALUES = YES

# adds additional code to prevent arithmetic overflows. MAX or MIN values stays. Not yet guaranteed!! Just improved
# Values = YES, NO apple：may be we can use later, original:NO 
ARITHMETIC_OVERFLOW_CHECK = YES

# increases resolution for throttle statistics and throttle counting, in very rare cases.
# this will be only true if an output channel is used for throttle calculation, and this channel has changed limits, e.g. -90% to +80%
# if you use standard limits for throttle or directly the throttle stick you do not need to bother about this option
# not valid for ARM based transmitters like Taranis, They do not have this rounding issue at all
# Values = YES, NO
ACCURAT_THROTTLE_STATS = YES

# HAPTIC Mods
# Values = YES, NO
HAPTIC = NO

# SmartieParts 2.2 Backlight support (inverted)
# Values = YES, NO
SP22 = NO

# Fade Backlight using PWM
# Values = YES, NO
PWM_BACKLIGHT = NO

# SPLASH on START
# Values = YES, NO, FIXED
SPLASH = $(GUI)

# GRAPHICS (sliders / checkboxes / bold / batt graph)
GRAPHICS = YES
BOLD = YES
BATTGRAPH = NO

# FONT TYPE
# Values = STD, SQT5
FONT = STD

# PPM center adjustable
# Values = YES, NO
PPM_CENTER_ADJUSTABLE = NO

# PPM symmetrical limits
# Values = YES, NO
PPM_LIMITS_SYMETRICAL = NO

# PPM display unit
# Values = PERCENT_PREC1, PERCENT_PREC0, US
PPM_UNIT = PERCENT_PREC1

# OverrideCHxx functions enable
# Values = YES, NO
OVERRIDE_CHANNEL_FUNCTION = YES

# Dangerous functions available in Custom Functions (range test, bind, module off)
# Values = YES, NO
DANGEROUS_MODULE_FUNCTIONS = NO

# FrSky Hub
# Values = YES, NO
FRSKY_HUB = NO

# FrSky FAS current sensor offset compensation
# Values = YES, NO
FAS_OFFSET = NO

# WS HowHigh
# Values = YES, NO
WS_HOW_HIGH = YES

# Gauges in Telemetry screen
# Values = YES, NO
GAUGES = YES

# GPS
# Values = YES, NO
GPS = NO

# Vario
# Values = YES, NO
VARIO = YES

# RTCLOCK
# Values = YES, NO
# default = YES (means enabled when available)
RTCLOCK = YES

# Hardware volume
# Values = YES, NO
# default = YES (AR9X board doesn't have a hardware volume control as delivered but it can be fitted optionally)
HARDWARE_VOLUME = YES

# BLUETOOTH
# Values = YES, NO
BLUETOOTH = NO

# PXX (FrSky PCM) protocol
PXX = NO

# DSM2 (Spektrum) protocol
# Values = NO, SERIAL, PPM
DSM2 = NO

# HUBSAN protocol
# Values = NO, YES
HUBSAN = NO

# DSM/PPM TX caddy PCB (Gruvin)
# For auto-switching switching between DSM2/X and Fr-Sky transmitter modules
# Values = YES, NO
TX_CADDY = NO

#IR protocols (SILV / TRAC09 / PICZ / SWIFT)
IRPROTOS = NO

# TRANSLATIONS
# Values = cz, de, en, fi, fr, it, se, es, pl, pt, nl
TRANSLATIONS = EN

# TTS
# Values = en, fr, hu
TTS = $(TRANSLATIONS)

# UNITS
# Values = imperial, metric
UNITS = METRIC

# Default Tx Mode
# Values = NO / 1 / 2 / 3 / 4
DEFAULT_MODE = NO

# EEPROM_PROGRESS_BAR
# Values = YES, NO
EEPROM_PROGRESS_BAR = NO

# Shutdown confirmation (for boards which have soft power)
# Values = NO, YES
SHUTDOWN_CONFIRMATION = NO

# Activates the DEBUG Mode (all boards)
# On G9X V4 board, when VOICE=NO:
#  PORTH:7 --> Transition LOW to HIGH = start of setupPulses(). Transition HIGH to LOW = end of setupPulses().
#  PORTH:6 --> LOW->HIGH = start of mixer calcs. HIGH->LOW, end of mixer calcs.
# Values = YES, NO
DEBUG = YES

# Activate Command Line Interpreter
# Values = NO, YES
CLI = NO

# Activate writing of SPORT telemetry received data to sport.log file
# Values = YES, NO
SPORT_FILE_LOG = YES

# Timers Count
# Values = 1, 2, 3 (on ARM boards)
TIMERS = 2

# Flight Modes
# Values = YES, NO
FLIGHT_MODES = YES

# Curves
# Values = YES, NO
CURVES = YES

# Global Vars
# Values = YES, NO
GVARS = NO

# Enable another 3 positions switch to replace a pot
# Values = NO, POT1, POT2, POT3,
3POS = NO

# Lua interpreter embedded
# Values = NO, NO_MODEL_SCRIPTS, YES
LUA = NO

# Menu Lock modes
# Values = NO, READONLY, NOMENUS
MENUS_LOCK = NO

# USB options in the main image
# Values = JOYSTICK, MASSSTORAGE, SERIAL
# JOYSTICK: The TX will appear as a joystick
# MASSSTORAGE: The TX will appear as a drive
# SERIAL: The TX will appear as a serial port
USB = SERIAL

# Mixers monitor view added to Channels monitor one
# Values = NO, YES
# YES - Adds mixers output view to Channels monitor screen,
# pressing [ENT] switches between views.
# NO - classic Channels monitor
MIXERS_MONITOR = YES

# Use newlib-nano
# Values = NO, YES
NANO = NO

# Use custom bin based allocator for Lua
# Values = NO, YES
# YES - use our bin based allocator
# NO - use default Lua allocator
USE_BIN_ALLOCATOR = NO

# Enable trace of events into Trace Buffer for debugging purposes
# Activating any of these options also activates DEBUG and DEBUG_TRACE_BUFFER
# Values = NO, YES
#   TRACE_SD_CARD     SD card operations (read, write, etc)
#   TRACE_FATFS       FatFs file operations
#   TRACE_AUDIO       Audio processing (not yet implemented)
TRACE_SD_CARD = NO
TRACE_FATFS = NO
TRACE_AUDIO = NO

# Enable double buffering for LCD. Only for TARANIS PLUS and 9XE targets.
# Activating requires about 6kB of RAM, but it enables menus task to
# immediately start to compose a new LCD image while the current one is
# being written to LCD by DMA.
# If this option is NO, then first function that affects LCD buffer is
# delayed until DMA transfer is finished.
# Values = NO, YES
LCD_DUAL_BUFFER = YES

# Enable internal module PPM mode for Taranis
# Values = NO, YES
TARANIS_INTERNAL_PPM = YES

# Support for D16-EU only (no D8, no LR12 which are not EU compatible)
# Value = NO, YES
SUPPORT_D16_EU_ONLY = NO

# Enable disk IO simulation in simulator
# Simulator will use FatFs module and simulated IO layer that
# uses "./sdcard.image" file as image of SD card.
# This file must contain whole SD card from first to last sector
# Values = NO, YES
SIMU_DISKIO = NO

# Enable audio in simulator
# Values = NO, YES
SIMU_AUDIO = NO

# Enable Watchdog test (DANGEROUS)
# Values = NO, YES
WATCHDOG_TEST = NO

# Treat compiler warnings as errors
# Values = NO, YES
WARNINGS_AS_ERRORS = NO

# Enable saving of Lua byte-code of all loaded scripts
# Values = NO, YES
LUA_COMPILER = YES

# Enable ADC jitter measurement
# Values = NO, YES
JITTER_MEASURE = YES

# Enable ADC jitter filtering
# Values = NO, YES
JITTER_FILTER = YES

#------- END BUILD OPTIONS ---------------------------

# Define programs and commands.
SHELL = sh

IMG2LBM = -python ../util/img2lbm.py
BIN2LBM = -python ../util/bin2lbm.py
FONT2PNG = -python ../util/font2png.py
TRANSLATE = -python ../util/translate.py

# apple:information about version
VERSION_MAJOR    = 0
VERSION_MINOR    = 0
VERSION_REVISION = 0

VERSION = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_REVISION)$(OPENTX_VERSION_SUFFIX)

THIRDPARTY = thirdparty
COOSDIR = $(THIRDPARTY)/CoOS
FATFSDIR = $(THIRDPARTY)/FatFs
LUADIR = $(THIRDPARTY)/Lua/src

SRC =
CPPSRC =
INCDIRS = . translations
LUADEP =
DEBUG_TRACE_BUFFER = NO

# MCU name
ifneq ($(PCB), $(filter $(PCB), STD 9X 9XR STD128 9X128 9XR128 9X2561 9XR2561 GRUVIN9X MEGA2560 SKY9X 9XRPRO AR9X TARANIS))
  $(error Sorry, the PCB $(PCB) is not supported yet in OpenTX)
endif

GVARS_VARIANT   = +1
FRSKY_VARIANT   = +2
3POS_VARIANT    = +4
MAVLINK_VARIANT = +8
M128_VARIANT    = +32768
M2561_VARIANT   = +16384
EEPROM_VARIANT  = 0

GUIMODELSRC = gui/$(GUIDIRECTORY)/menu_model.cpp gui/$(GUIDIRECTORY)/menu_model_select.cpp gui/$(GUIDIRECTORY)/menu_model_setup.cpp gui/$(GUIDIRECTORY)/menu_model_inputs_mixes.cpp gui/$(GUIDIRECTORY)/menu_model_limits.cpp gui/$(GUIDIRECTORY)/menu_model_logical_switches.cpp gui/$(GUIDIRECTORY)/menu_model_custom_functions.cpp gui/$(GUIDIRECTORY)/menu_model_telemetry.cpp
GUIGENERALSRC = gui/$(GUIDIRECTORY)/menu_general.cpp gui/$(GUIDIRECTORY)/menu_general_setup.cpp gui/$(GUIDIRECTORY)/menu_general_trainer.cpp gui/$(GUIDIRECTORY)/menu_general_version.cpp gui/$(GUIDIRECTORY)/menu_general_calib.cpp

BITMAPS = bitmaps/sticks.lbm

FONTS = $(patsubst %.png,%.lbm,$(wildcard fonts/std/*.png))



ifeq ($(PCB), TARANIS)
  # TARANIS radio
  ARCH = ARM
  LCDSIZE = 212
  GUIDIRECTORY = Taranis
  ifeq ($(PCBREV), REVPLUS)
    MCU = cortex-m3
    FLAVOUR = taranis-plus
    CPPDEFS = -DREVPLUS
    ifeq ($(LCD_DUAL_BUFFER), YES)
      CPPDEFS += -DLCD_DUAL_BUFFER
    endif
    HAPTIC = YES
	#added by apple
    EXTRABOARDSRC += telemetry/mavlink.cpp  
  else ifeq ($(PCBREV), REV4a)
    MCU = cortex-m3
    FLAVOUR = taranis
    CPPDEFS = -DREV4a
  else
    MCU = cortex-m3
    FLAVOUR = taranis
    CPPDEFS = -DREV4
  endif
  ifeq ($(SPORT_FILE_LOG), YES)
    CPPDEFS += -DSPORT_FILE_LOG
  endif
  ifeq ($(TRACE_SD_CARD), YES)
    DEBUG = YES
    DEBUG_TRACE_BUFFER = YES
    CPPDEFS += -DTRACE_SD_CARD
  endif
  ifeq ($(TRACE_FATFS), YES)
    DEBUG = YES
    DEBUG_TRACE_BUFFER = YES
    CPPDEFS += -DTRACE_FATFS
  endif
  ifeq ($(TRACE_AUDIO), YES)
    DEBUG = YES
    DEBUG_TRACE_BUFFER = YES
    CPPDEFS += -DTRACE_AUDIO
  endif
  ifeq ($(DEBUG_TRACE_BUFFER), YES)
    CPPDEFS += -DDEBUG_TRACE_BUFFER
  endif
  ifeq ($(TARANIS_INTERNAL_PPM), YES)
    CPPDEFS += -DTARANIS_INTERNAL_PPM
  endif
  ifeq ($(SUPPORT_D16_EU_ONLY), YES)
    CPPDEFS += -DMODULE_D16_EU_ONLY_SUPPORT
  endif
  TRGT = arm-none-eabi-
  OPT = s
  BITMAPS += $(patsubst %.png,%.lbm,$(wildcard bitmaps/Taranis/*.png)) bitmaps/Taranis/mainmenu.lbm
  SDCARD = YES
  THR_TRACE = YES
  EXT = FRSKY_SPORT
  AUTOSWITCH = YES
  AUTOSOURCE = YES
  DBLKEYS = YES
  TIMERS = 2
  CPPDEFS += -DTIMERS=$(TIMERS)
  PPM_LIMITS_SYMETRICAL = YES
  PPM_CENTER_ADJUSTABLE = YES
  DSM2 = PPM
  CPPDEFS += -DPCBTARANIS -DCPUARM -DCPUSTM32 -DVIRTUALINPUTS -DLUAINPUTS -DXCURVES -DEEPROM_RLC -DAUDIO -DPXX -DSBUS -DCROSSFIRE
  INCDIRS += targets/taranis $(THIRDPARTY) $(COOSDIR) $(COOSDIR)/kernel $(COOSDIR)/portable
  CPPDEFS += -DHSE_VALUE=12000000
  STM32LIBPATH= $(THIRDPARTY)/STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries
  CPPDEFS += -DEEPROM_VARIANT=0
  INCDIRS += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/inc $(STM32LIBPATH)/CMSIS/Device/ST/STM32F2xx/Include $(STM32LIBPATH)/CMSIS/include
  LDSCRIPT = targets/taranis/stm32f2_flash.ld
  STM32USBPATH = $(THIRDPARTY)/STM32_USB-Host-Device_Lib_V2.1.0/Libraries
  INCDIRS += $(STM32USBPATH)/STM32_USB_OTG_Driver/inc
  INCDIRS += $(STM32USBPATH)/STM32_USB_Device_Library/Core/inc
  INCDIRS += $(STM32USBPATH)/STM32_USB_Device_Library/Class/msc/inc
  INCDIRS += $(STM32USBPATH)/STM32_USB_Device_Library/Class/hid/inc
  INCDIRS += $(STM32USBPATH)/STM32_USB_Device_Library/Class/cdc/inc
  GUIGENERALSRC += gui/$(GUIDIRECTORY)/menu_general_hardware.cpp
  BOARDSRC = main_arm.cpp targets/taranis/board_taranis.cpp
  EXTRABOARDSRC += targets/taranis/configure_pins.cpp targets/taranis/lcd_driver.cpp targets/taranis/spi_driver.cpp targets/taranis/i2c_driver.cpp targets/taranis/aspi.c targets/taranis/delays.c serial.cpp targets/taranis/flash_driver.cpp
  SRC += $(COOSDIR)/kernel/core.c $(COOSDIR)/kernel/hook.c $(COOSDIR)/kernel/task.c $(COOSDIR)/kernel/event.c $(COOSDIR)/kernel/time.c $(COOSDIR)/kernel/timer.c $(COOSDIR)/kernel/flag.c $(COOSDIR)/kernel/mutex.c $(COOSDIR)/kernel/serviceReq.c $(COOSDIR)/portable/GCC/port.c $(COOSDIR)/portable/arch.c
  SRC +=  targets/taranis/usb_driver.c
  EEPROMSRC = eeprom_common.cpp eeprom_rlc.cpp eeprom_conversions.cpp
  PULSESSRC = pulses/pulses_arm.cpp pulses/ppm_arm.cpp pulses/pxx_arm.cpp pulses/crossfire.cpp
  CPPSRC += tasks_arm.cpp audio_arm.cpp sbus.cpp telemetry/telemetry.cpp
  CPPSRC += targets/taranis/pulses_driver.cpp targets/taranis/keys_driver.cpp targets/taranis/adc_driver.cpp 
  CPPSRC += targets/taranis/power_driver.cpp  targets/taranis/gui.cpp targets/taranis/menus.cpp targets/taranis/audio_driver.cpp  targets/taranis/pico.cpp
  CPPSRC += targets/taranis/usart1_driver.cpp  
  CPPSRC += targets/taranis/usart2_driver.cpp 
  CPPSRC += targets/taranis/usart3_driver.cpp 
  CPPSRC += targets/taranis/usart4_driver.cpp
  CPPSRC += bmp.cpp gui/$(GUIDIRECTORY)/view_channels.cpp gui/$(GUIDIRECTORY)/view_about.cpp gui/$(GUIDIRECTORY)/view_text.cpp debug.cpp
  CPPSRC += ftp.cpp
  CPPSRC += loadboot.cpp
  CPPSRC += apple.cpp 
  
  SRC += targets/taranis/system_stm32f2xx.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/misc.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_gpio.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_dbgmcu.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_rcc.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_spi.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_i2c.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_dma.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_rtc.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_pwr.c
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_usart.c
  SRC += $(STM32LIBPATH)/CMSIS/Device/ST/STM32F2xx/Source/Templates/gcc_ride7/startup_stm32f2xx.s
  #added by apple
  SRC += $(STM32LIBPATH)/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_fsmc.c
	
  SRC += targets/taranis/usb_bsp.c targets/taranis/usbd_desc.c
  SRC += $(STM32USBPATH)/STM32_USB_OTG_Driver/src/usb_core.c
  SRC += $(STM32USBPATH)/STM32_USB_OTG_Driver/src/usb_dcd.c
  SRC += $(STM32USBPATH)/STM32_USB_OTG_Driver/src/usb_dcd_int.c
  SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Core/src/usbd_core.c
  SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Core/src/usbd_ioreq.c
  SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Core/src/usbd_req.c
  EXTRABOARDSRC += targets/taranis/usbd_usr.cpp
  SRC += syscalls.c
  ifeq ($(USB), JOYSTICK)
    CPPDEFS += -DUSB_JOYSTICK
    SRC += targets/taranis/usbd_hid_joystick.c
  else ifeq ($(USB), SERIAL)
    CPPDEFS += -DUSB_SERIAL
    SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c
    EXTRABOARDSRC += targets/taranis/usbd_cdc.cpp
  else
    CPPDEFS += -DUSB_MASS_STORAGE
    SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Class/msc/src/usbd_msc_data.c
    SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Class/msc/src/usbd_msc_scsi.c
    SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Class/msc/src/usbd_msc_bot.c
    SRC += $(STM32USBPATH)/STM32_USB_Device_Library/Class/msc/src/usbd_msc_core.c
    EXTRABOARDSRC += targets/taranis/usbd_storage_msd.cpp
  endif
  ifeq ($(NANO), YES)
    # use newlib-nano for linking
    NEWLIB_NANO_FLAGS = --specs=nano.specs -u _printf_float
    CPPDEFS += -DNANO
  endif
  ifeq ($(GVARS), YES)
    GUIMODELSRC += gui/$(GUIDIRECTORY)/menu_model_gvars.cpp
  endif
  ifneq ($(LUA), NO)
    ifeq ($(LUA), YES)
      CPPDEFS += -DLUA_MODEL_SCRIPTS
      GUIMODELSRC += gui/$(GUIDIRECTORY)/menu_model_custom_scripts.cpp
    endif
    CPPDEFS += -DLUA
    INCDIRS += $(LUADIR)
    CPPSRC += lua/interface.cpp lua/api_general.cpp lua/api_lcd.cpp lua/api_model.cpp
    LUASRC = $(LUADIR)/lapi.c $(LUADIR)/lcode.c $(LUADIR)/lctype.c $(LUADIR)/ldebug.c $(LUADIR)/ldo.c $(LUADIR)/ldump.c $(LUADIR)/lfunc.c $(LUADIR)/lgc.c $(LUADIR)/llex.c $(LUADIR)/lmem.c \
           $(LUADIR)/lobject.c $(LUADIR)/lopcodes.c $(LUADIR)/lparser.c $(LUADIR)/lstate.c $(LUADIR)/lstring.c $(LUADIR)/ltable.c $(LUADIR)/lrotable.c $(LUADIR)/ltm.c $(LUADIR)/lundump.c $(LUADIR)/lvm.c $(LUADIR)/lzio.c \
           $(LUADIR)/lbaselib.c $(LUADIR)/linit.c $(LUADIR)/lmathlib.c $(LUADIR)/lbitlib.c $(LUADIR)/loadlib.c $(LUADIR)/lauxlib.c $(LUADIR)/ltablib.c $(LUADIR)/lcorolib.c $(LUADIR)/liolib.c $(LUADIR)/lstrlib.c
    SRC += $(LUASRC)
    ifeq ($(PCBREV), REV9E) 
      LUADEP = lua_exports_taranis_x9e.inc
    else
      LUADEP = lua_exports_taranis.inc
    endif
	ifeq ($(USE_BIN_ALLOCATOR), YES)
	  CPPDEFS += -DUSE_BIN_ALLOCATOR
	  CPPSRC += bin_allocator.cpp
	endif
  endif
  EXTRABOARDSRC += $(FATFSDIR)/ff.c $(FATFSDIR)/fattime.c $(FATFSDIR)/option/ccsbcs.c targets/taranis/diskio.cpp
  CPPSRC += sdcard.cpp logs.cpp rtc.cpp targets/taranis/rtc_driver.cpp 
  GUIGENERALSRC += gui/$(GUIDIRECTORY)/menu_general_sdmanager.cpp gui/$(GUIDIRECTORY)/menu_general_diagkeys.cpp gui/$(GUIDIRECTORY)/menu_general_diaganas.cpp
  CPPDEFS += -DSDCARD -DVOICE -DRTCLOCK
  INCDIRS += $(FATFSDIR) $(FATFSDIR)/option
  ifeq ($(DSM2), PPM)
    CPPSRC += pulses/dsm2_arm.cpp
  endif
  ifeq ($(HUBSAN), YES)
    CPPDEFS += -DHUBSAN
    CPPSRC += pulses/hubsan.cpp
    CPPSRC += targets/taranis/a7105_driver.cpp
  endif
  ifeq ($(HAPTIC), YES)
    CPPDEFS += -DHAPTIC
    CPPSRC += haptic.cpp
    CPPSRC += targets/taranis/haptic_driver.cpp
  endif
  ifeq ($(MIXERS_MONITOR), YES)
    CPPDEFS += -DMIXERS_MONITOR
  endif
  ifeq ($(WATCHDOG_TEST), YES)
    CPPDEFS += -DWATCHDOG_TEST
  endif
  ifeq ($(JITTER_MEASURE), YES)
    CPPDEFS += -DJITTER_MEASURE
  endif
  ifeq ($(JITTER_FILTER), YES)
    CPPDEFS += -DJITTER_FILTER
  endif
endif

CC      = $(TRGT)gcc
CPP     = $(TRGT)cpp
AS      = $(TRGT)as
OBJCOPY = $(TRGT)objcopy
OBJDUMP = $(TRGT)objdump
SIZE    = $(TRGT)size
NM      = $(TRGT)nm

REMOVE = rm -f
REMOVEDIR = rm -rf

SLEEP = sleep

# Processor frequency.
F_CPU = 16000000

# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# Target file name (without extension).
TARGET = opentx

# List C++ source files here. (C dependencies are automatically generated.)
ifeq ($(PCB), $(filter $(PCB), SKY9X 9XRPRO AR9X TARANIS))
  TTS_SRC = translations/tts_cz.cpp translations/tts_de.cpp translations/tts_en.cpp translations/tts_es.cpp translations/tts_fr.cpp translations/tts_it.cpp translations/tts_nl.cpp translations/tts_pt.cpp translations/tts_sk.cpp translations/tts_se.cpp translations/tts_pl.cpp translations/tts_hu.cpp
else
  STD_TTS_SRC = translations/tts_$(shell sh -c "echo $(TTS) | tr '[:upper:]' '[:lower:]'").cpp
  TTS_SRC = $(shell sh -c "if test -f $(STD_TTS_SRC); then echo $(STD_TTS_SRC); else echo translations/tts_en.cpp; fi")
endif

ifeq ($(GUI), YES)
  GUISRC = gui/$(GUIDIRECTORY)/helpers.cpp gui/$(GUIDIRECTORY)/navigation.cpp gui/$(GUIDIRECTORY)/popups.cpp gui/$(GUIDIRECTORY)/widgets.cpp gui/$(GUIDIRECTORY)/menus.cpp $(GUIMODELSRC) $(GUIGENERALSRC) gui/$(GUIDIRECTORY)/view_main.cpp gui/$(GUIDIRECTORY)/view_statistics.cpp
  CPPDEFS += -DGUI
endif

CPPSRC += opentx.cpp functions.cpp strhelpers.cpp $(PULSESSRC) switches.cpp curves.cpp mixer.cpp stamp.cpp $(GUISRC) $(EEPROMSRC) timers.cpp trainer_input.cpp

ifeq ($(GUI), YES)
  GUISRC += gui/$(GUIDIRECTORY)/lcd.cpp gui/$(GUIDIRECTORY)/splash.cpp gui/$(GUIDIRECTORY)/fonts.cpp
endif

GUISRC += keys.cpp maths.cpp translations.cpp $(TTS_SRC)

# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DBGFMT = dwarf-2

CPPDEFS += -DTRANSLATIONS_$(TRANSLATIONS)

# TTS language
CPPDEFS += -DTTS_$(TTS)

ifeq ($(UNITS), IMPERIAL)
  CPPDEFS += -DIMPERIAL_UNITS
endif

ifneq ($(DEFAULT_MODE), NO)
  CPPDEFS += -DDEFAULT_MODE=$(DEFAULT_MODE)
endif

# If POTS are used for fields modification
ifeq ($(NAVIGATION), POT1)
  CPPDEFS += -DNAVIGATION_POT1
endif
ifeq ($(NAVIGATION), POT2)
  CPPDEFS += -DNAVIGATION_POT2
endif
ifeq ($(NAVIGATION), POT3)
  CPPDEFS += -DNAVIGATION_POT3
endif
ifeq ($(NAVIGATION), POTS)
  CPPDEFS += -DNAVIGATION_POT1 -DNAVIGATION_POT2 -DNAVIGATION_POT3
endif
ifeq ($(NAVIGATION), STICKS)
  CPPDEFS += -DNAVIGATION_STICKS
endif
ifeq ($(NAVIGATION), ROTENC)
  CPPDEFS += -DROTARY_ENCODER_NAVIGATION
endif

ifeq ($(TURNIGY_TRANSMITTER_FIX), YES)
  ifeq ($(PCB), $(filter $(PCB), TARANIS))
    $(warning TURNIGY_TRANSMITTER_FIX is not available on this radio)
  else
    CPPDEFS += -DMODULE_ALWAYS_SEND_PULSES
  endif
endif

ifeq ($(LCD), ST7565R)
  CPPDEFS += -DLCD_ST7565R
else ifeq ($(LCD), ERC12864FSF)
  CPPDEFS += -DLCD_ERC12864FSF
else ifeq ($(LCD), ST7920)
  CPPDEFS += -DLCD_ST7920
endif   

ifeq ($(AUTOSWITCH), YES)
  CPPDEFS += -DAUTOSWITCH
endif

ifeq ($(AUTOSOURCE), YES)
  CPPDEFS += -DAUTOSOURCE
endif

ifeq ($(FRSKY_STICKS), YES)
  CPPDEFS += -DFRSKY_STICKS
endif

ifeq ($(DBLKEYS), YES)
  ifeq ($(PCB), $(filter $(PCB), 9XR 9XR128 9XR2561))
    $(warning Sorry, DBLKEYS doesnt work on 9XR)
  else
    CPPDEFS += -DDBLKEYS
  endif
endif

ifeq ($(SPLASH), YES)
  CPPDEFS += -DSPLASH
endif

ifeq ($(SPLASH), FIXED)
  CPPDEFS += -DSPLASH -DFSPLASH
endif

ifeq ($(GRAPHICS), YES)
  CPPDEFS += -DGRAPHICS
endif

ifeq ($(BOLD), YES)
  CPPDEFS += -DBOLD_FONT
endif

ifneq ($(PCB), $(filter $(PCB), TARANIS))
  ifeq ($(BATTGRAPH), YES)
    CPPDEFS += -DBATTGRAPH
  endif
endif

ifneq ($(FONT), STD)
  FONTDIR = $(shell sh -c "echo $(FONT) | tr '[:upper:]' '[:lower:]'")
  FONTS += $(patsubst %.png,%.lbm,$(wildcard fonts/$(FONTDIR)/*.png))
  INCDIRS += fonts/$(FONTDIR)
endif

INCDIRS += gui
INCDIRS += fonts/std

ifeq ($(THR_TRACE), YES)
  CPPDEFS += -DTHRTRACE
endif

CPPDEFS += -DTIMERS=$(TIMERS)

ifeq ($(FLIGHT_MODES), YES)
  CPPDEFS += -DFLIGHT_MODES
  GUIMODELSRC += gui/$(GUIDIRECTORY)/menu_model_flightmodes.cpp
endif

ifeq ($(CURVES), YES)
  CPPDEFS += -DCURVES
  GUIMODELSRC += gui/$(GUIDIRECTORY)/menu_model_curves.cpp
endif

ifeq ($(GVARS), YES)
  CPPDEFS += -DGVARS
  EEPROM_VARIANT += ${GVARS_VARIANT}
endif

ifeq ($(MENUS_LOCK), READONLY)
  CPPDEFS += -DMENUS_LOCK=1
endif

ifeq ($(MENUS_LOCK), NOMENUS)
  CPPDEFS += -DMENUS_LOCK=2
endif

ifeq ($(CORRECT_NEGATIVE_VALUES), YES)
  CPPDEFS += -DCORRECT_NEGATIVE_SHIFTS
endif

ifeq ($(ARITHMETIC_OVERFLOW_CHECK), YES)
  CPPDEFS += -DPREVENT_ARITHMETIC_OVERFLOW
endif

ifeq ($(ACCURAT_THROTTLE_STATS), YES)
  CPPDEFS += -DACCURAT_THROTTLE_TIMER
endif

# EXTENSIONS (for Telemetry protocols)
ifneq ($(EXT), $(filter $(EXT), STD FRSKY FRSKY_SPORT ARDUPILOT NMEA JETI MAVLINK TELEMETREZ))
  $(error Sorry, the EXT $(EXT) is not supported yet in openTx)
endif

ifeq ($(EXT), STD)
  CPPDEFS += -DEXTSTD
endif

ifeq ($(EXT), ARDUPILOT)
  CPPDEFS += -DARDUPILOT
  CPPSRC += telemetry/ardupilot.cpp
endif

ifeq ($(EXT), NMEA)
  CPPDEFS += -DNMEA
  CPPSRC += telemetry/nmea.cpp
endif

ifeq ($(EXT), JETI)
  CPPDEFS += -DJETI
  CPPSRC += telemetry/jeti.cpp
endif

ifeq ($(EXT), MAVLINK)
  CPPDEFS += -DMAVLINK
  INCDIRS += thirdparty targets/common_avr/
  CPPSRC += telemetry/mavlink.cpp gui/$(GUIDIRECTORY)/view_mavlink.cpp targets/common_avr/serial_driver.cpp
  EEPROM_VARIANT += ${MAVLINK_VARIANT}
endif

ifeq ($(EXT), TELEMETREZ)
  CPPDEFS += -DTELEMETREZ
endif

ifeq ($(EXT), $(filter $(EXT), FRSKY FRSKY_SPORT TELEMETREZ))
  CPPDEFS += -DFRSKY

  ifeq ($(ARCH), ARM)
    CPPSRC += telemetry/frsky.cpp telemetry/frsky_d_arm.cpp
  else
    CPPSRC += telemetry/frsky.cpp telemetry/frsky_d.cpp
  endif

  ifeq ($(EXT), FRSKY_SPORT)
    CPPSRC += telemetry/frsky_sport.cpp
    CPPSRC += crc16.cpp
    CPPDEFS += -DFRSKY_SPORT
  endif

  CPPSRC += gui/$(GUIDIRECTORY)/view_telemetry.cpp

  ifeq ($(FRSKY_HUB), YES)
    CPPDEFS += -DFRSKY_HUB
  endif

  ifeq ($(WS_HOW_HIGH), YES)
    CPPDEFS += -DWS_HOW_HIGH
  endif

  ifeq ($(GAUGES), YES)
    CPPDEFS += -DGAUGES
  endif

  ifeq ($(GPS), YES)
    CPPDEFS += -DGPS
  endif

  ifeq ($(VARIO), YES)
    CPPSRC += vario.cpp
    CPPDEFS += -DVARIO
  endif
  EEPROM_VARIANT += ${FRSKY_VARIANT}
endif

ifeq ($(SIMU), YES)
  CPPDEFS += -DDEBUG
  CPPSRC += dump.cpp
else ifeq ($(CLI), YES)
  CPPDEFS += -DCLI -DDEBUG
  CPPSRC += cli.cpp dump.cpp
else ifeq ($(DEBUG), YES)
  CPPDEFS += -DDEBUG
  CPPSRC += dump.cpp
endif

ifeq ($(EEPROM_PROGRESS_BAR), YES)
  CPPDEFS += -DEEPROM_PROGRESS_BAR
endif

ifeq ($(SHUTDOWN_CONFIRMATION), YES)
  CPPDEFS += -DSHUTDOWN_CONFIRMATION
endif

RUN_FROM_FLASH = 1

### Global Build-Option Directives ###

ifeq ($(PPM_CENTER_ADJUSTABLE), YES)
  CPPDEFS += -DPPM_CENTER_ADJUSTABLE
endif

ifeq ($(PPM_LIMITS_SYMETRICAL), YES)
  CPPDEFS += -DPPM_LIMITS_SYMETRICAL
endif

ifeq ($(OVERRIDE_CHANNEL_FUNCTION), YES)
  CPPDEFS += -DOVERRIDE_CHANNEL_FUNCTION
endif

ifeq ($(DANGEROUS_MODULE_FUNCTIONS), YES)
  CPPDEFS += -DDANGEROUS_MODULE_FUNCTIONS
endif

ifeq ($(PPM_UNIT), US)
  CPPDEFS += -DPPM_UNIT_US
endif

ifeq ($(PPM_UNIT), PERCENT_PREC1)
  CPPDEFS += -DPPM_UNIT_PERCENT_PREC1
endif

ifeq ($(PPM_UNIT), PERCENT_PREC0)
  CPPDEFS += -DPPM_UNIT_PERCENT_PREC0
endif

ifeq ($(FAI), YES)
  CPPDEFS += -DFAI
endif
ifeq ($(FAI), CHOICE)
  CPPDEFS += -DFAI_CHOICE
endif

ifeq ($(HELI), YES)
  CPPDEFS += -DHELI
  GUIMODELSRC += gui/$(GUIDIRECTORY)/menu_model_heli.cpp
endif

ifeq ($(TEMPLATES), YES)
  ifneq ($(PCB), $(filter $(PCB), TARANIS))
    CPPDEFS += -DTEMPLATES
    CPPSRC += templates.cpp
    GUIMODELSRC += gui/$(GUIDIRECTORY)/menu_model_templates.cpp
  endif
endif

ifeq ($(PXX), YES)
  CPPDEFS += -DPXX
endif

ifeq ($(DSM2), SERIAL)
  CPPDEFS += -DDSM2 -DDSM2_SERIAL
endif

ifeq ($(DSM2), PPM)
  CPPDEFS += -DDSM2 -DDSM2_PPM
endif

ifeq ($(TX_CADDY), YES)
  CPPDEFS += -DTX_CADDY
endif

ifeq ($(IRPROTOS), YES)
  CPPDEFS += -DIRPROTOS
endif

ifeq ($(LUA_COMPILER), YES)
  CPPDEFS += -DLUA_COMPILER
endif

#---------------- Compiler Options C++ ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CPPFLAGS = -g$(DBGFMT)
CPPFLAGS += $(CPPDEFS)
CPPFLAGS += -Wall
CPPFLAGS += -fno-exceptions
CPPFLAGS += -Wno-strict-aliasing
CPPFLAGS += -Wformat -Wreturn-type -Wunused -Wuninitialized -Wunknown-pragmas -Wno-switch -Wtype-limits
INCFLAGS = $(patsubst %,-I%,$(INCDIRS))

ifeq ($(WARNINGS_AS_ERRORS), YES)
  CPPFLAGS += -Werror
endif

MATH_LIB = -lm

#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += $(MATH_LIB)

# Define Messages
# English
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_COMPILING = Compiling C++:
MSG_CLEANING = Cleaning project:

# Combine all necessary flags and optional flags.
# Add target processor to flags.

CPPFLAGS += -DFLAVOUR=\"$(FLAVOUR)\"
ARMCPPFLAGS = -c -mcpu=$(MCU) -mthumb -fomit-frame-pointer -fverbose-asm -Wa,-ahlms=opentx.lst -DRUN_FROM_FLASH=1 -O$(OPT) $(CPPFLAGS) -fdata-sections -ffunction-sections $(INCFLAGS) $(GENDEPFLAGS)
AVRCPPFLAGS = -mmcu=$(MCU) -I. -x c++ -O$(OPT) $(CPPFLAGS) $(INCFLAGS) $(GENDEPFLAGS) -fwhole-program

ifeq ($(ARCH), AVR)
  GCCVERSIONGTE462 := $(shell expr 4.6.2 \<= `$(CC) -dumpversion`)
  ifeq ($(GCCVERSIONGTE462),1)
    CPPFLAGS += -flto
  endif

  AVRCPPFLAGS += -fno-inline-small-functions

  GCCVERSIONGTE470 := $(shell expr 4.7.0 \<= `$(CC) -dumpversion`)
  ifeq ($(GCCVERSIONGTE470),1)
    AVRCPPFLAGS += -mstrict-X
  endif
endif

MAJ_VER = ${shell sh -c "grep \"MAJ_VERS\" opentx.h | cut -d\  -f3 | egrep -o \"[[:digit:]]\""}
MIN_VER = ${shell sh -c "grep \"MIN_VERS\" opentx.h | cut -d\  -f3"}

.SECONDARY:

# Default target.
all: begin gccversion sizebefore build sizeafter end

# Change the build target to build a HEX file or a library.
ifeq ($(ARCH), AVR)
  build: stamp_header tra lbm allsrc.cpp elf remallsrc hex
else ifeq ($(PCB), $(filter $(PCB), SKY9X 9XRPRO AR9X TARANIS))
  build: stamp_header tra lbm bootloader.lbm $(LUADEP) allsrc.cpp elf remallsrc bin
else
  build: stamp_header tra lbm allsrc.cpp elf remallsrc bin
endif

# build bootloader
ifeq ($(PCB), $(filter $(PCB), TARANIS))
bootloader.lbm: targets/taranis/bootloader/bootloader_ramBoot.bin
	$(BIN2LBM) $^ $@
.PHONY targets/taranis/bootloader/bootloader_ramBoot.bin:
	@echo "Building bootloader..."
	$(MAKE) -C targets/taranis/bootloader PCB=$(PCB) PCBREV=$(PCBREV) NANO=$(NANO) WARNINGS_AS_ERRORS=$(WARNINGS_AS_ERRORS)
else ifeq ($(PCB), $(filter $(PCB), 9XRPRO AR9X))
bootloader.lbm:
	@echo "Fetching Mike's bootloader..."
	wget https://github.com/MikeBland/OpenRcBootloader/releases/download/V1.9/bootflash8.lbm
	mv bootflash8.lbm $@
else ifeq ($(PCB), SKY9X)
bootloader.lbm:
	@echo "Fetching Mike's bootloader..."
	wget https://github.com/MikeBland/OpenRcBootloader/releases/download/V1.9/bootflash4.lbm
	mv bootflash4.lbm $@
endif

tra: translations/en.h translations/cz.h translations/de.h translations/es.h translations/fi.h translations/fr.h translations/it.h translations/nl.h translations/pl.h translations/pt.h translations/se.h
lbm: fonts bitmaps

elf: $(TARGET).elf
bin: $(TARGET).bin
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss
sym: $(TARGET).sym

# Build stamp-file
stamp_header:
	@echo "// Automatically generated file (Makefile) - do not edit" > stamp-opentx.h
	@echo "#define DATE_STR \"`date +%Y-%m-%d | tr -d '\r\n'`\"" >> stamp-opentx.h
	@echo "#define TIME_STR \"`date +%H:%M:%S | tr -d '\r\n'`\"" >> stamp-opentx.h
	@echo "#define VERS_STR \"$(VERSION)\"" >> stamp-opentx.h
	@echo "" >> stamp-opentx.h
	@echo "#define VERSION_MAJOR     $(VERSION_MAJOR)" >> stamp-opentx.h
	@echo "#define VERSION_MINOR     $(VERSION_MINOR)" >> stamp-opentx.h
	@echo "#define VERSION_REVISION  $(VERSION_REVISION)" >> stamp-opentx.h
	@echo "#define GIT_STR \"$(shell git rev-parse HEAD)\"" >> stamp-opentx.h

opentx-stamp:
	@echo "#define DATE_STR \"`date +%Y-%m-%d`\"" > stamp-opentx.txt
	@echo "#define TIME_STR \"`date +%H:%M:%S`\"" >> stamp-opentx.txt
	@echo "#define VERS_STR \"$(VERSION)\"" >> stamp-opentx.txt
	@echo "#define GIT_STR \"$(shell git rev-parse HEAD)\"" >> stamp-opentx.txt
	@cat stamp-opentx.txt

translations/en.h: translations/en.h.txt
	@echo
	@echo "Create English language specific header files"
	$(TRANSLATE) translations/en.h.txt translations/en.h en

translations/cz.h: translations/cz.h.txt
	@echo
	@echo "Create Czech language specific header files"
	$(TRANSLATE) translations/cz.h.txt translations/cz.h cz

translations/de.h: translations/de.h.txt
	@echo
	@echo "Create German language specific header files"
	$(TRANSLATE) translations/de.h.txt translations/de.h de

translations/es.h: translations/es.h.txt
	@echo
	@echo "Create Spanish language specific header files"
	$(TRANSLATE) translations/es.h.txt translations/es.h es

translations/fi.h: translations/fi.h.txt
	@echo
	@echo "Create Finnish language specific header files"
	$(TRANSLATE) translations/fi.h.txt translations/fi.h fi

translations/fr.h: translations/fr.h.txt
	@echo
	@echo "Create French language specific header files"
	$(TRANSLATE) translations/fr.h.txt translations/fr.h fr

translations/it.h: translations/it.h.txt
	@echo
	@echo "Create Italian language specific header files"
	$(TRANSLATE) translations/it.h.txt translations/it.h it

translations/nl.h: translations/nl.h.txt
	@echo
	@echo "Create Nederlands language specific header files"
	$(TRANSLATE) translations/nl.h.txt translations/nl.h nl

translations/pl.h: translations/pl.h.txt
	@echo
	@echo "Create Polish language specific header files"
	$(TRANSLATE) translations/pl.h.txt translations/pl.h pl

translations/pt.h: translations/pt.h.txt
	@echo
	@echo "Create Portugese language specific header files"
	$(TRANSLATE) translations/pt.h.txt translations/pt.h pt

translations/se.h: translations/se.h.txt
	@echo
	@echo "Create Swedish language specific header files"
	$(TRANSLATE) translations/se.h.txt translations/se.h se

FONTNAME = Roboto

.PHONY: fonts
fonts: $(FONTS)

fonts/%.lbm: fonts/%.png
	$(IMG2LBM) $^ $@ $(LCDSIZE)

fonts/font_tinsize.lbm:
	$(FONT2PNG) $(FONTNAME) 5 fonts/font_tinsize.png
	$(IMG2LBM) fonts/font_tinsize.png $@ $(LCDSIZE) 4bits

fonts/font_stdsize.lbm:
	$(FONT2PNG) $(FONTNAME) 8 fonts/font_stdsize.png
	$(IMG2LBM) fonts/font_stdsize.png $@ $(LCDSIZE) 4bits

fonts/font_midsize.lbm:
	$(FONT2PNG) $(FONTNAME) 10 fonts/font_midsize.png
	$(IMG2LBM) fonts/font_midsize.png $@ $(LCDSIZE) 4bits

fonts/font_dblsize.lbm:
	$(FONT2PNG) $(FONTNAME) 14 fonts/font_dblsize.png
	$(IMG2LBM) fonts/font_dblsize.png $@ $(LCDSIZE) 4bits

.PHONY: bitmaps
bitmaps: $(BITMAPS)

bitmaps/sticks.lbm: bitmaps/sticks.xbm
	$(IMG2LBM) $^ $@ $(LCDSIZE) 1bit 4

bitmaps/9X/%.lbm: bitmaps/9X/%.png
	$(IMG2LBM) $^ $@ $(LCDSIZE) 1bit

bitmaps/Taranis/%.lbm: bitmaps/Taranis/%.png
	$(IMG2LBM) $^ $@ $(LCDSIZE) 4bits

bitmaps/Taranis/mainmenu.png: ../util/taranisicons.py $(wildcard bitmaps/Taranis/std/*.png)
	@python ../util/taranisicons.py

ifeq ($(OS),Windows_NT)
  PARSER = avr-cpp
else
  PARSER = gcc -E -x c++
endif

lua_exports_taranis.inc: myeeprom.h ../util/luaexport.py
	@echo "Generating a list of Lua exported constants in lua_exports_taranis.txt"
	@$(PARSER) -DEXPORT -DCPUARM -DPCBTARANIS -DLUA -DVIRTUALINPUTS myeeprom.h | grep LEXP > lua_exports_taranis.txt
	@echo "Parsing and generating C Lua exports in lua/lua_exports_taranis.inc"
	@$(PYTHON) ../util/luaexport.py $(VERSION) lua_exports_taranis.txt lua/lua_exports_taranis.inc lua_fields_taranis.txt
	@rm lua_exports_taranis.txt

lua_exports_taranis_x9e.inc: myeeprom.h ../util/luaexport.py
	@echo "Generating a list of Lua exported constants in lua_exports_taranis_x9e.txt"
	@$(PARSER) -DEXPORT -DCPUARM -DPCBTARANIS -DREVPLUS -DREV9E -DLUA -DVIRTUALINPUTS myeeprom.h | grep LEXP > lua_exports_taranis_x9e.txt
	@echo "Parsing and generating C Lua exports in lua/lua_exports_taranis_x9e.inc"
	@python ../util/luaexport.py $(VERSION) lua_exports_taranis_x9e.txt lua/lua_exports_taranis_x9e.inc lua_fields_taranis_x9e.txt
	@rm lua_exports_taranis_x9e.txt

# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.

begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo

# Display size of file.
ifeq ($(ARCH), AVR)
  MCU_FLAG = $(shell $(SIZE) --help | grep -- --mcu > /dev/null && echo --mcu=$(MCU) )
  FORMAT_FLAG = $(shell $(SIZE) --help | grep -- --format=.*avr > /dev/null && echo --format=avr )
  ELFSIZE = $(SIZE) $(MCU_FLAG) $(FORMAT_FLAG) $(TARGET).elf
else
  ELFSIZE = $(SIZE) $(TARGET).elf
endif

sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); fi

# Display compiler version information.
gccversion:
	@$(CC) --version

FOXINC=-I/usr/local/include/fox-1.6 -I/usr/include/fox-1.6 \
       -I$(FOXPATH)/include \
       -I/opt/local/include/fox-1.6
FOXLIB=-L/usr/local/lib \
       -L$(FOXPATH)/src/.libs \
       -L/opt/local/lib \
       -lFOX-1.6 \
       -Wl,-rpath,$(FOXPATH)/src/.libs

SIMUDEFS = -DSIMU -DLUA_USE_APICHECK
ifeq ($(SIMU_AUDIO), YES)
  AUDIOLIB = -L/usr/lib/x86_64-linux-gnu/ -lSDLmain -lSDL
  AUDIOINC = -I/usr/include/SDL
  SIMUDEFS += -DSIMU_AUDIO
endif

ifeq ($(SIMU_DISKIO), YES)
  CPPSRC += $(FATFSDIR)/ff.c $(FATFSDIR)/option/ccsbcs.c
  SIMUDEFS += -DSIMU_DISKIO
endif

simu: $(LUADEP) stamp_header allsimusrc.cpp Makefile simu.cpp targets/simu/simpgmspace.cpp *.h tra lbm eeprom.bin
	g++ $(CPPFLAGS) $(SIMUCPPFLAGS) $(INCFLAGS) simu.cpp allsimusrc.cpp $(LUASRC) targets/simu/simpgmspace.cpp -MD $(SIMUDEFS) -O0 -o simu $(FOXINC) $(FOXLIB) $(AUDIOINC) $(AUDIOLIB) -pthread -fexceptions

eeprom.bin:
	dd if=/dev/zero of=$@ bs=1 count=2048


# Create final output files (.hex, .eep) from ELF output file.
ifeq ($(ARCH), AVR)
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@
else
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O ihex opentx.elf opentx.hex
endif

%.bin: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O binary  $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Concatenate all sources files in one big file to optimize size
allsrc.cpp: Makefile $(BOARDSRC) $(CPPSRC) $(EXTRABOARDSRC)
	@rm -f allsrc.cpp
	for f in $(BOARDSRC) $(CPPSRC) $(EXTRABOARDSRC) ; do echo "# 1 \"$$f\"" >> allsrc.cpp; cat "$$f" >> allsrc.cpp; done

allsimusrc.cpp: Makefile $(BOARDSRC) $(CPPSRC)
	@rm -f allsimusrc.cpp
	for f in $(BOARDSRC) $(CPPSRC) ; do echo "# 1 \"$$f\"" >> allsimusrc.cpp; cat "$$f" >> allsimusrc.cpp; done

remallsrc:
	$(REMOVE) allsrc.cpp
	$(REMOVE) allsimusrc.cpp

# Link: create ELF output file from object files.
ifeq ($(ARCH), AVR)
opentx.elf: allsrc.cpp
	@echo
	@echo $(MSG_COMPILING) $@
	$(CC) $(AVRCPPFLAGS) $< --output $@ $(LDFLAGS)
else
%.o: %.c
	$(CC) $(ARMCPPFLAGS) -Wimplicit $< -o $@
%.o: %.s
	$(AS) $< -o $@
TMP = $(SRC:.c=.o)
OBJS =  $(TMP:.s=.o)
opentx.elf: allsrc.cpp $(OBJS)
	@echo
	@echo $(MSG_COMPILING) $@
	$(CC) $(ARMCPPFLAGS) $< -o allsrc.o
	$(CC) $(OBJS) allsrc.o -mcpu=$(MCU) -mthumb -nostartfiles -lm -T$(LDSCRIPT) -Wl,-Map=$(TARGET).map,--cref,--no-warn-mismatch,--gc-sections $(NEWLIB_NANO_FLAGS) -o $@
endif

# Target: clean project.
clean: begin clean_list end

CLEAN_SRC=$(filter %.c,$(SRC))
clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) simu
	$(REMOVE) gtests
	$(REMOVE) gtest.a
	$(REMOVE) gtest_main.a
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).eep
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(TARGET).lst
	$(REMOVE) $(CLEAN_SRC:.c=.s)
	$(REMOVE) `find . -name "*.o"`
	$(REMOVE) *.d
	$(REMOVE) *.lst
	$(REMOVE) allsrc.cpp
	$(REMOVE) allsimusrc.cpp
	$(REMOVE) stamp-opentx.h
	$(REMOVE) *.lbm
	$(REMOVE) bitmaps/*.lbm
	$(REMOVE) bitmaps/*/*.lbm
	$(REMOVE) bitmaps/*/*/*.lbm
	$(REMOVE) fonts/*.lbm
	$(REMOVE) fonts/*/*.lbm
	$(REMOVE) translations/*.h
	$(REMOVE) lua_exports*.txt lua/lua_exports*.inc lua_fields*.txt
	$(MAKE) -C targets/taranis/bootloader clean

#### Install

DATE = ${shell sh -c "date +%F:%T" }
AVRCOMMAND = $(AVRDUDE) $(AVRDEVICE)

WRITE_COMMAND = $(AVRCOMMAND) -U flash:w:opentx.hex:i
ifeq ($(RESTORE_EEPROM), YES)
  BACKUP_EEPROM = YES
  ifeq ($(DOUBLE_WRITE_OK), YES)
    WRITE_COMMAND += -U eeprom:w:$(DATA_DIR)/eeprom-$(DATE).hex:i
  else
    SECOND_WRITE_COMMAND = -U eeprom:w:$(DATA_DIR)/eeprom-$(DATE).hex:i
  endif
endif

ifeq ($(BACKUP_FLASH), YES)
  BACKUP_ARGS += -U eeprom:r:$(DATA_DIR)/eeprom-$(DATE).hex:i $(BACKUPFLASH)
endif
ifeq ($(BACKUP_EEPROM), YES)
  BACKUP_ARGS += -U flash:r:$(DATA_DIR)/opentx-$(DATE).hex:i
endif

install: all
ifdef BACKUP_ARGS
	$(AVRCOMMAND) $(BACKUP_ARGS)
	$(SLEEP) $(DELAY)
endif
	$(WRITE_COMMAND)
ifdef	SECOND_WRITE_COMMAND
	$(SLEEP) $(DELAY)
	$(SECOND_WRITE_COMMAND)
endif

#### GOOGLE TESTS

GTEST_INCDIR = /usr/include/gtest
GTEST_SRCDIR = /usr/src/gtest/

# Where to find user code.
USER_DIR = ./

# All Google Test headers.  Usually you shouldn't change this
# definition.
GTEST_HEADERS = $(GTEST_INCDIR)/*.h \
                $(GTEST_INCDIR)/internal/*.h

# House-keeping build targets.

# Builds gtest.a and gtest_main.a.

# Usually you shouldn't tweak such internal variables, indicated by a
# trailing _.
GTEST_SRCS = $(GTEST_SRCDIR)/src/*.cc $(GTEST_HEADERS)

# For simplicity and to avoid depending on Google Test's
# implementation details, the dependencies specified below are
# conservative and not optimized.  This is fine as Google Test
# compiles fast and for ordinary users its source rarely changes.
gtest-all.o : $(GTEST_SRCS)
	$(CXX) $(CPPFLAGS) -I$(GTEST_SRCDIR) -I$(GTEST_INCDIR) -c $(GTEST_SRCDIR)/src/gtest-all.cc

# Builds a sample test.  A test should link with either gtest.a or
# gtest_main.a, depending on whether it defines its own main()
# function.

#use all .cpp files from tests/ dir
GTEST_TESTS_SRCS = $(shell find tests/ -type f -name '*.cpp')

gtests: allsimusrc.cpp $(GTEST_TESTS_SRCS) targets/simu/simpgmspace.cpp *.h tests/gtests.h gtest-all.o
	g++ -std=gnu++0x $(CPPFLAGS) $(SIMUCPPFLAGS) allsimusrc.cpp $(LUASRC) $(GTEST_TESTS_SRCS) targets/simu/simpgmspace.cpp ${INCFLAGS} -I$(GTEST_INCDIR) -I/usr/include/qt4 -o gtests -lpthread -MD -DSIMU -lQtCore -lQtGui gtest-all.o -fexceptions

