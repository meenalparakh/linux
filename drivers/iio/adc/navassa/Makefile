SRCS = $(addprefix devices/adrv9001/private/src/, $(notdir $(wildcard $(src)/devices/adrv9001/private/src/*.c))) \
	$(addprefix devices/adrv9001/private/src/b0/, $(notdir $(wildcard $(src)/devices/adrv9001/private/src/b0/*.c))) \
	$(addprefix devices/adrv9001/public/src/, $(notdir $(wildcard $(src)/devices/adrv9001/public/src/*.c))) \
	common/adi_logging/adi_common_log.c \
	common/adi_common.c  \
	common/adi_error/adi_common_error.c  \
	common/adi_hal/adi_common_hal.c \
	platforms/linux_platform.c \
	third_party/jsmn/jsmn.c \
	adrv9002.c \
	adrv9002_conv.c \
	adrv9002_init_data.c

ccflags-y += -I$(src)/devices/adrv9001/private/include \
	-I$(src)/devices/adrv9001/private/include/b0 \
	-I$(src)/devices/adrv9001/public/include  \
	-I$(src)/boards/adrv9001/private/include  \
	-I$(src)/boards/adrv9001/public/include/  \
	-I$(src)/common/ \
	-I$(src)/common/adi_hal/  \
	-I$(src)/common/adi_error/  \
	-I$(src)/common/adi_libc/  \
	-I$(src)/common/adi_logging/  \
	-I$(src)/platforms/ \
	-I$(src)/third_party/jsmn/ \
	-I$(src)/third_party/adi_pmag_macros/ \
	-DADRV9001_CHANNELID_CHECK=0 \
	-DCHECK_VCO_CAL=0 \
	-DRC_TUNE_UNIT_TEST=0 \
	-DSI_REV_B0 \
	-DWAIT_FOR_TEMP=0 \
	-DADI_DYNAMIC_PROFILE_LOAD \
	-DADI_COMMON_VERBOSE=1 \
	-DADI_ADRV9001_ARM_VERBOSE \
	-DADI_VALIDATE_PARAMS


obj-y = $(SRCS:.c=.o)
