COMPONENT_ADD_INCLUDEDIRS :=  include .

CPPFLAGS += -Wno-attributes -Wno-narrowing -DAAC_PLUS -DHQ_SBR -DPARAMETRICSTEREO

CODEC_LIB_PATH := $(COMPONENT_PATH)/lib/libcodecs.a
COMPONENT_ADD_LDFLAGS += $(CODEC_LIB_PATH)
COMPONENT_ADD_LINKER_DEPS += $(CODEC_LIB_PATH)
