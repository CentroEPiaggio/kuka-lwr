# Makefile
# 
# Stanford University
# Department of Computer Science
# Artificial Intelligence Laboratory
# Gates Computer Science Building 1A
# 353 Serra Mall
# Stanford, CA 94305-9010
# USA
#
# http://cs.stanford.edu/groups/manips
#
# Author : Torsten Kroeger, tkr@stanford.edu
# Date   : November 2011
#
#

all:

include ../Makefile.global

export

# ********************************************************************************
# ********************************************************************************
# MacOS
# ********************************************************************************
# ********************************************************************************

ifeq ($(TARGET_OS),__MACOS__)

include ../TargetDependencies.global

export

# ---------- Objects ----------

FastResearchInterfaceLibrary_SRCS  := $(shell find $(SRC_DIR)/FastResearchInterfaceLibrary -name '*.cpp')
FastResearchInterfaceLibrary_OBJS  := $(patsubst $(SRC_DIR)/FastResearchInterfaceLibrary/%.cpp, $(OBJ_DIR)/%.o, $(FastResearchInterfaceLibrary_SRCS))
MACOSAbstraction_SRCS              := $(shell find $(SRC_DIR)/MACOSAbstraction -name '*.cpp')
MACOSAbstraction_OBJS              := $(patsubst $(SRC_DIR)/MACOSAbstraction/%.cpp, $(OBJ_DIR)/%.o, $(MACOSAbstraction_SRCS))

$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/FastResearchInterfaceLibrary/%.cpp
	@echo $(LINE1)
	$(CC) $< -o $@
	
$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/MACOSAbstraction/%.cpp
	@echo $(LINE1)
	$(CC) $< -o $@	

# -------- OutputFiles --------

$(LIB_DIR)/libFastResearchInterfaceLibrary.$(LIB_FILE_EXT): $(FastResearchInterfaceLibrary_OBJS) $(MACOSAbstraction_OBJS)
	@echo $(LINE1)
	$(AR) $@ $^

# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all: $(LIB_DIR)/libFastResearchInterfaceLibrary.$(LIB_FILE_EXT)
	@echo Built directory contents
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	

# ****************************************************************
# Entry point for 'do_clean'
# ****************************************************************

do_clean:
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	rm -f $(FastResearchInterfaceLibrary_OBJS) $(MACOSAbstraction_OBJS)
	rm -f $(LIB_DIR)/libFastResearchInterfaceLibrary.$(LIB_FILE_EXT)

include ../ExternalTargets.global

endif

	

