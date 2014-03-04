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
# QNX
# ********************************************************************************
# ********************************************************************************

ifeq ($(TARGET_OS),_NTO_)

include ../TargetDependencies.global

export

SRCS         := $(shell find $(SRC_DIR)/LWRGettingStartedExamples -name '*.cpp')
OBJS         := $(patsubst $(SRC_DIR)/LWRGettingStartedExamples/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
EXES         := $(patsubst $(SRC_DIR)/LWRGettingStartedExamples/%.cpp, $(EXE_DIR)/%, $(SRCS))

# ---------- Objects ----------

$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/LWRGettingStartedExamples/%.cpp
	@echo $(LINE1)
	$(CC) $< -o $@

# -------- OutputFiles --------

$(EXE_DIR)/%: $(OBJ_DIR)/%.$(OBJECT_FILE_EXT)
	@echo $(LINE1)
	$(CL) -o $@ $^ -lm -lsocket -lFastResearchInterfaceLibrary -lTypeIRML

# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all: $(EXES)
	@echo Built directory contents
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	

# ****************************************************************
# Entry point for 'do_clean'
# ****************************************************************

do_clean:
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	rm -f $(OBJS)
	rm -f $(EXES)

include ../ExternalTargets.global

endif

	

