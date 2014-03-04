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

# ---------- Objects ----------

SRCS  := $(shell find $(SRC_DIR)/TypeIRML -name '*.cpp')
OBJS  := $(patsubst $(SRC_DIR)/TypeIRML/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))


$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/TypeIRML/%.cpp
	@echo $(LINE1)
	$(CC) $< -o $@

# -------- OutputFiles --------

$(LIB_DIR)/libTypeIRML.$(LIB_FILE_EXT): $(OBJS)
	@echo $(LINE1)
	$(AR) $@ $^

# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all: $(LIB_DIR)/libTypeIRML.$(LIB_FILE_EXT)
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
	rm -f $(LIB_DIR)/libTypeIRML.$(LIB_FILE_EXT)

include ../ExternalTargets.global

endif

	

