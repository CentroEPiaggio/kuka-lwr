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

include Makefile.global


export

# ********************************************************************************
# ********************************************************************************
# LINUX
# ********************************************************************************
# ********************************************************************************

ifeq ($(TARGET_OS),__LINUX__)

include TargetDependencies.global

export


# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all:
	@echo $(LINE2)
	@echo Building the entire project tree... [$(VARIANT)]
	@echo Operating system: $(OS_NAME)
	cd FastResearchInterfaceLibrary;make $(VARIANT)
	cd TypeIRML;make $(VARIANT)
	cd FastResearchInterfaceTest;make $(VARIANT)
	cd LWRGettingStartedExamples;make $(VARIANT)	
	@echo This is the end of the build process [$(VARIANT)].
	@echo $(LINE2)


# ****************************************************************
# Entry point for 'do_clean'
# ****************************************************************
do_clean:
	@echo $(LINE2)
	@echo Cleaning up the entire project tree... [$(VARIANT)]
	@echo Operating system: $(OS_NAME)
	cd FastResearchInterfaceLibrary;make $(VARIANT)
	cd TypeIRML;make $(VARIANT)
	cd FastResearchInterfaceTest;make $(VARIANT)
	cd LWRGettingStartedExamples;make $(VARIANT)
	@echo This is the end of the clean up process [$(VARIANT)].
	@echo $(LINE2)

include ExternalTargets.global

endif



