all: installed
TARBALL = build/collada-dom-2.2.zip
TARBALL_URL = http://pr.willowgarage.com/downloads/collada-dom-2.2.zip
SOURCE_DIR = build/colladadom
INITIAL_DIR = build/collada-dom
UNPACK_CMD = unzip
TARBALL_PATCH = collada-dom.patch mac-x86_64.patch
MD5SUM_FILE = collada-dom-2.2.zip.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

ROOT = $(shell rospack find colladadom)

# Copied from build/colladadom/dom/Makefile:
# os: 'linux', 'mac', 'windows', or 'ps3'. Use the 'uname' command to decide the
# default value. To detect when we're on Windows we'll check to see if we're
# running on Cygwin or MinGW.
OS := linux
ifneq ($(shell uname | grep -i darwin),)
OS := mac
else ifneq ($(or $(shell uname | grep -i cygwin),$(shell uname | grep -i mingw)),)
OS := windows
endif

installed: wiped $(SOURCE_DIR)/unpacked
	@echo "ROOT is: $(ROOT)"
	-mkdir -p $(ROOT)/include
	-mkdir -p $(ROOT)/lib
	@echo "making it"
	cd $(SOURCE_DIR)/dom && make $(ROS_PARALLEL_JOBS)
	cp -rf $(SOURCE_DIR)/dom/include/* $(ROOT)/include/
	cp -rf $(SOURCE_DIR)/dom/build/$(OS)-1.5/*.* $(ROOT)/lib/
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf $(ROOT)/include $(ROOT)/lib installed

wipe: clean
	rm -rf build

.PHONY : clean download wipe
