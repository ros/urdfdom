all: installed
TARBALL = build/collada-dom-2.2.zip
TARBALL_URL = http://sourceforge.net/projects/collada-dom/files/Collada%20DOM/Collada%20DOM%202.2/collada-dom-2.2.zip/download
SOURCE_DIR = build/colladadom
INITIAL_DIR = build/collada-dom
UNPACK_CMD = unzip
TARBALL_PATCH=use_1_5.patch
MD5SUM_FILE = collada-dom-2.2.zip.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

ROOT = $(shell rospack find colladadom)

installed: wiped $(SOURCE_DIR)/unpacked
	@echo "ROOT is: $(ROOT)"
	-mkdir -p $(ROOT)/include
	-mkdir -p $(ROOT)/include/colladadom
	-mkdir -p $(ROOT)/lib
	@echo "making it"
	cd $(SOURCE_DIR)/dom && make $(ROS_PARALLEL_JOBS)
	cp -rf $(SOURCE_DIR)/dom/include/* $(ROOT)/include/colladadom/
	mv $(ROOT)/include/1.5/dom $(ROOT)/include/dom
	cp -f $(SOURCE_DIR)/dom/build/linux-1.5/*.* $(ROOT)/lib/
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
