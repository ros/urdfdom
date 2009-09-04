all: installed

TARBALL = build/ConvexDecomposition.zip
TARBALL_URL = http://pr.willowgarage.com/downloads/ConvexDecomposition.zip
SOURCE_DIR = build/convex_decomposition
INITIAL_DIR = build/ConvexDecomposition
UNPACK_CMD = unzip
TARBALL_PATCH=convex_decomposition.patch

include $(shell rospack find mk)/download_unpack_build.mk

ROOT = $(shell rospack find convex_decomposition)/convex_decomposition

installed: wiped $(SOURCE_DIR)/unpacked
	@echo "making it"
	@echo "ROOT is: $(ROOT)"
	-mkdir -p $(ROOT)
	-mkdir -p $(ROOT)/bin
	cd $(SOURCE_DIR) ; make $(ROS_PARALLEL_JOBS); make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf $(ROOT) installed

wipe: clean
	rm -rf build

.PHONY : clean download wipe
