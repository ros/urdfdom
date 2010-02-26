all: installed

REVISION=258
PACKAGE_NAME=`rospack find assimp`
# Regular download
TARBALL_NAME = assimp-r$(REVISION)
TARBALL = build/$(TARBALL_NAME).tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/$(TARBALL_NAME).tar.gz
TARBALL_PATCH = assimp.patch
UNPACK_CMD = tar xzf
SOURCE_DIR = build/assimp_svn
include $(shell rospack find mk)/download_unpack_build.mk

# SVN checkout (only for package maintainers)
SVN_DIR = $(SOURCE_DIR)
#build/assimp_svn
SVN_REVISION = -r $(REVISION)
SVN_URL = https://assimp.svn.sourceforge.net/svnroot/assimp/trunk
include $(shell rospack find mk)/svn_checkout.mk

ASSIMP_DIR = $(shell rospack find assimp)
BOOST_INCLUDE_DIRS = $(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS = $(shell rosboost-cfg --lib_dirs signals,thread)
BOOST_LIBRARIES = $(shell rosboost-cfg --lflags signals,thread)
CMAKE_ARGS = -D CMAKE_INSTALL_PREFIX=$(PACKAGE_NAME) \
             -D CMAKE_BUILD_TYPE=RelWithDebInfo \
             -D boost_include_dirs=$(BOOST_INCLUDE_DIRS) \
             -D boost_library_dirs=$(BOOST_LIBRARY_DIRS) \
             -D boost_libraries="$(BOOST_LIBRARIES)"

# Build tarball (only for package maintainers)
build_tarball: $(SVN_DIR)
	-cd $(SVN_DIR)/build && make clean
	-cd $(SVN_DIR) && rm -rf build
	cd build && tar --exclude="test" --exclude="workspaces" --exclude="tools" --exclude="scripts" --exclude="samples" --exclude="port" --exclude="obj" --exclude="bin" --exclude=".svn" -czf ../$(TARBALL) assimp_svn

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && mkdir -p build && cd build && cmake $(CMAKE_ARGS) .. && export PARALLEL_JOBS=ROS_PARALLEL_JOBS && make $(ROS_PARALLEL_JOBS) && make install
	mkdir -p $(PACKAGE_NAME)/include/assimp
	cp -r $(SOURCE_DIR)/include/* $(PACKAGE_NAME)/include/assimp/
	#touch installed

clean:
	-cd $(SVN_DIR)/build && make clean
	-rm -f installed
	-rm -rf include lib

wipe: clean
	rm -rf build

