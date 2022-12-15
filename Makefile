CC=/usr/bin/clang
CXX=/usr/bin/clang++
# CC=/usr/bin/gcc
# CXX=/usr/bin/g++
build_type=Release

.SILENT:
all: build/CMakeLists.txt.copy
	$(info Build type is [${build_type}])
	$(MAKE) --no-print-directory -C build

set_debug:
	$(eval build_type=Debug)

debug_all: | set_debug all

clean:
	rm -rf build bin lib

build/CMakeLists.txt.copy: CMakeLists.txt Makefile
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) \
		-DCMAKE_CXX_COMPILER=$(CXX) \
		-DCMAKE_C_COMPILER=$(CC) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy
