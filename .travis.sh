#!/bin/sh

# Utils

PWD=$(pwd)
CMAKE_DIR="$PWD/cmake"
OPENCV_DIR="$PWD/opencv"

export PATH=$CMAKE_DIR/bin:$PATH
export PATH=/usr/local/opt/opencv3/share/OpenCV:$PATH
export PATH=$OPENCV_DIR/share/OpenCV:$PATH

download_check ()
{
	URL="$1"
	FILENAME="$2"
	SHA256="$3"
	wget --no-clobber $URL
	echo "Performing checksum"
	echo "$SHA256  $FILENAME" | sha256sum --check
}

extract ()
{
	FILENAME="$1"
	DIR="$2"
	mkdir -p $DIR
	echo "Extracting"
	tar  --extract --file $FILENAME --directory $DIR
}

install_cmake ()
{
	CMAKE_VERSION_MAJOR=3
	CMAKE_VERSION_MINOR=7
	CMAKE_VERSION_PATCH=2
	CMAKE_VERSION="$CMAKE_VERSION_MAJOR.$CMAKE_VERSION_MINOR.$CMAKE_VERSION_PATCH"
	CMAKE_FILENAME="cmake-$CMAKE_VERSION.tar.gz"
	# check https://cmake.org/files/v3.7/cmake-3.7.2-SHA-256.txt
	CMAKE_SHA256=dc1246c4e6d168ea4d6e042cfba577c1acd65feea27e56f5ff37df920c30cae0
	CMAKE_URL="https://cmake.org/files/v$CMAKE_VERSION_MAJOR.$CMAKE_VERSION_MINOR/$CMAKE_FILENAME"
	CMAKE_BUILD_DIR="$PWD"

	cd $PWD
	if ! download_check $CMAKE_URL $CMAKE_FILENAME $CMAKE_SHA256; then
		echo "Aborting"
		return 1
	fi

	extract $CMAKE_FILENAME $CMAKE_BUILD_DIR
	cd "$CMAKE_BUILD_DIR/cmake-$CMAKE_VERSION"
	mkdir -p $CMAKE_DIR
	./bootstrap --prefix="$CMAKE_DIR" --parallel=2 --system-curl --system-zlib &&\
	make -j2 install
}

install_opencv ()
{
	OPENCV_VERSION_MAJOR=3
	OPENCV_VERSION_MINOR=0
	OPENCV_VERSION_PATCH=0
	OPENCV_VERSION="$OPENCV_VERSION_MAJOR.$OPENCV_VERSION_MINOR.$OPENCV_VERSION_PATCH"
	OPENCV_FILENAME="$OPENCV_VERSION.tar.gz"
	OPENCV_SHA256=4dd618de44852b21f0f63709e648567b33b85de07ff2abdeb53fd4edafae2692
	OPENCV_URL="https://github.com/opencv/opencv/archive/$OPENCV_FILENAME"
	OPENCV_BUILD_DIR="$PWD"

	if ! download_check $OPENCV_URL $OPENCV_FILENAME $OPENCV_SHA256; then
		echo "Aborting"
		return 1
	fi

	cd $PWD
	extract $OPENCV_FILENAME $OPENCV_BUILD_DIR
	cd "$OPENCV_BUILD_DIR/opencv-$OPENCV_VERSION"
	mkdir -p $OPENCV_DIR build install
	cd build
	cmake .. -DCMAKE_INSTALL_PREFIX=$OPENCV_DIR -DCMAKE_BUILD_TYPE=Release\
		-DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF\
		-DBUILD_WITH_DEBUG_INFO=OFF\
		-DBUILD_opencv_apps=OFF\
		-DBUILD_opencv_calib3d=OFF\
		-DBUILD_opencv_core=ON\
		-DBUILD_opencv_features2d=OFF\
		-DBUILD_opencv_flann=OFF\
		-DBUILD_opencv_hal=ON\
		-DBUILD_opencv_highgui=ON\
		-DBUILD_opencv_imgcodecs=ON\
		-DBUILD_opencv_imgproc=ON\
		-DBUILD_opencv_ml=OFF\
		-DBUILD_opencv_objdetect=OFF\
		-DBUILD_opencv_photo=OFF\
		-DBUILD_opencv_python2=OFF\
		-DBUILD_opencv_shape=OFF\
		-DBUILD_opencv_stitching=OFF\
		-DBUILD_opencv_superres=OFF\
		-DBUILD_opencv_ts=OFF\
		-DBUILD_opencv_video=OFF\
		-DBUILD_opencv_videoio=ON\
		-DBUILD_opencv_videostab=OFF\
		-DBUILD_opencv_viz=OFF\
		-DBUILD_opencv_world=OFF &&\
	make -j2 install
}

# Linux

function install_linux ()
{
	install_cmake
	install_opencv
}

# OSX

function install_osx ()
{
	brew tap homebrew/science
	brew upgrade cmake
	brew install eigen opencv3
	brew install tinyxml2 doxygen
}

function build ()
{
	mkdir build && cd build
	cmake ..
	make -j2
}

function doc ()
{
	mkdir build && cd build
	cmake ..
	make doc
}

# Common

function install_f ()
{
	if [ "$TRAVIS_OS_NAME" == "linux" ]; then
		install_linux
	elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
		install_osx
	fi
}

case $1 in
	install ) install_f;;
	build ) build;;
	doc ) doc;;
esac
