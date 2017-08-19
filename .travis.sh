#!/bin/sh

function install_f ()
{
	brew tap homebrew/science
	brew upgrade cmake
	brew install eigen opencv3
	brew install tinyxml2 doxygen
}

function build ()
{
	export PATH=/usr/local/opt/opencv3/share/OpenCV:$PATH
	mkdir build && cd build
	cmake ..
	make -j2
}

function doc ()
{
	export PATH=/usr/local/opt/opencv3/share/OpenCV:$PATH
	mkdir build && cd build
	cmake ..
	make doc
}

case $1 in
	install ) install_f;;
	build ) build;;
	doc ) doc;;
esac
