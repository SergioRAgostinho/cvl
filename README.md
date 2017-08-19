# Welcome to (yet another) Computer Vision Library (CVL) documentation

I hope it will shed some light on how the whole API is structured.

## Introduction

This library was designed to be cross-platform on all three major desktop operative systems, although it was mainly developed on two machines running Mac OS X and Ubuntu. This makes it likely that additional changes are required in order to have it running on Windows.

The code is licensed under the [Apache License 2.0](https://tldrlegal.com/license/apache-license-2.0-(apache-2.0)).

Currently there's not much you can do with `cvl` other than running the `vgm_grabber` example. That example in particular loads a model from the dataset and renders its wireframe on top of the groundtruth footage provided. On its own it might look practically useless, but in order to achieve it a number of useful baseline code was implemented, which will definitely be worth something to any user wanting to make use of that dataset from C++. Then again this is version 0.1 of the library. Please refer to the Wiki page of the example for further information.

## Requirements

### Compiler support

`cvl` is targeting C++14 feature set (which should become the default supported mode , therefore you'll need a compiler which supports it. Out of the top of my head here's some of the things which the library currently makes use:
-	Aliases
-	`auto` runtime type deduction
-	for-range loops
-	`std::unique_ptr` and `std::shared_ptr`
-	`emplace_back ()`
-	Universal references and `std::forward` for perfect forwarding functionalities
-	`std::forward_list`
-	`std::thread`, `std::async` and `std::future`
-	`constexpr`
-	`noexcept`
-	(and the list goes on...)

### Dependencies

Below is the current list of dependencies. `M` stands for mandatory and `R` is recommended.


| Library/Tool/App | Type | Version | Comments |
|------------------|------|---------|----------|
| CMake			   | M    | 3.7     | The core build system. |
| pkg-config	   |      |         | It aids a lot the search for dependencies. |
| Eigen	   		   | M    | 3       | The foundation for `cvl`'s linear algebra functionalities. |
| OpenCV   		   | M    | 3       | `core`, `videoio`, `highgui` and `imgproc` modules. |
| nanoflann		   | R    | HEAD    | A stripped down version of FLANN, used in the `search` modele. I submitted the [formula](https://github.com/Homebrew/homebrew-science/pull/6139) recently to [homebrew-science](https://github.com/Homebrew/homebrew-science)|
| TinyXML2		   |      | 	    | Used to parse XML for the `VgmGrabber` in `io`. |
| MATLAB		   |      | 	    | `cvl` makes use of the `mat` and `mx`libraries to read out the data from the VGM Dataset models with it. |
| GoogleTest	   |      | 	    | Enables building the unit tests. Coverage is less than minimal at this point. |
| Doxygen	   	   |      | 1.8.0   | Used to generate the documentation. |


## Frequently Asked Questions (F.A.Q.)

1. 	**Q: Why is was the namespace `ht` used instead of `cvl` for instance?**

	A: `ht` is an abbreviation for HipsterTech, the name of an [organization](https://github.com/HipsterTech) I'm a co-founder of, and which one day might do something noteworthy and (not necessarily) useful.

2. 	**Q: Why does the overall API an module structure look awfully close to [Pointcloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)?**

	A: I'm one of its maintainers, so it's only natural that I take advantage of some of the successful design patterns it uses.

3. 	**Q: Why did you set the target versions of C++ and CMake to such high values?**

	A: This is a new library and I don't like having backwards compatibility preventing me from using recent features. Ultimately no one likes to reinvent the wheel.
