# ORB_SLAM2-PythonBindings
A python wrapper for ORB_SLAM2, which can be found at [https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2).
This branch is designed to work with the base version of ORB_SLAM2, with a couple of minimal API changes to access the system output.
These changes can be found in the "ORB_SLAM2 changes" folder,
and should simply add 3 functions to the ORB_SLAM2 System class.

## License
This code is licensed under the BSD Simplified license, although it requires and links to ORB_SLAM2, which is available under the GPLv3 license

## Prerequisites
Boost::Python and ORB_SLAM2

This has been tested on ubuntu 14.04 and built against Python3, although it does not rely on any python3 features.
If you're running into difficulty, check which version of python the Boost::Python you're using was built against.
In particular, at time of writing, the version installed by ubuntu's apt repository was built against python2.
Incorrect python versions will fail when the module is imported into python, not at compile time.
