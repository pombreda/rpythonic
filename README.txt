RPythonic: Beta7 - April, 2012
Copyright: Brett Hartshorn (bhartsho@yahoo.com)
License: BSD


INSTALLING:
	apt-get install python-ply libreadline-dev

	Required to compile RPython:
		pypy source code

	Optional: 
		java
		android SDK/NDK
		llvm and llvm-gcc



NOTES:
	C++ Wrapper Gen - in progress...
	see the "scripts" folder for wrapper generator examples

PYPY NOTES:
	modify:
		pypy/translator/c/debug_print.c
			line 92, remove the ifndef block using "clock_gettime"
