#!/bin/bash

./gen-gtk-wrappers.py --gtk3 --output=../../pyppet/pyppet

./generate-wrappers.py --blender --output=../../pyppet/pyppet
./generate-wrappers.py --sdl --ode --opencv --openal --freenect-sync --fluid --fftw --output=../../pyppet/pyppet
./generate-wrappers.py --wiiuse --output=../../pyppet/pyppet

./generate-wrappers.py --avcodec --avformat --output=../../pyppet/pyppet
./generate-wrappers.py --libmlt --output=../../pyppet/pyppet

#./generate-wrappers.py --opengl --output=../../pyppet/pyppet

