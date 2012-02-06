#!/bin/bash
./generate-wrappers.py --blender --output=../../pyppet/pyppet
./generate-wrappers.py --sdl --ode --opencv --openal --freenect-sync --opengl --wiiuse --fluid --fftw --output=../../pyppet/pyppet
./gen-gtk-wrappers.py --gtk3 --output=../../pyppet/pyppet

