#!/usr/bin/python
# updated june 2012

import os, sys, time, ctypes
import openGL as gl
import xlib as X

display = X.OpenDisplay()
print(display)

screen = X.DefaultScreen( display )
print(screen)

attrs = (ctypes.c_int*15)()
attrs[ 0 ] = gl.GLX_RGBA
attrs[ 1 ] = gl.GLX_DOUBLEBUFFER
attrs[ 2 ] = gl.GLX_RED_SIZE
attrs[ 3 ] = 8
attrs[ 4 ] = gl.GLX_BLUE_SIZE
attrs[ 5 ] = 8
attrs[ 6 ] = gl.GLX_GREEN_SIZE
attrs[ 7 ] = 8
attrs[ 8 ] = gl.GLX_ALPHA_SIZE
attrs[ 9 ] = 8
attrs[10] = gl.GLX_DEPTH_SIZE
attrs[11] = 8
attrs[12] = gl.GLX_STENCIL_SIZE
attrs[13] = 0
attrs[14] = 0


visual = gl.XChooseVisual(
	display,
	screen,
	attrs
)
print(visual)

context = gl.XCreateContext(
	display,
	visual,
	None, 	# shareList - ctypes.POINTER(__GLXcontextRec))
	True	# direct rendering
)
print(context)

	

print('trying to create window')
if 0:		# problem with colormap
	colormap = X.CreateColormap(
		display,
		X.RootWindow( display, screen ),
		visual.visual,
		X.AllocNone
	)
	winattrs = X.SetWindowAttributes()
	winattrs.colormap = colormap
	winattrs.border_pixel = 0
	window = X.CreateWindow(
		display,
		X.RootWindow( display, screen ),	#visual.screen ?
		0, 0,
		640, 480,
		0,	# border width
		visual.depth,
		X.InputOutput,
		visual.visual,
		X.CWBorderPixel | X.CWColormap | X.CWEventMask,
		ctypes.pointer( winattrs.POINTER )
	)

window = X.CreateSimpleWindow(
	display,
	X.RootWindow(display, screen),
	10, 10, 200, 200, 1,
	X.BlackPixel(display,screen),
	X.WhitePixel(display,screen),
)
print('window created', window)

#X.SelectInput( display, window, X.ExposeMask...

print('trying to map window')
X.MapWindow( display, window )

gl.XMakeCurrent( display, window, context )

print('loop...')
while True:
	X.FillRectangle(
		display, window, X.DefaultGC(display,screen),
		20, 20, 10, 10 
	)

X.CloseDisplay( display )

print('test complete')

