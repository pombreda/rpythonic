#!/usr/bin/python
import os, sys, time, ctypes
import xlib as X

display = X.OpenDisplay()
print(display)
screen = X.DefaultScreen( display )
print(screen)

window = X.CreateSimpleWindow(
	display,
	X.RootWindow(display, screen),
	10, 10, 200, 200, 1,
	X.BlackPixel(display,screen),
	X.WhitePixel(display,screen),
)
print(window)

X.SelectInput( display, window, X.ExposureMask | X.KeyPressMask )

X.MapWindow( display, window )

event = X.Event()

while True:
	X.NextEvent( display, event )

	X.FillRectangle(
		display, window, X.DefaultGC(display,screen),
		20, 20, 10, 10 
	)

	if event.type == X.KeyPress:	# TODO fixme
		break

X.CloseDisplay( display )

print('test complete')

