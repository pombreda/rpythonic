#!/usr/bin/pypy
# this also works in Python2 and Python3
# to run this you must have compiled and installed libwayland-server.so to /usr/local/lib/

import os, sys, time, ctypes
import wayland_server as wl

loop = wl.event_loop_create()

def fd_dispatch( fd, mask, data ):
	print(fd, mask, data)
	data.contents.value = 420
	return 0

ptr = ctypes.pointer( ctypes.c_int() )
source = loop.add_fd( 1, wl.EVENT_READABLE, fd_dispatch, ptr )
source.check()

loop.dispatch( 0 )

assert ptr.contents.value == 420
source.remove()
loop.destroy()

print('wayland server test complete')

