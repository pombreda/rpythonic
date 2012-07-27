#!/usr/bin/pypy
# this also works in Python2 and Python3
# to run this you must have compiled and installed libwayland-server.so to /usr/local/lib/

import os, sys, time, ctypes
import socket
import wayland_server as wl

sock1, sock2 = socket.socketpair( socket.AF_UNIX )
display = wl.display_create()
print(display)
client = wl.client_create( display, sock2.fileno() )
print(client)

TEST = False
def mycallback( listener, data ):
	global TEST
	print('CALLBACK', listener, data )
	TEST = True

functype = wl.wl_listener.CSTRUCT._fields_[-1][-1]
funcptr = functype( mycallback )

listener = wl.wl_listener().POINTER
listener.contents.notify = funcptr
client.add_destroy_listener( listener )

listener = client.get_destroy_listener( funcptr )
assert listener.POINTER.contents

client.destroy()
assert TEST
print('wayland client test complete')

