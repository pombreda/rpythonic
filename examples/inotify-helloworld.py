#!/usr/bin/pypy
import os, sys, select, time, ctypes
import inotify
import unistd

fd = inotify.init()
assert fd != -1
print('fd',fd)

wd = inotify.add_watch(fd, '/tmp/hi', inotify.ALL_EVENTS)
assert wd != -1
print('wd',wd)

size = ctypes.sizeof( inotify.inotify_event.CSTRUCT )
#print('size of struct', size )

buff = (ctypes.c_char * size)()
ptr = ctypes.cast( buff, ctypes.POINTER(ctypes.c_void_p) )

while True:
	read, write, errors = select.select( [fd], [], [] )
	if read:
		unistd.read( fd, ptr, size )
		#print(''.join(buff))
		event = ctypes.cast( ptr, ctypes.POINTER(inotify.inotify_event.CSTRUCT) ).contents
		print(event)
		#print('wd',event.wd)
		#print('mask',event.mask)
		#print('cookie',event.cookie)
		#print('len', event.C_len)

		mask = event.mask
		if mask & inotify.ACCESS: print( 'access file' )
		if mask & inotify.ATTRIB: print('attribs changed')
		if mask & inotify.CLOSE_WRITE: print('close write')
		if mask & inotify.CLOSE_NOWRITE: print('closed (no write)')
		if mask & inotify.CREATE: print('create')
		if mask & inotify.DELETE: print('something in directory removed')
		if mask & inotify.DELETE_SELF: print('removed')
		if mask & inotify.MODIFY: print('modified')

		if mask & inotify.MOVE_SELF: print('moved')
		if mask & inotify.MOVED_FROM: print('file moved out of watched dir')
		if mask & inotify.MOVED_TO: print('moved into watched dir')
		if mask & inotify.OPEN: print('open')



