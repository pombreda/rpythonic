#!/usr/bin/python
#!/usr/bin/pypy
# this also works in python3
'''
libev ctypes helloworld test1
"touch /tmp/hi"
run this script
run a few more times "touch /tmp/hi"

'''

import os, sys, time, ctypes
import libev as ev

print('libev version: %s.%s' %(ev.version_major(), ev.version_minor()))

main = ev.default_loop( 2 )	# ev.EVBACKEND_POLL = 2
print(main)

def ev_init(watcher, callback):
	watcher.active = 0
	watcher.pending = 0
	cb = ev.ev_watcher._fields_[-1][-1]( callback )
	watcher.cb = cb

def ev_stat_init( watcher, callback, path, interval=0.0 ):
	ev_init( watcher, callback )
	watcher.path = path
	watcher.interval = interval

watcher = ev.ev_stat()
def mycallback( loop, watcher, revents ):
	print(loop, watcher, revents)

ev_init( watcher, mycallback )
ev_stat_init( watcher, mycallback, '/tmp/hi' )
ptr = ctypes.pointer( watcher )
ev.stat_start( main, ptr )

main.run()

print('watchers', main.pending_count())

main.suspend()
main.resume()

ev.stat_stop( main, ptr )

main.loop_destroy()

print('test done')
