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

assert ev.EVBACKEND_POLL == 2
main = ev.default_loop( ev.EVBACKEND_POLL )

watcher = ev.ev_stat()
def mycallback( loop, watcher, revents ):
	print(loop, watcher, revents)

ev.init( watcher, mycallback )
ev.stat_init( watcher, mycallback, '/tmp/hi' )
ptr = ctypes.pointer( watcher )
ev.stat_start( main, ptr )

main.run()

print('watchers', main.pending_count())

main.suspend()
main.resume()

ev.stat_stop( main, ptr )

main.loop_destroy()

print('test done')
