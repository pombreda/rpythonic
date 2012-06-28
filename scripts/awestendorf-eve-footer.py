'''
Copyright 2011, Aaron Westendorf, All Rights Reserved.
https://github.com/awestendorf/eve/blob/master/LICENSE
Copyright (c) 2011 Aaron Westendorf
Updated June 2012 by Brett for libev-4.11

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Define the constants used from libev/ev.h in ev.c. The case is preserved.
Spec'd to libev-4.04. Attempted to use documentation straight from libev for structs.
'''
ev_tstamp = ctypes.c_double

class ev_loop(ctypes.Structure):
  '''
  Assuming built wth EV_MULTIPLICITY.
  '''
  _fields_ = [
    ('ev_rt_now', ev_tstamp),
  ]

class _ev_watcher( ctypes.Structure ):
  _fields_ = [
    ('active', ctypes.c_int),
    ('pending', ctypes.c_int),
    ('priority', ctypes.c_int),
    ('data', ctypes.c_void_p),
    ('cb', ctypes.c_void_p),
  ]

class ev_watcher(ctypes.Structure):
  '''base class, nothing to see here unless you subclass'''
  _fields_ = [
    ('active', ctypes.c_int),
    ('pending', ctypes.c_int),
    ('priority', ctypes.c_int),  # TODO: handle "#if EV_MINPRI == EV_MAXPRI"
    ('data', ctypes.c_void_p),
    ('cb', ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.POINTER(ev_loop), ctypes.POINTER(_ev_watcher), ctypes.c_int,))
  ]

# TODO: Determine if subclassing structs combines the _fields_ parameter

class ev_watcher_list(ctypes.Structure):
  '''base class, nothing to see here unless you subclass'''
  _fields_ = ev_watcher._fields_ + [
    ('next', ctypes.c_void_p), # TODO: make this real
  ]

class ev_watcher_time(ctypes.Structure):
  '''base class, nothing to see here unless you subclass'''
  _fields_ = ev_watcher._fields_ + [
    ('at', ev_tstamp),
  ]

class ev_io(ctypes.Structure):
  '''
  invoked when fd is either EV_READable or EV_WRITEable
  revent EV_READ, EV_WRITE
  '''
  _fields_ = ev_watcher_list._fields_ + [
    ('fd', ctypes.c_int), # ro
    ('events', ctypes.c_int), # ro
  ]

class ev_timer(ctypes.Structure):
  '''
  invoked after a specific time, repeatable (based on monotonic clock)
  revent EV_TIMEOUT
  '''
  _fields_ = ev_watcher_time._fields_ + [
    ('repeat', ev_tstamp), # rw
  ]

class ev_periodic(ctypes.Structure):
  '''
  invoked at some specific time, possibly repeating at regular intervals (based on UTC)
  revent EV_PERIODIC
  '''
  _fields_ = ev_watcher_time._fields_ + [
    ('offset', ev_tstamp), # rw
    ('interval', ev_tstamp), # rw
    ('callback', ctypes.CFUNCTYPE(ev_tstamp, ctypes.c_void_p, ev_tstamp))
  ]

class ev_signal(ctypes.Structure):
  '''
  invoked when the given signal has been received
  revent EV_SIGNAL
  '''
  _fields_ = ev_watcher_list._fields_ + [
    ('signum', ctypes.c_int), # ro
  ]

class ev_child(ctypes.Structure):
  '''
  invoked when sigchld is received and waitpid indicates the given pid
  revent EV_CHILD
  does not support priorities
  '''
  _fields_ = ev_watcher_list._fields_ + [
    ('flags', ctypes.c_int), # private
    ('pid', ctypes.c_int), # ro
    ('rpid', ctypes.c_int), # rw, holds the received pid
    ('rstatus', ctypes.c_int), # rw, holds the exit status, use the macros from sys/wait.h
  ]

class stat(ctypes.Structure):
  '''
  Defines sys/stat.h
  '''
  # TODO: Fill this in, figure something else out. Could just skip ev_stat
  # support.
  '''
  struct stat {
     dev_t     st_dev;     /* ID of device containing file */
     ino_t     st_ino;     /* inode number */
     mode_t    st_mode;    /* protection */
     nlink_t   st_nlink;   /* number of hard links */
     uid_t     st_uid;     /* user ID of owner */
     gid_t     st_gid;     /* group ID of owner */
     dev_t     st_rdev;    /* device ID (if special file) */
     off_t     st_size;    /* total size, in bytes */
     blksize_t st_blksize; /* blocksize for file system I/O */
     blkcnt_t  st_blocks;  /* number of 512B blocks allocated */
     time_t    st_atime;   /* time of last access */
     time_t    st_mtime;   /* time of last modification */
     time_t    st_ctime;   /* time of last status change */
  };
  '''

# TODO: implement win32 support
'''
typedef struct _stati64 ev_statdata;
# else
typedef struct stat ev_statdata;
'''
ev_statdata = stat

class ev_stat(ctypes.Structure):
  '''
  invoked each time the stat data changes for a given path
  revent EV_STAT
  '''
  _fields_ = ev_watcher_list._fields_ + [
    ('timer', ev_timer), # private
    ('interval', ev_tstamp), # ro
    ('path', ctypes.c_char_p), # ro
    ('prev', ev_statdata), # ro
    ('attr', ev_statdata), # ro
    ('wd', ctypes.c_int), # wd for inotify, fd for kqueue 
  ]

class ev_idle(ctypes.Structure):
  '''
  invoked when the nothing else needs to be done, keeps the process from blocking
  revent EV_IDLE
  '''
  _fields_ = ev_watcher._fields_[:]

class ev_prepare(ctypes.Structure):
  '''
  invoked for each run of the mainloop, just before the blocking call
  you can still change events in any way you like
  revent EV_PREPARE
  '''
  _fields_ = ev_watcher._fields_[:]

class ev_check(ctypes.Structure):
  '''
  invoked for each run of the mainloop, just after the blocking call
  revent EV_CHECK
  '''
  _fields_ = ev_watcher._fields_[:]

#if EV_FORK_ENABLE
class ev_fork(ctypes.Structure):
  '''
  the callback gets invoked before check in the child process when a fork was detected
  revent EV_FORK
  '''
  _fields_ = ev_watcher._fields_[:]

class ev_cleanup(ctypes.Structure):
  '''
  is invoked just before the loop gets destroyed
  revent EV_CLEANUP
  '''
  _fields_ = ev_watcher._fields_[:]

class ev_embed(ctypes.Structure):
  '''
  used to embed an event loop inside another
  the callback gets invoked when the event loop has handled events, and can be 0
  '''
  _fields_ = ev_watcher._fields_ + [
    ('other', ctypes.c_void_p), # ro TODO: make this real
    ('io', ev_io), # private
    ('prepare', ev_prepare), # private
    ('check', ev_check), # unused
    ('timer', ev_timer), # unused
    ('periodic', ev_periodic), # unused
    ('idle', ev_idle), # unused
    ('fork', ev_fork), # private
    ('cleanup', ev_cleanup), # unused
  ]

#############################################

def ev_init(watcher, callback):
	watcher.active = 0
	watcher.pending = 0
	cfunctype = ev_watcher._fields_[-1][-1]
	watcher.cb = cfunctype( callback )
init = ev_init

def ev_stat_init( watcher, callback, path, interval=0.0 ):
	ev_init( watcher, callback )
	watcher.path = path
	watcher.interval = interval
stat_init = ev_stat_init

