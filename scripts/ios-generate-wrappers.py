#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic
rpythonic.set_cache( '../cache' )
rpythonic.set_pypy_root( '../../pypy' )
################################


## requires iOS headers ##
USR_INC = SDK_ROOT = '../../ios-sdk-2.0/'
assert os.path.isdir( SDK_ROOT )

includes = ['./ios-hacks']
defines = ['TARGET_OS_IPHONE', '__arm__', '__LITTLE_ENDIAN__', '__GNUC__', '__APPLE_CPP__', '_ANSI_SOURCE']
undefines = ['__i386__', '__x86_64__']
library = '<undefined>'

name = 'GLES'
header = os.path.join(os.path.join( USR_INC, 'OpenGLES' ), 'ES1')
header = os.path.join( header, 'glext.h' )		# glext superset of gl.h
#library = os.path.join( USR_LIB, 'libGLESv1_CM.so' )
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines,
	ctypes=False, platform='iOS'
	)

name = 'OpenAL'
header = os.path.join(os.path.join( USR_INC, name ), 'al.h')
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines,
	ctypes=False, platform='iOS'
	)

name = 'QuartzCore'
header = os.path.join(os.path.join( USR_INC, name ), 'QuartzCore.h')
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines, undefines=undefines,
	ctypes=False, platform='iOS'
	)


name = 'CoreFoundation'
header = os.path.join(os.path.join( USR_INC, name ), 'CoreFoundation.h')
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines, undefines=undefines,
	ctypes=False, platform='iOS'
	)

name = 'CoreGraphics'
header = os.path.join(os.path.join( USR_INC, name ), 'CoreGraphics.h')
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines, undefines=undefines,
	ctypes=False, platform='iOS'
	)

if 0:	# Invalid char constant 'lpcm'
	name = 'CoreAudio'
	header = os.path.join(os.path.join( USR_INC, name ), 'CoreAudioTypes.h')
	rpythonic.wrap( 
		name, header=header, library=library, 
		includes=includes, system_include=USR_INC,
		defines=defines, undefines=undefines,
		ctypes=False, platform='iOS'
		)

	name = 'AudioToolbox'
	header = os.path.join(os.path.join( USR_INC, name ), 'AudioToolbox.h')
	rpythonic.wrap( 
		name, header=header, library=library, 
		includes=includes, system_include=USR_INC,
		defines=defines, undefines=undefines,
		ctypes=False, platform='iOS'
		)



if 0:	# contains obj-c
	name = 'CoreLocation'
	header = os.path.join(os.path.join( USR_INC, name ), 'CoreLocation.h')
	rpythonic.wrap( 
		name, header=header, library=library, 
		includes=includes, system_include=USR_INC,
		defines=defines, undefines=undefines,
		ctypes=False, platform='iOS'
		)

	name = 'MediaPlayer'
	header = os.path.join(os.path.join( USR_INC, name ), 'MediaPlayer.h')
	rpythonic.wrap( 
		name, header=header, library=library, 
		includes=includes, system_include=USR_INC,
		defines=defines, undefines=undefines,
		ctypes=False, platform='iOS'
		)



