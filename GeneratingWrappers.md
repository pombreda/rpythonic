#generating wrappers to C libraries.

# Introduction #

Its likely you will need to work with several C libraries in your project.  Writing wrappers by hand is very slow, error prone, and tedious.  Other older wrapper generators are usually based on GccXML, which is no longer maintained.  RPythonic integrates with the latest PyCParser to generate ctypes and RFFI bindings.  The ctypes bindings will generate an object oriented interface, and work like many hand-written wrappers hiding the ugly ctypes details as much as possible.  The process is very simple, just a single function call, rpythonic.wrap( 'name', 'path/to/myheader.h' )


![http://2.bp.blogspot.com/-oVoPWCvKERU/TfHykGDmzUI/AAAAAAAAAHc/iNJ-9cMoh5M/s1600/Screenshot-9.png](http://2.bp.blogspot.com/-oVoPWCvKERU/TfHykGDmzUI/AAAAAAAAAHc/iNJ-9cMoh5M/s1600/Screenshot-9.png)

# Overview #
  * import rpythonic
  * call rpythonic.wrap( 'name', 'path/to/header.h' )

## Simple Example ##
```
mod = rpythonic.load( 'SDL' )
if not mod:  # if not found in cache, generate new wrapper
	rpythonic.wrap( 
		'SDL', 
		header='/usr/include/SDL/SDL.h', 
		ctype=True, # defaults to true
		rffi=True, # generate RFFI wrappers, defaults to false
	)

```



## Mix Custom Wrapper Code Example ##
```
mod = rpythonic.load( 'ode' )
if not mod:
	defines = ['dDOUBLE']
	footer = '''
### ode headers sometimes define a return type as a pointer when it might be a vec3 or vec4 ###
Vector3 = ctypes.c_double * 3
Vector4 = ctypes.c_double * 4
def _ode_convert_to_vector3_( pointer=None ):
	v = ctypes.cast( pointer, ctypes.POINTER(Vector3) )
	return v.contents[0], v.contents[1], v.contents[2]
def _ode_convert_to_vector4_( pointer=None ):
	v = ctypes.cast( pointer, ctypes.POINTER(Vector4) )
	return v.contents[0], v.contents[1], v.contents[2], v.contents[3]

for func in ( dBodyGetPosition, dBodyGetRotation, dBodyGetLinearVel, dBodyGetAngularVel, dBodyGetForce, dBodyGetTorque  ):
	func.return_wrapper = _ode_convert_to_vector3_
	func.object_oriented = True

for func in ( dBodyGetQuaternion,  ):
	func.return_wrapper = _ode_convert_to_vector4_
	func.object_oriented = True
########### end of manual patch #########
	'''
	rpythonic.wrap( 'ode', header='/usr/include/ode/ode.h', 
		defines=defines, rffi=True, ctypes_footer=footer 
	)
```