#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic

if sys.argv[-1].startswith('--output='):
	rpythonic.set_cache( sys.argv[-1].split('=')[-1] )
else:
	rpythonic.set_cache('../examples' )


ALL = '--all' in sys.argv

debug=0
if '--emokit' in sys.argv or ALL:
	#apt-get install libmcrypt-dev liboscpack-dev
	#git clone http://github.com/qdot/emokit.git
	rpythonic.wrap( 'emokit', 
		header='/usr/local/include/libepoc.h',
		library = '/usr/local/lib/libepoc.so',
	)


if '--python3' in sys.argv or ALL:
	footer = 'PyRun_SimpleString = PyRun_SimpleStringFlags'
	rpythonic.wrap( 'python3', 
		header='/usr/include/python3.2/Python.h',
		library = '/usr/lib/libpython3.2mu.so',
		ctypes_footer = footer,
	)



if '--ogre' in sys.argv:
	defines = []
	ignore_ogre = 'MeshSerializerImpl MeshSerializerImpl_v1_4 MeshSerializerImpl_v1_3 MeshSerializerImpl_v1_2 MeshSerializerImpl_v1_1 InstancedGeometry::MaterialBucket MaterialBucket ScriptCompiler StringUtil'.split()
	ignore_ogre += 'SharedPtr ExceptionFactory ControllerManager UTFString AnimableValue'.split()
	ignore_ogre += 'RenderQueueInvocationList RenderQueueInvocationIterator ConstShadowTextureConfigIterator ConstEnabledAnimationStateIterator'.split()
	ignore_ogre += 'Any AnyNumeric AnimableObject Animation DefaultWorkQueueBase'.split()
	ignore_rtss = 'UniformParameter NormalMapLighting TargetRenderState'.split()

	rpythonic.wrap( 'Ogre', 
		header='/usr/local/include/OGRE/Ogre.h', 
		insert_headers = ['/usr/local/include/OGRE/RTShaderSystem/OgreRTShaderSystem.h'],
		library='/usr/local/lib/libOgreMain.so',
		dynamic_libs = ['OgreMain', 'OgreRTShaderSystem'],
		defines=defines, ctypes=True, rffi=True, 
		ignore_classes = ignore_ogre + ignore_rtss,
		ignore_functions = ['Ogre::any_cast'],
		cplusplus=True
	)
	assert 0


if '--qt' in sys.argv:
    
	includes = [ 
		'/usr/include/qt4', 
		'/usr/include/qt4/QtCore', 
		'/usr/include/qt4/QtGui',
	]

	insert_headers = []

	rpythonic.wrap( 'Qt', 
		header='/usr/include/qt4/QtCore/QtCore',
		includes=includes, insert_headers=insert_headers,
		library = '',
		dynamic_libs=[ 'QtCore' ],
		ignore_classes = (),
		cplusplus=True,
	)


if '--g3d' in sys.argv:
	ignore_funcs = ['G3D::'+n for n in 'toString glGetCurrentContext gcchtonl zipfileExists stringPtrCompare failureHook assertionHook debugPrint consolePrint consolePrintHook'.split()]
	rpythonic.wrap( 'G3D', 
		header='/usr/local/include/G3D/G3DAll.h',
		static_libs=[ 'G3D' ],
		ignore_classes = [],
		ignore_functions = ignore_funcs,
		cplusplus=True,
	)


if '--irr' in sys.argv:
	#apt-get install libxxf86vm-dev
	#cd irrlicht-1.7.2/source/Irrlicht
	#make sharedlib
	root = '../../irrlicht-1.7.2/source/Irrlicht/'
	insert = []
	for h in 'CIrrDeviceSDL.h CIrrDeviceLinux.h CSceneManager.h COpenGLDriver.h'.split():
		insert.append( root+h )
	#for h in os.listdir( root ):
	#	if h.endswith('.h'):
	#		insert.append( root+h )

	rpythonic.wrap( 'Irrlicht', 
		header='/usr/local/include/irrlicht/irrlicht.h',
		insert_headers = insert,
		includes=['../../irrlicht-1.7.2/source/Irrlicht/'],
		library = '/usr/local/lib/libIrrlicht.so',
		dynamic_libs=[ 'Irrlicht' ],
		ignore_classes = [],	#'SColorHSL'],    # CppHeaderParser bug: "toRGB1" should be private
		cplusplus=True,
	)
#'SMaterialLayer', 'IDynamicMeshBuffer', 'CVertexBuffer', 'CIndexBuffer', 'CDynamicMeshBuffer', 'IImage', 'IGUIElement', 'IMaterialRenderer',  ],

nroot = '../../naali'
if os.path.isdir( nroot ) and '--naali' in sys.argv:
	includes = [ 
		'../../naali-deps/install/include',	# for kNetFwd.h and others
		nroot, 
		nroot+'/Core', 
		nroot+'/Scene', 
		nroot+'/Interfaces', 
		nroot+'/Input',
		'/usr/include/qt4', 
		'/usr/include/qt4/QtCore', 
		'/usr/include/qt4/QtGui',
	]

	insert_headers = []
	for h in 'SceneAPI.h Entity.h EntityAction.h IAttribute.h IComponent.h SceneEvents.h EC_Name.h'.split():
		insert_headers.append( '%s/Scene/%s' %(nroot,h) )

	rpythonic.wrap( 'Naali', 
		header='%s/Foundation/Framework.h' %nroot,
		includes=includes, insert_headers=insert_headers,
		library = '',
		ignore_classes = ('ConsoleCallbackInterface', 'QVariant', 'IComponentFactory'),
		cplusplus=True,
	)






broot = '/usr/local/include/bullet'	# tested with bullet 2.77, something broken in 2.78?
if os.path.isdir( broot ) and '--bullet' in sys.argv:
	includes = [ broot ]
	# DOUBLE is off by default
	#undefines = ['BT_USE_DOUBLE_PRECISION']	# build your bullet with FLOATING point precision

	rpythonic.wrap( 'BulletPhysics', 
		header='%s/btBulletDynamicsCommon.h' %broot,
		insert_headers=['%s/btBulletCollisionCommon.h' %broot],
		includes=includes,
		library = '/usr/local/lib/libBulletDynamics.so',
		dynamic_libs = ['LinearMath', 'BulletCollision', 'BulletDynamics'],
		ignore_classes = 'btMultiSapBroadphase CProfileIterator'.split(),
		cplusplus=True,
	)

	#rpythonic.wrap( 'BulletCollision', 
	#	includes=includes,
	#	library = '/usr/local/lib/BulletCollision/libBulletCollision.so',
	#	cplusplus=True,
	#)


if '--sdl' in sys.argv or ALL:
	rpythonic.wrap( 
		'SDL', 
		header='/usr/include/SDL/SDL.h', 
		strip_prefixes = ['SDL_'],
	)


if '--ode' in sys.argv or ALL:
	defines = ['dDOUBLE']
	footer = '''
### ode headers sometimes define a return type as a pointer, when it should be an array ###
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
	#ODE_API dJointID dJointCreateContact (dWorldID, dJointGroupID, const dContact *);
	rpythonic.wrap(
		'ode', 
		header='/usr/include/ode/ode.h', 
		ctypes_footer=footer,
		strip_prefixes = ['d'],
		defines = ['dDOUBLE'],
	)




if '--opencv' in sys.argv or ALL:
	## need to inject some aliases and globals ##
	footer = '''
IPL_DEPTH_1U = 1
IPL_DEPTH_8U = 8
IPL_DEPTH_16U = 16
IPL_DEPTH_32F = 32
IPL_DEPTH_SIGN = 0x80000000
IPL_DEPTH_8S = IPL_DEPTH_SIGN | 8
IPL_DEPTH_16S = IPL_DEPTH_SIGN | 16
IPL_DEPTH_32S = IPL_DEPTH_SIGN | 32

IPL_DATA_ORDER_PIXEL = 0
IPL_DATA_ORDER_PLANE = 1

IPL_ORIGIN_TL = 0
IPL_ORIGIN_BL = 1

IPL_ALIGN_4BYTES =  4
IPL_ALIGN_8BYTES =  8
IPL_ALIGN_16BYTES = 16
IPL_ALIGN_32BYTES = 32
IPL_ALIGN_DWORD =  IPL_ALIGN_4BYTES
IPL_ALIGN_QWORD =  IPL_ALIGN_8BYTES

IPL_GET_TILE_TO_READ =  1
IPL_GET_TILE_TO_WRITE = 2
IPL_RELEASE_TILE = 4

IPL_LUT_LOOKUP = 0
IPL_LUT_INTER = 1

CV_AA = 16
CV_WHOLE_SEQ = CvSlice(0,9999999)
cvConvert = lambda a,b: cvConvertScale( a, b, 1.0, 0.0 )

IplImage.CvtColor = lambda a, b, t: cvCvtColor(a, b, t )
IplImage.Convert = lambda a, b: cvConvertScale( a, b, 1.0, 0.0 )

'''
	rpythonic.wrap( 
		'cv', 
		defines = ['_MMINTRIN_H_INCLUDED', '_XMMINTRIN_H_INCLUDED', '_EMMINTRIN_H_INCLUDED'],
		header='/usr/include/opencv/cv.h', 
		insert_headers = ['/usr/include/opencv/cvtypes.h'],
		ctypes_footer = footer,
		strip_prefixes = ['cv'],
	)


	defines = []
	rpythonic.wrap(
		'highgui',
		defines = ['_MMINTRIN_H_INCLUDED', '_XMMINTRIN_H_INCLUDED', '_EMMINTRIN_H_INCLUDED'],
		header='/usr/include/opencv/highgui.h',
		strip_prefixes = ['cv'],
	)

if '--opencvaux' in sys.argv or ALL:
	rpythonic.wrap( 
		'cvaux', 
		header='/usr/include/opencv/cvaux.h', 
		strip_prefixes = ['cv'],
	)


if '--openal' in sys.argv or ALL:
	rpythonic.wrap(
		'openal', 
		header='/usr/include/AL/al.h', 
		insert_headers = ['/usr/include/AL/alc.h'],
		strip_prefixes='al alc AL_'.split(),
	)

if '--alut' in sys.argv or ALL:

	rpythonic.wrap( 'alut', header='/usr/include/AL/alut.h',  )


if '--freenect' in sys.argv or ALL:
	#git clone https://github.com/OpenKinect/libfreenect.git

	rpythonic.wrap( 
		'libfreenect', 
		header='/usr/include/libfreenect.h',
		strip_prefixes = ['freenect_', '_freenect_', 'FREENECT_'],
	)

if '--freenect-sync' in sys.argv or ALL:

	rpythonic.wrap(
		'libfreenect_sync', 
		header='/usr/include/libfreenect_sync.h',
		strip_prefixes = ['freenect_'],
	)



if '--opengl' in sys.argv or ALL:
	rpythonic.wrap( 'openGL', header='/usr/include/GL/gl.h', library='/usr/lib/libGL.so' )

if '--openglu' in sys.argv or ALL:
	rpythonic.wrap( 'openGLU', header='/usr/include/GL/glu.h', library='/usr/lib/libGLU.so' )

if '--openglut' in sys.argv or ALL:
	rpythonic.wrap( 'openGLUT', header='/usr/include/GL/glut.h', library='/usr/lib/libglut.so' )

if '--openjpeg' in sys.argv or ALL:
	rpythonic.wrap( 'openjpeg', header='/usr/include/openjpeg.h' )


if '--vnc' in sys.argv:
	mod = rpythonic.load( 'vncserver', debug=debug )
	if not mod:
		rpythonic.wrap( 'vncserver', header='/usr/include/rfb/rfb.h' )
	else:
		print( mod )

	mod = rpythonic.load( 'vncclient', debug=debug )
	if not mod:
		rpythonic.wrap( 'vncclient', header='/usr/include/rfb/rfbclient.h' )
	else:
		print( mod )




if '--wiiuse' in sys.argv or ALL:
	#git clone http://github.com/rpavlik/wiiuse.git
	footer = '''
def IS_PRESSED(dev, button): return ((dev.contents.buttons & button) == button)

def IS_HELD(dev, button): return ((dev.contents.buttons_held & button) == button)

def IS_RELEASED(dev, button): return  ((dev.contents.buttons_released & button) == button)

def IS_JUST_PRESSED(dev, button): return  (IS_PRESSED(dev, button) and not IS_HELD(dev, button))

def WIIUSE_USING_ACC(dev):  return ((dev.contents.state & 0x020) == 0x020)

def WIIUSE_USING_EXP(dev): return ((dev.contents.state & 0x040) == 0x040)

def WIIUSE_USING_IR(dev): return ((dev.contents.state & 0x080) == 0x080)

def WIIUSE_USING_SPEAKER(dev): return ((dev.contents.state & 0x100) == 0x100)

WIIUSE_SMOOTHING = 0x01
WIIUSE_CONTINUOUS = 0x02
WIIUSE_ORIENT_THRESH = 0x04
WIIUSE_INIT_FLAGS = (WIIUSE_SMOOTHING | WIIUSE_ORIENT_THRESH)

# from wiiuse_internal.h #
WIIMOTE_STATE_CONNECTED	= 0x0008
WIIMOTE_STATE_RUMBLE	= 0x0010
WIIMOTE_STATE_ACC = 0x0020
WIIMOTE_STATE_EXP = 0x0040
WIIMOTE_STATE_IR	= 0x0080
WIIMOTE_STATE_SPEAKER	 = 0x0100
WIIMOTE_STATE_IR_SENS_LVL1 = 0x0200
WIIMOTE_STATE_IR_SENS_LVL2 = 0x0400
WIIMOTE_STATE_IR_SENS_LVL3 = 0x0800
WIIMOTE_STATE_IR_SENS_LVL4 = 0x1000
WIIMOTE_STATE_IR_SENS_LVL5 = 0x2000
WIIMOTE_INIT_STATES = WIIMOTE_STATE_IR_SENS_LVL3

	'''
	rpythonic.wrap( 'wiiuse', 
		header='/usr/local/include/wiiuse.h',
		library = '/usr/local/lib/libwiiuse.so',
		ctypes_footer = footer,
		strip_prefixes = ['wiiuse_'],
	)


if '--fluid' in sys.argv or ALL:
	rpythonic.wrap( 'fluidsynth', 
		header='/usr/include/fluidsynth.h',
		library = '/usr/lib/libfluidsynth.so',
		strip_prefixes = ['fluid_'],
	)


if '--xlib' in sys.argv:		# this was just for testing
	rpythonic.wrap( 'xlib', 
		header='/usr/include/X11/Xlib.h',
	)

if '--fftw' in sys.argv or ALL:
	rpythonic.wrap( 'fftw', 
		header='/usr/include/fftw3.h',
		library = '/usr/lib/libfftw3.so',
		strip_prefixes = ['fftw_', 'FFTW_'],
	)

if '--avcodec' in sys.argv or ALL:
	rpythonic.wrap( 'avcodec', 
		header='/usr/include/libavcodec/avcodec.h',
		library = '/usr/lib/libavcodec.so',
		strip_prefixes = ['AV', 'FF_'],
	)

if '--avformat' in sys.argv or ALL:
	rpythonic.wrap( 'avformat', 
		header='/usr/include/libavformat/avformat.h',
		library = '/usr/lib/libavformat.so',
		strip_prefixes = ['AV', 'FF_'],
	)

if '--libmlt' in sys.argv or ALL:
	rpythonic.wrap( 'libmlt', 
		header='/usr/include/mlt/framework/mlt.h',
		library = '/usr/lib/libmlt.so',
		strip_prefixes = ['mlt_'],
	)

if '--verse' in sys.argv or ALL:
	# mkdir verse2
	# cd verse2
	# svn checkout  https://dev.nti.tul.cz/repos/verse2/verse2/trunk
	rpythonic.wrap( 'libverse', 
		header='../../verse2/trunk/include/verse.h',
		library = '/usr/lib/libverse.so',
		strip_prefixes = ['ve_'],
	)

if '--test' in sys.argv:
	rpythonic.wrap( 'testing', 
		header='./test.h',
	)

