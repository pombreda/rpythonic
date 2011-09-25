#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic

debug=0




if '--python3' in sys.argv:
	footer = 'PyRun_SimpleString = PyRun_SimpleStringFlags'
	rpythonic.wrap( 'python3', 
		header='/usr/include/python3.2/Python.h',
		library = '/usr/lib/libpython3.2mu.so',
		ctypes_footer = footer,
	)


broot = '../../blender'
if os.path.isdir( broot ) and '--blender' in sys.argv:
	includes = []
	for d in 'extern/bullet2/src intern/ghost source/blender/ikplugin source/blender/blenloader source/blender/gpu source/blender/windowmanager source/blender/editors/include source/blender/render/extern/include source/blender/imbuf source/blender/makesrna source/blender/makesdna source/blender/blenkernel source/blender/blenlib intern/guardedalloc'.split():
		includes.append( os.path.join( broot, d ) )

	#rpythonic.wrap( 'bpy', 
	#	header='./custom-sources/RNA_blender.h',
	#	includes=includes,
	#	library = 'bpy.so',
	#)

	rpythonic.wrap( 'blender', 
		header='./libblender/blender.h',	# needs RNA_blender.h
		includes=includes,
		insert_headers = [ os.path.join( broot, 'source/creator/creator.c' ) ],
		library = 'bpy.so',
	)
	assert 0

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


if '--ogrepaged' in sys.argv:
	root = '/usr/local'
	#sudo apt-get install libois-dev
	includes = [ 
		#root+'/include', 
		'/usr/local/include/OGRE',	# assume Ogre was built from source #
		'%s/include/PagedGeometry' %root,
	]
	insert_headers = []
	for h in 'BatchPage.h ImpostorPage.h TreeLoader3D.h'.split():
		insert_headers.append( '%s/include/PagedGeometry/%s' %(root,h) )

	rpythonic.wrap( 'OgrePaged', 
		header='%s/include/PagedGeometry/PagedGeometry.h' %root,
		includes=includes, insert_headers=insert_headers,
		library = '%s/lib/libPagedGeometry.a' %root,
		dynamic_libs = ['OgreMain', 'OgrePaging', 'OgreTerrain', 'OgreProperty'],
		static_libs = ['PagedGeometry'],
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


if '--sdl' in sys.argv:
	defines = []
	rpythonic.wrap( 
		'SDL', 
		header='/usr/include/SDL/SDL.h', 
		#insert_headers = ['/usr/include/SDL/SDL_opengl.h'],
		defines=defines, rffi=True 
	)


if '--ode' in sys.argv:
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
	rpythonic.wrap( 'ode', header='/usr/include/ode/ode.h', 
		defines=defines, rffi=True, ctypes_footer=footer 
	)




if '--opencv' in sys.argv:
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
		header='/usr/include/opencv/cv.h', 
		insert_headers = ['/usr/include/opencv/cvtypes.h'],
		ctypes_footer = footer
	)

	rpythonic.wrap( 
		'cvaux', 
		header='/usr/include/opencv/cvaux.h', 
	)

	defines = []
	rpythonic.wrap( 'highgui', header='/usr/include/opencv/highgui.h', defines=defines )

if '--openaudio' in sys.argv:
	defines = []
	rpythonic.wrap( 'openal', header='/usr/include/AL/al.h', defines=defines )

	defines = []
	rpythonic.wrap( 'alut', header='/usr/include/AL/alut.h', defines=defines )


if '--freenect' in sys.argv:
	defines = []
	rpythonic.wrap( 'libfreenect', header='/usr/local/include/libfreenect/libfreenect.h', defines=defines)

	defines = []
	rpythonic.wrap( 'libfreenect_sync', header='/usr/local/include/libfreenect/libfreenect_sync.h', defines=defines)

GINCLUDE = [
	'/usr/include/gtk-2.0/',
	'/usr/include/glib-2.0/',
	'/usr/lib/glib-2.0/include/',
	'/usr/include/cairo/',
	'/usr/include/pango-1.0/',
	'/usr/include/atk-1.0/',
	'/usr/include/gdk-pixbuf-2.0/',
	'/usr/lib/i386-linux-gnu/glib-2.0/include/',		# glibconfig.h
	'/usr/lib/gtk-2.0/include/',				# gdkconfig.h
]

if '--wnck' in sys.argv:
	#sudo apt-get install libwnck-dev
	rpythonic.wrap( 'wnck', 
		defines = ['WNCK_I_KNOW_THIS_IS_UNSTABLE'],
		includes=[ '/usr/include/libwnck-1.0'] + GINCLUDE,
		header='/usr/include/libwnck-1.0/libwnck/libwnck.h',
		library = '/usr/lib/libwnck-1.so',
	)

if '--gtk' in sys.argv:
	includes = [
		'/usr/include/gtk-2.0/',
		'/usr/include/glib-2.0/',
		'/usr/lib/glib-2.0/include/',
		'/usr/include/pango-1.0/',
		'/usr/include/atk-1.0/',
		'/usr/lib/i386-linux-gnu/glib-2.0/include/',		# glibconfig.h
		'/usr/lib/gtk-2.0/include/',				# gdkconfig.h
	]
	footer = '''

gtk_window_new.defaults[0] = GTK_WINDOW_TOPLEVEL

def connect( ptr, name, func, data=None ):
	gtk_signal_connect_while_alive( ptr, name, func, data, ptr )


RPYTHONIC_AUTOPREFIX_IGNORE.append( 'gdk_' )
RPYTHONIC_AUTOPREFIX_IGNORE.append( 'atk_' )
RPYTHONIC_AUTOPREFIX_IGNORE.append( 'Gdk' )
RPYTHONIC_AUTOPREFIX_IGNORE.append( 'Atk' )

for o in (GtkVBox, GtkHBox): o._rpythonic_parent_classes_.append( GtkBox )
for o in (GtkCheckButton,): o._rpythonic_parent_classes_.append( GtkToggleButton )
for o in (GtkHScale, GtkVScale): o._rpythonic_parent_classes_.append( GtkScale )


GTK_WIDGET_CLASSES = {
	GtkButton : gtk_button_new_with_label,
	GtkAdjustment : gtk_adjustment_new,
	GtkHScale : gtk_hscale_new,
	GtkVScale : gtk_vscale_new,
	GtkEntry : gtk_entry_new,
	GtkLabel : gtk_label_new,

	GtkToggleButton : gtk_toggle_button_new_with_label,
	GtkCheckButton : gtk_check_button_new_with_label,

}
GTK_CONTAINER_CLASSES = {
	GtkEventBox : gtk_event_box_new,
	GtkExpander : gtk_expander_new,
	GtkFixed : gtk_fixed_new,
	GtkFrame : gtk_frame_new,
	GtkWindow : gtk_window_new,
	GtkVBox : gtk_vbox_new,
	GtkHBox : gtk_hbox_new,
	GtkNotebook : gtk_notebook_new,
}
for d in (GTK_WIDGET_CLASSES, GTK_CONTAINER_CLASSES):
	for o in d:
		o._rpythonic_parent_classes_.append( GtkWidget )
		o._rpythonic_parent_classes_.append( GtkContainer )
		d[ o ].return_wrapper = o
		s = "lambda *args, **kw: %s(*args, **kw)"%d[o].name
		globals()[ o.__name__[3:] ] = eval(s)
		print( o.__name__, s )
	'''

	rpythonic.wrap( 
		'gtk', 
		header='/usr/include/gtk-2.0/gtk/gtk.h', 
		includes=includes, ctypes_footer=footer,
		library= '/usr/lib/libgtk-x11-2.0.so',
	)


if '--gtk3' in sys.argv:
	mod = rpythonic.load( 'gtk3', debug=debug )
	if not mod:
		includes = [
			'/usr/include/gtk-3.0/',
			'/usr/include/glib-2.0/',
			'/usr/lib/glib-2.0/include/',
			'/usr/lib/i386-linux-gnu/glib-2.0/include/',		# glibconfig.h
			'/usr/include/pango-1.0/',					# pango.h
			'/usr/include/cairo/',
			'/usr/include/gdk-pixbuf-2.0/',
			'/usr/include/atk-1.0/',
		]
		rpythonic.wrap( 'gtk3', 
			header='/usr/include/gtk-3.0/gtk/gtk.h', 
			library='/usr/lib/libgtk-3.so',
			includes=includes,
		)
	else:
		print( mod )


if '--opengl' in sys.argv:
	rpythonic.wrap( 'openGL', header='/usr/include/GL/gl.h', library='/usr/lib/libGL.so' )

if '--openglu' in sys.argv:
	rpythonic.wrap( 'openGLU', header='/usr/include/GL/glu.h', library='/usr/lib/libGLU.so' )

if '--openglut' in sys.argv:
	rpythonic.wrap( 'openGLUT', header='/usr/include/GL/glut.h', library='/usr/lib/libglut.so' )

if '--openjpeg' in sys.argv:
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




if '--wiiuse' in sys.argv:
	mod = rpythonic.load( 'wiiuse', debug=debug )
	if not mod:
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
			header='./custom-sources/rpavlik-wiiuse-0.14.0/src/wiiuse.h',
			library = 'libwiiuse.so',
			ctypes_footer = footer
		)
	else:
		print( mod )


# missing#webkit/webkitversion.h
#mod = rpythonic.load( 'webkit', debug=debug )
#if not mod:
#	rpythonic.wrap( 'webkit', 
#		header='/usr/include/webkit-1.0/webkit/webkit.h',
#		library = '/usr/lib/libwebkitgtk-1.0.so',
#	)
#else:
#	print( mod )

#apt-get install libxslt-dev libxml2-dev
if 0:
	mod = rpythonic.load( 'xml2', debug=debug )
	if not mod:
		rpythonic.wrap( 'xml2', 
			header='/usr/include/libxml2/libxml/tree.h',
			library = '/usr/lib/libxml2.so',
		)
	else:
		print( mod )









