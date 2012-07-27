#!/usr/bin/python
import os,sys, time

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
rpythonic.set_android_sdk_root( '../../android-sdk-linux_x86/' )
rpythonic.set_android_ndk_root( '../../android-ndk-r5/' )
rpy = rpythonic.RPython('android')

gl = rpy.rimport( 'GLES', namespace=globals() )
e = rpy.rimport( 'EGL' )
glue = rpy.rimport('android_native_app_glue')

## API ##
get_app_state = rffi.llexternal('rpythonic_get_app_state', [], lltype.Ptr(glue.android_app) )
setup_glue = rffi.llexternal('rpythonic_setup_glue', [ lltype.Ptr(glue.android_app) ], rffi.VOIDP )
swap_buffers = rffi.llexternal('rpythonic_swap_buffers', [ rffi.VOIDP ], rffi.INT )
mainloop = rffi.llexternal('rpythonic_mainloop', [ rffi.VOIDP ], lltype.Void )

#/*
# * Send a simple string to the log.
# */
#int __android_log_write(int prio, const char *tag, const char *text);
log = rffi.llexternal('__android_log_write', [ rffi.INT, rffi.CCHARP, rffi.CCHARP ], rffi.INT )
def rprint( s, tag='rpythonic' ): log( 5, tag, s )	# 3 is debug level, 4 is info level, 5 is warn level


def reset_viewport( width, height ):
	ratio = float( width ) / float( height )
	glViewport(0, 0, width, height)

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	#/* Set our perspective */
	#gluPerspective(45.0, ratio, 0.1, 100.0);

	#/* Make sure we're chaning the model view and not the projection */
	glMatrixMode(GL_MODELVIEW);

	#/* Reset The View */
	glLoadIdentity();


def redraw_callback():
	vec3_t = rffi.CFixedArray( rffi.DOUBLE, 3 )
	vec4_t = rffi.CFixedArray( rffi.DOUBLE, 4 )
	colors_t = rffi.CFixedArray( vec4_t, 4 )
	verts_t = rffi.CFixedArray( vec3_t, 4 )

	colors = lltype.malloc( colors_t, flavor='raw' )
	verts = lltype.malloc( verts_t, flavor='raw' )
	for i in range(4):
		colors[i][0] = 1.0
		if i==0: verts[i][1] = 1.0
		elif i==1: verts[i][0] = -1.0; verts[i][1] = -1.0
		elif i==2: verts[i][0] = 1.0; verts[i][1] = -1.0


	width = 320; height = 240

	#/* Enable smooth shading */
	glShadeModel(GL_SMOOTH);

	#/* Set the background black */
	glClearColor(0.0, 0.0, 0.0, 0.0);

	#/* Depth buffer setup */
	glClearDepthf(1.0);

	#/* Enables Depth Testing */
	glEnable(GL_DEPTH_TEST);

	#/* The Type Of Depth Test To Do */
	glDepthFunc(GL_LEQUAL);
	#/* Really Nice Perspective Calculations */
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	reset_viewport( width, height )

	for i in range( 1000 ):
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )

		#/* Enable COLOR and VERTEX arrays */
		glEnableClientState(GL_COLOR_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);

		#/* Setup pointers to COLOR and VERTEX arrays */
		glColorPointer(4, GL_FLOAT, 0, colors);
		glVertexPointer(3, GL_FLOAT, 0, verts);
		#glColorPointer(4, GL_DOUBLE, 0, colors);	# no GL_DOUBLE on Android?
		#glVertexPointer(3, GL_DOUBLE, 0, verts);

		#/* Move Left 1.5 Units And Into The Screen 6.0 */
		glLoadIdentity();
		glTranslatef(-1.5, 0.0, -6.0);
		glDrawArrays(GL_TRIANGLES, 0, 3);

		swap_buffers( engine )
		time.sleep(0.01)


@rpy.standalone
def myentrypoint( _app_ ):
	#app = rffi.cast( lltype.Ptr(glue.android_app), _app_ )
	for i in range(10):
		rprint( 'please work' )

	app = get_app_state()
	engine = setup_glue( app )

	#display, context, surface = setup_egl( app )
	#rpy.engine.setup_display()

	mainloop( engine )



pak = rpy.compile()
print( pak.apk )
pak.upload()
print( 'logcat?' )
if raw_input().strip(): pak.logcat()

