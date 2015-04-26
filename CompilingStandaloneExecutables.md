#compiling standalone executables for Linux and Android.

# Introduction #

It is a quick process to compile a standalone for Linux, just download the latest PyPy source code, set the path in your script, and run the example below.

Compiling standalone's for Android is more complex and still experimental; you will need to download the Android SDK and NDK, get the PyPy source code and modify it.


# Overview #
  * import rpythonic
  * create a RPython instance, and set the target type
  * decorate the entry-point as standalone
  * call rpy.compile()


## Example - OpenGL SDL ##
```
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
rpy = rpythonic.RPython( 'linux' )
gl = rpy.rimport( 'openGL', namespace=globals() )
glu = rpy.rimport( 'openGLU', namespace=globals() )
sdl = rpy.rimport( 'SDL' )

def reset_viewport( width, height ):
	ratio = float( width ) / float( height )
	glViewport(0, 0, width, height)

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	#/* Set our perspective */
	gluPerspective(45.0, ratio, 0.1, 100.0);

	#/* Make sure we're chaning the model view and not the projection */
	glMatrixMode(GL_MODELVIEW);

	#/* Reset The View */
	glLoadIdentity();

@rpy.standalone
def myentrypoint():
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
	sdl.SDL_Init( sdl.SDL_INIT_VIDEO )
	sdl.SDL_SetVideoMode( width, height, 32, sdl.SDL_GL_ACCELERATED_VISUAL )#SDL_GL_DOUBLEBUFFER )
	sdl.SDL_WM_SetCaption( 'hello world', None )

	#/* Enable smooth shading */
	glShadeModel(GL_SMOOTH);

	#/* Set the background black */
	glClearColor(0.0, 0.0, 0.0, 0.0);

	#/* Depth buffer setup */
	glClearDepth(1.0);

	#/* Enables Depth Testing */
	glEnable(GL_DEPTH_TEST);

	#/* The Type Of Depth Test To Do */
	glDepthFunc(GL_LEQUAL);
	#/* Really Nice Perspective Calculations */
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	reset_viewport( width, height )

	for i in range( 1000 ):
		sdl.SDL_PumpEvents()
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )

		#/* Enable COLOR and VERTEX arrays */
		glEnableClientState(GL_COLOR_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);

		#/* Setup pointers to COLOR and VERTEX arrays */
		#glColorPointer(4, GL_FLOAT, 0, colors);
		#glVertexPointer(3, GL_FLOAT, 0, verts);
		glColorPointer(4, GL_DOUBLE, 0, colors);
		glVertexPointer(3, GL_DOUBLE, 0, verts);

		#/* Move Left 1.5 Units And Into The Screen 6.0 */
		glLoadIdentity();
		glTranslatef(-1.5, 0.0, -6.0);
		glDrawArrays(GL_TRIANGLES, 0, 3);


		sdl.SDL_GL_SwapBuffers()
		time.sleep(0.01)

	sdl.SDL_Quit()


package = rpy.compile()
package.test()
package.save( './openGLES-test1' )
```