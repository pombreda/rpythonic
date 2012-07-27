#!/usr/bin/python
# updated may9th 2011
# trick, the head will contain many sub rects, but the bounding points to not touch
# while the hand will have many rects with touching bounding points.

import os, sys, ctypes, thread
if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
rpythonic.set_cache('../cache')
freenect = rpythonic.module( 'libfreenect_sync' )
assert freenect

cvaux = rpythonic.module( 'cvaux' )
assert cvaux
gui = rpythonic.module( 'highgui', secondary=cvaux )
assert gui
cv = rpythonic.module( 'cv', secondary=gui )
assert cv
import time

IPL_DEPTH_16S = 2147483664
IPL_DEPTH_8S = 2147483656
CV_WHOLE_SEQ = cv.CvSlice(0,9999999)

lock = thread.allocate_lock()

class Shape(object):
	HAND = None
	storage_hull = cv.CreateMemStorage(0)
	storage_defects = cv.CreateMemStorage(0)

	def touches( self, other ):
		pt1, pt2 = self.rectangle
		if other.rectangle[0] == pt1 or other.rectangle[1] == pt2:
			if other not in self.touching: self.touching.append( other )
		return other in self.touching

	def contains( self, other ):
		#if self.touches( other ): return False
		s1, s2 = self.rectangle
		o1, o2 = other.rectangle
		if s1[0] <= o1[0] and s1[1] <= o1[1] and s2[0] >= o2[0] and s2[1] >= o2[1]:
			if other not in self.children:
				self.children.append( other )
				other.parents.append( self )
		return other in self.children

	def draw_bounds( self, image, mode='rectangle' ):
		if mode=='rectangle':
			cv.Rectangle(	image, self.rectangle[0], self.rectangle[1],
				(64,0,128), 1, 7, 0)
		else:
			cv.Circle( image, self.center, int(self.width), (128,0,128), 2, 7, 0 )


	def draw_defects( self, image ):
		for d in self.defects:
			start,end,depth = d
			cv.Line(image, start, end,
				(0,64,128),
				1,
				8, 
				0
			)
			cv.Line(image, end, depth,
				(0,64,128),
				2,
				8, 
				0
			)
		if len( self.defects ) >= 2 and self.center_defects:
			cv.Circle( image, self.center_defects, 14, (255,255,0), 5, 7, 0 )

	def __init__( self, poly, depth ):
		self.depth = depth
		self.points = []
		self.touching = []
		self.children = []
		self.parents = []
		xavg = []; yavg = []
		xmin = ymin = xmax = ymax = None
		for j in range( poly.contents.total ):
			_ptr = cv.GetSeqElem( poly, j )
			point = ctypes.cast(_ptr, ctypes.POINTER(cv.Point) )
			self.points.append( point.contents )
			x = point.contents.x; y = point.contents.y
			xavg.append( x ); yavg.append( y )
			if j == 0:
				xmin = xmax = x
				ymin = ymax = y
			else:
				if x < xmin: xmin = x
				if x > xmax: xmax = x
				if y < ymin: ymin = y
				if y > ymax: ymax = y
		n = len(self.points)
		self.weight = ( sum(xavg)/float(n), sum(yavg)/float(n) )
		self.width = xmax - xmin
		self.height = ymax - ymin
		self.center = ( int(xmin + (self.width/2)), int(ymin + (self.height/2)) )
		self.rectangle = ( (xmin,ymin), (xmax,ymax) )

		self.dwidth = xmax - xmin
		self.dheight = ymax - ymin
		self.dcenter = ( xmin + (self.dwidth/2), ymin + (self.dheight/2) )
		self.drectangle = ( (xmin,ymin), (xmax,ymax) )

		self.defects = []
		self.center_defects = None
		self.convex = cv.CheckContourConvexity( poly )
		if not self.convex:
			T = 80
			dxavg = []; dyavg = []
			hull = cv.ConvexHull2( poly, self.storage_hull, 1, 0 )
			defects = cv.ConvexityDefects( poly, hull, self.storage_defects )
			n = defects.contents.total
			for j in range( n ):
				d = ctypes.cast( cv.GetSeqElem( defects, j ), ctypes.POINTER(cv.ConvexityDefect) )
				start = d.contents.start
				end = d.contents.end
				depth = d.contents.depth_point
				## ignore large defects ##
				if abs(end.contents.x - depth.contents.x) > T or abs(end.contents.y - depth.contents.y) > T or abs(start.contents.x - end.contents.x) > T or abs(start.contents.y - end.contents.y) > T: continue
				dxavg.append( depth.contents.x )
				dyavg.append( depth.contents.y )
				self.defects.append( (start.contents, end.contents, depth.contents) )

			xmin = ymin = 999999
			xmax = ymax = -1
			if self.defects:
				n = len(self.defects)
				self.center_defects = ( int(sum(dxavg)/float(n)), int(sum(dyavg)/float(n)) )
				for j,f in enumerate( self.defects ):
					s,e,d = f
					if s.x < xmin: xmin = s.x
					if e.x < xmin: xmin = e.x
					if s.x > xmax: xmax = s.x
					if e.x > xmax: xmax = e.x
					if s.y < ymin: ymin = s.y
					if e.y < ymin: ymin = e.y
					if s.y > ymax: ymax = s.y
					if e.y > ymax: ymax = e.y

				self.dwidth = xmax - xmin
				self.dheight = ymax - ymin
				self.dcenter = ( xmin + (self.dwidth/2), ymin + (self.dheight/2) )
				self.drectangle = ( (xmin,ymin), (xmax,ymax) )

		if False:		# throws error, some value not in range? #
			dompoints = cv.FindDominantPoints( poly, 
				self.storage_dom, cv.CV_DOMINANT_IPAN,
				7.0,		# min distance
				9.0, 		# max distance
				7.0,		# neighborhood distance
				150.0,	# max angle
			)
			print dompoints

		cv.ClearMemStorage( self.storage_hull )
		cv.ClearMemStorage( self.storage_defects )

class Threaded(object):
	def start(self):
		self.active = True
		self.loops = 0
		thread.start_new_thread( self.loop, () )


class Kinect( Threaded ):
	def __init__(self, ocv):
		self.active = False
		self.ocv = ocv
		self._lenpix = 480 * 640 * 2
		self.buffer_type = ctypes.c_void_p	#(ctypes.c_char * lenpix)
		self.buffer = self.buffer_type() #_ref = ctypes.cast(buff, ctypes.c_void_p)
		self.pointer = ctypes.pointer( self.buffer )
		print( 'setting leds' )
		freenect.sync_set_led( freenect.LED_YELLOW, 0 )

	def loop(self):
		print( 'starting kinect thread' )
		while self.active:
			self.loops += 1
			status = freenect.sync_get_depth( ctypes.byref(self.pointer), 0, 0, 
				freenect.FREENECT_DEPTH_11BIT
			)
			#lock.acquire()		# seems safe even without locking?
			cv.SetData( self.ocv.depth16raw, self.pointer, 640*2 )
			#lock.release()
		freenect.sync_set_led( freenect.LED_OFF, 0 )
		freenect.sync_stop()
		print( 'exit kinect thread', self.loops )

mem = cv.CreateMemStorage(0)
class OpenCVStage2( Threaded ):
	def __init__(self, ocv):
		self.active = False
		self.ocvstage1 = ocv
		self.contours_image = cv.cvCreateImage((640,480), 8, 3)
		self.storage_poly = cv.CreateMemStorage(0)
		cv.NamedWindow('contours', 1)


	def loop(self):
		while self.active:
			self.loops += 1
			#lock.acquire()
			#buff = list( self.ocvstage1.sweep_buffer )
			#lock.release()
			cv.SetZero( self.contours_image )
			shapes = []
			#for contours in buff:
			for i in range(12):
				lock.acquire()
				contours = self.ocvstage1.sweep_buffer[i]
				lock.release()
				if not contours: continue

				cv.DrawContours(
					self.contours_image, 
					contours,
					(64,64,128), 	# external color
					(255,255,0),	# hole color
					1, # max levels
					2, # thickness
					8, # linetype
					(0, 0)
				)

				poly = cv.ApproxPoly( 	 # returns the first element
					contours, 
					ctypes.sizeof( cv.Contour ), 
					self.storage_poly,
					cv.CV_POLY_APPROX_DP,
					30.0, 1 )
				#print dir(poly.contents.first.contents)
				contours = ctypes.pointer( poly )
				cv.DrawContours(
					self.contours_image, 
					contours.contents,
					(255,0,128), 	# external color
					(255,255,0),	# hole color
					1, # max levels
					1, # thickness
					8, # linetype
					(0, 0)
				)

				segments = []	#self.breakdown_polygon( poly )
				for seg in segments:
					contours = ctypes.pointer( seg )
					cv.DrawContours(
						self.contours_image, 
						contours.contents,
						(255,0,128-thresh), 	# external color
						(255,255,0),	# hole color
						1, # max levels
						4, # thickness
						8, # linetype
						(0, 0)
					)

				shapes.append( Shape( poly, 0 ) )

			for s1 in shapes:
				for s2 in shapes:
					if s1 is not s2:
						s1.touches( s2 )
						s1.contains( s2 )		# parents, children

			#Shape.HAND = None
			hand = head = None
			for s in shapes:
				## these numbers are magic
				if len(s.children) >= 8 and s.width < 200 and s.height < 180 and s.width > 100 and s.height > 150:
					if not s.touching:
						head = s  # end of magic
						break

			best = []; maybe = []; poor = []
			for s in shapes:
				if not s.defects: continue
				if head and s in head.children: continue

				if len(s.touching) >= 2 and len(s.defects) >= 2:
					if s.dwidth < 320 and s.dheight < 340:
						best.append( s )
						if not hand or ( s.dwidth <= hand.dwidth and s.dheight <= hand.dheight ):
							hand = s
					else: maybe.append( s )
				elif len(s.defects) >= 2:
					poor.append( s )

			if hand: Shape.HAND = hand
			if not hand:
				if maybe:
					for s in maybe:
						if not hand or ( s.dwidth <= hand.dwidth and s.dheight <= hand.dheight ):
							hand = s

				if not hand and poor and Shape.HAND:
					hx,hy = Shape.HAND.center_defects
					for s in poor:
						if not hand or ( s.dwidth <= hand.dwidth and s.dheight <= hand.dheight ):
							sx,sy = s.center_defects
							if abs(hx-sx) < 20 and abs(hy-sy) < 20:
								hand = s

			if head: head.draw_bounds( self.contours_image, 'circle' )
			#for s in shapes: s.draw_defects( self.contours_image )
			if hand:
				hand.draw_defects( self.contours_image )


			cv.ClearMemStorage( self.storage_poly )
			cv.ShowImage( 'contours', self.contours_image )
			cv.ShowImage( 'depth', self.ocvstage1.depth8 )
			cv.WaitKey(1)
		print( 'thread3 exit', self.loops)

class OpenCV( Threaded ):
	def breakdown_polygon( self, poly ):
		segments = []
		prev = None
		start = 0
		n = poly.contents.total
		for idx in range( n ):
			_ptr = cv.GetSeqElem( poly, idx )
			point = ctypes.cast(_ptr, ctypes.POINTER(cv.Point) )
			x = point.contents.x; y = point.contents.y
			if prev:
				px,py = prev
				dx = abs(px-x); dy = abs(py-y)
				print 'delta', dx, dy
				if (abs(px-x) > 25 or abs(py-y) > 25):# or idx == n-1:
					print 'segs', start, idx
					if idx-start > 5:
						seg = cv.SeqSlice( poly, cv.Slice(start,idx-1), self.storage_poly, 0 )
						segments.append( seg )
					start = idx
			prev = (x,y)

		return segments



	def __init__(self):
		self.active = False
		self.depth16raw = cv.CreateImage((640,480), IPL_DEPTH_16S, 1)
		self.depth16 = cv.CreateImage((640,480), IPL_DEPTH_16S, 1)
		self.depth8 = cv.cvCreateImage((640,480), 8, 1)

		self.sweep_thresh = [ cv.cvCreateImage((640,480), 8, 1) for i in range(12) ]
		self.sweep_buffer = [None]*12
		self.storage = cv.CreateMemStorage(0)
		self.storage_poly = cv.CreateMemStorage(0)

		cv.NamedWindow('depth', 1)
		cv.SetZero( self.depth16raw )

	def loop(self):
		print( 'starting opencv thread' )
		self.active = True
		while self.active:
			self.loops += 1
			#lock.acquire()
			cv.ConvertScale( self.depth16raw, self.depth8, 0.18, 0 )
			#lock.release()
			# blur helps?
			cv.Smooth( self.depth8, self.depth8, cv.CV_BLUR, 12, 12, 0.1, 0.1 )
			cv.ClearMemStorage( self.storage )
			thresh = 100
			buff = []
			for i, img in enumerate(self.sweep_thresh):
				cv.Threshold( self.depth8, img, thresh, 256, cv.CV_THRESH_BINARY_INV )
				thresh += 2

				seq = cv.CvSeq()
				contours = ctypes.pointer( ctypes.pointer( seq ) )
				cv.FindContours(img, self.storage, contours, ctypes.sizeof( cv.Contour ), cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_APPROX_SIMPLE, (0,0) )
				#cv.FindContours(img, self.storage, contours, ctypes.sizeof( cv.Chain ), cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_CODE, (0,0) )
				_total = 0
				try: _total = contours.contents.contents.total
				except: continue

				poly = cv.ApproxPoly( 	 # returns the first element
					contours.contents, 
					ctypes.sizeof( cv.Contour ), 
					self.storage_poly,
					cv.CV_POLY_APPROX_DP,
					3.0, 1 )
				contours = ctypes.pointer( poly )
				buff.append( cv.SeqSlice( contours.contents, cv.Slice(0,_total), mem, 1 ) )

				lock.acquire()
				self.sweep_buffer[ i ] = buff[-1]
				lock.release()

			#lock.acquire()
			#self.sweep_buffer = list( buff )
			#lock.release()

		print( 'exit opencv thread', self.loops )

o = OpenCV()
k = Kinect( o )
k.start()		# thread 1
time.sleep(1)
o.start()		# thread 2
o2 = OpenCVStage2( o )
o2.start()	# thread 3

START = time.time()
LOOPS = 0
while time.time() - START < 30:	# main thread
	LOOPS += 1
	try:
		print( 'waiting...' )
		#time.sleep(1)
		a = 0.0		# make sure the main thread still has some CPU time to do work
		for i in range(1000 * 1000): a += i*2.5
	except:
		print( 'user exit' )
		break

o.active = False
o2.active = False
k.active = False
time.sleep(1)

print('threaded kinect test done')
print( LOOPS )



