#!/usr/bin/python
# updated may19th 2011 - almost fully working again

import os, sys, ctypes, thread, math
if '..' not in sys.path: sys.path.append( '..' )
import rpythonic

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

Klock = thread.allocate_lock()
lock = thread.allocate_lock()

SWEEPS = 32
SWEEP_STEP = 2

class Point(object):
	def __len__(self): return 2
	def __getitem__(self,key):
		if key == 0: return self.x
		elif key == 1: return self.y
		else: raise IndexError
	def __init__(self, x,y): self.x = x; self.y = y
	def scale( self, other ): return Point( self.x*other.x, self.y*other.y )
	def dot( self, other ): return self.x*other.x + self.y*other.y
	def length( self ): return math.sqrt( self.dot(self) )
	def angle( self, other ):
		dot = self.dot( other )
		length = self.length() * other.length()
		if not length: return .0
		else: return math.acos( dot / length )

a=Point(1,1)
b=Point(10,90)
print round(a.angle( b ),6)

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
				(64,0,128), 1, 8, 0)
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
			cv.Circle( image, self.center_defects, 24, (255,255,0), 15, 7, 0 )

	def draw_variance(self, image ):
		if len(self.points) < 3: return
		color = (128,80,80); width = 1
		if self.avariance > 30: color = (255,155,155)
		elif self.avariance > 10: color = (255,80, 80)
		else: color = (225,60, 60)

		for i in range(0,len(self.points),2):
			w = self.avariance_points[i]
			a = self.points[i-1]
			b = self.points[i]
			cv.Line( image, (a.x,a.y), (b.x,b.y), color, width+int(w*0.15), 8, 0 )


	def __init__( self, poly, depth ):
		self.depth = depth
		self.points = []
		self.angles = [.0]
		self.touching = []
		self.children = []
		self.parents = []
		xavg = []; yavg = []
		xmin = ymin = xmax = ymax = None
		for j in range( poly.contents.total ):
			_ptr = cv.GetSeqElem( poly, j )
			point = ctypes.cast(_ptr, ctypes.POINTER(cv.Point) )
			x = point.contents.x; y = point.contents.y
			p = Point(x,y)
			if self.points:
				a = math.degrees(self.points[-1].angle( p ))
				self.angles.append( a )
				
			self.points.append( p )
			xavg.append( x ); yavg.append( y )
			if j == 0:
				xmin = xmax = x
				ymin = ymax = y
			else:
				if x < xmin: xmin = x
				if x > xmax: xmax = x
				if y < ymin: ymin = y
				if y > ymax: ymax = y

		self.avariance = .0
		self.avariance_points = [.0,.0]
		if self.angles:
			print self.angles
			prev = self.angles[0]
			for a in self.angles[1:]:
				v = abs( prev - a )
				self.avariance_points.append( v )
				self.avariance += v
				prev = a
			#print 'variance', self.avariance
			#print 'variance-points', self.avariance_points
			#print 'len len', len(self.points), len(self.avariance_points)

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
				D = ctypes.cast( cv.GetSeqElem( defects, j ), ctypes.POINTER(cv.ConvexityDefect) )
				s = D.contents.start.contents
				e = D.contents.end.contents
				d = D.contents.depth_point.contents
				start	= ( s.x, s.y )
				end		= ( e.x, e.y )
				depth 	= ( d.x, d.y )

				## ignore large defects ##
				if abs(end[0] - depth[0]) > T or abs(end[1] - depth[1]) > T or abs(start[0] - end[0]) > T or abs(start[1] - end[1]) > T: continue

				dxavg.append( depth[0] )
				dyavg.append( depth[1] )
				self.defects.append( (start, end, depth) )

			xmin = ymin = 999999
			xmax = ymax = -1
			if self.defects:
				n = len(self.defects)
				self.center_defects = ( int(sum(dxavg)/float(n)), int(sum(dyavg)/float(n)) )
				for j,f in enumerate( self.defects ):
					s,e,d = f
					if s[0] < xmin: xmin = s[0]
					if e[0] < xmin: xmin = e[0]
					if s[0] > xmax: xmax = s[0]
					if e[0] > xmax: xmax = e[0]
					if s[1] < ymin: ymin = s[1]
					if e[1] < ymin: ymin = e[1]
					if s[1] > ymax: ymax = s[1]
					if e[1] > ymax: ymax = e[1]

				self.dwidth = xmax - xmin
				self.dheight = ymax - ymin
				self.dcenter = ( xmin + (self.dwidth/2), ymin + (self.dheight/2) )
				self.drectangle = ( (xmin,ymin), (xmax,ymax) )

		cv.ClearMemStorage( self.storage_hull )
		cv.ClearMemStorage( self.storage_defects )

class Threaded(object):
	def start(self):
		self.active = True
		self.loops = 0
		thread.start_new_thread( self.loop, () )


class Kinect( Threaded ):
	def __init__(self):
		self.active = False
		self._lenpix = 480 * 640 * 2
		self.buffer_type = ctypes.c_void_p
		self.buffer = self.buffer_type()
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
			Klock.acquire()		# seems safe even without locking?
			cv.SetData( OCVThread1.DEPTH16RAW, self.pointer, 640*2 )
			Klock.release()
		freenect.sync_set_led( freenect.LED_OFF, 0 )
		freenect.sync_stop()
		print( 'exit kinect thread', self.loops )

mem = cv.CreateMemStorage(0)
#cv.ClearMemStorage( self.mem2 )
print 'releasing mem'
cv.ReleaseMemStorage( ctypes.pointer(mem()) )
print 'mem releaseed'
class OCVThread2( Threaded ):
	def __init__(self):
		self.active = False
		self.contours_image = cv.cvCreateImage((640,480), 8, 3)
		#self.storage_poly = cv.CreateMemStorage(0)
		cv.NamedWindow('contours', 1)

	def update_frame( self, shapes ):
		img = self.contours_image

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
		if hand:
			hand.draw_defects( self.contours_image )


		lock.acquire()
		cv.ShowImage( 'contours', self.contours_image() )
		cv.WaitKey(1)	# waitkey is called in other ocv thread
		lock.release()

	def loop(self):
		frame = []
		trash = []
		while self.active:
			self.loops += 1
			batch = []
			#f not len(OCVThread1.BUFFER) > 1: time.sleep(0.01)
			lock.acquire()
			while OCVThread1.BUFFER:
				batch.append( OCVThread1.BUFFER.pop() )
			lock.release()
			if not batch:
				time.sleep(0.01)
				trash = []
			while batch:
				P = batch.pop()
				if P.index == 0:
					self.update_frame( frame )		# final draws and shows
					cv.SetZero( self.contours_image )
					frame = [ Shape( P.poly2, P.depth ) ]
				else:
					frame.append( Shape( P.poly2, P.depth ) )
				P.draw( self.contours_image )	# do this outside of update_frame
				s = frame[-1]
				s.draw_defects( self.contours_image )
				s.draw_bounds( self.contours_image )
				s.draw_variance( self.contours_image )

				#del P		# can randomly segfault?, let python GC del P
				trash.append( P )

		print( 'ocv thread2 exit', self.loops)


class OCVThread1( Threaded ):
	BUFFER = []
	DEPTH16RAW = cv.CreateImage((640,480), IPL_DEPTH_16S, 1)
	def __init__(self):
		self.active = False
		self.depth16raw = OCVThread1.DEPTH16RAW
		self.depth16 = cv.CreateImage((640,480), IPL_DEPTH_16S, 1)
		self.depth8 = cv.cvCreateImage((640,480), 8, 1)

		self.sweep_thresh = [ cv.cvCreateImage((640,480), 8, 1) for i in range(SWEEPS) ]
		self.storage = cv.CreateMemStorage(0)
		#self.storage_poly = cv.CreateMemStorage(0)

		cv.NamedWindow('depth', 1)
		cv.SetZero( self.depth16raw )

	def loop(self):
		print( 'starting opencv thread' )
		self.active = True
		while self.active:
			self.loops += 1
			Klock.acquire()
			cv.ConvertScale( self.depth16raw, self.depth8, 0.18, 0 )
			Klock.release()

			# blur helps?
			#cv.Smooth( self.depth8, self.depth8, cv.CV_BLUR, 16, 16, 0.1, 0.1 )
			#cv.Smooth( self.depth8, self.depth8, cv.CV_GAUSSIAN, 13, 13, 0.1, 0.1 )

			lock.acquire()		# this part of opencv is not threadsafe
			cv.ShowImage( 'depth', self.depth8() )	#TODO()
			cv.WaitKey(1)
			lock.release()

			thresh = 80
			index = 0
			for img in self.sweep_thresh:
				cv.ClearMemStorage( self.storage )

				cv.Threshold( self.depth8, img, thresh, 255, cv.CV_THRESH_BINARY_INV )
				#cv.Canny( img, img, 0, 255, 3 )	# too slow
				seq = cv.CvSeq()
				contours = ctypes.pointer( ctypes.pointer( seq() ) )
				cv.FindContours(img, self.storage, contours, ctypes.sizeof( cv.Contour()(type=True) ), cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_APPROX_SIMPLE, (0,0) )

				_total = 0
				try: _total = contours.contents.contents.total
				except:
					thresh += SWEEP_STEP
					continue

				P = ReducedPolygon( contours, index, thresh )
				lock.acquire()
				self.BUFFER.append( P )
				lock.release()
				index += 1
				thresh += SWEEP_STEP


		print( 'exit opencv thread1', self.loops )


class ReducedPolygon( object ):
	def copy_buffer( self, memory ):	# clear local mem on copy?
		return cv.SeqSlice( self.poly2, cv.Slice(0,self.total), memory, 1 )
	def __del__(self):
		#cv.ClearMemStorage( self.mem1 )
		#cv.ClearMemStorage( self.mem2 )
		cv.ReleaseMemStorage( ctypes.pointer(self.mem1) )
		cv.ReleaseMemStorage( ctypes.pointer(self.mem2) )

	def __init__(self, contours, index, depth ):
		self.index = index		# used from other thread, so it knows to clear drawing surface on zero.
		self.depth = depth		# kinect depth level
		self.mem1 = cv.CreateMemStorage(0)
		self.mem2 = cv.CreateMemStorage(0)

		self.poly1 = cv.ApproxPoly( 	 # pass1
			contours.contents, 
			ctypes.sizeof( cv.Contour ), 
			self.mem1,
			cv.CV_POLY_APPROX_DP,
			3.0, 1 )
		self.poly2 = cv.ApproxPoly( 	 # pass2
			self.poly1, 
			ctypes.sizeof( cv.Contour ), 
			self.mem2,
			cv.CV_POLY_APPROX_DP,
			20.0, 1 )

		self.total = self.poly2.contents.total

	def draw(self, image, lowres=False):
		cv.DrawContours(
			image, 
			self.poly1,
			(255,64,128), 	# external color
			(255,255,0),	# hole color
			1, # max levels
			1, # thickness
			cv.CV_AA, # linetype
			(0, 0)
		)
		if lowres:
			cv.DrawContours(
				image, 
				self.poly2,
				(128,64,64), 	# external color
				(255,255,0),	# hole color
				1, # max levels
				1, # thickness
				8, # linetype
				(0, 0)
			)


def run():
	k = Kinect()
	k.start()
	time.sleep(1)		# wait for kinect
	o = OCVThread1()
	o.start()
	o2 = OCVThread2()
	o2.start()
	while True:	# main thread
		try:
			#print( 'waiting...' )
			time.sleep(10)
		except:
			print( 'user exit' )
			break
	o.active = False
	o2.active = False
	k.active = False
	time.sleep(1)
	print('threaded kinect test done')

if __name__ == '__main__': run()




