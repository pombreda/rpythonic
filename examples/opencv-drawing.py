#!/usr/bin/python
import os,sys, time, ctypes
import opencv_core as cv
import opencv_highgui as gui

from random import Random
import colorsys


vec4 = ctypes.c_double * 4

def random_color(random):
    """
    Return a random color
    """
    icolor = random.randint(0, 0xFFFFFF)
    d = vec4( icolor & 0xff, (icolor >> 8) & 0xff, (icolor >> 16) & 0xff )
    return cv.Scalar(d)

if __name__ == '__main__':

    # some "constants"
    width = 1000
    height = 700
    window_name = "Drawing Demo"
    number = 100
    delay = 5
    line_type = 8	#cv.CV_AA  # change it to 8 to see non-antialiased graphics
    
    # create the source image
    print( 'creating image' )
    image = cv.CreateImage( (width, height), 8, 3)

    # create window and display the original picture in it
    gui.NamedWindow(window_name, 1); print( window_name )
    cv.SetZero(image)
    gui.ShowImage(window_name, image )

    # create the random number
    random = Random()

    # draw some lines
    for i in range(number):
        pt1 =  cv.Point(random.randrange(-width, 2 * width),
                          random.randrange(-height, 2 * height))
        pt2 =  cv.Point(random.randrange(-width, 2 * width),
                          random.randrange(-height, 2 * height))
        cv.Line(image, pt1, pt2,
                   random_color(random),
                   random.randrange(0, 10),
                   line_type, 0)
        
        gui.ShowImage(window_name, image)
        gui.WaitKey(delay)

    # draw some rectangles
    for i in range(number):
        pt1 =  cv.Point(random.randrange(-width, 2 * width),
                          random.randrange(-height, 2 * height))
        pt2 =  cv.Point(random.randrange(-width, 2 * width),
                          random.randrange(-height, 2 * height))
        cv.Rectangle(image, pt1, pt2,
                        random_color(random),
                        random.randrange(-1, 9),
                        line_type, 0)
        
        gui.ShowImage(window_name, image)
        gui.WaitKey(delay)

    # draw some ellipes
    for i in range(number):
        pt1 =  cv.Point(random.randrange(-width, 2 * width),
                          random.randrange(-height, 2 * height))
        sz =  cv.Size(random.randrange(0, 200),
                        random.randrange(0, 200))
        angle = random.randrange(0, 1000) * 0.180
        cv.Ellipse(image, pt1, sz, angle, angle - 100, angle + 200,
                        random_color(random),
                        random.randrange(-1, 9),
                        line_type, 0)
        
        gui.ShowImage(window_name, image)
        gui.WaitKey(delay)

    # init the list of polylines
    nb_polylines = 2
    polylines_size = 3
    pt = [0,] * nb_polylines
    for a in range(nb_polylines):
        pt [a] = [0,] * polylines_size

    # draw some polylines
    for i in range(number):
        for a in range(nb_polylines):
            for b in range(polylines_size):
                pt [a][b] =  (random.randrange(-width, 2 * width),
                                     random.randrange(-height, 2 * height))
        cv.PolyLine(image, pt, 1,
                       random_color(random),
                       random.randrange(1, 9),
                       line_type, 0)

        gui.ShowImage(window_name, image)
        gui.WaitKey(delay)

    # draw some filled polylines
    for i in range(number):
        for a in range(nb_polylines):
            for b in range(polylines_size):
                pt [a][b] =  (random.randrange(-width, 2 * width),
                                     random.randrange(-height, 2 * height))
        cv.FillPoly(image, pt,
                       random_color(random),
                       line_type, 0)

        cv.ShowImage(window_name, image)
        cv.WaitKey(delay)

    # draw some circles
    for i in range(number):
        pt1 =  (random.randrange(-width, 2 * width),
                          random.randrange(-height, 2 * height))
        cv.Circle(image, pt1, random.randrange(0, 300),
                     random_color(random),
                     random.randrange(-1, 9),
                     line_type, 0)
        
        cv.ShowImage(window_name, image)
        cv.WaitKey(delay)

    # draw some text
    for i in range(number):
        pt1 =  (random.randrange(-width, 2 * width),
                          random.randrange(-height, 2 * height))
        font = cv.InitFont(random.randrange(0, 8),
                              random.randrange(0, 100) * 0.05 + 0.01,
                              random.randrange(0, 100) * 0.05 + 0.01,
                              random.randrange(0, 5) * 0.1,
                              random.randrange(0, 10),
                              line_type)

        cv.PutText(image, "Testing text rendering!",
                      pt1, font,
                      random_color(random))
        
        cv.ShowImage(window_name, image)
        cv.WaitKey(delay)

    # prepare a text, and get it's properties
    font = cv.InitFont(cv.CV_FONT_HERSHEY_COMPLEX,
                          3, 3, 0.0, 5, line_type)
    text_size, ymin = cv.GetTextSize("OpenCV forever!", font)
    pt1 = ((width - text_size[0]) / 2, (height + text_size[1]) / 2)
    image2 = cv.CloneImage(image)

    # now, draw some OpenCV pub ;-)
    for i in range(0, 512, 2):
        cv.SubS(image2, cv.ScalarAll(i), image)
        (r, g, b) = colorsys.hsv_to_rgb((i % 100) / 100., 1, 1)
        cv.PutText(image, "OpenCV forever!",
                      pt1, font, cv.RGB(255 * r, 255 * g, 255 * b))
        cv.ShowImage(window_name, image)
        cv.WaitKey(delay)

    # wait some key to end
    cv.WaitKey(0)
