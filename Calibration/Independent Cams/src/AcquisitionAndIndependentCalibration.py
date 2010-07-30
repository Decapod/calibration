#--------------------------------------------------------------------------------------------------------------------------------------------------
# Acquisition Code Integrated with the Calibration Code for Independent/Stereo Camera Calibration			-- Supported	
# Also Integrated the calibValid/stereoValid code to obtain the RMS Error in pixels					-- Supported			   
# Imran Zafar 														-- Dated 18th June, 2010								
# Test with python AcquisitionAndIndependentCalibration.py				[data paths, size, and samples number has been hard coded] 
#--------------------------------------------------------------------------------------------------------------------------------------------------
from opencv import *
from opencv.cv import *
from opencv.highgui import *
import os
import sys
import math
#--------------------------------------------------------------------------------------------------------------------------------------------------
TotalSamples 	= 8											# 	Total number of sample images
prefix_left 	= "/home/zafar/Downloads/elmelegy-stereo/dcCalibrate/calibTools/data2/Left/IMG_00"	# 	Image Prefix for left camera
prefix_right 	= "/home/zafar/Downloads/elmelegy-stereo/dcCalibrate/calibTools/data2/Right/IMG_00"	#	Image Prefix for right camera
ext		= "JPG" 
#--------------------------------------------------------------------------------------------------------------------------------------------------
# getch() Function: Pressing any key to capture but ESC key which is used for exiting purposes
#--------------------------------------------------------------------------------------------------------------------------------------------------
def getch():
    import tty, termios
    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ord(ch)
#--------------------------------------------------------------------------------------------------------------------------------------------------
# parse_cameras_port_txt() Function: Returns the camera ports for the cameras, to capture with gphoto2 command it is necessary to give the
#				     the port number of the camera when acquiring and saving files
#--------------------------------------------------------------------------------------------------------------------------------------------------
def parse_cameras_port_txt(imf):
	f = open(imf)
	cameras_port = []
	for line in f:
		if not line.startswith('Canon'):
			continue
		values = line.split()
		if len(values[-1])==4:
			continue
		cameras_port.append(values[-1])
	f.close()
	return cameras_port
#--------------------------------------------------------------------------------------------------------------------------------------------------
# capture_images_from_cameras() Function: Acquires the images from the cameras depending on the number of samples requested.
#					  Same function can be used for independent cameras and stereo capture.					  
#--------------------------------------------------------------------------------------------------------------------------------------------------
def capture_images_from_cameras(nFrames):
	imgNum = 0
	os.system("gphoto2 --auto-detect > cameras_port.txt")
	cameras_port = parse_cameras_port_txt("cameras_port.txt")

	while imgNum<nFrames:
	    print "Press any key to capture the image, <Escape> to end.\n"
	    c = getch()
	    if c==27:                           							#	Escape Key
	      break

	    for cmr_nr in range(0,len(cameras_port)):
		if cmr_nr == 0:										# 	Left Camera
		    filename=prefix_left + "%(s)02i"%{"s":imgNum}+"."+ ext
		    os.system("gphoto2 --port "+cameras_port[cmr_nr]+" --capture-image >/dev/null")
		    print "Capturing image to file: '"+filename+"' ... ",
		    os.system("gphoto2 --port "+cameras_port[cmr_nr]+" -P --new --force-overwrite --filename="+filename)
		    print "Done.\n"
     
		if cmr_nr == 1:										# 	Right Camera
		    filename=prefix_right + "%(s)02i"%{"s":imgNum}+"."+ ext
		    os.system("gphoto2 --port "+cameras_port[cmr_nr]+" --capture-image >/dev/null")
		    print "Capturing image to file: '"+filename+"' ... ",
		    os.system("gphoto2 --port "+cameras_port[cmr_nr]+" -P --new --force-overwrite --filename="+filename)
		    print "Done.\n"
    
	    imgNum += 1
	return True
#--------------------------------------------------------------------------------------------------------------------------------------------------
# square() Function: Utility function, calaculates the square of a number and returns it
#--------------------------------------------------------------------------------------------------------------------------------------------------
def square(x): 
  return x*x 
#--------------------------------------------------------------------------------------------------------------------------------------------------
# validate() Function: Validates the independent camera calibration and returns the calibration accuracy in pixels
#--------------------------------------------------------------------------------------------------------------------------------------------------
def validate(intrinsic_matrix, distortion_coeffs, imagePoints, objectPoints, pointCounts, numFrames, board_n, nbFrames):
	
	if numFrames<1:
		print "Need at least one frame to validate calibration! ... I will do nothing."
		return -1

	if intrinsic_matrix==None or distortion_coeffs==None:
		print "I need the intriniscs and distortion mtrices ... I will do nothing."
		return -1
	
	frame_image_points  = cvCreateMat(board_n,2,CV_32FC1)
	frame_object_points = cvCreateMat(board_n,3,CV_32FC1)
	frame_rotation = cvCreateMat(1,3,CV_32FC1)
	cvZero(frame_rotation)
	frame_translation = cvCreateMat(1,3,CV_32FC1)
	cvZero(frame_translation)  
	expected_imagepoints  = cvCreateMat(board_n,2,CV_32FC1)
	totalRmsErr=0.0
	for frame in range (numFrames):

		step = frame*board_n
		i,j=step,0
        	while j<board_n:
			frame_image_points [j][0] = imagePoints[i][0]
			frame_image_points [j][1] = imagePoints[i][1]
			frame_object_points[j][0] = objectPoints[i][0] 
			frame_object_points[j][1] = objectPoints[i][1] 
			frame_object_points[j][2] = objectPoints[i][2]
			i+=1
			j+=1
             
                cvFindExtrinsicCameraParams2(frame_object_points, frame_image_points,  intrinsic_matrix, distortion_coeffs, frame_rotation, frame_translation)
        	cvProjectPoints2(frame_object_points, frame_rotation, frame_translation, intrinsic_matrix, distortion_coeffs, expected_imagepoints)

        	rmsErr=0.0
	        for i in range (board_n):
          		rmsErr += square(frame_image_points[i][0]-expected_imagepoints[i][0]) +  square(frame_image_points[i][1]-expected_imagepoints[i][1]) 
          		# print "[ ", image_points[i], " ] --> [ ", expected_imagepoints[i], " ]"
		
		totalRmsErr += rmsErr
		rmsErr = math.sqrt(rmsErr/board_n) 
        	# print  "Image no. %(#)i has rms err = %(s1)f pixels\n"%{"s1":rmsErr,"#":frame}

	totalRmsErr = math.sqrt (totalRmsErr/board_n/nbFrames)   
	# print "Total RMS error = %(s1)f pixels.\n"%{"s1": totalRmsErr}
	return totalRmsErr
#--------------------------------------------------------------------------------------------------------------------------------------------------
# 					Calibrating on the basis of captured sample images
#--------------------------------------------------------------------------------------------------------------------------------------------------
capture_images_from_cameras(TotalSamples)									# 	Acquiring images from the cameras
fnameLeadingZero = 1   	              										# 	Does the image file name has leading zeros?

# Need to hard code this
board_w  	= 6												#	int(sys.argv[1])
board_h  	= 8												#	int(sys.argv[2])
n_boards 	= TotalSamples											#	int(sys.argv[3])
leftpath 	= "/home/zafar/Downloads/elmelegy-stereo/dcCalibrate/calibTools/data2/Left"			#	sys.argv[4]
rightpath 	= "/home/zafar/Downloads/elmelegy-stereo/dcCalibrate/calibTools/data2/Right"			#	sys.argv[5]
prefix   	= "IMG_00"											#	sys.argv[6]                   
ext 		= "JPG"												#	sys.argv[7]                   

board_n 	= board_w * board_h
board_sz 	= cvSize( board_w, board_h )
totalRmsErr	= 0.0												#	RMS Error in Pixels

# Defining the Image Display Window
cvNamedWindow("Calibration", 0) 										#  	CV_WINDOW_AUTOSIZE

# Allocating Storage
image_points  	= cvCreateMat(n_boards*board_n,2,CV_32FC1)
object_points 	= cvCreateMat(n_boards*board_n,3,CV_32FC1)
point_counts  	= cvCreateMat(n_boards,1,CV_32SC1)

cam_nr   = 0													# 	Left Camera
datapath = leftpath

while cam_nr<2:
  
  corner_count  = 0
  successes 	= 0
  imgNum        = 0

# Start reading the files
  if fnameLeadingZero:
      filename=datapath+"/"+prefix+"%(s)02i"%{"s":imgNum}+"."+ext
  else:
      filename=datapath+"/"+prefix+"%(s)i"%{"s":imgNum}+"."+ext   
  print "Reading image from file: ", filename, "   ... ",
  image = cvLoadImage( filename )
  assert(image);
  print "Done."
  img_sz =cvGetSize(image)
  print "Image of Size: ", img_sz.width, " x ", img_sz.height
  gray_image = cvCreateImage(img_sz,8,1)    									#	Subpixel

  while imgNum < n_boards:    
    if fnameLeadingZero:
      filename=datapath+"/"+prefix+"%(s)02i"%{"s":imgNum}+"."+ext
    else:
      filename=datapath+"/"+prefix+"%(s)i"%{"s":imgNum}+"."+ext   
    print "Reading image from file: ", filename, "   ... ",
    imgNum += 1
    image = cvLoadImage(filename)
    if image == None:
      print "Oops.\n"
      continue
    
    print "Done."
# Done Reading

    # Find chessboard corners:
    found, corners = cvFindChessboardCorners(image, board_sz, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS)   # &corner_count CV_CALIB_CB_NORMALIZE_IMAGE
    corner_count   = len(corners)
    
    # Get Subpixel accuracy on those corners
    cvCvtColor(image, gray_image, CV_BGR2GRAY)
    cvFindCornerSubPix(gray_image, corners, cvSize(11,11),cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ))
    print "Found %(s1)i corners out of %(#)i."%{"s1":corner_count,"#":board_n}
    
    # Draw it
    cvDrawChessboardCorners(image, board_sz, corners, found)
    cvShowImage( "Calibration", image )
    
    # If we got a good board, add it to our data
    if found and (corner_count == board_n):
      step = successes*board_n
      i,j=step,0
      while j<board_n: 
	image_points[i][0]   = corners[j].x 	# CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x
	image_points [i][1]  = corners[j].y 	# CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y
	object_points [i][0] = j/board_w  	# CV_MAT_ELEM(*object_points,float,i,0) = j/board_w
	object_points [i][1] = j%board_w 	# CV_MAT_ELEM(*object_points,float,i,1) = j%board_w
	object_points [i][2] = 0.0  		# CV_MAT_ELEM(*object_points,float,i,2) = 0.0
	i+=1
	j+=1
	    
      point_counts [successes] = board_n  	# CV_MAT_ELEM(*point_counts, int,successes,0) = board_n
      print "This view is to be used for calibration.\n"
      successes += 1

    # Handle pause/unpause and ESC
    c = cvWaitKey(100)
    # End collection loop

  print "\nFound %(s)i images, with %(#)i 2D points each.\n"%{"s":successes, "#":corner_count}

  # Allocate required matrices
  object_points2 = cvCreateMat(successes*board_n,3,CV_32FC1)
  image_points2  = cvCreateMat(successes*board_n,2,CV_32FC1)
  point_counts2  = cvCreateMat(successes,1,CV_32SC1)

  intrinsic_matrix    = cvCreateMat(3,3,CV_32FC1)
  distortion_coeffs   = cvCreateMat(4,1,CV_32FC1)
  rotation_vectors    = cvCreateMat(successes,3,CV_32FC1)
  translation_vectors = cvCreateMat(successes,3,CV_32FC1)

  cv.cvZero(rotation_vectors)
  cv.cvZero(translation_vectors)

  for i in range(successes*board_n):
    image_points2[i][0]  = image_points[i][0]
    image_points2[i][1]  = image_points[i][1]
    object_points2[i][0] = object_points[i][0]
    object_points2[i][1] = object_points[i][1] 
    object_points2[i][2] = object_points[i][2] 

  for i in range(successes):
    point_counts2[i] = point_counts[i]

  # At this point we have all of the chessboard corners we need.
  # Initialize the intrinsic matrix such that the two focal
  # lengths have a ratio of 1.0

  intrinsic_matrix[0,0] = 1.0
  intrinsic_matrix[1,1] = 1.0

  # This is the correct way to define cvCalibrateCamera2, it takes 7 arguments not 9 according to opencv1.0! In opencv2.0 the same function but with 9 parameters is used
  cvCalibrateCamera2( object_points2, image_points2, point_counts2, img_sz, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, CV_CALIB_ZERO_TANGENT_DIST)  # + CV_CALIB_FIX_ASPECT_RATIO 


  cvSave(datapath+"/Intrinsics.xml",intrinsic_matrix)
  cvSave(datapath+"/Distortion.xml",distortion_coeffs)
  
  # Calculating Total RMS Error in Pixels for Camera "n" once calibration is finished
  totalRmsErr = validate(intrinsic_matrix, distortion_coeffs, image_points, object_points, point_counts, TotalSamples, board_n, successes)
  print "Total RMS Error = %(s1)f pixels.\n"%{"s1": totalRmsErr}

  cam_nr  += 1											# 	Right Camera
  datapath = rightpath										

print "\nIndependent Camera Calibration Done!"
#------------------------------------------------------------------- Independent Camera Calibration Done ------------------------------------------------------------------------------------------