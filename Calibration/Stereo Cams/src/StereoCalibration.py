##########################################################################################
#			DECAPOD v0.5 CALIBRATION MODULE					 #
##########################################################################################
# Input : Some images captured by each camera for the chessboard for calibration.
#       : All calibration images for each camera are assumed in one containing folder
#
# Dated : June, 2010
##########################################################################################
from opencv import *
from opencv.cv import *
from opencv.highgui import *
import os
import sys
import math
import subprocess
import re
#--------------------------------------------------------------------------------------------------------------------------------------------------

# All the cameras in this list are not tested, only Canon G10 is tested, 
# the list was populated to test the profiling part
SUPPORTED_CAMERAS_LIST = [  "Canon PowerShot G10",
			    "Canon PowerShot G11",
			    "Canon EOS 550D",
			    "Canon Lorem Ipsum 8000", 
			    "Canon Lorem Ipsum 8100", 
			    "Nikon D90",
			    "Nikon Dolor Sit F5"  ]

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
    c = f.read()
    cameras_port = re.findall(r'usb:\d.*$',c,re.M)
    f.close()
    return cameras_port
#--------------------------------------------------------------------------------------------------------------------------------------------------
# parse_cameras_model_txt() Function: Returns the camera model for the cameras
#--------------------------------------------------------------------------------------------------------------------------------------------------
def parse_cameras_model_txt(imf):
  camera_model_name = ""
  
  for line in open(imf, 'r'):
      words_list = line.split()
      
  i = len(words_list) - 2
  for line in open(imf, 'r'):
      while i>=0:
	  camera_model_name = `words_list[i]` + " " + camera_model_name
	  i = i - 1
  return camera_model_name
#--------------------------------------------------------------------------------------------------------------------------------------------------
# check_2_supported_models() Function: Returns 'True' if there are 2 or more supported cameras of the same model are connected.
#--------------------------------------------------------------------------------------------------------------------------------------------------
def check_2_supported_models(imf):
    f = open(imf)
    c = f.read()
    max = 0
    for supported_cam in SUPPORTED_CAMERAS_LIST:
        cam_num = c.count(supported_cam)
        if cam_num > max: max = cam_num
    f.close()
    if max > 2:
        return True
    else:
        return False
#--------------------------------------------------------------------------------------------------------------------------------------------------
# project_path() Function: Gets the project folders path by subtracting last two folders from script's path
#--------------------------------------------------------------------------------------------------------------------------------------------------
def project_path():
    l = re.split(r'(\W+)', sys.path[0])
    i = 0
    path = ""
    while i < (len(l)-5):
        path = path + l[i]
        i+=1
    return path
#--------------------------------------------------------------------------------------------------------------------------------------------------
# capture_images_from_cameras() Function: Acquires the images from the cameras. In order to acquire images from the the two cameras
#					  nearly simultaneously processes have been used. It works well, the capture is pretty fast
#					  as tested with Canon G10 Cameras
#--------------------------------------------------------------------------------------------------------------------------------------------------
def capture_images_from_cameras():
    
    # Read camera ports, it is important to give port numbers while capturing
    os.system("gphoto2 --auto-detect > "+datapath+"cameras_port.txt")
    cameras_port = parse_cameras_port_txt(datapath+"cameras_port.txt")
    
    ###### Camera Error Checking to see if both cameras are connected/matching/supported
    if len(cameras_port) < 2:
        print "Cannot find 2 cameras, DECAPOD requires two matching supported cameras... Exiting."
        exit()
        
    if check_2_supported_models(datapath+"cameras_port.txt") == False:
        print "Cannot find 2 cameras of the same supported model, DECAPOD requires two matching supported cameras... Exiting."
        exit()
    ##### Camera Error Checking Done
    
    # Wait for taking the image
    print "Press any key to capture the image with an image window selected, <q> to end.\n"
    
    # getch() was not working nicely, so instead cvWaitKey() is used 
    c = cvWaitKey()									
    if c=='q':			# press 'q' while one image window is selected to quit                                                         
        exit()
    
    # Taking the images
    p1 = subprocess.Popen("gphoto2 --port "+cameras_port[0]+" --capture-image >/dev/null", shell=True)
    p2 = subprocess.Popen("gphoto2 --port "+cameras_port[1]+" --capture-image >/dev/null", shell=True)

    sts2 = os.waitpid(p2.pid, 0)[1]
    sts1 = os.waitpid(p1.pid, 0)[1]
    # Done taking images
    
    # Saving the images on data folder
    filename=leftpath+"/"+prefix+"%(s)i"%{"s":imgNum}+"."+ext
    p1 = subprocess.Popen("gphoto2 --port "+cameras_port[0]+" -P --new --force-overwrite --filename="+filename, shell=True)
    filename=rightpath+"/"+prefix+"%(s)i"%{"s":imgNum}+"."+ext
    p2 = subprocess.Popen("gphoto2 --port "+cameras_port[1]+" -P --new --force-overwrite --filename="+filename, shell=True)

    sts2 = os.waitpid(p2.pid, 0)[1]
    sts1 = os.waitpid(p1.pid, 0)[1]
    # Done saving images

    return True
#--------------------------------------------------------------------------------------------------------------------------------------------------
# Main
#--------------------------------------------------------------------------------------------------------------------------------------------------
board_w  	= 6					# Board width, hard coded
board_h  	= 8					# Board height, hard coded
nFrames 	= 5					# Number of samples, hard coded
datapath    	= project_path()+"Data/data4/"
leftpath 	= datapath+"Left"
rightpath   	= datapath+"Right"
matrixpath  	= datapath+"Matrices"
prefix   	= "img_"
ext 		= "jpg"                                   

board_n 	= board_w * board_h
board_sz 	= cvSize( board_w, board_h )

#ALLOCATE STORAGE
f_matrix = cvCreateMat(3,3,CV_32FC1)
e_matrix = cvCreateMat(3,3,CV_32FC1)
R = cvCreateMat(3,3,CV_64F)
T = cvCreateMat(3,1,CV_64F)
cvZero(f_matrix)
cvZero(e_matrix)
cvZero(R)
cvZero(T)
   
image_pointsR  = cvCreateMat(nFrames*board_n,1,CV_32FC2)
image_pointsL  = cvCreateMat(nFrames*board_n,1,CV_32FC2)
object_points = cvCreateMat(nFrames*board_n,3,CV_32FC1)
point_counts  = cvCreateMat(nFrames,1,CV_32SC1)

#Declare the Windows & Resize
cvNamedWindow("Left Camera", 0) # CV_WINDOW_AUTOSIZE didn't work
cvNamedWindow("Right Camera", 0)
cvResizeWindow("Left Camera", 512, 409)
cvResizeWindow("Right Camera", 512, 409)
cvMoveWindow("Left Camera", 0, 30)
cvMoveWindow("Right Camera", 520, 30)

corner_count = 0
successes    = 0
imgNum       = 0

#-----Start reading the files

while imgNum < nFrames:
  
    #capture the images first
    capture_images_from_cameras()
    
    # now read the images
    #### Begin with the left camera

    filenameL=leftpath+"/"+prefix+"%(s)i"%{"s":imgNum}+"."+ext
    imageL = cvLoadImage(filenameL)
    if imgNum == 0:
        assert(imageL);
        print "\nLeft image asserted."
        img_szL =cvGetSize(imageL)
        print "Image of Size: ", img_szL.width, " x ", img_szL.height
        gray_imageL = cvCreateImage(img_szL,8,1)    # subpixel
    if imageL == None:
        print "Oops. No image.\n" 
        continue
    print "Done."
    
   
    #Find chessboard corners:
    found, corners = cvFindChessboardCorners(imageL, board_sz, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS)   #&corner_count
    corner_count = len(corners)
    
    #If we got a good board, add it to our data
    if (found==0) or (corner_count != board_n):
        print "Couldn't find the chessboard, need to take another image!"
        continue

    #Get Subpixel accuracy on those corners
    cvCvtColor(imageL, gray_imageL, CV_BGR2GRAY)
    cvFindCornerSubPix(gray_imageL, corners, cvSize(11,11),cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1))
    print "Found %(s1)i corners out of %(#)i."%{"s1":corner_count,"#":board_n}

    #Draw it
    cvDrawChessboardCorners(imageL, board_sz, corners, found)
    cvShowImage( "Left Camera", imageL )

    #save the image and object points    
    step = successes*board_n
    i,j=step,0
    while j<board_n: 
        image_pointsL[i] = cvPoint2D32f(corners[j].x, corners[j].y)
        object_points [i][0] = j/board_w  
        object_points [i][1] = j%board_w 
        object_points [i][2] = 0.0  
        i+=1
        j+=1    
   
    #### Now proceed to with the right camera

    filenameR = rightpath+"/"+prefix+"%(s)i"%{"s":imgNum}+"."+ext
    imageR = cvLoadImage( filenameR )
    if imgNum == 0:
        assert(imageR);
        print "\nRight image asserted."
        img_szR = cvGetSize(imageR)
        print "Image of Size: ", img_szR.width, " x ", img_szR.height
        gray_imageR = cvCreateImage(img_szR,8,1)    #subpixel
    if imageR == None:
        print "Oops. No image.\n" 
        continue
    print "Done."
    
   
    #Find chessboard corners:
    found, corners = cvFindChessboardCorners(imageR, board_sz, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS)   #&corner_count
    corner_count = len(corners)

    #If we got a good board, add it to our data
    if (found==0) or (corner_count != board_n):
        print "Couldn't find the chessboard, need to take another image!"
        continue

    #Get Subpixel accuracy on those corners
    cvCvtColor(imageR, gray_imageR, CV_BGR2GRAY)
    cvFindCornerSubPix(gray_imageR, corners, cvSize(11,11),cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1))
    print "Found %(s1)i corners out of %(#)i."%{"s1":corner_count,"#":board_n}

    #Draw it
    cvDrawChessboardCorners(imageR, board_sz, corners, found)
    cvShowImage( "Right Camera", imageR )
    
    #save the image points
    i,j=step,0
    while j<board_n: 
        image_pointsR[i] = cvPoint2D32f(corners[j].x, corners[j].y)
        i += 1
        j += 1    
            
    point_counts [successes] = board_n  
    print "This view is to be used for calibration.\n"
    successes +=1
    imgNum += 1
    
    # end collection loop

print "\nIn total, found %(s)i image pairs with %(#)i 2D points each.\n"%{"s":successes, "#":corner_count}

# Single Calibration
##################################################
#Right Cam
intrinsicRC = cvCreateMat(3,3,CV_32FC1)
distortionRC = cvCreateMat(4,1,CV_32FC1)
Rrotation_vectors = cvCreateMat(successes,3,CV_32FC1)
Rtranslation_vectors = cvCreateMat(successes,3,CV_32FC1)

cv.cvZero(Rrotation_vectors)
cv.cvZero(Rtranslation_vectors)

intrinsicRC[0,0] = 1.0
intrinsicRC[1,1] = 1.0

cvCalibrateCamera2(object_points, image_pointsR, point_counts, img_szR, intrinsicRC, distortionRC, Rrotation_vectors, Rtranslation_vectors, CV_CALIB_ZERO_TANGENT_DIST)  # + CV_CALIB_FIX_ASPECT_RATIO 

# If these files are saved then the same data can be used for independent camera calibration as well
#cvSave(matrixpath+"/IntrinsicsRC.xml",intrinsicRC)
#cvSave(matrixpath+"/DistortionRC.xml",distortionRC)

#####
#Left Cam

intrinsicLC = cvCreateMat(3,3,CV_32FC1)
distortionLC = cvCreateMat(4,1,CV_32FC1)
Lrotation_vectors = cvCreateMat(successes,3,CV_32FC1)
Ltranslation_vectors = cvCreateMat(successes,3,CV_32FC1)

cv.cvZero(Lrotation_vectors)
cv.cvZero(Ltranslation_vectors)

intrinsicLC[0,0] = 1.0
intrinsicLC[1,1] = 1.0

cvCalibrateCamera2(object_points, image_pointsL, point_counts, img_szL, intrinsicLC, distortionLC, Lrotation_vectors, Ltranslation_vectors, CV_CALIB_ZERO_TANGENT_DIST)  # + CV_CALIB_FIX_ASPECT_RATIO 

# If these files are saved then the same data can be used for independent camera calibration as well
#cvSave(matrixpath+"/IntrinsicsLC.xml",intrinsicLC)
#cvSave(matrixpath+"/DistortionLC.xml",distortionLC)

# Single Calibration done
#########################################################


cvStereoCalibrate(object_points, image_pointsL, image_pointsR, point_counts, intrinsicLC, distortionLC, intrinsicRC, distortionRC, img_szL, 
            R, T, e_matrix, f_matrix, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
            CV_CALIB_FIX_INTRINSIC + CV_CALIB_ZERO_TANGENT_DIST )   # CV_CALIB_FIX_INTRINSIC + CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_INTRINSIC+CV_CALIB_FIX_ASPECT_RATIO+CV_CALIB_SAME_FOCAL_LENGTH 


#Now we can prepare the two cameras for rectification
Q   = cvCreateMat(4,4,CV_64F)
PLC = cvCreateMat(3,4,CV_64F)   #CV_32FC1
PRC = cvCreateMat(3,4,CV_64F)
RLC = cvCreateMat(3,3,CV_64F)
RRC = cvCreateMat(3,3,CV_64F)
cvZero(PLC)
cvZero(PRC)
cvZero(RLC)
cvZero(RRC)

cvStereoRectify(intrinsicLC, intrinsicRC, distortionLC, distortionRC, img_szL, R, T, RLC, RRC, PLC, PRC, Q)

# Calibration Data Saved as .xml files
cvSave(matrixpath+"/Q.xml", Q)					# Perspective Transformation Matrix [disparity to depth mapping]
cvSave(matrixpath+"/Rotation.xml", R) 				# Rotation Matrix
cvSave(matrixpath+"/Translation.xml", T) 			# Translation Matrix
cvSave(matrixpath+"/ProjectionRC.xml", PRC) 			# Projection Matrix right
cvSave(matrixpath+"/RotationRC.xml", RRC) 			# Rotation Matrix right
cvSave(matrixpath+"/Fundamental.xml", f_matrix) 		# Fundamental Matrix
cvSave(matrixpath+"/Essential.xml", e_matrix) 			# Essential Matrix
cvSave(matrixpath+"/ProjectionLC.xml", PLC) 			# Projection Matrix left
cvSave(matrixpath+"/RotationLC.xml", RLC) 			# Rotation Matrix left
cvSave(matrixpath+"/IntrinsicsRC.xml",intrinsicRC)		# Right Camera Intrinsic Matrix
cvSave(matrixpath+"/DistortionRC.xml",distortionRC)		# Right Camera Distortion Matrix
cvSave(matrixpath+"/IntrinsicsLC.xml",intrinsicLC)		# Left Camera Intrinsic Matrix
cvSave(matrixpath+"/DistortionLC.xml",distortionLC)		# Left Camera Distortion Matrix

print "Stereo Calibration done and obtained data are saved!\n"

#----------Stereo Calibration Done.


#----------Testing the Stereo Calibration

# ALLOCATE STORAGE

lineR = cvCreateMat(board_n,3,CV_32FC1)
lineL = cvCreateMat(board_n,3,CV_32FC1)
image_pointsR2  = cvCreateMat(board_n,1,CV_32FC2)
image_pointsL2  = cvCreateMat(board_n,1,CV_32FC2)
image_pointsR3  = cvCreateMat(board_n,1,CV_32FC2)
image_pointsL3  = cvCreateMat(board_n,1,CV_32FC2)

imgNum      = 0
successes   = 0
totalRmsErr = 0.0

#undistort and check error for each pair of images
while imgNum < nFrames:
   
    j = 0
    i = imgNum*board_n
    
    while j<board_n:
        image_pointsL2[j] = image_pointsL[i]
        image_pointsR2[j] = image_pointsR[i]
        j += 1
        i += 1 
          
    print "Undistorting: "
    cvUndistortPoints(image_pointsL2, image_pointsL3, intrinsicLC, distortionLC, RLC, PLC)
    cvUndistortPoints(image_pointsR2, image_pointsR3, intrinsicRC, distortionRC, RRC, PRC)
      
    #find the corresponding epipolar lines
    cvComputeCorrespondEpilines(image_pointsR3, 2, f_matrix, lineR)
    cvComputeCorrespondEpilines(image_pointsL3, 1, f_matrix, lineL)
    
    #calculating the rms error
    rmsErr = 0.0
    for i in range (board_n):
        rmsErr += (image_pointsR3[i][0]*lineL[i][0] +  image_pointsR3[i][1]*lineL[i][1] + lineL[i][2])**2
        rmsErr += (image_pointsL3[i][0]*lineR[i][0] +  image_pointsL3[i][1]*lineR[i][1] + lineR[i][2])**2
   
    totalRmsErr += rmsErr
    rmsErr = math.sqrt(rmsErr/board_n/2.0)
    print  "Image no. %(#)i has rms err = %(s1)f pixels\n"%{"s1":rmsErr,"#":imgNum}

    successes +=1
    imgNum += 1

print "\nIn total, found %(s)i image pairs with %(#)i 2D points each.\n"%{"s":successes, "#":corner_count}
if successes!=0:
      totalRmsErr = math.sqrt (totalRmsErr/board_n/2.0/successes)
print "Total RMS error = %(s1)f pixels.\n"%{"s1": totalRmsErr}
