import cv2 as cv
import numpy as np
import open3d as o3d


import os
import re
import time
import copy
import socket
import threading
import datetime
import k4a


range2unit = 1/255
colorPieceDic = {
     1: "Yellow",
     2: "Blue",
     3: "Red",
     4: "Green",
     5: "Purple"
}
lower_color_dic = {
    "Yellow": np.array([100*range2unit, 100*range2unit, 0*range2unit]),
    "Blue": np.array([0*range2unit, 30*range2unit, 97*range2unit]),
    "Red": np.array([100*range2unit, 10*range2unit, 0*range2unit]),
    "Green": np.array([4*range2unit, 30*range2unit, 30*range2unit]),
	"Purple": np.array([30*range2unit, 50*range2unit, 90*range2unit])
}
upper_color_dic = {
    "Yellow": np.array([230*range2unit, 220*range2unit, 20*range2unit]),
    "Blue": np.array([40*range2unit, 100*range2unit, 220*range2unit]),
    "Red": np.array([230*range2unit, 70*range2unit, 80*range2unit]),
    "Green": np.array([40*range2unit, 140*range2unit, 100*range2unit]),
	"Purple": np.array([100*range2unit, 140*range2unit, 160*range2unit])
}

imgBlack =np.zeros((1080, 1920, 3), dtype=np.uint8)
imgBlackDepth = np.zeros((1080, 1920, 3), dtype=np.uint16)
img1 = imgBlack
img2 = imgBlack
img3 = imgBlack
img4 = imgBlack+100

imgColor = imgBlack
imgDepth = imgBlackDepth

centerPoint = None
centerPointX = None
centerPointY = None

pcdGlobal = None
pcdImg = None
origin = None
orientation = None

WINDOW_WIDTH = 1152     #    1920 *0.6
WINDOW_HEIGHT = 648     #   1080 *0.6

threadLock = threading.Lock()
timestamp0 = None
timestamp0 = "1006_1710"


###
### class MyPipeline
###

class MyDevice:
    def __init__(self):
        self.device0 = k4a.Device()
        self.deviceConfig0 = None
        
        self.calibration0 = None
        self.transformation0 = None
        
        self.capture0 = None
        self.sample0 = None

        self.color0 = None
        self.depth0 = None
        self.depthTransformed0 = None

        self.color0 = cv.imread("C:/Users/ACER/Downloads/TFG2/data/color/"+ timestamp0 +".jpg")
        self.depthTransformed0 = cv.imread("C:/Users/ACER/Downloads/TFG2/data/depth/"+ timestamp0 +".png")

        ##############################
        ##############################
        ###
        ### THIS PART CAN BE UNCOMMENTED IF YOU ARE CONNECTED TO THE KINECT
        ###
        """
        # Open a device
        if self.device0.get_device_count() == 0:
            print("No K4A devices found")
            os._exit(1)

        self.device0 = k4a.Device.open()
        if self.device0.serial_number == None:
            print("Device opening failed")
            os._exit(1)
        #
        #
        #info from device :
        #    print(self.device0.serial_number)
        #    print(self.device0.hardware_version)
        #    print(self.device0.color_ctrl_cap)
        #    print(self.device0.sync_in_connected)
        #    print(self.device0.sync_out_connected)


        # Configuartion a device before starting cameras
        self.deviceConfig0 = k4a.DeviceConfiguration(
            color_format= k4a.EImageFormat.COLOR_BGRA32,
            color_resolution=k4a.EColorResolution.RES_1080P,
            depth_mode=k4a.EDepthMode.NFOV_UNBINNED,
            camera_fps=k4a.EFramesPerSecond.FPS_5,
            #By default:
            #    synchronized_images_only=0,
            #    depth_delay_off_color_usec=0;
            #    wired_sync_mode=,
            #    subordinate_delay_off_master_usec=,
            #    disable_streaming_indicator=,
        )
        #
        #
        #info from device :
        #   print(deviceConfig0.camera_fps)
        #   print(deviceConfig0.color_format)
        #   print(deviceConfig0...
    

        # Start camera from the device
        status = self.device0.start_cameras(self.deviceConfig0)        
        if status == k4a.EStatus.FAILED:
            print("Device configuration failed")
            os._exit(1)


        # Start IMU from the device (only after start_cameras)
        status = self.device0.start_imu()        
        if status == k4a.EStatus.FAILED:
            print("Start IMU failed")
            os._exit(1)
        """
    def capture(self, calib):
        """
        # get IMU sample (-1 to wait until it gets a sample)
        self.sample0 = self.device0.get_imu_sample(-1)
        #
        #info from sample:
        #   print(self.sample0)

        # get capture
        self.capture0 = self.device0.get_capture(-1)
        #
        #info from capture:
        #   print(self.capture0.color)
        #   print(self.capture0.depth)
        #   print(self.capture0.ir)
        #   print(self.capture0.temperature)

        self.color0 = self.capture0.color
        colorMatrix0 = self.color0.data
        colorWidth0 = self.color0.width_pixels
        colorHeight0 = self.color0.height_pixels
        colorStride0 = self.color0.stride_bytes
        
        self.depth0 = self.capture0.depth
        depthMatrix0 = self.depth0.data
        depthWidth0 = self.depth0.width_pixels
        depthHeight0 = self.depth0.height_pixels
        depthStride0 = self.depth0.stride_bytes

        self.calibration0 = self.device0.get_calibration(
            self.deviceConfig0.depth_mode,
            self.deviceConfig0.color_resolution
        )
        #
        #
        #info from calibration:
        # print("---camera calibration parameters---")
        # print(self.calibration0.depth_cam_cal)
        # print(self.calibration0.color_cam_cal)
        # print(self.calibration0.extrinsics)  
        # print(self.calibration0.depth_mode)
        # print(self.calibration0.color_resolution)

        self.transformation0 = k4a.Transformation(self.calibration0)

        self.depthTransformed0 = self.transformation0.depth_image_to_color_camera(self.depth0)
        
        if calib:
            self.getCalibration()

        global timestamp0
        timestamp0 = datetime.datetime.now().strftime("%d%m_%H%M")
        
        #save color and depth images
        #cv.imwrite("C:/Users/ACER/Downloads/TFG2/data/color/"+ timestamp0 +".jpg", colorMatrix0)
        #cv.imwrite("C:/Users/ACER/Downloads/TFG2/data/depth/"+ timestamp0 +".png", self.depthTransformed0.data)
        #cv.imwrite("C:/Users/ACER/Downloads/TFG2/data/depth/"+ timestamp0 +".png", depthMatrix0)
        """
        global img1
        with threadLock:
            img1 = cv.cvtColor(self.color0.data, cv.COLOR_BGRA2BGR)         
    
    def getCalibration(self):
        boardWidth0 = 8
        boardHeighth0 = 5
        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 0.001)
                #### type	    The type of termination criteria 
                    # Terminate after some number of iterations (MAX_ITER) 
                    # or when the convergence metric reaches epsilon (EPS)
                #### maxCount	The maximum number of iterations or elements to compute (two ways to end it)
                #### epsilon	The desired accuracy or change in parameters at which the iterative algorithm stops

        # prepare object points: (0,0,0), (1,0,0), (2,0,0)...., (6,5,0)
        objp = np.zeros((boardWidth0*boardHeighth0,3), np.float32)
        objp[:,:2] = np.mgrid[0:boardWidth0,0:boardHeighth0].T.reshape(-1,2)

        # Arrays to store object points and image points from all images
        objPoints = [] # 3d point in real world space
        imgPoints = [] # 2d points in image plane.

        cv.namedWindow('Calibrate Frames', cv.WINDOW_NORMAL)

        for i in range (1,15):
            self.capture(False)
            img0 = self.color0.data
            imgWidth0 = img0.shape[0]
            imgHeight0 = img0.shape[1]

            #Transform color image into gray image (next function only works in gray scale)
            grayImg = cv.cvtColor(img0,cv.COLOR_BGR2GRAY)
            #Finds the positions of internal corners of the chessboard.
            #Returns all in 'corners' (array) and set 'ret' as True
            ret, corners = cv.findChessboardCorners(grayImg, (boardWidth0,boardHeighth0),None)

            # If worked findChessboardCorners, then refine and add object and image points
            if ret == True:
                print("calibImg")
                #Set into the list the points of the chessboard
                objPoints.append(objp)
                #Set into the list the points of the chessboard from the image
                imgPoints.append(corners)   

                #Finds the sub-pixel accurate location of the corners
                cornersSubPix = cv.cornerSubPix(grayImg,corners,(11,11),(-1,-1),criteria)
                # Draws chessboard corners in the image
                img0 = cv.drawChessboardCorners(img0, (boardWidth0,boardHeighth0), cornersSubPix,ret)
                
            else:
                print("there was not found any chessboard corners")
                
            # cv.resizeWindow('Calibrate Frames', int(imgHeight0/2), int(imgWidth0/2))
            # cv.imshow('Calibrate Frames',img0)
            # cv.waitKey(100)
            
        cv.destroyAllWindows()

        #print("Object Points:", objPoints)
        #print("Image Points:", imgPoints)

        ### Calibration: Get the homography (relation between object points and image points)
        calibRet, cameraMat, dist, rvecs, tvecs = cv.calibrateCamera(objPoints, imgPoints, (imgHeight0,imgWidth0),None,None)

        print("Calibrated:", calibRet)
        print("Camera Matrix:", cameraMat)
        ### [focal_legth_x  0               optical_center_cx]
        ### [0              focal_legth_y   optical_center_cy]
        ### [0              0               1                ]
        print("Distortion Parameters:", dist)
        print("Rotation Vectors:", rvecs)
        print("Translation Vectors:", tvecs)

        # Save the variables back to the text file
        with open("C:/Users/ACER/Downloads/TFG2/data/text/cameraCalibration.txt", "w") as file:
            file.write("Matrix = " + str(cameraMat) + "\n")
            file.write("Distortion = " + str(dist) + "\n")
            file.write("Rotation Vectors = " + str(rvecs) + "\n")
            file.write("Translation Vectors = " + str(tvecs) + "\n")
           
    def getWorkObject(self):
        copy0 = np.copy(self.color0.data)

        global img1
        with threadLock:
            img1 = cv.cvtColor(copy0, cv.COLOR_BGRA2BGR) 
        
        correct = False
        while correct == False:
            # Transform image for circle detection
            gray0 = cv.cvtColor(copy0, cv.COLOR_BGR2GRAY)
            #_, gray1 = cv.threshold(gray0, 10, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
            gray1 = cv.adaptiveThreshold(gray0, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, blockSize= 101, C= 20)
            gray2 = cv.medianBlur(gray1, 7)

            FindCircle = cv.HoughCircles(gray2, cv.HOUGH_GRADIENT, 1, 330, param1= 200, param2= 10, minRadius= 8, maxRadius = 15)
            circlesList = np.uint16(np.around(FindCircle)) 

            # Visualize circles
            #cv.namedWindow('bbb', cv.WINDOW_NORMAL)
            #cv.resizeWindow('bbb', WINDOW_WIDTH, WINDOW_HEIGHT)

            # Draw circles
            distances = []
            for i in circlesList[0][:][:] :
                
                if i[0]>150:
                    cv.circle(copy0, (i[0], i[1]), i[2], (255,0,0), 2)
                    cv.circle(copy0, (i[0], i[1]), 2, (255,0,0), 2)
                    distance = np.sqrt((i[0] - 0) ** 2 + (i[1] - 0) ** 2)
                    distances.append(distance)

            # Get the origin of the local reference system
            sorted_indices = np.argsort(distances)
            
            # cv.imshow("bbb", copy0)
            # cv.waitKey(0)
            global centerPoint
            global centerPointX
            global centerPointY
            centerPoint = circlesList[0][sorted_indices[0]]
            centerPointX = circlesList[0][sorted_indices[1]]
            centerPointY = circlesList[0][sorted_indices[2]]
            # centerPointA = circlesList[0][sorted_indices[3]]


            # Draw the axis
            if(circlesList[0][sorted_indices[1]][1] < circlesList[0][sorted_indices[2]][1]):
                centerPointY = circlesList[0][sorted_indices[1]]
                centerPointX = circlesList[0][sorted_indices[2]]
                print(centerPointX[1])
                print(centerPointY[1])
            
            print(centerPointX)
            if centerPoint[0]>200 and centerPointX[0] < 800 and centerPointX[0] > 200 and centerPointY[0] > 1000:
                correct = True

        cv.line(copy0, (centerPoint[0], centerPoint[1]), (centerPointX[0], centerPointX[1]), (0, 0, 255), 2)
        cv.line(copy0, (centerPoint[0], centerPoint[1]), (centerPointY[0], centerPointY[1]), (0, 255, 0), 2)

        #cv.imshow("bbb", copy0)
        #cv.waitKey(0)
        # Save images for pipeline recognition
        global imgColor
        global imgDepth
        imgColor = self.color0
        imgDepth = self.depthTransformed0.astype(np.uint16)
        
        global img2
        with threadLock:
            img2 = cv.cvtColor(copy0, cv.COLOR_BGRA2BGR) 

    def getPCD(self, empty):
        
        tic3 = time.time()
        tic4 = time.time()

        depthO3D = o3d.io.read_image("C:/Users/ACER/Downloads/TFG2/data/depth/" + timestamp0 + ".png")
        colorO3D = o3d.io.read_image("C:/Users/ACER/Downloads/TFG2/data/color/" + timestamp0 + ".jpg")
        
        #- Transform color and depth images to Open3D Images
        #global imgColor
        #global imgDepth
        #rgb0 = cv.cvtColor(imgColor, cv.COLOR_BGRA2RGB)
        #colorO3D = o3d.geometry.Image(rgb0)
        #depthO3D = o3d.geometry.Image(imgDepth)
        
        print("---")
        print(colorO3D)
        print(depthO3D)
        #o3d.visualization.draw_geometries([colorO3D])
        #o3d.visualization.draw_geometries([depthO3D])


        tic4 = time.time()
        #-  Create RGBD image from color and depth picture files
        Rgbd0 = o3d.geometry.RGBDImage.create_from_color_and_depth(colorO3D, depthO3D, convert_rgb_to_intensity = False)
                            ####    (color, depth, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=True)
                            ####    The color image is by default (True) converted into a grayscale image, stored in float ranged in [0, 1]. 
                            ####    The color image is now converted into a 3-channel image, stored in float ranged in [0, 1].
                            ####    The depth image is stored in float, representing the depth value in meters.
        #print(Rgbd0)
        """read intrinsics from TXT
        # Read the matrices from the text file
        with open("C:/Users/ACER/Downloads/TFG2/data/text/cameraCalibration.txt", "r") as file:
            lines = file.readlines()
            intrinsicValues = eval(lines[0].split("=")[1].strip())
        
        cx = intrinsicValues[0][2]
        cy = intrinsicValues[1][2]
        fx = intrinsicValues[0][0]
        fy = intrinsicValues[1][1]"""

        """#- Get intrinsics parameters
        intrinsicParameters = self.calibration0.color_cam_cal.intrinsics.parameters
        intrinsicValues = str(intrinsicParameters).split(', ')  # Split the string by comma and space
        values = {}

        for parameter in intrinsicValues:
            key, value = parameter.split('=')
            values[key] = float(value)

        cx = values['cx']
        cy = values['cy']
        fx = values['fx']
        fy = values['fy']"""

        cx=955.677429
        cy=552.134888
        fx=921.360779
        fy=921.394775

        intrinsicMatrix = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]])
        
        width0 = imgColor.shape[0]
        height0 = imgColor.shape[1]

        #- get Camera Pinhole for accurate 3D reconstruction
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width0, height0, intrinsicMatrix)


         #- Create Point Cloud from RGBD Image and intrinsic parameters of the camera
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            Rgbd0, o3d.camera.PinholeCameraIntrinsic(camera_intrinsic))
    
        print("create PCD {:.0f} [ms]".format(1000 * (time.time() - tic3)))
        #print(pcd)
        #o3d.io.write_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcd/pcd" + "00" + ".ply", pcd)

    
        # print("create PCD {:.0f} [ms]".format(1000 * (time.time() - tic3)))

                            #### create_from_rgbd_image(
                            ####    image, intrinsic, extrinsic=(with default value), project_valid_depth_only=True)

                            # pcd.points = o3d.utility.Vector3dVector(points)
                            # pcd.colors = o3d.utility.Vector3dVector(colors)

        #o3d.visualization.draw_geometries([pcd], front=front, lookat=lookat, up=up, zoom=zoom)
        #o3d.visualization.draw_geometries([pcd])



        #################################################
        #################################################
        #################################################
        #################################################
        #################################################

        empty = True
        if empty == True:
            #- Image pixel coordinates and depth(mm to m)
            imgDepth = np.asarray(depthO3D)
            x1 = centerPoint[0]
            y1 = centerPoint[1]
            depth1 = imgDepth[y1][x1]/1000
            #depth1 = 0.52
            #print(centerPoint)
            #print(depth1)

            x2 = centerPointX[0] 
            y2 = centerPointX[1]
            depth2 = imgDepth[y2][x2]/1000 #change order (y,x)
            #depth2 = 0.52
            #print(centerPointX)
            #print(depth2)

            x3 = centerPointY[0] 
            y3 = centerPointY[1]
            depth3 = imgDepth[y3][x3]/1000 #change order (y,x)
            #depth3 = 0.51
            #print(centerPointY)
            #print(depth3)

            #- Calculate 3D coordinates in meters
            point1 = np.dot(np.linalg.inv(intrinsicMatrix), np.array([x1, y1, 1])) * depth1
            num_points = 100
            color = np.array([0, 0, 240])
            distances = np.linalg.norm(np.asarray(pcd.points) - point1, axis=1)
            sorted_indices = np.argsort(distances)
            colors = np.asarray(pcd.colors)
            colors[sorted_indices[:num_points]] = color
            pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))  # Convert colors to Vector3dVector

            point2 = np.dot(np.linalg.inv(intrinsicMatrix), np.array([x2, y2, 1])) * depth2
            num_points = 100
            color = np.array([0, 0, 240])
            distances = np.linalg.norm(np.asarray(pcd.points) - point2, axis=1)
            sorted_indices = np.argsort(distances)
            colors = np.asarray(pcd.colors)
            colors[sorted_indices[:num_points]] = color
            pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))  # Convert colors to Vector3dVector

            #- Calculate 3D coordinates in meters
            point3 = np.dot(np.linalg.inv(intrinsicMatrix), np.array([x3, y3, 1])) * depth3
            num_points = 100
            color = np.array([0, 0, 240])
            distances = np.linalg.norm(np.asarray(pcd.points) - point3, axis=1)
            sorted_indices = np.argsort(distances)
            colors = np.asarray(pcd.colors)
            colors[sorted_indices[:num_points]] = color
            pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))  # Convert colors to Vector3dVector

            #print("pointsss")
            #print(point1)
            #print(point2)
            #print(point3)
            
            """pointOffset = np.array([point1[0]+0.03, point1[1]+0.03, point1[2]])
            num_points = 100
            color = np.array([100, 0, 240])
            distances = np.linalg.norm(np.asarray(pcd.points) - pointOffset, axis=1)
            sorted_indices = np.argsort(distances)
            colors = np.asarray(pcd.colors)
            colors[sorted_indices[:num_points]] = color
            pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))  # Convert colors to Vector3dVector"""
            
            
            #o3d.visualization.draw_geometries([pcd])

            # Subtract the centroid from the points and return them to the pcd
            points = np.array(pcd.points) - point1
            pcd.points = o3d.utility.Vector3dVector(points)

            #- Get and normalize the X and Y vectors
            vectorX = point2 - point1
            vectorY = point3 - point1
            vectorX_normalized = vectorX / np.linalg.norm(vectorX)
            vectorY_normalized = vectorY / np.linalg.norm(vectorY)

            # Calculate the z-axis vector using the cross product
            vectorZ = np.cross(vectorX_normalized, vectorX_normalized)

            eulerZ = np.arctan2(vectorX_normalized[1], vectorY_normalized[1])
            eulerY = np.arctan2(vectorX_normalized[0], vectorX_normalized[2])
            eulerX = np.arctan2(vectorY_normalized[2], vectorZ[2])
            
            rotZ = np.array([[np.cos(eulerZ), -np.sin(eulerZ), 0, 0],
                                        [np.sin(eulerZ), np.cos(eulerZ), 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
            eulerY = np.pi
            rotY = np.array([[np.cos(eulerY), 0, np.sin(eulerY), 0],
                                        [0, 1, 0, 0],
                                        [-np.sin(eulerY), 0, np.cos(eulerY), 0],
                                        [0, 0, 0, 1]])
            eulerX = -0.01
            rotX = np.array([[1, 0, 0, 0],
                                        [0, np.cos(eulerX), -np.sin(eulerX), 0],
                                        [0, np.sin(eulerX), np.cos(eulerX), 0],
                                        [0, 0, 0, 1]])
            
            pcd.transform(rotZ)
            pcd.transform(rotY)
            pcd.transform(rotX)
            #The pcd result is generated upside down
            #pcd.transform([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
            coord_axesA = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            coord_axesA.transform(np.identity(4))
            #o3d.visualization.draw_geometries([pcd, coord_axesA])


            print(np.linalg.norm(point3 - point1))
            scaling_factor = 0.47 / np.linalg.norm(point3 - point1)  # Calculate the scaling factor

            # Rescale the point cloud by multiplying each point's coordinates
            rescaled_pointcloud = np.array(pcd.points) * scaling_factor
            pcd.points = o3d.utility.Vector3dVector(rescaled_pointcloud)

            #- Create a mask to filter the point cloud based on depth
            maskX = np.logical_and(np.array(pcd.points)[:, 0] > 0, np.array(pcd.points)[:, 0] < 0.35)
            maskY = np.logical_and(np.array(pcd.points)[:, 1] > 0, np.array(pcd.points)[:, 1] < 0.47)
            maskZ= np.logical_and(np.array(pcd.points)[:, 2] > 0.01, np.array(pcd.points)[:, 2] < 0.05)
            combinedMasks = np.logical_and(maskX, np.logical_and(maskY, maskZ))
            pcd2 = pcd.select_by_index(np.where(combinedMasks)[0])
            #o3d.visualization.draw_geometries([pcd2])

            print("create and filter PCD {:.0f} [ms]".format(1000 * (time.time() - tic4)))
            #o3d.visualization.draw_geometries([pcd2])
            maskZ= np.logical_and(np.array(pcd.points)[:, 2] > -0.005, np.array(pcd.points)[:, 2] < 0.05)
            combinedMasks = np.logical_and(maskX, np.logical_and(maskY, maskZ))
            pcdImg0 = pcd.select_by_index(np.where(combinedMasks)[0])
            #o3d.visualization.draw_geometries([pcdImg0])
            print(pcd2)
            print(pcdImg0)
            print(pcd)
            #yellow
            pcd1 = o3d.io.read_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Yellow" + ".ply")
            pcd1.transform(rotZ)
            pcd1.transform(rotY)
            rescaled_pointcloud = np.array(pcd1.points) * scaling_factor
            pcd.points = o3d.utility.Vector3dVector(rescaled_pointcloud)
            o3d.io.write_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Yellow" + "2.ply", pcd1)
            #blue
            pcd1 = o3d.io.read_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Blue" + ".ply")
            pcd1.transform(rotZ)
            pcd1.transform(rotY)
            rescaled_pointcloud = np.array(pcd1.points) * scaling_factor
            pcd.points = o3d.utility.Vector3dVector(rescaled_pointcloud)
            o3d.io.write_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Blue" + "2.ply", pcd1)
            # o3d.visualization.draw_geometries([pcdImg0, pcd1])
            #red
            pcd1 = o3d.io.read_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Red" + ".ply")
            pcd1.transform(rotZ)
            pcd1.transform(rotY)
            rescaled_pointcloud = np.array(pcd1.points) * scaling_factor
            pcd.points = o3d.utility.Vector3dVector(rescaled_pointcloud)
            o3d.io.write_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Red" + "2.ply", pcd1)
            # o3d.visualization.draw_geometries([pcdImg0, pcd1])
            #green
            pcd1 = o3d.io.read_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Green" + ".ply")
            pcd1.transform(rotZ)
            pcd1.transform(rotY)
            rescaled_pointcloud = np.array(pcd1.points) * scaling_factor
            pcd.points = o3d.utility.Vector3dVector(rescaled_pointcloud)
            o3d.io.write_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Green" + "2.ply", pcd1)
            # o3d.visualization.draw_geometries([pcdImg0, pcd1])

            #purple
            pcd1 = o3d.io.read_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Purple" + ".ply")
            pcd1.transform(rotZ)
            pcd1.transform(rotY)
            rescaled_pointcloud = np.array(pcd1.points) * scaling_factor
            pcd.points = o3d.utility.Vector3dVector(rescaled_pointcloud)
            o3d.io.write_point_cloud ("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Purple" + "2.ply", pcd1)
            # o3d.visualization.draw_geometries([pcdImg0, pcd1])


            #o3d.visualization.draw_geometries([pcdImg0, pcd1])

            global origin
            origin = point1
           
            #o3d.visualization.draw_geometries([pcd, pcdColor, coord_axesA])
            
            global pcdGlobal
            global pcdImg
            global img3
            pcdGlobal = pcd2
            pcdImg = pcdImg0
            pcdImg.transform([[0, 1, 0, 0], [ -1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

            with threadLock:
                        
                img3 = pcd2img(pcdImg)

            red=False
            if red:
                    
                    maskX = np.logical_and(np.array(pcd.points)[:, 0] > -0, np.array(pcd.points)[:, 0] < 0.4)
                    maskY = np.logical_and(np.array(pcd.points)[:, 1] > -0, np.array(pcd.points)[:, 1] < 0.4)
                    maskZ= np.logical_and(np.array(pcd.points)[:, 2] > -0.02, np.array(pcd.points)[:, 2] < -0.01)

                    combinedMasks = np.logical_and(maskX, np.logical_and(maskY, maskZ))

                    # Filter the point cloud based on the mask
                    pcd2 = pcd.select_by_index(np.where(combinedMasks)[0])
                    
                    a = 0
                    rotation_matrix = np.array([
                        [np.cos(a), -np.sin(a), 0, -0.03],
                        [np.sin(a), np.cos(a), 0, -0.03],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])
                    pcd2.transform(rotation_matrix)
                    o3d.visualization.draw_geometries([pcd2, pcd])
                    o3d.io.write_point_cloud("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Red" + ".ply",pcd2)
    
            yellow=False
            if yellow:
                    
                    maskX = np.logical_and(np.array(pcd.points)[:, 0] > -0, np.array(pcd.points)[:, 0] < 0.4)
                    maskY = np.logical_and(np.array(pcd.points)[:, 1] > -0, np.array(pcd.points)[:, 1] < 0.4)
                    maskZ= np.logical_and(np.array(pcd.points)[:, 2] > -0.02, np.array(pcd.points)[:, 2] < -0.01)

                    combinedMasks = np.logical_and(maskX, np.logical_and(maskY, maskZ))

                    # Filter the point cloud based on the mask
                    pcd2 = pcd.select_by_index(np.where(combinedMasks)[0])
                    
                    a = 0
                    rotation_matrix = np.array([
                        [np.cos(a), -np.sin(a), 0, -0.03],
                        [np.sin(a), np.cos(a), 0, -0.03],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])
                    pcd2.transform(rotation_matrix)
                    o3d.visualization.draw_geometries([pcd2, pcd])
                    o3d.io.write_point_cloud("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Yellow" + ".ply",pcd2)
    
            purple=False
            if purple:
                    
                    maskX = np.logical_and(np.array(pcd.points)[:, 0] > -0, np.array(pcd.points)[:, 0] < 0.4)
                    maskY = np.logical_and(np.array(pcd.points)[:, 1] > -0, np.array(pcd.points)[:, 1] < 0.4)
                    maskZ= np.logical_and(np.array(pcd.points)[:, 2] > -0.02, np.array(pcd.points)[:, 2] < -0.01)

                    combinedMasks = np.logical_and(maskX, np.logical_and(maskY, maskZ))

                    # Filter the point cloud based on the mask
                    pcd2 = pcd.select_by_index(np.where(combinedMasks)[0])
                    
                    a = 0
                    rotation_matrix = np.array([
                        [np.cos(a), -np.sin(a), 0, -0.03],
                        [np.sin(a), np.cos(a), 0, -0.03],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])
                    pcd2.transform(rotation_matrix)
                    o3d.visualization.draw_geometries([pcd2, pcd])
                    o3d.io.write_point_cloud("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Purple" + ".ply",pcd2)
    
            green=False
            if green:
                    
                    maskX = np.logical_and(np.array(pcd.points)[:, 0] > -0, np.array(pcd.points)[:, 0] < 0.4)
                    maskY = np.logical_and(np.array(pcd.points)[:, 1] > -0, np.array(pcd.points)[:, 1] < 0.4)
                    maskZ= np.logical_and(np.array(pcd.points)[:, 2] > -0.02, np.array(pcd.points)[:, 2] < -0.01)

                    combinedMasks = np.logical_and(maskX, np.logical_and(maskY, maskZ))

                    # Filter the point cloud based on the mask
                    pcd2 = pcd.select_by_index(np.where(combinedMasks)[0])
                    
                    a = 0
                    rotation_matrix = np.array([
                        [np.cos(a), -np.sin(a), 0, -0.03],
                        [np.sin(a), np.cos(a), 0, -0.03],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])
                    pcd2.transform(rotation_matrix)
                    o3d.visualization.draw_geometries([pcd2, pcd])
                    o3d.io.write_point_cloud("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Green" + ".ply",pcd2)
    
            blue=False
            if blue:
                    
                    maskX = np.logical_and(np.array(pcd.points)[:, 0] > -0, np.array(pcd.points)[:, 0] < 0.4)
                    maskY = np.logical_and(np.array(pcd.points)[:, 1] > -0, np.array(pcd.points)[:, 1] < 0.4)
                    maskZ= np.logical_and(np.array(pcd.points)[:, 2] > -0.02, np.array(pcd.points)[:, 2] < -0.01)

                    combinedMasks = np.logical_and(maskX, np.logical_and(maskY, maskZ))

                    # Filter the point cloud based on the mask
                    pcd2 = pcd.select_by_index(np.where(combinedMasks)[0])
                    
                    a = 0
                    rotation_matrix = np.array([
                        [np.cos(a), -np.sin(a), 0, -0.03],
                        [np.sin(a), np.cos(a), 0, -0.03],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])
                    pcd2.transform(rotation_matrix)
                    o3d.visualization.draw_geometries([pcd2, pcd])
                    o3d.io.write_point_cloud("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + "Blue" + ".ply",pcd2)
         
    def close(self):
        print("Closing devices...")
        self.device0.stop_cameras()
        self.device0.stop_imu()

###
### class MyPipeline
###

class MyPipeline:
    
    def __init__(self):
        self.idx = 1
        self.colorPiece = colorPieceDic.get(self.idx)

    def readMatrixTXT(self, idx):
        self.idx = idx
        self.colorPiece = colorPieceDic.get(self.idx)

        with open("C:/Users/ACER/Downloads/TFG2/data/text/Transformations_"+ self.colorPiece +".txt", "r") as file:
            file_data = file.read()

    # Extract the matrices using regular expressions
        matrix_lines = re.findall(r"\[\[.*?\]\]", file_data)

        ActualT = [[float(val) for val in line.strip("[]").split(",")] for line in matrix_lines[0].split("],[")]
        RobotT = [[float(val) for val in line.strip("[]").split(",")] for line in matrix_lines[1].split("],[")]
        
        last_line = file_data.splitlines()[-1].strip()
        if last_line.startswith("change = "):
            change = last_line.split("=")[1].strip()

        return ActualT, RobotT, change

    def writeMatrixTXT(self, 
                       idx, 
                       ActualT, 
                       RobotT,
                       change):
        
        self.idx = idx
        self.colorPiece = colorPieceDic.get(self.idx)

        # Init Matrices
        if ActualT == None:
            ActualT = np.zeros((2, 4))
            ActualT[0][0] = idx 

        if RobotT == None:
            RobotT = np.zeros((2, 4))
            RobotT[0][0] = idx  

        # Format the matrix elements to desired precision and convert to string
        ActualT = np.array2string(np.around(ActualT, decimals=4), separator=',', formatter={'float_kind': lambda x: "{:+.4f}".format(x)})
        # Remove newline characters from formatted matrix string
        ActualT = ActualT.replace('\n', '')
        ActualT = ActualT.replace(' ', '')

        # Format the matrix elements to desired precision and convert to string
        RobotT = np.array2string(np.around(RobotT, decimals=4), separator=',', formatter={'float_kind': lambda x: "{:+.4f}".format(x)})
        # Remove newline characters from formatted matrix string
        RobotT = RobotT.replace('\n', '')
        RobotT = RobotT.replace(' ', '')

        #print(ActualT)
        #print(RobotT)
        #print(change)

        # Save the variables back to the text file
        with open("C:/Users/ACER/Downloads/TFG2/data/text/Transformations_"+ self.colorPiece +".txt", "w") as file:
            file.write("ActualT = " + str(ActualT) + "\n")
            file.write("RobotT = " + str(RobotT) + "\n")
            file.write("change = " + str(change) + "\n")
        
        print(self.colorPiece + " (" + str(self.idx) + ")")


###
### Functions
###

def initSocket():
    # Create a TCP/IP socket
    clientSocket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clientSocket2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Define server's IP address and port number
    serverIP1 = "192.168.0.210" 
    serverIP2 = "192.168.0.229" 
    serverPort = 8000 

    # Connect to the RobotStudio virtual controller server
    clientSocket1.connect((serverIP1, serverPort))

    # Connect to the RobotStudio real controller server
    clientSocket2.connect((serverIP2, serverPort))

    # Send a message to the RobotStudio controllers
    data = "Hello, server!!!"
    clientSocket1.sendall(data.encode())
    clientSocket2.sendall(data.encode())
    print("data000: " + data )
    # Receive a response from the RobotStudio controller
    response1 = clientSocket1.recv(1024).decode()
    print("Response received (1):", response1)
    response2 = clientSocket2.recv(1024).decode()
    print("Response received (2):", response2)

    return clientSocket1, clientSocket2

def find_color(colorPiece, pcd0):
    lower_color = lower_color_dic.get(colorPiece)
    upper_color = upper_color_dic.get(colorPiece)

    # Create a boolean mask to filter {color} points and close tones
    mask1 = np.logical_and(np.all(np.array(pcd0.colors) >= lower_color, axis=1),
                            np.all(np.array(pcd0.colors) <= upper_color, axis=1))
    pcd1= pcd0.select_by_index(np.where(mask1)[0])
    #o3d.visualization.draw_geometries([pcd1])
    outlier_cloud = pcd0.select_by_index(mask1,invert=True)
    outlier_cloud.paint_uniform_color([1,0,0])
    #o3d.visualization.draw_geometries([pcd1,outlier_cloud])



    if(len(pcd1.points) == 0):
        return pcd1, False
    # Remove isolated points
    _, inliers = pcd1.remove_radius_outlier(200, 0.02) #100 neighbours in radius = 2cm
    pcd2 = pcd1.select_by_index(inliers)
    outlier_cloud = pcd1.select_by_index(inliers,invert=True)
    outlier_cloud.paint_uniform_color([1,0,0])
    #o3d.visualization.draw_geometries([pcd2,outlier_cloud])

    if(len(pcd2.points) < 200):
        return pcd2, False
    else:
        return pcd2, True
    
    """if(len(inlier_cloud.points) == 0):
        return pcd2, False
    # Get the nearest points of the selected points (the piece)
    pcd0_tree = o3d.geometry.KDTreeFlann(pcd1)
    print("Find its 200 nearest neighbors")
    center = np.mean(np.asarray(inlier_cloud.points), axis=0)
	# Perform the k-NN search with the query point
    #[k2, idx2, _] = pcd0_tree.search_knn_vector_3d(center, 1800)
    [k2, idx2, _] = pcd0_tree.search_knn_vector_3d(center, 1800)
    pcd3 = pcd1.select_by_index(idx2)
    #print(pcd2)
    #o3d.visualization.draw_geometries([pcd3])
    

    mask3 = np.logical_and(np.array(pcd3.colors) >= lower_color, np.array(pcd3.colors) <= upper_color)
    idx3 = np.where(mask3)[0]
    pcd4 = pcd3.select_by_index(idx3)
    #print(pcd3)
    #o3d.visualization.draw_geometries([pcd4])
    
    if(len(pcd4.points) < 50):
        isPiece = False
    else:
        isPiece = True

    return pcd3, isPiece"""

def pipeline(pcd0, pcd1, idx, color, Mode):
    errorICP = 7e-06
    attempVal = 20
    pcd_objectA = o3d.io.read_point_cloud("C:/Users/ACER/Downloads/TFG2/data/pcdDataset/pcd" + color + ".ply")
    pcd_object0 = copy.deepcopy(pcd_objectA)
    #pcd_object0.paint_uniform_color([1,0,0])
    euler = np.pi
    rot = np.array([[np.cos(euler), 0, np.sin(euler), 0],
                                        [0, 1, 0, 0],
                                        [-np.sin(euler), 0, np.cos(euler), 0],
                                        [0, 0, 0, 1]])
    euler = np.pi/2
    rot2 = np.array([[np.cos(euler), -np.sin(euler), 0, 0.005],
                                        [np.sin(euler), np.cos(euler), 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
    eulerX = np.pi
    rotX = np.array([[1, 0, 0, 0],
                     [0, np.cos(eulerX), -np.sin(eulerX), 0],
                    [0, np.sin(eulerX), np.cos(eulerX), 0],
                    [0, 0, 0, 1]])
    
    pcd_object0.transform(rot)
    pcd_object0.transform(rot2)
    #o3d.visualization.draw_geometries([pcd_objectA])
    coord_axesA = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    coord_axesA.transform(np.identity(4))
    print(pcd_object0)
    #o3d.visualization.draw_geometries([pcd_object0, coord_axesA])
    ###################


    tic = time.time()

    size = 0.003
    #ESCENA original (0.001)
    voxel_scene = pcd1.voxel_down_sample(size)
    #OBJETO	original (0.001)
    voxel_obj= pcd_object0.voxel_down_sample(size)

    toc = 1000 * (time.time() - tic)
    print("Voxel {:.0f} [ms]".format(toc))
    print(voxel_scene)
    print(voxel_obj)
    if(idx == 1):
        errorICP = 7e-06
        attempVal = 30
    #o3d.visualization.draw_geometries([voxel_scene,voxel_obj])

    ###################
    errorICP = 1
    attemp = 0
    while errorICP > 4.3e-06 and attemp < attempVal:
        attemp += 1
        start = time.time()
        radius_normal = size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        #ESCENA
        voxel_scene.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=40))
        #OBJETO
        voxel_obj.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=40))

        # Compute the centroid of the point cloud
        centroid0 = np.mean(np.asarray(voxel_scene.points), axis=0)
        centroid = np.mean(np.asarray(voxel_obj.points), axis=0)

        # Calculate the dot product between each normal and the vector pointing towards the centroid
        dot_products0 = np.dot(np.asarray(voxel_scene.normals), centroid0)
        dot_products = np.dot(np.asarray(voxel_obj.normals), centroid)

        # Reshape dot_products to match the shape of pcd0.normals
        dot_products0 = dot_products0.reshape(-1, 1)
        dot_products = dot_products.reshape(-1, 1)

        # Invert the normals that have a negative dot product
        voxel_scene.normals = o3d.utility.Vector3dVector(np.where(dot_products0 < 0, -np.asarray(voxel_scene.normals), np.asarray(voxel_scene.normals)))
        voxel_obj.normals = o3d.utility.Vector3dVector(np.where(dot_products < 0, -np.asarray(voxel_obj.normals), np.asarray(voxel_obj.normals)))

        # Visualize the point cloud with aligned normals
        #o3d.visualization.draw_geometries([voxel_scene, voxel_obj])

        toc = 1000 * (time.time() - tic)
        print("Calcular normales {:.0f} [ms]".format(toc))

        ##################
        """
        tic = time.time()

        #ESCENA
        keypoints = o3d.geometry.keypoint.compute_iss_keypoints(voxel_scene,0.005,0.005,0.5,0.5) 
        #OBJETO
        keypoints_obj = o3d.geometry.keypoint.compute_iss_keypoints(voxel_obj,0.005,0.005,0.5,0.5) 

        toc = 1000 * (time.time() - tic)
        print("Keypoints {:.0f} [ms]".format(toc))
        print(keypoints)
        print(keypoints_obj)
        #o3d.visualization.draw_geometries([keypoints,keypoints_obj])
        """
        ##################

        tic = time.time()
        radius_feature = size * 5

        #ESCENA
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(voxel_scene,o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=60))
        #pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(keypoints,o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)) 
        #OBJETO
        obj_fpfh = o3d.pipelines.registration.compute_fpfh_feature(voxel_obj,o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=60))
        #obj_fpfh = o3d.pipelines.registration.compute_fpfh_feature(keypoints_obj,o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)) 

        toc = 1000 * (time.time() - tic)
        print("Descriptor {:.0f} [ms]".format(toc))
        print(pcd_fpfh)
        print(obj_fpfh)

        ##################

        tic01 = time.time()
        distance_threshold = 0.0025 #con 0.0005 va

        # Perform point cloud registration using FPFH features
        #result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        #    voxel_obj, voxel_scene, obj_fpfh, pcd_fpfh, max_correspondence_distance=0.05,
        #    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        #    ransac_n=4, criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
        #)

        voxelCopy = copy.deepcopy(voxel_obj)
        voxelICP = copy.deepcopy(voxel_obj)

        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(voxelCopy, voxel_scene, obj_fpfh, pcd_fpfh, True, distance_threshold,o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], o3d.pipelines.registration.RANSACConvergenceCriteria(10000000, 0.9999))
        #result1 = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(voxel_obj, voxel_scene, obj_fpfh, pcd_fpfh, True, distance_threshold,o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.9999))
        #result2 = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(voxel_obj, voxel_scene, obj_fpfh, pcd_fpfh, True, distance_threshold,o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.9999))
        #result3 = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(voxel_obj, voxel_scene, obj_fpfh, pcd_fpfh, True, distance_threshold,o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.9999))
        #result4 = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(voxel_obj, voxel_scene, obj_fpfh, pcd_fpfh, True, distance_threshold,o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.9999))

        #Transformation1 = result1.transformation
        #Transformation2 = result2.transformation
        #Transformation3 = result3.transformation
        #Transformation4 = result4.transformation
        #Transformation0 = (Transformation1 + Transformation2 + Transformation3 + Transformation4)/4
        Transformation0 = result.transformation
        #Transformation0 = result_icp.transformation
        voxelICP.paint_uniform_color([1,0,0])
        #voxelICP.transform([[0, 1, 0, 0], [ -1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        #o3d.visualization.draw_geometries([pcd0, voxelICP])

        #print(Transformation1)
        #print(Transformation2)
        #print(Transformation3)
        #print(Transformation4)
        print(Transformation0)

        voxelCopy.transform(Transformation0)
        voxelCopy.paint_uniform_color([1,0,0])
        #o3d.visualization.draw_geometries([pcd0, voxelCopy])

        toc = 1000 * (time.time() - tic01)
        print("Transformation {:.0f} [ms]".format(toc))

        ##################
        ##################
        
        ##-------------------------------------------------------------------------------------------------------------------------------------------------
        #RANSAC ERROR

        tree = o3d.geometry.KDTreeFlann(voxel_scene)

        distance = 0

        totalPoints = len(np.array(voxelCopy.points))

        for i in range (totalPoints):
            
            p = voxelCopy.points[i]
            [k,i1,dist] = tree.search_knn_vector_3d(p,1)
            distance = distance + dist[0]
        errorRansac = distance/totalPoints
        print("ransac error:")
        print(errorRansac)	
        
        ##----------------------------------------------------------------------------------------------------------------------------------------------------
        #ICP
        
        tic = time.time()
        distance_threshold = 0.003 * 0.4
        result_icp = o3d.pipelines.registration.registration_icp(voxelCopy, voxel_scene, distance_threshold , result.transformation, o3d.pipelines.registration.TransformationEstimationPointToPlane())

        #o3d.visualization.draw_geometries([pcd0, voxelCopy])
        voxelICP.transform(result_icp.transformation)    
        #o3d.visualization.draw_geometries([pcd0, voxelICP])
        toc = 1000 * (time.time() - tic)
        print("ICP {:.0f} [ms]".format(toc))
        
        ##-------------------------------------------------------------------------------------------------------------------------------------------------
        #ICP ERROR

        tree = o3d.geometry.KDTreeFlann(voxel_scene)

        distance = 0

        puntos = len(np.array(voxelICP.points))

        for i in range (puntos):

            p = voxelICP.points[i]
            [k,i1,dist] = tree.search_knn_vector_3d(p,1)
        
            distance = distance + dist[0]
            errorICP = distance/puntos

        print("error icp:")
        print(errorICP)
        print("==============")
        print("==============")
        stop = 1000 * (time.time() - start)
        print("todoo {:.0f} [ms]".format(stop))
    

    ##################
    ##################
    ##################
    print("attemp:" + str(attemp))
    Transformation0 = result_icp .transformation
    voxelICP.paint_uniform_color([1,0,0])
    #voxelICP.transform([[0, 1, 0, 0], [ -1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    #o3d.visualization.draw_geometries([pcd1, voxelICP, coord_axesA])
    #o3d.visualization.draw_geometries([pcd0, voxelICP, coord_axesA])

    x = Transformation0[0,3]
    y = Transformation0[1,3]
    z = Transformation0[2,3]
    #z = 0

    # Roll (X-axis rotation)
    roll = np.arctan2(Transformation0[1,2], Transformation0[2,2])

    # Pitch (Y-axis rotation)
    pitch = np.arctan2(-Transformation0[0,2], np.sqrt(Transformation0[1,2]**2 + Transformation0[2,2]**2))

    # Yaw (Z-axis rotation)
    yaw = np.arctan2(Transformation0[1,1], Transformation0[1,0]) 
    if idx == 4:
        if yaw > 0:
            yaw - np.pi
        elif yaw < 0:
            yaw + np.pi
    if idx == 3:
        yaw = 0

    else:
        if yaw > (np.pi/2):
            yaw = yaw - np.pi
        elif yaw < (-np.pi/2):
            yaw = yaw + np.pi
    

    #s =  np.sqrt((1 + Transformation0[0,0] + Transformation0[1,1] + Transformation0[2,2]) / 4)
    s = np.cos(yaw/2)
    #vx = np.sqrt((1 + Transformation0[0,0] - Transformation0[1,1] - Transformation0[2,2]) / 4)
    vx = 0
    #vy = np.sqrt((1 - Transformation0[0,0] + Transformation0[1,1] - Transformation0[2,2]) / 4)
    vy = 0
    #vz = np.sqrt((1 - Transformation0[0,0] - Transformation0[1,1] + Transformation0[2,2]) / 4)
    vz = np.sin(yaw/2)
    
    Matrix0 = [[idx, x, y, z],[s, vx, vy, vz]]
    #print(np.degrees(yaw))
    print(Matrix0)    

    global img2
    global img3
    global img4
    global pcdImg


    with threadLock:
        pcdCopy = copy.deepcopy(pcdImg)
        euler = np.pi/2
        rot2 = np.array([[np.cos(euler), -np.sin(euler), 0, 0],
                                        [np.sin(euler), np.cos(euler), 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
        pcdCopy.transform(rot2)
        pcdCopy += voxelICP
        euler = -np.pi/2
        rot2 = np.array([[np.cos(euler), -np.sin(euler), 0, 0],
                                        [np.sin(euler), np.cos(euler), 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
        pcdCopy.transform(rot2)
        img4 = pcd2img(pcdCopy)


    ##################
    ##################
    ##################
    ##################
    ##################

    MyPipeline0 = MyPipeline()
    ActualT, RobotT, change = MyPipeline0.readMatrixTXT(idx)


    change = "false"
    if(Mode == "robotPos"):
        RobotT = Matrix0
        change = "true"

    elif(Mode == "workerPos"):
        # If it is above some error
        if(abs(x-ActualT[0][1]) > 0.02 or 
           abs(y-ActualT[0][2]) > 0.02 or
           abs(z-ActualT[0][3]) > 0.02 or
           abs(vz-ActualT[1][3]) > np.sin(np.radians(15))):
            print("It's different..................")
            ActualT = Matrix0
            change = "true"
        else:
            print("It's NOT different..................")

    MyPipeline0.writeMatrixTXT( idx, ActualT, RobotT, change)

def pcd2img(pcd):
    #scaling_factor = 1.5
    #rescaled_pointcloud = np.array(pcd.points) * scaling_factor
    #pcd.points = o3d.utility.Vector3dVector(rescaled_pointcloud)

    # Create the Open3D visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=1920, height=1080)

    # Add the point cloud to the visualization
    vis.add_geometry(pcd)

    # Set the camera viewpoint parameters
    
#    front = [ -0.65285193404481656, -0.49729776527148872, 0.57138365821042325 ]
#    lookat = [ 0.17499960504227169, 0.23499987732914263, 0.0085968757436495798 ]
#    up = [ -0.29026103210455956, 0.86097597608703758, 0.41769474720490429 ]
#    zoom = 0.1


#    vis.get_view_control().set_front(front)
#    vis.get_view_control().set_lookat(lookat)
#    vis.get_view_control().set_up(up)
#    vis.get_view_control().set_zoom(zoom)

    # Capture the rendered image
    vis.poll_events()
    vis.update_renderer()
    image = vis.capture_screen_float_buffer()

    # Convert the image to a numpy array
    image_np = (np.array(image)*255).astype(np.uint8)

    # Close the visualization window
    vis.destroy_window()

    # Calculate the dimensions of the image
    height, width = image_np.shape[:2]

    # Define the desired zoom factor
    zoom_factor = 1.7  # Increase this value for a larger zoom

    # Calculate the ROI dimensions
    roi_width = int(width / zoom_factor)
    roi_height = int(height / zoom_factor)

    # Calculate the ROI coordinates
    roi_x = int((width - roi_width) / 2)
    roi_y = int((height - roi_height) / 2)

    # Extract the ROI from the image
    zoomed_image = image_np[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

    image_np1 = cv.resize(zoomed_image, (img2.shape[1], img2.shape[0]))       
    img0 = cv.cvtColor(image_np1, cv.COLOR_RGBA2BGR)
    return img0

#################################
#################################
#################################

def viewer():
    # Create a fixed-size window for displaying the frames
    cv.namedWindow('Viewer', cv.WINDOW_NORMAL)
    cv.resizeWindow('Viewer', WINDOW_WIDTH, WINDOW_HEIGHT)

    while True:
        # Read the processed image (acquire the lock first)
        with threadLock:
            # Create a composite image with the original and processed frames side by side
            imagesRow1 = cv.hconcat([img1, img2])
            imagesRow2 = cv.hconcat([img3, img4])
            imagesMatrix = cv.vconcat([imagesRow1,imagesRow2])
            
            # Display the composite frame
            cv.imshow('Viewer', imagesMatrix)

        if cv.waitKey(1) == 13:  # Exit when Enter key is pressed
            break

    cv.destroyAllWindows()

def main():
    tic = time.time()

    print("---------- Initial Setup ----------")

    global img1
    global img2
    global img3
    global img4

    global pcdGlobal

    ActualT = None
    RobotT= None
    change = False

    MyPipeline0 = MyPipeline()

    for idx in range (5):
        MyPipeline0.writeMatrixTXT(idx+1, ActualT, RobotT, change)
    
    MyDevice0 = MyDevice()
    #MyDevice0.getCalibration()
    
    ticInitSocket = time.time()
    clientSocket1, clientSocket2 = initSocket()
    print("TOTAL initSocket {:.0f} [ms]".format(1000 * (time.time() - ticInitSocket)))


    # Create a thread for displaying the modified image
    display_thread = threading.Thread(target=viewer)
    display_thread.start()

    print("Settings {:.0f} [ms]".format(1000 * (time.time() - tic)))
    timeRobot = time.time()
    timeWorker = time.time()
    
    while True:
        print("---------- Starting station analysis ----------")
        

        data = "ready"
        clientSocket1.sendall(data.encode())
        clientSocket2.sendall(data.encode())
        print("data000: " + data )

        # Receive a response from the RobotStudio controller
        response1 = clientSocket1.recv(1024).decode()
        print("Response Mode (1):", response1)
        response2 = clientSocket2.recv(1024).decode()
        print("Response Mode (2):", response2)

        tic1 = time.time()
        MyDevice0.capture(False)
        print("TOTAL capture {:.0f} [ms]".format(1000 * (time.time() - tic1)))

        MyDevice0.getWorkObject()
        #MyDevice0.cropImages() 
        tic2 = time.time()
        MyDevice0.getPCD(True)
        print("TOTAL PCD {:.0f} [ms]".format(1000 * (time.time() - tic2)))
        if response1 == "robotPos":
            print("TOTAL timeWorker {:.0f} [ms]".format(1000 * (time.time() - timeWorker)))


        if response1 == "pickplace" or response2 == "pickplace":
                data = " "
                print("No Pipeline")

        elif response1 == "robotPos" or response1 == "workerPos":
            for idx in range(5):
                pcd0 = copy.deepcopy(pcdGlobal) 
                colorPiece = colorPieceDic.get(idx+1)

                pcd1, isPiece = find_color(colorPiece, pcd0)
    
                if(isPiece):
                    print("---------- Detected " + colorPiece + " Piece ----------")
                    pipeline(pcd0, pcd1, idx+1, colorPiece, response2)
                
                ActualT, RobotT, change = MyPipeline0.readMatrixTXT(idx+1)

                if change == "true":
                        print(response1)
                        if response1 == "robotPos":
                            print("RobotT in robot WS")
                            matrix = str(RobotT)
                            OffsetT = RobotT.copy()
                            data = "robotDone"
                            
                        elif response1 == "workerPos":
                            print("ActualT in worker WS")
                            matrix = str(ActualT)
                            OffsetT = ActualT.copy()
                            data = "workerDone"
                    
                        else:
                            matrix = " "
                            data = "Error"

                        print("=========")

                        OffsetT[0][3] += 0.000
                        OffsetT[0][2] += 0.000
                        OffsetT[0][1] += 0.000
                        clientSocket1.sendall(str(OffsetT).encode())
                        clientSocket2.sendall(matrix.encode())
                        print("data000 =" + matrix)

                        # Receive a response from the RobotStudio controller
                        response1 = clientSocket1.recv(1024).decode()
                        print("Response received (1):", response1)
                        response2 = clientSocket2.recv(1024).decode()
                        print("Response received (2):", response2)
                        #discardBuffer(clientSocket1)
                        #discardBuffer(clientSocket2)
            
            print(".")
            
            clientSocket1.sendall(data.encode())
            clientSocket2.sendall(data.encode())
            print("data000 =" + data )
            #discardBuffer(clientSocket1)
            #discardBuffer(clientSocket2)

            # Receive a response from the RobotStudio controller
            response1 = clientSocket1.recv(1024).decode()
            print("Response received (1):", response1)
            response2 = clientSocket2.recv(1024).decode()
            print("Response received (2):", response2)
            print("TOTAL timeRobot {:.0f} [ms]".format(1000 * (time.time() - timeRobot)))


        if cv.waitKey(1) == 13:  # Exit when Enter key is pressed
            break
    ###
    MyDevice0.close()
    clientSocket1.close()
    clientSocket2.close()

def mainTest():
    tic = time.time()
    tictodo = time.time()

    global img1
    global img2
    global img3
    global img4

    global pcdGlobal

    ActualT = None
    RobotT= None
    change = False
    
    MyDevice0 = MyDevice()

    display_thread = threading.Thread(target=viewer)
    display_thread.start()
    
    print("Settings {:.0f} [ms]".format(1000 * (time.time() - tic)))
    
    while True:
        #MyDevice0.capture(False)
        MyDevice0.getWorkObject()
        tic2 = time.time()
        MyDevice0.getPCD(True)
        print("TOTAL PCD {:.0f} [ms]".format(1000 * (time.time() - tic2)))
        response1 = "robotPos"

        for idx in range(5):
            pcd0 = copy.deepcopy(pcdGlobal) 
            colorPiece = colorPieceDic.get(idx+1)
            print(colorPiece)
            pcd1, isPiece = find_color(colorPiece, pcd0)
            print(isPiece)
            print("qqqqqqqqqqqqqqqqqq")
            #o3d.visualization.draw_geometries([pcd1])
            if(isPiece):
                pipeline(pcd0, pcd1, idx+1, colorPiece, response1)

        print("completo {:.0f} [ms]".format(1000 * (time.time() - tictodo)))

        if cv.waitKey(1) == 13:  # Exit when Enter key is pressed
            break
    ###
    #MyDevice0.close()
    #clientSocket1.close()
    #clientSocket2.close()

    # Wait for both threads to finish
    processing_thread.join()
    display_thread.join()

if __name__ == '__main__':
    # Create a thread for image processing and modification
    processing_thread = threading.Thread(target=mainTest)
    processing_thread.start()