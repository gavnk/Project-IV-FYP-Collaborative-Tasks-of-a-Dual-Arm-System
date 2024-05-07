#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from skimage.morphology import skeletonize
from matplotlib import pyplot as plt

class ObjectLevelChecker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_level_checker', anonymous=True)

        # Set up a subscriber to the RealSense camera image topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

        # Set up a CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        level, processed_image = self.is_object_level(cv_image)
        if level:
            print("The object is level.")
        else:
            print("The object is not level.")
        
        # Display the image with lines
        cv2.imshow("Level Detection", processed_image)
        cv2.waitKey(1)

    def is_object_level(self, image):
         # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # STEP 3.1 define a set of Sobel kernels to compute the gradient in the X-direction and Y-direction
        xSobel = np.array([[-1,0,1],[-2,0,2],[-1,0,1]])
        ySobel = xSobel.T

        # STEP 3.2 use cv2.filter2D to estimate the gradients in the X and Y-directions
        Ix = cv2.filter2D(gray,ddepth=cv2.CV_64F,kernel=xSobel)
        Iy = cv2.filter2D(gray,ddepth=cv2.CV_64F,kernel=ySobel)

        #STEP 3.3 combine the result of STEP 3.2 to compute a gradient magnitude image
        G = np.sqrt(Ix**2 + Iy**2)

        #STEP 3.4 Compute an edge-map from G: for each pixel in G above a threshold of 20 set the corresponding pixel in E to 1
        E = G>20

        ## Some image encodings cause the edge detector to create phantom
        ## edges around the border for the image. Here we just zero them out
        ## to avoid them corrupting results later
        E[0:5,:] = 0
        E[-1:-5:,:] = 0
        E[:,0:5] = 0
        E[:,-1:-5] = 0
        E = skeletonize(E)
        plt.imshow(E)
        plt.show()

       
        
        # Apply Gaussian blur to reduce noise
       # blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
       # edges = cv2.Canny(blur, 50, 150, apertureSize=3)
        # Find lines using Hough Transform
       # lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
        #STEP 7.1 here we use np.array to define an array containing [rho_min, rho_max, rho_delta]
        rho_range = np.array([-400, 400, 2])
        #STEP 7.2 here we use np.array to define an array containing [theta_min, theta_max, theta_delta]
        theta_range = np.array([0, np.pi, np.pi/180])
        #STEP 7.3 here we compute the Hough transform using the above rho and theta ranges
        A = self.Hough(E, rho_range, theta_range)
        A = self.nonmax(A,5)

        #plt.imshow(A,vmin=0, vmax=120)
        #plt.set_cmap('hot')
        #plt.gca().set_aspect(A.shape[1]/A.shape[0])

        #STEP 9: Here we use the extract_peaks function to find all peaks greater than a value of 170
        lines = self.extract_peaks(A, 120, rho_range, theta_range)
        print(lines.shape)

        D = gray.copy()

        #STEP 10: Here we add code to use drawline to visualise each detected line on the image D
        for line in lines:
            self.drawline(D, line)

       # plt.imshow(D)
      #  plt.show()

        intersections = []

        # TASK 11: Add code here that uses the intersection function to compute the intersection
        # between each pair of detected lines, where the angle between the lines is greater than
        # 10 degrees (i.e. np.pi/18 radians). Each intersection should be appended to the
        # intersections list
        for line1 in lines:
            for line2 in lines:
                if (np.abs(line1[1] - line2[1]) > np.pi/18):
                    pt = self.intersection(line1, line2)
                    intersections.append(pt)

       # D = gray.copy()
      #  i = 0
       # for i in intersections:
       #     cv2.circle(D,(i[0][0],i[0][1]),4,(255,0,0),thickness=3)

        #plt.imshow(D)
       # plt.show()
        
        level = False

      #  if lines is not None:
       #     for line in lines:
       #         x1, y1, x2, y2 = line[0]
       #         angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
        #        if abs(angle) < 10 or abs(angle) > 170:
         #           level = True
         #           cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw horizontal line in green

        return level, D
    

    def Hough(self,E, rho_range, theta_range):
    # Computes the Hough transform for a given input edge map
     #   Args:
     #       E (np.array) - edge image
     #       rho_range (np.array) = [rho_min, rho_max, rho_delta]
     #       theta_range (np.array) = [theta_min, theta_max, theta_delta]
##
    #    Returns:
     #       np.array: 2D Accumulator Array
   # '''
        theta_list = np.arange(theta_range[0], theta_range[1], theta_range[2])
        rho_list = np.arange(rho_range[0], rho_range[1], rho_range[2])

        # Construct Accumulator
        Acc = np.zeros((rho_list.shape[0],theta_list.shape[0]))

        # Raster scan image
        for row in range(0,E.shape[0]):
            for col in range(0,E.shape[1]):
                # If we are on an edge
                if E[row,col]==1:
                    # Compute the rho values
                    # Note how all rho values are calculated in a single
                    # statement (i.e. by computing over all the thetas)
                    rhos = col*np.cos(theta_list) + row*np.sin(theta_list)

                    # Map the rhos to Accumulator indices such that rho_min
                    # corresponds to index 0, rho_min + rho_delta corresponds to
                    # index 1, etc.
                    rho_indices = np.round((rhos - rho_range[0])/rho_range[2]).astype(int)

                    # Create a binary mask to indentify which rho_indices are valid
                    # Since rho values can be outside the rho range specified in the
                    # input, the results indices would be outside the range of the
                    # Accumulator array -- so we need to identify these the following
                    # binary mack
                    mask = np.logical_and((rho_indices >= 0), (rho_indices < rho_list.shape[0]))

                    # Compute the set of theta indices
                    # Since we compute the rho values for every theta
                    # the theta indices include every index
                    theta_indices = np.arange(0,theta_list.shape[0])

                    # Increment the elements of the Accumulator array
                    # Again notice how we can pick out the valid rho, theta
                    # pairs using the mask, and then index into and increment
                    # each corresponding element of the Accumulator in a single
                    # statement
                    Acc[rho_indices[mask],theta_indices[mask]] +=1

        return Acc
    
    def nonmax(self,A, size = 1):
        M = np.zeros(A.shape)
        for col in range(size,A.shape[1]-size):
            for row in range(size,A.shape[0]-size):
                #STEP 8: Here we add code to copy the value of A[row,col] through to M[row,col] iff it is greater than all of the the *other* values
                # in the neighbour from [row-size, col-size] to [row+size, col+size]
                d = A[row,col]
                A[row,col]=0
                M[row, col] = d if (d > np.max(A[row-size:row+size, col-size:col+size])) else 0
                A[row,col] = d
        return M
    
    def extract_peaks(self,A, thresh, r_range, theta_range):
        ind = np.where(A>thresh)
        theta_list = np.arange(theta_range[0], theta_range[1], theta_range[2])
        r_list = np.arange(r_range[0], r_range[1], r_range[2])

        return np.vstack((r_list[ind[0]], theta_list[ind[1]])).T
    
    def drawline(self,img, line):
        r,theta = line
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*r
        y0 = b*r
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(img,(x1,y1),(x2,y2),(255,0,255),2)

    def intersection(self,line1, line2):
  #  """Finds the intersection of two lines given in Hesse normal form.
#
  #  Returns closest integer pixel locations.
  #  See https://stackoverflow.com/a/383527/5087436
  #  """
        rho1, theta1 = line1
        rho2, theta2 = line2
        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]
        ])
        b = np.array([[rho1], [rho2]])
        try:
            x0, y0 = np.linalg.solve(A, b)
        except:
            print("Singular matrix -- You should check that the lines are not close to parallel"
                "e.g. check that lines are not within 10 degrees of each other")
            raise
        x0, y0 = int(np.round(x0)), int(np.round(y0))
        return [[x0, y0]]

if __name__ == '__main__':
    # Instantiate the object level checker
    checker = ObjectLevelChecker()
    
    # Spin to keep the script from exiting until the node is stopped
    rospy.spin()
    cv2.destroyAllWindows()
