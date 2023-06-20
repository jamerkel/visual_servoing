#!/usr/bin/python

import cv2
import imutils
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging.
	Press any key to continue.
	"""
	winname = "Image"
	cv2.namedWindow(winname)        # Create a named window
	cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
	cv2.imshow(winname, img)
	cv2.waitKey()
	cv2.destroyAllWindows()

def cd_sift_ransac(img, template):
	"""
	Implement the cone detection using SIFT + RANSAC algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	# Minimum number of matching features
	MIN_MATCH = 10
	# Create SIFT
	sift = cv2.xfeatures2d.SIFT_create()

	# Compute SIFT on template and test image
	kp1, des1 = sift.detectAndCompute(template,None)
	kp2, des2 = sift.detectAndCompute(img,None)

        #img_des = cv2.drawKeypoints(img, kp2, None)
        #image_print(img_des)

	# Find matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	# Find and store good matches
	good = []
	for m,n in matches:
		if m.distance < 0.75*n.distance:
			good.append(m)

	# If enough good matches, find bounding box
	if len(good) > MIN_MATCH:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		# Create mask
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                print(M)
                
		matchesMask = mask.ravel().tolist()

		h1 = template.shape[0]
                w1 = template.shape[1]
		pts = np.float32([ [0,0],[0,h1-1],[w1-1,h1-1],[w1-1,0] ]).reshape(-1,1,2)
                print(pts)
                
		########## YOUR CODE STARTS HERE ##########
                
                box_pts = cv2.perspectiveTransform(pts, M)
                

                x, y, w, h = cv2.boundingRect(box_pts.reshape(-1, 2))
                '''
                #image_print(
                cv2.rectangle(img, (x,y), (x+w, y+h), (0,0,255), 2)
                combined = np.hstack((img, img))
                
                for i in range(len(good)):
                    kpt = src_pts[i]
                    kpi = dst_pts[i]
                    pt1 = (np.int32(kpt.pt[0]), np.int32(kpt.pt[1]))
                    pt2 = (np.int32(kpi.pt[0]), np.int32(kpi.pt[1]))
                    cv2.line(combined, pt1, pt2, (0, 255, 0), 1)

                image_print(combined)'''

		x_min = x
                y_min = y
                x_max = x+w
                y_max = y+h

		########### YOUR CODE ENDS HERE ###########

		# Return bounding box
		return ((x_min, y_min), (x_max, y_max))
	else:

		print "[SIFT] not enough matches; matches: ", len(good)

		# Return bounding box of area 0 if no match found
		return ((0,0), (0,0))

def cd_template_matching(img, template):
	"""
	Implement the cone detection using template matching algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	template_canny = cv2.Canny(template, 50, 200)

	# Perform Canny Edge detection on test image
	grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img_canny = cv2.Canny(grey_img, 50, 200)

	# Get dimensions of template
	(img_height, img_width) = img_canny.shape[:2]

	# Keep track of best-fit match
        best_match = 0

	# Loop over different scales of image
	for scale in np.linspace(1.5, .5, 50):
		# Resize the image
		resized_template = imutils.resize(template_canny, width = int(template_canny.shape[1] * scale))
		(h,w) = resized_template.shape[:2]
		# Check to see if test image is now smaller than template image
		if resized_template.shape[0] > img_height or resized_template.shape[1] > img_width:
			continue

		########## YOUR CODE STARTS HERE ##########
		# Use OpenCV template matching functions to find the best match
		# across template scales.

                check_fit = cv2.matchTemplate(img_canny, resized_template, cv2.TM_CCOEFF_NORMED)

                _, max_value, _, max_pt = cv2.minMaxLoc(check_fit)

                if max_value > best_match:
                    best_match = max_value

		    # Remember to resize the bounding box using the highest scoring scale
		    # x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled
		    bounding_box = (max_pt,(max_pt[0] + w, max_pt[1] + h))
		########### YOUR CODE ENDS HERE ###########

        #print(best_match)

        #cv2.rectangle(img, bounding_box[0], bounding_box[1], (0,255,0), 2)
        #image_print(img)

	return bounding_box
'''
if __name__ == '__main__':
    img = cv2.imread('test_images_localization/basement_fixed.png')
    temp_read = cv2.imread('test_images_localization/map_scrap8.png')

    image_print(temp_read)

    temp = cv2.cvtColor(temp_read, cv2.COLOR_BGR2GRAY)
    
    cd_template_matching(img, temp)

    #cd_sift_ransac(img, temp)'''
