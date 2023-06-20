#!/usr/bin/python

import cv2
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
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

        # orange range in BGR

        min_orange = np.array([0, 40, 175])
        max_orange = np.array([35, 245, 255])
        
        #create and apply mask to original image
        mask = cv2.inRange(img, min_orange, max_orange)

        # from binary mask, find 'trace' of cone from contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cone = None
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area >= 250: #pixel minimum to be considered a cone
                cone = contour

        if cone is not None:
            x, y, w, h = cv2.boundingRect(cone)

	    bounding_box = ((x,y),(x+w,y+h))

        else:
            bounding_box = ((0,0),(0,0))

        #cv2.rectangle(img, bounding_box[0], bounding_box[1], (0,0,255),2)
        #image_print(img)

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
'''
if __name__ == '__main__':
    img_path = 'test_images_cone/test9.jpg'
    img = cv2.imread(img_path)

    cd_color_segmentation(img, None)'''
