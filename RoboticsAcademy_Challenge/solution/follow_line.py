from GUI import GUI
from HAL import HAL
import numpy as np
import cv2

acc_error = 0.0
prev_error = 0.0
p, i, d = 0.005, 0.00001, 0.027
while True:
    image = HAL.getImage()
    image_hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Creating masks for both ranges of red
    mask1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2
    result = cv2.bitwise_and(image, image, mask=red_mask)

    M = cv2.moments(red_mask)
    if M["m00"] != 0:
        # Calculate x, y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # Drawing a circle at the center
        cv2.circle(image, (cX, cY), 5, (255, 255, 255), -1)
        GUI.showImage(result)

        error = (image.shape[1]/2) - cX
        print('error = ', error)
        acc_error += error
        diff_error = error - prev_error
        u = p * error + i * acc_error + d * diff_error
        HAL.setV(3)
        HAL.setW(u)
        prev_error = error
    