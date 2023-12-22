import cv2

def detect_green(current_frame):
    image = current_frame
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(img_hsv, (36, 0, 0), (70, 255,255))
    target = cv2.bitwise_and(image,image, mask=mask1)
    green_pixel_count = cv2.countNonZero(mask1)

    if green_pixel_count > 3000:
        return True
    else: 
        return False

def detect_sign(current_frame, sign_left, sign_right):
    img1 = sign_left
    img2 = sign_right

    sift = cv2.SIFT_create()

    keypoints_1, descriptors_1 = sift.detectAndCompute(img1, None)
    keypoints_2, descriptors_2 = sift.detectAndCompute(img2, None)
    keypoints_3, descriptors_3 = sift.detectAndCompute(current_frame, None)

    # feature matching
    bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

    matches_left = bf.match(descriptors_1, descriptors_3)
    matches_right = bf.match(descriptors_2, descriptors_3)
    matches_left = sorted(matches_left, key=lambda x: x.distance)
    matches_right = sorted(matches_right, key=lambda x: x.distance)
    num_matches_l = len(matches_left)
    num_matches_r = len(matches_right)

    
    if num_matches_l > num_matches_r:
        return 1 # left turn
    else:
        return 2 # right turn

