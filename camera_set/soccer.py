import cv2

#image_ball = cv2.imread("data/ball.jpg", 0)
#cv2.imshow("ball_orig", image_ball)

cam = cv2.VideoCapture(0)

def nothing(x):
    pass

cv2.namedWindow("result")

cv2.createTrackbar("minb", "result",0,255,nothing)
cv2.createTrackbar("ming", "result",0,255,nothing)
cv2.createTrackbar("minr", "result",0,255,nothing)
cv2.createTrackbar("maxb", "result",0,255,nothing)
cv2.createTrackbar("maxg", "result",0,255,nothing)
cv2.createTrackbar("maxr", "result",0,255,nothing)

while True:

    error, image_ball = cam.read()
    image_ball = cv2.resize(image_ball, (240,160))

    image_ball = cv2.GaussianBlur(image_ball,(3,3),0)
    #cv2.imshow("gaussian", image_ball)
    cv2.imshow("RGB", image_ball)
    hsv_image_ball = cv2.cvtColor(image_ball, cv2.COLOR_BGR2HSV)

    cv2.imshow("HSV", hsv_image_ball)

    minb = cv2.getTrackbarPos("minb", "result")
    ming = cv2.getTrackbarPos("ming", "result")
    minr = cv2.getTrackbarPos("minr", "result")
    maxb = cv2.getTrackbarPos("maxb", "result")
    maxg = cv2.getTrackbarPos("maxg", "result")
    maxr = cv2.getTrackbarPos("maxr", "result")

    
    
    bin_image_ball = cv2.inRange(hsv_image_ball, (0, 168, 150), (13, 255, 255))
    
    #bin_image_ball = cv2.inRange(hsv_image_ball, (minb, ming, minr), (maxb, maxg, maxr))


    
    cv2.imshow("Binary", bin_image_ball)
    bin_image_ball = cv2.erode(bin_image_ball,None,iterations=1)
    bin_image_ball=cv2.dilate(bin_image_ball,None,iterations=1)
    cv2.imshow("Binary_erode", bin_image_ball)
    moments = cv2.moments(bin_image_ball)
    area = moments['m00']
    if(area > 5000):
    	x = (moments['m10'] - 120)/(120*12)
    	y = (moments['m01'] - 80)/(80*12)
    	print('x = ',x,' y = ',y)
    """contours = cv2.findContours(bin_image_ball, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours = contours[1]
    if len(contours):
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        cv2.drawContours(image_ball, contours, 0, (255, 0, 255), 3)
        #cv2.imshow("Contour", image_ball)

        (x, y), radius = cv2.minEnclosingCircle(contours[0])
        center = (int(x), int(y))
        radius = int(radius)
        cv2.circle(image_ball, center, radius, (0, 255, 0), 2)
        cv2.imshow("Circle", image_ball)
        #print(radius)
        #l = -0.125*radius+34
        #l = -0.0001*(radius**3) + 0.0258*(radius**2)-2.8669*radius+134.64
        #print(l)"""


    if cv2.waitKey(1)==ord('q'):
        break

cv2.destroyAllWindows()