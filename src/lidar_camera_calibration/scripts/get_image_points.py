import cv2
import rospy
refPt=[]
image = cv2.imread("test2.png")
def mouse_callback(event, x, y, flags, params):
	if event == cv2.EVENT_LBUTTONUP:
		refPt.append((x,y))
		print(x, y)
print(image.shape)
cv2.namedWindow("image")
cv2.setMouseCallback("image", mouse_callback)
while True:
	cv2.imshow("image", image)
	key = cv2.waitKey(1) & 0xFF

	# if the 'c' key is pressed, break from the loop
	if key == ord("c"):
		break
cv2.destroyAllWindows()
