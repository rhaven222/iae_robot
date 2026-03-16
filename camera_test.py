import cv2
#0 uses the usb connection where the camera is plugged in /dev/video®
pic = cv2. VideoCapture(0)

while True:
    ret, frame = pic. read()
    if not ret:
        print( "Camera not found" )
        break
    cv2. imshow( "Camera", frame )
    
    if cv2.waitKey(1) & Oxff == ord(*q'):
        break
        
pic. release()
cv2. destroyAllWindows ()