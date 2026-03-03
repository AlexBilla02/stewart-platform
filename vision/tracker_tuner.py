import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(2) 

cv2.namedWindow("Trackbars")
cv2.resizeWindow("Trackbars", 640, 480)

cv2.createTrackbar("L-H", "Trackbars", 0, 180, nothing)
cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U-H", "Trackbars", 180, 180, nothing)
cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)

print("Instructions:")
print("1. Move the sliders until target object becomes White.")
print("2. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L-H", "Trackbars")
    l_s = cv2.getTrackbarPos("L-S", "Trackbars")
    l_v = cv2.getTrackbarPos("L-V", "Trackbars")
    u_h = cv2.getTrackbarPos("U-H", "Trackbars")
    u_s = cv2.getTrackbarPos("U-S", "Trackbars")
    u_v = cv2.getTrackbarPos("U-V", "Trackbars")

    lower_color = np.array([l_h, l_s, l_v])
    upper_color = np.array([u_h, u_s, u_v])

    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Maschera (Punti isolati)", mask)
    cv2.imshow("Risultato Filtrato", result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"\nValues found:")
        print(f"LOWER_BALL = np.array([{l_h}, {l_s}, {l_v}])")
        print(f"UPPER_BALL = np.array([{u_h}, {u_s}, {u_v}])")
        break

cap.release()
cv2.destroyAllWindows()