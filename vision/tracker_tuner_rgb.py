import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(2) 

cv2.namedWindow("Tuner_RGB")
cv2.resizeWindow("Tuner_RGB", 640, 480)

# BGR ORDER (Blue, Green, Red)
cv2.createTrackbar("L-B", "Tuner_RGB", 0, 255, nothing)
cv2.createTrackbar("L-G", "Tuner_RGB", 0, 255, nothing)
cv2.createTrackbar("L-R", "Tuner_RGB", 0, 255, nothing)
cv2.createTrackbar("U-B", "Tuner_RGB", 255, 255, nothing)
cv2.createTrackbar("U-G", "Tuner_RGB", 255, 255, nothing)
cv2.createTrackbar("U-R", "Tuner_RGB", 255, 255, nothing)

print("Press 'q' to save values.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    l_b = cv2.getTrackbarPos("L-B", "Tuner_RGB")
    l_g = cv2.getTrackbarPos("L-G", "Tuner_RGB")
    l_r = cv2.getTrackbarPos("L-R", "Tuner_RGB")
    u_b = cv2.getTrackbarPos("U-B", "Tuner_RGB")
    u_g = cv2.getTrackbarPos("U-G", "Tuner_RGB")
    u_r = cv2.getTrackbarPos("U-R", "Tuner_RGB")

    lower_bgr = np.array([l_b, l_g, l_r])
    upper_bgr = np.array([u_b, u_g, u_r])

    mask = cv2.inRange(frame, lower_bgr, upper_bgr)
    
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Maschera B/N", mask)
    cv2.imshow("Oggetto Isolato", result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"\n--- BGR Values ---")
        print(f"LOWER_BALL = np.array([{l_b}, {l_g}, {l_r}])")
        print(f"UPPER_BALL = np.array([{u_b}, {u_g}, {u_r}])")
        break

cap.release()
cv2.destroyAllWindows()