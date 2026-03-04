import cv2
import numpy as np
import serial
import time

# --- Serial config ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- Vision config ---
# HSV Range to find white in platform
LOWER_WHITE = np.array([0, 0, 200])
UPPER_WHITE = np.array([180, 25, 255])

# HSV Range to distinguish ball 
LOWER_BALL = np.array([5, 120, 150])
UPPER_BALL = np.array([35, 255, 255])

def init_serial():
    try:
        return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    except serial.SerialException:
        print(f"Warning: Serial {SERIAL_PORT} not found. Test Mode ON.")
        return None

def main():
    ser = init_serial()
    cap = cv2.VideoCapture(1)
    
    # Low Res
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Press 'q' to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Blur image, reducing noise
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Find platform
        mask_white = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
        contours_plat, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        plat_center = None
        
        if contours_plat:
            # Select external contour
            c_plat = max(contours_plat, key=cv2.contourArea)
            # x and y will be the center of the platform
            ((x, y), radius) = cv2.minEnclosingCircle(c_plat)
            
            if radius > 50: 
                plat_center = (int(x), int(y))
                cv2.circle(frame, plat_center, int(radius), (255, 0, 0), 2)
                cv2.drawMarker(frame, plat_center, (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

        # Find ball
        mask_ball = cv2.inRange(hsv, LOWER_BALL, UPPER_BALL)
        mask_ball = cv2.erode(mask_ball, None, iterations=2)
        mask_ball = cv2.dilate(mask_ball, None, iterations=5)
        
        contours_ball, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_ball and plat_center:
            # Select the external contour of the ball
            c_ball = max(contours_ball, key=cv2.contourArea)
            M = cv2.moments(c_ball)
            
            if M["m00"] > 0:
                ball_x = int(M["m10"] / M["m00"])
                ball_y = int(M["m01"] / M["m00"])
                
                cv2.circle(frame, (ball_x, ball_y), 10, (0, 255, 0), -1)

                # relative position of the ball
                rel_x = ball_x - plat_center[0]
                rel_y = plat_center[1] - ball_y 

                # String format accepted by uart_comm.c
                data_str = f"{rel_x:.2f},{rel_y:.2f}\n"
                
                if ser:
                    ser.write(data_str.encode('ascii'))
                
                cv2.putText(frame, f"X: {rel_x}, Y: {rel_y}", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Tracking", frame)
        cv2.imshow("Ball Mask", mask_ball)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    if ser:
        ser.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()