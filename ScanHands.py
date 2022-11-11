import sys
import RPi.GPIO as GPIO
import time
import cv2 as cv
import mediapipe as mp


#Definition of servo pin on the camera Horizontal move
ServoCameraHorizontalPin = 11


#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)


#Ignore warning information
GPIO.setwarnings(False)

#capture video camera index 0
cap = cv.VideoCapture(0)

#Hands detection with mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands() # Create a hands object, use default of the method
mpDraw = mp.solutions.drawing_utils # Method to draw the landmarks of the hand detected

if not cap.isOpened():  
    print("Cannot open camera")
    exit()


#Motor pin initialization operation.
def init():
    global pwm_ENA
    global pwm_ENB
    global delaytime
    global pwm_servo
    global pwm_servo_cam
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(ServoCameraHorizontalPin , GPIO.OUT)
    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo_cam = GPIO.PWM(ServoCameraHorizontalPin, 50)
    pwm_servo.start(0)
    pwm_servo_cam.start(0)
    
    

def scan_hands_frame():
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        
    imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB) # ColorFrane BGR to RGB
    results = hands.process(imgRGB) #Process the frame to detect hand and give the result
    if results.multi_hand_landmarks:
        print ("Found")
        for handLMS in results.multi_hand_landmarks:
            mpDraw.draw_landmarks(frame, handLMS, mpHands.HAND_CONNECTIONS)
    cv.imshow('frame', frame)
    cv.waitKey(1)
    
    return results


def servo_cam_scan_hands():
    scan=None
    for pos in range(0, 181, 1):
        if scan!=None:
            
            break
        else:
            pwm_servo_cam.ChangeDutyCycle(2.5 + 10 * pos/180)
            time.sleep(0.009)
            pwm_servo_cam.ChangeDutyCycle(0)
            results=scan_hands_frame()
            scan = results.multi_hand_landmarks
            
            continue
        break
    if scan==None:
        
        for pos in reversed(range(0, 181, 1)):
            if scan!=None:
            
                break
            else:
                pwm_servo_cam.ChangeDutyCycle(2.5 + 10 * pos/180)
                time.sleep(0.009)
                pwm_servo_cam.ChangeDutyCycle(0)
                results=scan_hands_frame()
                scan = results.multi_hand_landmarks
                continue
            break
#Delay 2s   
    time.sleep(2)
    return scan
        


def main():
    """Main entry point for the script."""
    #The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.
    try:
        init()
        pwm_servo_cam.ChangeDutyCycle(2.5 + 10 * 0/180)
        time.sleep(0.009)
        pwm_servo_cam.ChangeDutyCycle(0)
        while True:
            scan=servo_cam_scan_hands()
            if scan!=None:
                print ("HAND FOUNDED !! ")
                time.sleep (5)
                break
            
    except KeyboardInterrupt:
        pass
    pwm_ENA.stop()
    pwm_ENB.stop()
    GPIO.cleanup()
    cap.release()
    cv.destroyAllWindows()
    pass


if __name__ == '__main__':
    sys.exit(main())

