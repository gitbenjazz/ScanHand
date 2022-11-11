
# USING MEDIAPIPE TO SCAN HANDS
## Raspberry Pi with a Yahboom Tank Robot

Yahboom Raspberry Pi Tank Robot use the latest Raspberry Pi 4B development board as the core controller. It comes with a Camera , two motors for each side of the vehicle, a distance sensor, a line Tracker, some leds, one servo to rotate the distance sensor within 180 degree and two servo ( horizontal and vertical) for the camera. 

![Yahboom Tank](https://github.com/YahboomTechnology/Raspberry-pi-G1-Tank/blob/master/Yahboom_PiTank.jpg?raw=true)


The aim of this project is just to use the camera to detect hands to be able to trigger an action. Here in this example, it's just a print text output that can be replaced by anything. 
To perceive the shape and motion of hands, we use a Google library named Mediapipe. MediaPipe Hands is a high-fidelity hand and finger tracking solution. It employs machine learning (ML) to infer 21 3D landmarks of a hand from just a single frame.
<!-- Links  -->

![landmarks](https://mediapipe.dev/images/mobile/hand_landmarks.png)

On a next repo, i'll describe how to interact with any of these 21 landmarks. Based on these differents positions, we will be able to trigger different actions. While coming naturally to people, robust real-time hand perception is a decidedly challenging computer vision task, these landmarsks positions can bring many different visual sign to interact with the robot and allowing hand gesture control.



###Code explained

We use opencv to process the frames and mediapipe to detect the hands
```python
    import cv2 as cv
    import mediapipe as mp
```

The camera is mounted on two servo ( horizontal and vertical). In this project we're using only the horizontal to scan from left to right with an angle of 180 degrees. 

![Servo](http://www.yahboom.net/Public/ueditor/php/upload/image/20220523/1653300312847198.png)

The servo interface is as below

![Servo_Interface](http://www.yahboom.net/Public/ueditor/php/upload/image/20190225/1551095858268604.png)


- The front servo is connected to servo interface J1. 
- The horizontal servo is connected to servo interface J2. 
- The vertical servo is connected to servo interface J3. 

According to the circuit schematic, and as we want to use the horizontal servo. We need to find the correspondant BCM for our code. In our case it's 11

J2---23(Physical pin)-----11(BCM)


```python
    ServoCameraHorizontalPin = 11
    GPIO.setmode(GPIO.BCM)

```

Using openCV, we create an object cap to process each frame of the Camera. In our case, the index is 0
```python
cap = cv.VideoCapture(0)
```


Using mediapipe, we detect the hands ( all the 21 Mark) and we draw the hands by connecting all the marks
```
mpHands = mp.solutions.hands
hands = mpHands.Hands() # Create a hands object, use default of the method
mpDraw = mp.solutions.drawing_utils # Draw the hand
```


### Code explained

scan_hands : Is used to process each frane and analyze if a hand is present
The for loop is to do the process of drawing on both hands (handLMS)
cv.imshow is used to output the frame on the screen, it has to be used with cv.waitKey() to work

```
def scan_hands_frame():
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        
    imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB) # Frame in color, we switch from BGR to RGB 
    results = hands.process(imgRGB) #Process the image to detect hand and give the result. If hand is detected, the output will be the position of each landmark otherwise it will be None
    if results.multi_hand_landmarks:
        print ("Hand Founded")
        for handLMS in results.multi_hand_landmarks:
            mpDraw.draw_landmarks(frame, handLMS, mpHands.HAND_CONNECTIONS)
    cv.imshow('frame', frame)
    cv.waitKey(1)
    
    return results
```



servo_cam_scan : Is used to rotate the camera from left to right and then right to left. For each pos, we call scan_hands. The result of scan_hands ( found or not found ) is adressed to the variable scan. We rotate the servo while scan is None. No hands detected

```
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
```



The main function call servo_cam_scan inside an infinite loop until a hand is detected and then scan not equal to None. Here we print a text but we can trigger any action we can imagine with the Tank

```
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
                print ("HANDS DETECTED !! ")
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

```


