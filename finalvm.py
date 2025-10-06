import cv2
import numpy as np
import pyautogui
import math
from cvzone.HandTrackingModule import HandDetector
import screen_brightness_control as sbc
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL

# Initialize camera and hand detector
cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.85, maxHands=2)

# Set resolution
cam_w, cam_h = 640, 480
cap.set(3, cam_w)
cap.set(4, cam_h)

# Screen resolution
screen_w, screen_h = pyautogui.size()

# Gesture box
box_w, box_h = 500, 300
box_x = (cam_w - box_w) // 2
box_y = (cam_h - box_h) // 2

# Volume setup
devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume_control = cast(interface, POINTER(IAudioEndpointVolume))

def get_distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

while True:
    success, img = cap.read()
    if not success:
        continue

    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)

    # Draw gesture area
    cv2.rectangle(img, (box_x, box_y), (box_x + box_w, box_y + box_h), (0, 255, 0), 2)
    cv2.putText(img, "Gesture Box", (box_x, box_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if hands:
        # One-hand gestures (mouse, click, scroll)
        if len(hands) == 1:
            hand = hands[0]
            lmList = hand["lmList"]
            fingers = detector.fingersUp(hand)

            ind_x, ind_y = lmList[8][:2]
            mid_x, mid_y = lmList[12][:2]
            thumb_x, thumb_y = lmList[4][:2]
            index_base_x, index_base_y = lmList[5][:2]

            # Mouse movement (Thumb + Index + Middle up)
            if fingers == [1, 1, 1, 0, 0]:
                if abs(thumb_x - index_base_x) < 30 and abs(thumb_y - index_base_y) < 30:
                    if box_x <= ind_x <= box_x + box_w and box_y <= ind_y <= box_y + box_h:
                        screen_x = np.interp(ind_x, [box_x, box_x + box_w], [0, screen_w])
                        screen_y = np.interp(ind_y, [box_y, box_y + box_h], [0, screen_h])
                        pyautogui.moveTo(screen_x, screen_y)

            # Scroll Down: Index + Middle up
            elif fingers == [0, 1, 1, 0, 0]:
                if abs(ind_x - mid_x) < 25:
                    pyautogui.scroll(-20)
                    cv2.putText(img, "Scroll Down", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            # Scroll Up: Index + Middle + Pinky up
            elif fingers == [0, 1, 1, 0, 1]:
                if abs(ind_x - mid_x) < 25:
                    pyautogui.scroll(20)
                    cv2.putText(img, "Scroll Up", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            # Left Click: Only index finger bent
            if fingers[1] == 0 and lmList[8][1] > lmList[6][1]:
                pyautogui.click()
                cv2.putText(img, "Left Click", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Right Click: Only middle finger bent
            if fingers[2] == 0 and lmList[12][1] > lmList[10][1]:
                pyautogui.rightClick()
                cv2.putText(img, "Right Click", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Two-hand gestures (brightness and volume

    

        elif len(hands) == 2:
            hand1, hand2 = hands[0], hands[1]

            # Identify left and right hands
            if hand1["type"] == "Left":
                left_hand, right_hand = hand1, hand2
            else:
                left_hand, right_hand = hand2, hand1

            lmL = left_hand["lmList"]
            lmR = right_hand["lmList"]
            fingersL = detector.fingersUp(left_hand)
            fingersR = detector.fingersUp(right_hand)

            

            # Brightness Control (Left hand thumb + index, pinky up)
            if fingersL[4] == 1:
                brightness_dist = get_distance(lmL[4], lmL[8])
                brightness_level = np.interp(brightness_dist, [30, 200], [0, 100])
                sbc.set_brightness(int(brightness_level))
                cv2.putText(img, f"Brightness: {int(brightness_level)}%", (50, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 200, 0), 2)

            # Volume Control (Right hand thumb + index, pinky NOT up)
            if fingersR[4] == 0:
                volume_dist = get_distance(lmR[4], lmR[8])
                volume_level = np.interp(volume_dist, [30, 200], [0.0, 1.0])
                volume_control.SetMasterVolumeLevelScalar(volume_level, None)
                cv2.putText(img, f"Volume: {int(volume_level * 100)}%", (50, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 255), 2)

    cv2.imshow("Virtual Controller", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()