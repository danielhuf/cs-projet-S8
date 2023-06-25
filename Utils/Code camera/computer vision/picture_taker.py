""" 
How to run the snippet:
python picture_taker.py 
"""

import cv2

#===================================================== Capturing Image ==============================================================
key = cv2. waitKey(1)
webcam = cv2.VideoCapture(0)
while True:
    try:
        check, frame = webcam.read()
        cv2.imshow("Capturing", frame)
        key = cv2.waitKey(1)
        if key == ord('s'): 
            cv2.imwrite(filename='img/saved_img.jpg', img=frame)
            webcam.release()
            cv2.waitKey(1650)
            cv2.destroyAllWindows()

            """"
            # Processing image...
            img_ = cv2.imread('img/saved_img.jpg', cv2.IMREAD_ANYCOLOR)

            # Converting RGB image to grayscale...
            gray = cv2.cvtColor(img_, cv2.COLOR_BGR2GRAY)

            # Resizing image to 28x28 scale...
            img_ = cv2.resize(gray,(28,28))

            # Saving image
            img_resized = cv2.imwrite(filename='saved_img-final.jpg', img=img_)
            """
        
            break

        elif key == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break
        
    except(KeyboardInterrupt):
        webcam.release()
        cv2.destroyAllWindows()
        break