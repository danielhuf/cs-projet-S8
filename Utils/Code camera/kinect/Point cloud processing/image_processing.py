import cv2
import numpy as np
import matplotlib.pyplot as plt


# define the range of colors in HSV space
lower_green = np.array([70, 30, 40])
upper_green = np.array([100, 255, 255])
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 100])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([100, 40, 40])
upper_blue = np.array([120, 255, 255])
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])



def empty(a):
    pass


def find_color():
    """Assist finding the color of one object with webcam"""

    cap = cv2.VideoCapture(0)

    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars", 640, 340)
    
    # Create track bars
    cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)  # hue
    cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
    cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)  # saturation
    cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
    cv2.createTrackbar("Val Min", "TrackBars", 0, 255,
                       empty)  # value
    cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

    # load image
    while(1):
        ret, frame = cap.read()
        h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

        # transfer the BGR image into HSV. We use HSV because it better implies the color information
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # # Draw a histogram to find the range of the color
        # h, s, v = cv2.split(hsv_img)
        # print(max(h.reshape(-1, 1)), min(h.reshape(-1, 1)),
        #       max(s.reshape(-1, 1)), max(v.reshape(-1, 1)))
        # for i in [h, s, v]:
        #     hist = cv2.calcHist([i], [0], None, [255], [0, 256])
        #     plt.plot(hist)
        # plt.legend(['h', 's', 'v'])
        # plt.show()

        # define the range
        lower_color = np.array([h_min, s_min, v_min])
        upper_color = np.array([h_max, s_max, v_max])
        # cv2.imshow('1', h)

        # Using inRange to extract the pixels
        mask = cv2.inRange(hsv_img, lower_color, upper_color)

        # Performing morphological operations on the extracted pixels to remove noise and unnecessary regions.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel)

        # find the contours of all connected areas
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # draw contours
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * area / perimeter**2

            if area < 100:  # or circularity < 0.5:
                # ignore too small area
                continue
            else:
                cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)

        # show result
        cv2.imshow('result', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break


def select_color(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask1 = cv2.bitwise_or(mask_green, mask_red1)
    mask2 = cv2.bitwise_or(mask_red2, mask_blue)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.bitwise_or(mask, mask_yellow)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)

    # cv2.imshow('mask', mask)

    # find all contours
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # draw all contours and find the biggest area
    max_area = 0
    max_ind = 0
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # perimeter = cv2.arcLength(contour, True)
        # circularity = 4 * np.pi * area / perimeter**2

        if area < 50:  # or circularity < 0.5:
            # ignore too small area
            continue
        elif area > max_area:
            max_ind = i
            max_area = area
        cv2.drawContours(img, [contour], -1, (0, 0, 255), 1)

    max_mask = np.zeros_like(mask)
    cv2.drawContours(max_mask, contours, max_ind, (255, 255, 255), -1)

    # cv2.imshow('result', img)
    # cv2.waitKey(0)

    return mask, max_mask


def kmeans(img, iter, k):
    """ This method can be used in clustering
    img: the image data
    iter: maximum iteration
    k: the number of clusters
    """
    # img = plt.imread(img_path)
    row = img.shape[0]
    col = img.shape[1]
    img = img.reshape(-1, 3)

    img = np.column_stack((img, np.ones(row*col)))
    # 1. Generae cluster centre randomly
    cluster_center = img[np.random.choice(row*col, k)]
    # 2.classification
    distance = [[] for i in range(k)]
    for i in range(iter):

        for j in range(k):
            distance[j] = np.sqrt(np.sum((img - cluster_center[j])**2, axis=1))

        img[:, 3] = np.argmin(distance, axis=0)
        # 3. calculate new cluster centre
        for j in range(k):
            cluster_center[j] = np.mean(img[img[:, 3] == j], axis=0)
    img = img[:, 3]
    image_show = img.reshape(row, col)

    plt.imshow(image_show, cmap='gray')
    plt.show()


if __name__ == '__main__':

    # find_color()

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        select_color(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
