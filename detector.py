import cv2
import numpy as np

global x
x = False

def change(a, b):
    global x
    x = True 

    return a > b


def detect_bag(action = None):

    global start_stream, amount_of_bags, bags, last_r

    if action == 'detect':

        if start_stream:
            err, resolution, image = vrep.simxGetVisionSensorImage(
                                    clientID, cameraID, 0, vrep.simx_opmode_buffer)
        else:
            start_stream = True
            err, resolution, image = vrep.simxGetVisionSensorImage(
                                    clientID, cameraID, 0, vrep.simx_opmode_streaming)
            # read image
            time.sleep(0.1)
            err, resolution, image = vrep.simxGetVisionSensorImage(
                                    clientID, cameraID, 0, vrep.simx_opmode_buffer)


        newImg = np.array(image, dtype=np.uint8)
        newImg.resize([resolution[0], resolution[1], 3])

        avg_color_per_row = np.average(newImg[128-64:128+64, 128-64:128+64  ], axis=0)
        avg_color = np.average(avg_color_per_row, axis=0)
        print('avg_color = ', avg_color)
            

        gray = cv2.cvtColor(newImg, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 3)
        image = newImg
        output = image.copy()

        # detect circles in the image
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 25)
        prev_amount = amount_of_bags 
        # ensure at least some circles were found
        if circles is not None:
            print('hey')

            # convert the (x, y) coordinates and radius of the circles to integers
            # circles = np.round(circles[0, :]).astype("int")
            circles = circles[0, :]
            r_max = 0
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                print ("r = " , r)
                print (len(circles))
                if r > r_max:
                    r_max = r
                    x_max = x
                    y_max = y
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle


            if abs(1 - last_r/r_max) > 0.1:
                amount_of_bags += 1
                bags.append(r)
            last_r = r_max


            if prev_amount != amount_of_bags:
                r_max = int(r_max)
                x_max = int(x_max)
                y_max = int(y_max)                

                cv2.circle(output, (x_max, y_max), r_max, (0, 255, 0), 4)
                cv2.rectangle(output, (x_max - 5, y_max - 5), (x_max + 5, y_max + 5), (0, 128, 255), -1)        
                name = 'output' + str(amount_of_bags) + '_' + str(len(circles)) + '.png'
                # show the output image
                cv2.imwrite(os.path.join(path , name), output[x_max-12:x_max+12, y_max-12:y_max+12])


if change(5,2) and x:
    print('oops')
else:
    print('wow')

list1 = [1, 3 ,5, 6,7 ,8 ]
list1 *= 0

print (list1)