# main script

import sim as vrep
import time
import math
# image processing
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os

global x, y, z, start_stream
global bags, amount_of_bags, last_r, result
global path
global low_green, low_yellow, low_purple
global high_green, high_yellow, high_purple
global colour_defined, colours, current_colour

start_stream = False
bags = []
amount_of_bags = 0
last_r = 0
path = '/home/evgesha/nti/copter_proj/imgs'
result = []

low_green = (76, 152, 0)
high_green = (145, 255, 24)

low_yellow = (55, 150, 150)
high_yellow = (116, 255, 255)

low_purple = (240, 135, 240)
high_purple = (255, 145, 255) 

colour_defined = False
colours = ['green', 'yellow', 'purple']
current_colour = None

check = {   'green': lambda x: is_green(x), 
            'yellow': lambda x: is_yellow(x), 
            'purple': lambda x: is_purple(x)  }

def go_to_x(x1, action = None):
    global x, y, z, start_stream

    err, Qpos = vrep.simxGetObjectPosition(
    clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)

    x = Qpos[0]
    y = Qpos[1]
    z = Qpos[2]

    if x < x1:
        sign = -1
    elif x > x1:
        sign = 1
    else:
        return 0

    while abs(x - x1) > 0.02 and len(result) < 3:
        err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (x, y, z), vrep.simx_opmode_oneshot_wait)
        detect_bag(action)
        time.sleep(0.2)
        x -= sign*0.02
        print(x)
        print('\n')
    

    err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (x1, y, z), vrep.simx_opmode_oneshot_wait)

    time.sleep(0.2)
    err, Qpos = vrep.simxGetObjectPosition(
    clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
    print(Qpos)

    x = x1
    start_stream = False


def go_to_y(y1):
    global x, y, z

    err, Qpos = vrep.simxGetObjectPosition(
    clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)

    x = Qpos[0]
    y = Qpos[1]
    z = Qpos[2]

    if y < y1:
        sign = -1
    elif y > y1:
        sign = 1
    else:
        return 0

    while abs(y - y1) > 0.01:
        err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (x, y, z), vrep.simx_opmode_oneshot_wait)
        time.sleep(0.2)
        y -= sign*0.01
        print(y)
        print('\n')

    err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (x, y1, z), vrep.simx_opmode_oneshot_wait)

    time.sleep(0.2)
    err, Qpos = vrep.simxGetObjectPosition(
    clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
    print(Qpos)

    y = y1

def go_to_z(z1):
    global x, y, z

    err, Qpos = vrep.simxGetObjectPosition(
    clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)

    x = Qpos[0]
    y = Qpos[1]
    z = Qpos[2]

    if z < z1:
        sign = -1
    elif z > z1:
        sign = 1
    else:
        return 0

    while abs(z - z1) > 0.01:
        err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (x, y, z), vrep.simx_opmode_oneshot_wait)
        time.sleep(0.2)
        z -= sign*0.01
        print(z)
        print('\n')
        # returnCode, linearVelocity, angularVelocity=vrep.simxGetObjectVelocity(clientID, QuadricopterT, vrep.simx_opmode_streaming)
        # returnCode, linearVelocity, angularVelocity=vrep.simxGetObjectVelocity(clientID, QuadricopterT, vrep.simx_opmode_buffer)

    err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (x, y, z1), vrep.simx_opmode_oneshot_wait)

    time.sleep(0.2)
    err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
    print(Qpos)
    z = z1



def circle_move(radius, height):

    go_to_z(height)

    err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)

    x = Qpos[0]
    y = Qpos[1]
    z = Qpos[2]

    time.sleep(2)
    theta = 0
    go_to_x(x + radius*np.cos(theta))
    while True:
        err = vrep.simxSetObjectPosition(
        clientID, QuadricopterT, -1, (x + radius*np.cos(theta), y + radius*np.sin(theta), z), vrep.simx_opmode_oneshot_wait)
        time.sleep(0.01)
        theta += np.pi / 360
        err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
        print(Qpos[0]**2 + Qpos[1]**2)


def square_move(square, height):

    err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)

    x = Qpos[0]
    y = Qpos[1]
    z = Qpos[2]

    go_to_z(height)
    # go_to_x(np.sqrt(square))
    # go_to_y(0)

    time.sleep(2)
    theta = 0
    while True:

        err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
        print(Qpos[0], Qpos[1])
        go_to_y(y - np.sqrt(square))

        err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
        print(Qpos[0], Qpos[1])
        go_to_x(x + np.sqrt(square))

        err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
        print(Qpos[0], Qpos[1])
        go_to_y(y)

        err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
        print(Qpos[0], Qpos[1])
        go_to_x(x)
        


def spiral_move(radius, height):
    
    err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)

    x = Qpos[0]
    y = Qpos[1]
    z = Qpos[2]

    go_to_z(height)

    z = height

    theta = 0
    go_to_x(x + radius*np.cos(theta))

    time.sleep(2)
    theta = 0
    while True:
        err = vrep.simxSetObjectPosition(
        clientID, QuadricopterT, -1, (x + radius*np.cos(theta), y + radius*np.sin(theta), z), vrep.simx_opmode_oneshot_wait)
        time.sleep(0.01)
        theta += np.pi / (90*radius + 1)
        radius /= 1.001
        err, Qpos = vrep.simxGetObjectPosition(clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
        print(Qpos[0]**2 + Qpos[1]**2)


def detect_bag(action = None):

    global start_stream, amount_of_bags, bags, last_r, result
    global low_green, low_yellow, low_purple
    global high_green, high_yellow, high_purple
    global colour_defined

    if action == 'detect':

        if start_stream:
            err, resolution, image = vrep.simxGetVisionSensorImage(
                                    clientID, cameraID, 0, vrep.simx_opmode_buffer)
        else:
            start_stream = True
            colour_defined = False

            err, resolution, image = vrep.simxGetVisionSensorImage(
                                    clientID, cameraID, 0, vrep.simx_opmode_streaming)
            # read image
            time.sleep(0.1)
            err, resolution, image = vrep.simxGetVisionSensorImage(
                                    clientID, cameraID, 0, vrep.simx_opmode_buffer)


        newImg = np.array(image, dtype=np.uint8)
        newImg.resize([resolution[0], resolution[1], 3])
        # print('avg_color = ', avg_color)
            

        gray = cv2.cvtColor(newImg, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 3)
        # image = newImg
        output = newImg
        # print(output.shape)

        # detect circles in the image
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2.1, 25)
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
            amount_of_bags += 1
            bags.append(r_max)
            


            if prev_amount != amount_of_bags:
                r_max = int(r_max)
                x_max = int(x_max)
                y_max = int(y_max)                

                # cv2.circle(output, (x_max, y_max), r_max, (0, 255, 0), 4)
                # cv2.rectangle(output, (x_max - 5, y_max - 5), (x_max + 5, y_max + 5), (0, 128, 255), -1)  
                sector = output[y_max-5:y_max+5, x_max-5:x_max+5]    
                avg_color_per_row = np.average(sector, axis=0)
                avg_color = np.average(avg_color_per_row, axis=0)
                # print(avg_color, type(avg_color))

                if colour_defined and is_colour_changed(sector):
                    result.append(max(bags[:-1]))
                    bags = [bags[-1]]
                    print(result)
                    print('colour changed!', current_colour)
                    if len(result) == 2:
                        result.append(bags[-1])
                else:
                    is_colour_changed(sector)
                    
            

                avg_color_per_row = np.average(sector, axis=0)
                avg_color = np.average(avg_color_per_row, axis=0)

                name = str(amount_of_bags) + '____' + str(avg_color) + '.png'
                # show the output image
                cv2.imwrite(os.path.join(path , name), sector)




def is_colour_changed(sector):

    if current_colour is None:
        colour_defined = False
        return False

    if check[current_colour](sector):
        return False
    else:
        return True
        

def detect_colour(sector):

    global current_colour, colour_defined, colours

    colour_defined = True

    if is_green(sector):
        current_colour = colours[0]
        return current_colour

    elif is_yellow(sector):
        current_colour = colours[1]
        return current_colour

    elif is_purple(sector):
        current_colour = colours[2]
        return current_colour

    current_colour = None
    colour_defined = False


def is_green(sector):
    green_mask = cv2.inRange(sector, low_green, high_green)
    if abs(get_avrg_colour(green_mask) - 255) <= 10:
        return True
    else:
        return False

def is_yellow(sector):
    yellow_mask = cv2.inRange(sector, low_yellow, high_yellow)
        if abs(get_avrg_colour(yellow_mask) - 255) <= 10:
            return True
        else:
            return False

def is_purple(sector):
    purple_mask = cv2.inRange(sector, low_purple, high_purple)
            if abs(get_avrg_colour(purple_mask) - 255) <= 10:
                return True
            else:
                return False







def get_avrg_colour(sector):
    avg_color_per_row = np.average(sector, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)
    print(avg_color)
    return avg_color


rad = math.radians

# just in case, close all opened connections
vrep.simxFinish(-1)

# connect to V-REP
# port is in remoteApiConnections.txt
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID != -1:
    print('Connected to remote API server')

# get handles
err, QuadricopterT = vrep.simxGetObjectHandle(
    clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("No Quadricopter")

err, Quadricopter = vrep.simxGetObjectHandle(
    clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("No Quadricopter")

err, cameraID = vrep.simxGetObjectHandle(
    clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("No camera")


err, frontCameraID = vrep.simxGetObjectHandle(
    clientID, 'Quadricopter_frontCamera', vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("No front camera")

# start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)


# Example: position / orientation
err, pos = vrep.simxGetObjectPosition(
    clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("Can't get position")
print("Position:", pos)

err, orient = vrep.simxGetObjectOrientation(
    clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("Can't get orientation")
print("Orientation:", orient)


err, Qpos = vrep.simxGetObjectPosition(
    clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)


print('Qpos' , Qpos)

x = Qpos[0]
y = Qpos[1]
z = Qpos[2]


go_to_y(0.55)
go_to_z(0.43)
go_to_x(0.47, 'detect') 

print('result =' , result)

# err = vrep.simxSetObjectOrientation(
#     clientID, QuadricopterT, QuadricopterT, (0, 0, rad(10)), vrep.simx_opmode_oneshot_wait)
# time.sleep(4)


err, Qpos = vrep.simxGetObjectPosition(
    clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)

print('Qpos =' , Qpos)



# Throw ball
# emptyBuff = bytearray()
# res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'Quadricopter', vrep.sim_scripttype_childscript,
#                                                                              'ThrowBallFunction', [], [], [""], emptyBuff, vrep.simx_opmode_blocking)
# time.sleep(1)


# # Example: image
# # open streaming
# err, resolution, image = vrep.simxGetVisionSensorImage(
#     clientID, cameraID, 0, vrep.simx_opmode_streaming)
# # read image
# time.sleep(0.1)
# err, resolution, image = vrep.simxGetVisionSensorImage(
#     clientID, cameraID, 0, vrep.simx_opmode_buffer)
# # convert byte array to image
# newImg = np.array(image, dtype=np.uint8)
# newImg.resize([resolution[0], resolution[1], 3])

# gray = cv2.cvtColor(newImg, cv2.COLOR_BGR2GRAY)
# image = newImg
# output = image.copy()

# # detect circles in the image
# circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 20)
# print(circles)
 
# # ensure at least some circles were found
# if circles is not None:
#     print('hey')
#     # convert the (x, y) coordinates and radius of the circles to integers
#     circles = np.round(circles[0, :]).astype("int")
 
#     # loop over the (x, y) coordinates and radius of the circles
#     for (x, y, r) in circles:
#         # draw the circle in the output image, then draw a rectangle
#         # corresponding to the center of the circle
#         cv2.circle(output, (x, y), r, (0, 255, 0), 4)
#         cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
 
#     # show the output image
#     cv2.imwrite("output1.png", np.hstack([image, output]))




# # save
# cv2.imwrite('image.png', image)
# print('Image is saved')
# gr = cv2.imread('image.png')
# time.sleep(1)

# finish work
vrep.simxFinish(clientID)
