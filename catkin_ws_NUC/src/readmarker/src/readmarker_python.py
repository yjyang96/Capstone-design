import numpy as np
import cv2
import cv2.aruco as aruco

cap = cv2.VideoCapture(1)

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH )
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT )

mtx = np.array([[636.792913,  0, 329.63907],[ 0, 635.580978, 238.191252],[ 0, 0, 1]])
dist = np.array([[0.028996, -0.136993, 0.002800, 0.000258, 0.00000]])


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

outImageCorners = np.array([[0, 0], [224, 0], [224, 224], [0, 224]])

while (True):
    ret, frame = cap.read()
    # operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)


    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    b_image1=0;                #Let image 1 is available
    b_image2=0;                #Let image 2 is available

    # width = frame.cols()         #Store Image's width, height in pixels
    # height = frame.rows()

    marker1X = [float('nan'),float('nan'),float('nan'),float('nan')]
    marker1Y = [float('nan'),float('nan'),float('nan'),float('nan')]
    marker2X = [float('nan'),float('nan'),float('nan'),float('nan')]
    marker2Y = [float('nan'),float('nan'),float('nan'),float('nan')]

    image_1 = np.zeros((224,224,3), np.uint8)
    image_2 = np.zeros((224,224,3), np.uint8)
    indice = {}
    aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers

    if np.all(ids != None):
        
        for i, id_ in enumerate(ids):
            indice[id_[0]-1] = i
            if id_[0] < 5:
                b_image1+=1
            else:
                b_image2+=1

        if b_image1 == 4:
            innerMarkerCorners = []
            innerMarkerCorners.append(corners[indice[0]][0][2])
            innerMarkerCorners.append(corners[indice[1]][0][3])
            innerMarkerCorners.append(corners[indice[2]][0][0])
            innerMarkerCorners.append(corners[indice[3]][0][1])
            for i in range(4):
                marker1X[i], marker1Y[i] = innerMarkerCorners[i]

            innerMarkerCorners = np.array(innerMarkerCorners)

            h1, status = cv2.findHomography(innerMarkerCorners, outImageCorners)
            image_1 = cv2.warpPerspective(frame, h1, (224, 224))

        if b_image2 == 4:
            innerMarkerCorners = []
            innerMarkerCorners.append(corners[indice[4]][0][2])
            innerMarkerCorners.append(corners[indice[5]][0][3])
            innerMarkerCorners.append(corners[indice[6]][0][0])
            innerMarkerCorners.append(corners[indice[7]][0][1])
            for i in range(4):
                marker2X[i], marker2Y[i] = innerMarkerCorners[i]

            innerMarkerCorners = np.array(innerMarkerCorners)

            h2, status = cv2.findHomography(innerMarkerCorners, outImageCorners)
            image_2 = cv2.warpPerspective(frame, h2, (224, 224))

    b_image1 = True if b_image1 == 4 else False
    b_image2 = True if b_image2 == 4 else False

    # Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.moveWindow('frame', 50, 20)
    
    showImg = np.concatenate((image_1, image_2), axis=0)
    cv2.imshow('out',showImg)
    cv2.moveWindow('out', int(width) + 50, 20)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()