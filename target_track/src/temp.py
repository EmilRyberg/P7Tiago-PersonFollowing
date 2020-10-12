import cv2 

webcam = cv2.VideoCapture(0)

qrDet = cv2.QRCodeDetector()

while True:
    check, frame = webcam.read()

    decodedText, points, _ = qrDet.detectAndDecode(frame)

    if points is not None:
        nrOfPoints = len(points)
        x = 0
        y = 0

        for i in range(nrOfPoints):
            nextPointIndex = (i+1) % nrOfPoints
            cv2.line(frame, tuple(points[i][0]), tuple(points[nextPointIndex][0]), (255,0,0), 5)
            x += points[i][0][0]
            y += points[i][0][1]
            
        x = int(x / 4) # cols
        y = int(y / 4) # rows
        print(x, y)

    cv2.imshow("Capturing", frame)
    cv2.waitKey(1)

