import cv2
camera=cv2.VideoCapture(0)
i = 0
while 1:
    (grabbed, img) = camera.read()
    cv2.imshow('img',img)
    if cv2.waitKey(1) & 0xFF == ord('j'):  # Press J to save image
        i += 1
        u = str(i)
        firename=str('./img'+u+'.jpg')
        cv2.imwrite(firename, img)
        print('writing：',firename)
    if cv2.waitKey(1) & 0xFF == ord('q'):   # Press Q to end
        break

