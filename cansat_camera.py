import cv2

cap = cv2.VideoCapture(0)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

fourcc  = cv2.VideoWriter_fourcc(*'XVID')

writer = cv2.VideoWriter('output.avi',fourcc,30.0,(width,height))

e1 = cv2.getTickCount()
while True:
    ret,img_color = cap.read()
    
    if ret == False:
        break
    cv2.imshow("Color",img_color)

    writer.write(img_color)
    
    e2 = cv2.getTickCount()
    time = (e2-e1)/cv2.getTickFrequency()

    if cv2.waitKey(1)&0xFF == 27:
        break
    if time > 60*50:
      break 
        
cap.release()
writer.release()
cv2.destroyAllWindows()
