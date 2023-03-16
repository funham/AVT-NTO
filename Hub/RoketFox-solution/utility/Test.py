import cv2
import os

cap = cv2.VideoCapture(0)

i = 0
while True:
    i += 1
    ret, frame = cap.read()
    
    if not ret:
        print('\nError: Cannot read frame')
        break
    
    if i % 10 == 0:
        outfolder = "Images"
        os.makedirs(outfolder, exist_ok=True)
        outpath = os.path.join(outfolder, f"img{i}.png")
        print(outpath)
        cv2.imwrite(outpath, frame)

    key = cv2.waitKey(0)
    if key == 27 or key == ord('q'):
        break
cv2.destroyAllWindows()