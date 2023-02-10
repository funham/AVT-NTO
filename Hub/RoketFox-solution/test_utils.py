from config import *


class ImageCapture:
    def __init__(self, path):
        self.path = path

    def read(self):
        path_list = [path for path in os.listdir(
            self.path) if path.endswith('.png')]
        img_idx = 0

        yield True, cv2.imread(path_list[img_idx])

        while True:
            key = cv2.waitKey(0)

            if key == 27:
                cv2.destroyAllWindows()
                yield False, None

            if key == ord('n'):
                img_idx += 1

            if key == ord('p'):
                img_idx -= 1

            img_idx = np.clip(img_idx, 0, len(path_list) - 1)

            cv2.destroyAllWindows()

            yield True, cv2.imread(f'{self.path}/{path_list[img_idx]}')
