import os
import cv2


class ImageCapture:
    def __init__(self, path):
        self.path = path
        self.reader = self.__reader()

    def read(self):
        return next(self.reader)

    def __reader(self):
        path_list = [path for path in os.listdir(
            self.path) if path.endswith('.png') or path.endswith('.jpg')]

        img_idx = 0
        path = f'{self.path}\{path_list[img_idx]}'

        yield True, cv2.imread(path)

        while True:
            key = cv2.waitKey(0)

            if key == 27 or key == ord('q'):
                cv2.destroyAllWindows()
                yield False, None

            if key == ord('n'):
                img_idx += 1

            if key == ord('p'):
                img_idx -= 1

            img_idx = img_idx % len(path_list)

            path = f'{self.path}\{path_list[img_idx]}'
            img = cv2.imread(path)
            yield True, img
