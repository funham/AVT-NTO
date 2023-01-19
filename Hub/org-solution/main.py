import numpy as np
import cv2
import math
import typing

from dataclasses import dataclass
from itertools import product, combinations
from functools import cached_property, lru_cache

@np.vectorize
def dist(p1, p2):
    '''Поиск расстояния между точками в `ℝⁿ`'''
    diff = (a - b for a, b in zip(p1, p2))
    return math.sqrt(np.sum(np.square(diff)))

def nearest_color(color):
    colors = {
        0: (255, 255, 255),  # white
        1: (40, 40, 40),  # black
        2: (95, 95, 230),  # red
        3: (80, 160, 250),  # orange
        4: (70, 240, 235),  # yellow
        5: (120, 200, 0),  # green
        6: (255, 205, 0),  # cyan
        7: (210, 120, 10),  # blue
        8: (225, 150, 90),  # violet
        9: (205, 145, 215) # magenta
    }  

    _, code = min((dist(color, colors.values()), colors.keys()))
    return code


def mean_color(img, m, n):
    '''mean color of `img` in `10`x`10` at `m`th, `n`th quadrant'''
    y, x = 45+m*100, 45+n*100
    return np.mean(img[y:y+10, x:x+10])


@dataclass(frozen=True, eq=True, order=True)
class CubeFace:
    area: float
    box: typing.Any
    bad: bool = False

    @cached_property
    def mask(self):
        m = np.zeros(frame.shape[:2])
        cv2.drawContours(m, [self.box], -1, 1, -1, cv2.LINE_AA)
        return m


@lru_cache
def cnt2face(cnt):
    '''Строит `CubeFace object` по контуру, если возможно, иначе `None`'''
    rect = cv2.minAreaRect(cnt)
    box = np.int0(cv2.boxPoints(rect))
    area = int(rect[1][0] * rect[1][1])
    side1, side2 = dist((box[0], box[1]), (box[1], box[2]))

    if area > 2500 and abs(side1 - side2) < min(side1, side2):
        return CubeFace(area, box)


def mark_colors(img, box):
    '''
    Растягиваем изображение груза в куб и находим цвета четырех областей,
    соответсвующих центрам квадратов цветовой маркировки
    '''
    dst = np.float32([(0, 0), (0, 200), (200, 200), (200, 0)])
    transform = cv2.getPerspectiveTransform(box.astype(np.float32), dst)
    face = cv2.warpPerspective(img, transform, (200, 200))

    return (mean_color(face, m, n) for m, n in product((1, 2), repeat=2))


def get_actual_marking(marking: list):
    '''"rotates" marking, till it gets correct'''

    for i in (0, 1, 3, 2):
        main = marking[i]
        aligned = marking[(i + 1) % 4]
        control = marking[(i - 1) % 4]
        remainder = marking[(i + 2) % 4]

        if main + aligned == 11 and main + control != 11:
            return (main, aligned, control, remainder)           
       
def main():
    frame = cv2.resize(frame, (640, 480))
    frame = frame[int(frame.shape[0] * 0.17):-int(frame.shape[0] * 0.1),
                int(frame.shape[1] * 0.27):-int(frame.shape[1] * 0.35)]

    blur = cv2.medianBlur(frame, 5)
    canny = cv2.Canny(blur, 50, 250)

    contours, _ = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    faces = list([cnt2face(cnt) for cnt in contours if cnt2face(cnt)])

    # Сравниваем друг с другом все маски верхних сторон кубов и оставляем только самые крупные,
    # на которые не накладываются более крупные маски
    for f1, f2 in combinations(faces, 2):
        if (f1.mask.flatten() @ f2.mask.flatten()) / np.sum(f1.mask) > 0.2:
                min(f1, f2).bad = True

    faces = (f for f in faces if not f.bad)

    for face in faces:
        marking = [nearest_color(c) for c in mark_colors(frame, face.box)]
        marking = get_actual_marking(marking)

        # Орги не прописали искомую маркировку и поиск координат груза
        if marking == TARGET_MARKING:
            loadOnDrone(coords)

if __name__ == '__main__':
    main()
