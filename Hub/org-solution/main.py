import numpy as np
import cv2
import math
from functools import cached_property


def dist(p1, p2):
    '''Поиск расстояния между двумя точками в `ℝⁿ`'''
    diff = (a - b for a, b in zip(p1, p2))
    return math.sqrt(np.sum(np.square(diff)))


vdist = np.vectorize(dist)


def nearest_color(color):
    colors = {0: (255, 255, 255),  # white
              1: (40, 40, 40),  # black
              2: (95, 95, 230),  # red
              3: (80, 160, 250),  # orange
              4: (70, 240, 235),  # yellow
              5: (120, 200, 0),  # green
              6: (255, 205, 0),  # cyan
              7: (210, 120, 10),  # blue
              8: (225, 150, 90),  # violet
              9: (205, 145, 215)}  # magenta

    _, code = min((vdist(color, colors.values()), colors.keys()))
    return code


frame = cv2.resize(frame, (640, 480))
# обрезаем кадр так, чтобы остался только контейнер с грузами
frame = frame[int(frame.shape[0] * 0.17):-int(frame.shape[0] * 0.1),
              int(frame.shape[1] * 0.27):-int(frame.shape[1] * 0.35)]

# Применяем размытие для более однородных цветов грузов
blur = cv2.medianBlur(frame, 5)
# Детектор Кенни находит прямые лини, находи контуры на изображении с линиями
canny = cv2.Canny(blur, 50, 250)

contours = cv2.findContours(
    canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

contours = contours[0] if len(contours) == 2 else contours[1]
# Перебираем контуры и проверям их площадь и соотношение сторон.
# Если они подходят под квадрат - сохраняем их в список


class CubeFace:
    def __init__(self, box, area):
        self.box = box
        self.area = area
        self.bad = False

    @cached_property
    def mask(self):
        mask = np.zeros(frame.shape[:2])
        cv2.drawContours(mask, [self.box], -1, 1, -1, cv2.LINE_AA)
        return mask

    def __lt__(self, other):
        return self.area < other.area


def square_cnts(cnt):
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    area = int(rect[1][0] * rect[1][1])
    side1, side2 = dist(box[0], box[1]), dist(box[1], box[2])

    if area > 2500 and abs(side1 - side2) < min(side1, side2):
        mask = np.zeros((frame.shape[0], frame.shape[1], 1))
        cv2.drawContours(mask, [box], -1, 1, -1, cv2.LINE_AA)

        return CubeFace(box, area)


faces = list(filter(lambda x: x is not None, [
    square_cnts(cnt) for cnt in contours]))


# Сравниваем друг с другом все маски и оставляем только самые крупные,
# на которые не накладываются более крупные маски
bad = set()
for i, f1 in enumerate(faces):
    for j, f2 in enumerate(faces[i+1:], i+1):
        if (f1.mask @ f2.mask) / np.sum(f1.mask) > 0.2:
            min(f1, f2).bad = True

faces = filter(lambda f: not f.bad, faces)


'''
for i in range(len(bboxes)):
    for j in range(i + 1, len(bboxes)):
        if np.any(cv2.bitwise_and(masks[i], masks[j])) and (masks[i] @ masks[j]) / np.sum(masks[i]) > 0.2:
            if areas[i] < areas[j]:
                bad.append(j)
            else:
                bad.append(i)

    for j in range(len(main_masks)):
        if np.sum(main_masks[j] * masks[i]) / np.sum(main_masks[j]) > 0.2:
            bad.append(i)
            break

    if i not in bad:
        main_masks.append(masks[i])
        main_bboxes.append(bboxes[i])
    else:
        ...'''

# Преобразуем части исходного изображения закрывающиеся масками в квадраты
for _, face in enumerate(faces):
    # Для каждого полученного изображения груза, растянутого в квадрат,
    # находим цвета четырех точек, соответствующим центрам квадров цветовой маркировки
    # переводим найденные цвета в цифры, которые они кодируютbbox = face.box
    dst = np.float32([(0, 0), (0, 200), (200, 200), (200, 0)])
    transform = cv2.getPerspectiveTransform(bbox.astype(np.float32), dst)
    face_img = cv2.warpPerspective(frame.copy(), transform, (200, 200))

    cur_colors = []
    for m in range(2):
        for n in range(2):
            point = face_img[50 + n * 100, 50 + m * 100]
            cur_colors.append(nearest_color(point))
# Таким образом, мы имеем координаты всех грузов и соответствующие им цифры.
# Осталось определить верный порядок расположения цифр.
    for color in cur_colors:
        if color[0] + color[1] == 11 and color[0] + color[2] != 11:
            final_color = color
        elif color[1] + color[3] == 11 and color[1] + color[0] != 11:
            final_color = [color[1], color[3], color[0], color[2]]
        elif color[3] + color[2] == 11 and color[3] + color[1] != 11:
            final_color = [color[3], color[2], color[1], color[0]]
        elif color[2] + color[0] == 11 and color[2] + color[3] != 11:
            final_color = [color[2], color[0], color[3], color[1]]
        else:
            final_color = color
            final_color = '#' + ''.join(hex(x) for x in final_color)

            all_colors.append(final_color)
            all_coords.append(coords)
