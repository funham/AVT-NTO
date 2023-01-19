import numpy as np
import cv2
import math

# поиск расстояния между двумя точками на плоскости
def find_distance_2(pts1, pts2):
    return math.sqrt(abs(pts1[0] - pts2[0]) ** 2 + abs(pts1[1] - pts2[1]) ** 2)

# поиск расстояния между двумя точками в трехмерном пространстве
def find_distance_3(pts1, pts2):
    return math.sqrt(abs(pts1[0] - pts2[0]) ** 2 + abs(pts1[1] - pts2[1]) ** 2 + abs(pts1[2] - pts2[2]) ** 2)

def find_color(color):
    # позволяет найти ближайший к данному на вход, цвет из словаря
    # цвета воспринимаются как точки в трехмерном пространстве
    # возвращает номер цвета
    colors = {
        0: (255, 255, 255), # white
        1: (40, 40, 40), # black
        2: (95, 95, 230), # red
        3: (80, 160, 250), # orange
        4: (70, 240, 235), # yellow
        5: (120, 200, 0), # green
        6: (255, 205, 0), # cyan
        7: (210, 120, 10), # blue
        8: (225, 150, 90), # violet
        9: (205, 145, 215) # magenta
    } 

    cur_col, minim = None, np.inf
    
    for num, col in colors.items():
        dist = find_distance_3(color, col)

        if dist < minim:
            minim = dist
            cur_col = num
    
    return cur_col

frame = cv2.resize(frame, (640, 480))

# обрезаем кадр так, чтобы остался только контейнер с грузами
frame = frame[int(frame.shape[0] * 0.17):-int(frame.shape[0] * 0.1), 
              int(frame.shape[1] * 0.27):-int(frame.shape[1] * 0.35)]

# Применяем размытие для более однородных цветов грузов
blur = cv2.medianBlur(frame, 5)
canny = cv2.Canny(blur, 50, 250)

# Детектор Кенни находит прямые лини, находи контуры на изображении с линиями
contours = cv2.findContours(canny.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]

# Перебираем контуры и проверям их площадь и соотношение сторон.
# Если они подходят под квадрат - сохраняем их в список
bboxes, areas = [], []
for cnt in contours:
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    area = int(rect[1][0] * rect[1][1])
    side1, side2 = find_distance_2(box[0], box[1]), find_distance_2(box[1], box[2])
    
    if area > 2500 and side1 * 0.5 < side2 < side1 * 1.5:
        bboxes.append(box)
        areas.append(area)

# Создаем список масок - соответствующих выбранным контурам
masks = []
for i in range(len(bboxes)):
    mask = np.zeros((frame.shape[0], frame.shape[1], 1))
    cv2.drawContours(mask, [bboxes[i]], -1, 1, -1, cv2.LINE_AA)
    masks.append(mask)

    # Сравниваем друг с другом все маски и оставляем только самые крупные,
    # на которые не накладываются более крупные маски

    bad = []
    for i in range(len(bboxes)):
        for j in range(i + 1, len(bboxes)):
            if np.any(cv2.bitwise_and(masks[i], masks[j])) and np.sum(masks[i] * masks[j]) / np.sum(masks[i]) > 0.2:
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
        break

# Преобразуем части исходного изображения закрывающиеся масками в квадраты
for k, bbox in enumerate(main_bboxes):
    dst = np.float32([(0, 0), (0, 200), (200, 200), (200, 0)])
    transform = cv2.getPerspectiveTransform(bbox.astype(np.float32), dst)
    new_image = cv2.warpPerspective(frame.copy(), transform, (200, 200))
    
    # Для каждого полученного изображения груза, растянутого в квадрат,
    # находим цвета четырех точек, соответствующим центрам квадров цветовой маркировки
    # переводим найденные цвета в цифры, которые они кодируют
    
    cur_colors = []
    for m in range(2):
        for n in range(2):
            point = new_image[50 + n * 100, 50 + m * 100]
            cur_colors.append(find_color(point))

    for color in cur_color:
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
            final_color = ''.join(str(x) for x in final_color)

        all_colors.append(final_color)
        all_coords.append(coords)
