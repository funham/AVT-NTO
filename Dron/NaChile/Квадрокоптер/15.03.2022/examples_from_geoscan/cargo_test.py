#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gs_module import CargoController
from time import sleep

cargo = CargoController() # создаем объект магнитного захвата
cargo.on() # включаем магнит
cargo.changeColor(255, 0, 0, 0) # меняем цвет 0-го светодиода на красный, первыми тремя аргументами передаем цвет светодиода в RGB, четвертым аргументом номер светодиода от 0 до 3
sleep(2) # ожидаем 2 секунды
cargo.changeAllColor(0, 255, 0) # меняем цвет всех светодиодов на зеленый
sleep(3) # ожидаем 3 секунды
cargo.off() # выключаем магнит
cargo.changeAllColor(0, 0, 0) # выключаем все лампочки