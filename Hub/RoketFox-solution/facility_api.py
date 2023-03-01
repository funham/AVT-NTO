import RPi.GPIO as GPIO
import time


GPIO.setwarnings(False)

DIR1 = 20   # Direction GPIO Pin Mag
STEP1 = 21  # Step GPIO Pin Mag
DIR2 = 16   # Direction GPIO Pin  Car
STEP2 = 12  # Step GPIO Pin Car
DIR3 = 6   # Direction GPIO Pin Lenta
STEP3 = 5  # Step GPIO Pin Lenta
DIR4 = 7   # Direction GPIO Pin Lift
STEP4 = 8  # Step GPIO Pin Lift

CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 75   # Steps per Revolution (360 / 7.5)  328 - last
Servo = 17  # servo pin
in1 = 11   # GPIO pin led 1 last - 24
in2 = 9   # GPIO pin led 2 last - 23
in3 = 3   # GPIO pin led 1 last - 24
in4 = 4   # GPIO pin led 2 last - 23

End = False
Step = 0
StepMagnet = 0
StepMove = 0
place = 0
FirstHoarder = 2250
SecondHoarder = 1450
ThirdHoarder = 650
FourthHoarder = 0

GPIO.setmode(GPIO.BCM)

GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(STEP1, GPIO.OUT)
GPIO.output(DIR1, CW)

GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)
GPIO.output(DIR2, CW)

GPIO.setup(DIR3, GPIO.OUT)
GPIO.setup(STEP3, GPIO.OUT)
GPIO.output(DIR3, CW)

GPIO.setup(DIR4, GPIO.OUT)
GPIO.setup(STEP4, GPIO.OUT)
GPIO.output(DIR4, CW)


GPIO.setup(Servo, GPIO.OUT)

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)

GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)


GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm = GPIO.PWM(Servo, 50)

MODE = (13, 19, 26)   # Microstep Resolution GPIO Pins
GPIO.setup(MODE, GPIO.OUT)
RESOLUTION = {'Full': (0, 0, 0),
              'Half': (1, 0, 0),
              '1/4': (0, 1, 0),
              '1/8': (1, 1, 0),
              '1/16': (0, 0, 1),
              '1/32': (1, 0, 1)}
GPIO.output(MODE, RESOLUTION['1/8'])

step_count = SPR * 32
delay = .0208 / 32


def MagnetFullDown():
    '''
    Опустить магнит на уровень грузового отсека Айкара для захвата груза из грузового автономного автомобиля.
    В окне отладки выводит название функции, если функция успешно завершилась.
    '''
    global StepMagnet
    while StepMagnet <= 7090:
        GPIO.output(DIR1, CCW)
        GPIO.output(STEP1, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP1, GPIO.LOW)
        time.sleep(delay)
        StepMagnet += 1
    print('MagnetFullDown')
    StepMagnet = 0
    print(StepMagnet)


def MagnetHalfDown():
    '''
    Опустить магнит в специальное рабочее положение - на уровень накопителя для захвата груза из соответствующего накопителя или выгрузки очередного груза в соответствующий накопитель.
    В окне отладки выводит название функции, если функция успешно завершилась.
    '''
    global StepMagnet
    while StepMagnet <= 2000:
        GPIO.output(DIR1, CCW)
        GPIO.output(STEP1, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP1, GPIO.LOW)
        time.sleep(delay)
        StepMagnet += 1
    print('MagnetHalfDown')
    StepMagnet = 0
    print(StepMagnet)


def MagnetUp():
    '''
    Поднять магнит в верхнее положение, удовлетворяющее условию
    (кубик не должен задевать в процессе дальнейшего горизонтального передвижения конвейерную ленту или накопители).
    В окне отладки выводит название функции, если функция успешно завершилась
    '''
    global End
    global StepMagnet
    while End == False:
        MagnetUp = GPIO.input(25)
        GPIO.output(DIR1, CW)
        GPIO.output(STEP1, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP1, GPIO.LOW)
        time.sleep(delay)
        StepMagnet += 1
        if MagnetUp == False:
            print('MagnetUp')
            time.sleep(0.2)
            End = True
    End = False
    StepMagnet = 0
    print(StepMagnet)


def MagnetOn():
    '''
    Включить магнит при помощи подачи напряжения на него.
    Во включенном состоянии магнит способен захватывать груз для дальнейшего перемещения в пространстве.
    Функция MagnetOff служит для обратного действия, отключая действия магнита.
    '''
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)


def MagnetOff():
    '''
    Выключить магнит при помощи остановки подачи напряжения на него;
    в выключенном состоянии магнит не способен захватывать груз для дальнейшего перемещения в пространстве;
    иными словами, магнит находится в «спящем режиме», пока не будет вызвана функция MagnetOn
    '''
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)


def MoveTo(coords):
    '''
    Общая функция управления шаговыми двигателями каретки с магнитом для ее передвижения по горизонтальной оси, параллельной основе (упору) производственной фабрики на определенное количество шагов, определяющее конечную точку в траектории перемещения каретки.
    Используется в более конкретных функциях по передвижению: ToFirstHoarder, ToSecondHoarder, ToThirdHoarder, ToFourthHoarder
    '''
    global Step
    if Step > place:
        while Step >= place:
            GPIO.output(DIR4, CCW)
            GPIO.output(STEP4, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(STEP4, GPIO.LOW)
            time.sleep(delay)
            Step -= 1
    else:
        while Step <= place:
            GPIO.output(DIR4, CW)
            GPIO.output(STEP4, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(STEP4, GPIO.LOW)
            time.sleep(delay)
            Step += 1


def ToFirstHoarder():
    '''
    Cпециальная функция перемещения каретки с магнитом в точку, лежащую на одной вертикальной оси с первым накопителем, обладающим выталкивателем для отправления грузов на конвейерную ленту.
    В окне отладки выводит название функции, если функция успешно завершилась
    '''
    global FirstHoarder
    global place
    place = FirstHoarder
    MoveTo()
    print('ToFirstHoarder')
    print(Step)


def ToSecondHoarder():
    '''
    Cпециальная функция перемещения каретки с магнитом в точку, лежащую на одной вертикальной оси с вторым накопителем, обладающим выталкивателем для отправления грузов на конвейерную ленту.
    В окне отладки выводит название функции, если функция успешно завершилась
    '''
    global SecondHoarder
    global place
    place = SecondHoarder
    MoveTo()
    print('ToSecondHoarder')
    print(Step)


def ToThirdHoarder():
    '''
    Cпециальная функция перемещения каретки с магнитом в точку, лежащую на одной вертикальной оси с третьим накопителем, обладающим выталкивателем для отправления грузов на конвейерную ленту.
    В окне отладки выводит название функции, если функция успешно завершилась
    '''
    global ThirdHoarder
    global place
    place = ThirdHoarder
    MoveTo()
    print('ToThirdHoarder')
    print(Step)


def ToFourthHoarder():
    '''
    Cпециальная функция перемещения каретки с магнитом в точку, лежащую на одной вертикальной оси с четвертым накопителем, обладающим выталкивателем для отправления грузов на конвейерную ленту.
    В окне отладки выводит название функции, если функция успешно завершилась.
    '''
    global FourthHoarder
    global place
    place = FourthHoarder
    MoveTo()
    print('ToFourthHoarder')
    print(Step)


def CarriageStart():
    '''
    Функция перемещения каретки в стартовую зону для дальнейшего получения груза, прибывшего в зону выгрузки на Айкаре.
    В окне отладки выводит название функции, если функция успешно завершилась.
    '''
    global End
    global Step
    while End == False:
        CarriageStart = GPIO.input(23)
        GPIO.output(DIR4, CW)
        GPIO.output(STEP4, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP4, GPIO.LOW)
        time.sleep(delay)
        Step = Step + 1
        if CarriageStart == False:
            print('CarriageStart')
            time.sleep(0.2)
            End = True
    End = False
    print(Step)


def CarriageEnd():
    '''
    Функция перемещения каретки в конечную зону.
    В окне отладки выводит название функции, если функция успешно завершилась.
    '''
    global End
    global Step
    while End == False:
        CarriageEnd = GPIO.input(22)
        GPIO.output(DIR4, CCW)
        # for x in range(step_count):
        GPIO.output(STEP4, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP4, GPIO.LOW)
        time.sleep(delay)
        Step = Step + 1
        if CarriageEnd == False:
            print('CarriageEnd')
            time.sleep(0.2)
            End = True
    End = False
    print(End)
    print(Step)
    Step = 0


def LiftUp():
    '''
    Cпециальная функция перемещения лифта с корзиной для перемещения на площадку для посадки квадрокоптера и дальнейшей передачи этого груза квадрокоптеру, находящуюся на крыше башни.
    В окне отладки выводит название функции, если функция успешно завершилась.
    '''
    global End
    global StepMove
    while End == False:
        LiftDown = GPIO.input(18)
        GPIO.output(DIR3, CW)
        GPIO.output(STEP3, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP3, GPIO.LOW)
        time.sleep(delay)
        StepMove += 1
        if LiftDown == False:
            print('LiftDown')
            time.sleep(0.2)
            End = True
    End = False
    StepMove = 0
    print(StepMove)


def LiftDown():
    '''
    Специальная функция перемещения лифта с корзиной для передачи груза на нижний уровень
    для получения нового поступающего груза в пространство корзины
    '''
    global End
    global StepMove
    while End == False:
        LiftUp = GPIO.input(27)
        GPIO.output(DIR3, CCW)
        # for x in range(step_count):
        GPIO.output(STEP3, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP3, GPIO.LOW)
        time.sleep(delay)
        StepMove += 1
        if LiftUp == False:
            print('LiftUp')
            time.sleep(0.2)
            End = True
    End = False
    print(End)
    print(StepMove)
    StepMove = 0


def LentaUp():
    '''
    Функция перемещения конвейерной ленты.
    '''
    global StepMove
    while StepMove <= 5000:
        GPIO.output(DIR2, CW)
        # for x in range(step_count):
        GPIO.output(STEP2, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP2, GPIO.LOW)
        time.sleep(delay)
        StepMove += 1
    print(StepMove)
    StepMove = 0


def Servo():
    '''
    Функция перемещения выталкивателя для отправки грузов из первого накопителя на конвейерную ленту.
    В зависимости от типа производимых действий (втягивание и выталкивание)
    в качестве отладки могут прийти следующие ключевые слова: ’ServoBackward’ и ’ServoForward’
    '''
    pwm.start(0)
    global End
    pwm.ChangeDutyCycle(5)
    while End == False:
        ServoForward = GPIO.input(10)
        if ServoForward == False:
            End = True
            print('ServoForward')
    End = False
    pwm.ChangeDutyCycle(10)
    while End == False:
        ServoBackward = GPIO.input(24)
        if ServoBackward == False:
            End = True
            print('ServoBackward')

    End = False
    print(End)


MagnetUp()
CarriageEnd()
CarriageStart()
MagnetUp()
MagnetFullDown()
MagnetOn()
MagnetUp()
ToFirstHoarder()
MagnetHalfDown()
MagnetOff()
MagnetUp()
Servo()
LiftDown()
LentaUp()
LiftUp()

GPIO.cleanup()
