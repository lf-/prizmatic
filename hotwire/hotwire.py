import pygame
import ctypes
import serial.tools.list_ports

c_uint8 = ctypes.c_uint8


class KeyInputInside(ctypes.LittleEndianStructure):
    _fields_ = [
        ("w", c_uint8, 1),
        ("a", c_uint8, 1),
        ("s", c_uint8, 1),
        ("d", c_uint8, 1),
        ("signal", c_uint8, 1),
    ]


class KeyInput(ctypes.Union):
    _fields_ = [("b", KeyInputInside), ("asbyte", c_uint8)]


pygame.init()
pygame.display.set_mode([100, 100])
done = False
clock = pygame.time.Clock()
out = KeyInput()
ser = serial.tools.list_ports.comports()[0].device
s = serial.Serial(ser, 9600)

while not done:
    clock.tick(20)
    for evt in pygame.event.get():
        if evt.type == pygame.QUIT:
            done = True
        elif evt.type == pygame.KEYDOWN or evt.type == pygame.KEYUP:
            if evt.key == pygame.K_ESCAPE:
                done = True
            keys = pygame.key.get_pressed()
            out.b.w = keys[pygame.K_s]
            out.b.a = keys[pygame.K_d]
            out.b.s = keys[pygame.K_w]
            out.b.d = keys[pygame.K_a]
            out.b.signal = keys[pygame.K_SPACE]
            # print(out.asbyte)
            s.write(bytes((out.asbyte,)))
    # print(s.read_all())
pygame.quit()

