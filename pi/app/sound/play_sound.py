import pygame
import time

BASE_PATH = "/home/pi/Code/Roshis-car/pi/app/sounds/"

pygame.mixer.init()
pygame.mixer.music.load(BASE_PATH + "error.mp3")
pygame.mixer.music.play()

print("This runs immediately while audio plays")
time.sleep(3)  # keep process alive