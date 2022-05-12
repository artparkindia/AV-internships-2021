# -*- coding: utf-8 -*-
"""
Created on Thu May 27 12:26:08 2021

@author: Awies Mohammad Mulla
"""
import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import pandas as pd

#Set acceleration, deceleration etc. according to your convenience
class Car:
    def __init__(self, x, y, angle=0.0, length=4, max_steering=30, max_acceleration=5.0):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 20
        self.brake_deceleration = 10
        self.free_deceleration = 2

        self.acceleration = 0.0
        self.steering = 0.0

    def update(self, dt):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        if self.steering:
            turning_radius = self.length / sin(radians(self.steering))
            angular_velocity = self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        self.angle += degrees(angular_velocity) * dt


class Game:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Car tutorial")
        width = 789
        height = 491
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.ticks = 60
        self.exit = False

    def run(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "car_2.png")
        background = os.path.join(current_dir, "maps/map_2.png")
        car_image = pygame.image.load(image_path)
        bkg_image = pygame.image.load(background)
        #select initial car pose according to map
        #map_1:-
        #car = Car(7.42048, 14.0683, 58) 
        #map_2:-
        car = Car(11.7803, 14.5866, 90) 
        #map_3:-
        #car = Car(9.43958, 14.7214, 90)
        ppu = 32

        while not self.exit:
            dt = self.clock.get_time() / 1000

            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            # User input
            pressed = pygame.key.get_pressed()

            if pressed[pygame.K_UP]:
                if car.velocity.x < 0:
                    car.acceleration = car.brake_deceleration
                else:
                    car.acceleration += 1 * dt
            elif pressed[pygame.K_DOWN]:
                if car.velocity.x > 0:
                    car.acceleration = -car.brake_deceleration
                else:
                    car.acceleration -= 1 * dt
            elif pressed[pygame.K_SPACE]:
                if abs(car.velocity.x) > dt * car.brake_deceleration:
                    car.acceleration = -copysign(car.brake_deceleration, car.velocity.x)
                else:
                    car.acceleration = -car.velocity.x / dt
            else:
                if abs(car.velocity.x) > dt * car.free_deceleration:
                    car.acceleration = -copysign(car.free_deceleration, car.velocity.x)
                else:
                    if dt != 0:
                        car.acceleration = -car.velocity.x / dt
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))

            if pressed[pygame.K_RIGHT]:
                car.steering -= 30 * dt
            elif pressed[pygame.K_LEFT]:
                car.steering += 30 * dt
            else:
                car.steering = 0
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

            # Logic
            car.update(dt)

            # Drawing
            self.screen.blit(bkg_image, [0,0])
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            self.screen.blit(rotated, car.position * ppu - (rect.width / 2, rect.height / 2))
            pygame.display.flip()
            X.append(car.position[0])
            Y.append(car.position[1])
            phi.append(car.angle)

            self.clock.tick(self.ticks)
        print(car.position)
        print(car.angle)
        pygame.quit()


if __name__ == '__main__':
    #Compile the pose of car at every timestep
    X = []
    Y = []
    phi = []
    game = Game()
    game.run()
    #Transform the coordinates from pygame to that of appropriate maps
    for i in range(len(X)):
        X[i] = X[i] * (31.6)
        Y[i] = Y[i] * (32.7)
        phi[i] = phi[i]-90
    #Genrate a csv file to use for localization
    Readings = {'X' : X, 'Y':Y, 'phi':phi}
    data = pd.DataFrame(Readings)
    data.to_csv("Readings/Readings_2.csv")   #Do appropriate labelling corresponding to map used
    
    
