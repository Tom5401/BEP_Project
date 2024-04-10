import os
import pygame
import numpy as np
from math import sin, radians, degrees, copysign
from pygame.math import Vector2


class Car:
    def __init__(self, x, y, angle=np.pi/2, length=0.2, max_steering=radians(20), max_acceleration=5.0):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 10.0
        self.free_deceleration = 0.805398773006135
        self.acceleration = 0.0
        self.steering = 0.0
        self.throttle = 0.0
        self.steering_input = 0.0
        self.point = [0,0]
        self.npoint = 0
        d = 1.5
        a = d/np.sqrt(2)
        b = 0.5*(d-a)
        self.points = np.array([[0,0],
            [b,a/2],
            [b+a,a/2],
            [b+a,-a/2],
            [b,-a/2],
            [0,0],
            [-b,a/2],
            [-(b+a),a/2],
            [-(b+a),-a/2],
            [-b,-a/2],
            [0,0]])

    def nextPoint(self):
        distP = np.sqrt((self.position.y-self.point[1])**2+(self.position.x-self.point[0])**2)
        if distP < 0.3:
            if self.npoint < (len(self.points)-1):
                self.npoint = self.npoint+1
                self.point = self.points[self.npoint]
            else:
                self.npoint = 0
                self.point = self.points[self.npoint]


    def throttleController(self,pressed):
        if pressed[pygame.K_UP]:
            self.throttle += 0.002
        elif pressed[pygame.K_DOWN]:
            self.throttle -= 0.002


    def steeringController(self):
        alpha = np.arctan2((self.position.y-self.point[1]),(self.position.x-self.point[0]))
        theta = alpha - self.angle - np.pi
        if theta > np.pi:
            theta = theta - 2*np.pi
        if theta < -np.pi:
            theta = theta + 2*np.pi
        self.steering_input = theta*1.5
        if abs(self.steering_input) > 1.0:
            self.steering_input = self.steering_input/abs(self.steering_input)
        info = 'rotation:',str(self.angle),'alpha:',str(alpha),'theta:',str(theta),'steering:',str(self.steering)
        #print(info)            


    def update(self, dt):
         # User input
        pressed = pygame.key.get_pressed()

        self.nextPoint()
        print(self.npoint,self.point)
        self.throttleController(pressed)
        self.steeringController()


        if self.throttle > 0.10 and self.velocity.x >= 0:
            self.acceleration = (-2*abs(self.velocity.x) + 55*self.throttle- 5.25)/1.630
        elif self.throttle < -0.10 and self.velocity.x <= 0:
            self.acceleration = -(-2*abs(self.velocity.x) + 55*abs(self.throttle) - 5.25)/1.630
        else:
            if abs(self.velocity.x) > dt * self.free_deceleration:
                self.acceleration = -copysign(self.free_deceleration, self.velocity.x)
            else:
                if dt != 0:
                    self.acceleration = -self.velocity.x / dt
        self.acceleration = max(-self.max_acceleration, min(self.acceleration, self.max_acceleration))

        self.steering = self.steering_input * self.max_steering
        self.steering = max(-self.max_steering, min(self.steering, self.max_steering))

        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        self.free_deceleration = (0.42571*abs(self.velocity.x) + 1.3128)/1.630

        if self.steering:
            turning_radius = self.length / sin(self.steering)
            angular_velocity = self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        self.position += self.velocity.rotate(degrees(self.angle)) * dt
        self.angle += angular_velocity * dt
        if self.angle > np.pi:
            self.angle = self.angle - 2*np.pi
        elif self.angle < -np.pi:
            self.angle = self.angle + 2*np.pi
        else:
            pass

class Game:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Jetracer simulation")
        self.white = (255, 255, 255)
        self.width = 1280
        self.height = 720
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('arial', 15)
        self.ticks = 60
        self.exit = False

    def center_origin(self, p):
        return (p[0] + self.width // 2, p[1] + self.height // 2)

    def run(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "car.png")
        car_image = pygame.image.load(image_path)
        car = Car(0, 0)
        ppu = 200
        x_bound = 2.8
        y_bound = 1.3
        border = pygame.Rect(self.width/2 - ppu*x_bound, self.height/2 - ppu*y_bound, ppu*x_bound*2, ppu*y_bound*2)

        while not self.exit:
            dt = self.clock.get_time() / 1000

            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            # Logic
            car.update(dt)

            # Drawing screen
            self.screen.fill((0, 0, 0))
            pygame.draw.rect(self.screen, self.white, border,  2)

            # Drawing info
            position_text = self.font.render('position:'+str(car.position), True,self.white)
            velocity_text = self.font.render('velocity:'+str(car.velocity.x), True,self.white)
            accelera_text = self.font.render('acceleration:'+str(car.acceleration), True,self.white)
            carangle_text = self.font.render('car angle:'+str(car.angle), True,self.white)
            throttle_text = self.font.render('throttle:'+str(car.throttle), True,self.white)
            steering_text = self.font.render('steering:'+str(car.steering_input), True,self.white)
            self.screen.blit(position_text, (0, 0))
            self.screen.blit(velocity_text, (0, 15))
            self.screen.blit(accelera_text, (0, 30))
            self.screen.blit(carangle_text, (0, 45))
            self.screen.blit(throttle_text, (0, 60))
            self.screen.blit(steering_text, (0, 75))

            # Drawing car
            rotated = pygame.transform.rotate(car_image, degrees(car.angle))
            rect = rotated.get_rect()
            self.screen.blit(rotated, self.center_origin(Vector2(car.position.x,-car.position.y) * ppu - (rect.width / 2, rect.height / 2)))
            pygame.display.flip()

            self.clock.tick(self.ticks)
        pygame.quit()


if __name__ == '__main__':
    game = Game()
    game.run()
