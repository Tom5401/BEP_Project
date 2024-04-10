import os
import pygame
import numpy as np
from scipy import spatial
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
        self.max_velocity = 100.0
        self.free_deceleration = 0.805398773006135
        self.acceleration = 0.0
        self.steering = 0.0
        self.throttle = 0.15
        self.steering_input = 0.0
        self.listLength = 500
        self.u = np.linspace(0,2*np.pi,self.listLength)
        path_x = 2*np.sin(self.u)
        path_y = 2*np.cos(self.u)*np.sin(self.u)
        self.path = np.array([path_x,path_y]).transpose()
        print(self.path)
        self.K1 = 5
        self.K2 = 10
        C0x = self.length/2*np.cos(self.angle) + self.position.x
        C0y = self.length/2*np.sin(self.angle) + self.position.y
        self.d_e_prev,self.i_prev = spatial.KDTree(self.path).query([C0x,C0y])
        print('initial index:',self.i_prev, self.d_e_prev)

        self.theta_d = []
        for i in range(self.listLength-1):
            point_vector = self.path[i-1] - self.path[i+1]
            self.theta_d.append(np.arctan2(point_vector[1],point_vector[0]))
        point_vector = self.path[self.listLength-1] - self.path[0]
        self.theta_d.append(np.arctan2(point_vector[1],point_vector[0]))
        


    def throttleController(self):
        self.throttle = 0.15

    def steeringController(self):
        Cx = self.length/2*np.cos(self.angle) + self.position.x
        Cy = self.length/2*np.sin(self.angle) + self.position.y

        if self.i_prev-5 < 0:
            path_seg =  np.concatenate((self.path[self.i_prev-5:], self.path[:self.i_prev+5]))
        else:
            path_seg = self.path[self.i_prev-5: self.i_prev+5]


        print('segment index:',self.i_prev-5,self.i_prev+5)
        d_abs,i = spatial.KDTree(path_seg).query([Cx,Cy])
        i += self.i_prev - 5
        self.i_prev = i


        theta_e = self.angle - self.theta_d[i]
        if theta_e > np.pi:
            theta_e = theta_e - 2*np.pi
        elif theta_e < -np.pi:
            theta_e = theta_e + 2*np.pi
        d_e = (Cy-self.path[i][1])*np.cos(self.theta_d[i]) - (Cx-self.path[i][0])*np.sin(self.theta_d[i])
        print(d_e, theta_e)
        self.steering_input = -(self.K1*d_e + self.K2*theta_e)
        if abs(self.steering_input) > 1.0:
            self.steering_input = self.steering_input/abs(self.steering_input)
        #info = 'rotation:',str(self.angle),'alpha:',str(alpha),'theta:',str(theta),'steering:',str(self.steering)
        #print(info)            


    def update(self, dt):
         # User input
        self.throttleController()
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
        car = Car(3,-1)
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
            last = [None,None]
            for p in car.path:
                p = p*ppu
                if last[0] != None:
                    pygame.draw.line(self.screen, (255,255,255), self.center_origin(last), self.center_origin(p))
                last = p
            pygame.draw.rect(self.screen, self.white, border,  2)

            # Drawing info
            position_text = self.font.render('position:['+str(car.position.x)[:5]+','+str(car.position.y)[:5]+']', True,self.white)
            velocity_text = self.font.render('velocity:'+str(car.velocity.x)[:5], True,self.white)
            accelera_text = self.font.render('acceleration:'+str(car.acceleration)[:5], True,self.white)
            carangle_text = self.font.render('car angle:'+str(car.angle)[:5], True,self.white)
            throttle_text = self.font.render('throttle:'+str(car.throttle)[:5], True,self.white)
            steering_text = self.font.render('steering:'+str(car.steering_input)[:5], True,self.white)
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
