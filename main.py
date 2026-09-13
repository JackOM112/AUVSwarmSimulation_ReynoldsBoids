import pygame
import numpy as np
import math

#Creates window
pygame.init()
Width, Height = 800, 600
Screen = pygame.display.set_mode((Width, Height))
pygame.display.set_caption("Boids Simulation")
Clock = pygame.time.Clock()

#Builds AUV blueprint
class AUV:
    def __init__(self):
        self.Position = np.array([np.random.uniform(0, Width), np.random.uniform(0, Height)], dtype=float)
        Angle = np.random.uniform(0, 2 * math.pi)
        self.Velocity = np.array([math.cos(Angle), math.sin(Angle)]) * 2
        self.Acceleration = np.zeros(2)
        self.MaxSpeed = 2.0

    #Updates AUVs' properties each frame
    def Update(self):
        self.Velocity = self.Velocity + self.Acceleration
        Speed = np.linalg.norm(self.Velocity)
        #Speed limit
        if Speed > self.MaxSpeed:
            self.Velocity = (self.Velocity / Speed) * self.MaxSpeed
        self.Position = self.Position + self.Velocity
        self.Acceleration = np.zeros(2)

        #Screen wrapping
        self.Position[0] = self.Position[0] % Width
        self.Position[1] = self.Position[1] % Height

    #Draws AUVs as a triangle
    def Draw(self, Surface):
        Angle = math.atan2(self.Velocity[1], self.Velocity[0])
        P1 = self.Position + np.array([math.cos(Angle), math.sin(Angle)]) * 12
        P2 = self.Position + np.array([math.cos(Angle + 2.5), math.sin(Angle + 2.5)]) * 8
        P3 = self.Position + np.array([math.cos(Angle - 2.5), math.sin(Angle - 2.5)]) * 8
        pygame.draw.polygon(Surface, (0, 255, 0), [P1, P2, P3])

#Calculates distance
def GetDistance(A, B):
    return np.linalg.norm(A - B)

#Spawns 40 AUVs and 4 obstacles
AUVs = [AUV() for _ in range(40)]
Obstacles = [np.array([np.random.uniform(100, 700), np.random.uniform(100, 500)]) for _ in range(4)]

#Loop set-up
Running = True
while Running:
    Screen.fill((0, 0, 0))
    MousePosition = np.array(pygame.mouse.get_pos(), dtype=float)

    #Quits the simulation
    for Event in pygame.event.get():
        if Event.type == pygame.QUIT:
            Running = False

    #Draws obstacles as a circle
    for Obstacle in Obstacles:
        pygame.draw.circle(Screen, (255, 0, 0), Obstacle.astype(int), 12)

    #Vectors created for the 3 core rules + obstacle avoidance
    for AUVObject in AUVs:
        Alignment, Cohesion, Separation, Avoidance = np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2)
        Total = 0

        #Checks nearby AUVs to calculate alignment, cohesion and separation forces
        for OtherAUV in AUVs:
            if AUVObject != OtherAUV:
                Distance = GetDistance(AUVObject.Position, OtherAUV.Position)
                if Distance < 60:
                    Alignment = Alignment + OtherAUV.Velocity
                    Cohesion = Cohesion + OtherAUV.Position
                    Separation = Separation + (AUVObject.Position - OtherAUV.Position) / (Distance ** 2)
                    Total = Total + 1

        #Averages the forces of the nearby AUVs to influence movement
        if Total > 0:
            Alignment = (Alignment / Total) * 0.02
            Cohesion = ((Cohesion / Total) - AUVObject.Position) * 0.02
            Separation = Separation * 5.0

        #Obstacle avoidance
        for Obstacle in Obstacles:
            DistanceToObstacle = GetDistance(AUVObject.Position, Obstacle)
            if DistanceToObstacle < 30:
                Avoidance = Avoidance + (AUVObject.Position - Obstacle) / DistanceToObstacle * 6

        #Cursor attraction
        CursorDistance = GetDistance(AUVObject.Position, MousePosition)
        CursorPull = np.zeros(2)
        if CursorDistance > 10:
            CursorPull = (MousePosition - AUVObject.Position) * 0.0005

        #Applies vector forces
        AUVObject.Acceleration = AUVObject.Acceleration + Alignment + Cohesion + Separation + Avoidance + CursorPull
        AUVObject.Update()
        AUVObject.Draw(Screen)

    #Updates the display 60 times / second
    pygame.display.flip()
    Clock.tick(60)

pygame.quit()
