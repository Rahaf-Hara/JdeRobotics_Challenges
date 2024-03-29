"""
Simulation of Brownian Motion in a 2D arena.

This script simulates the random movement of a robot within a bounded 2D arena,
mimicking Brownian Motion behavior. The robot moves continuously, changing
direction randomly upon colliding with the arena's boundaries, and varying its
speed based on a normal distribution to reflect natural variability in motion.
"""
import pygame
import numpy as np


class Robot:
    """
    Represents a robot that simulates Brownian Motion within a 2D arena,
    changing direction upon collisions with boundaries and varying its
    speed and color in a random manner.
    """
    def __init__(self, arena_size, mean_speed=5.0, speed_std=0.5):
        """
        Initializes the robot at the arena's center with a random direction,
        speed, and color.

        Args:
            arena_size (tuple): The dimensions of the arena (width, height).
            mean_speed (float): The mean speed of the robot.
            speed_std (float): The standard deviation of the robot's speed.
        """
        self.position = np.array([arena_size[0] / 2, arena_size[1] / 2])
        self.direction = np.random.uniform(0, 2 * np.pi)
        self.speed = np.abs(np.random.normal(loc=mean_speed, scale=speed_std))
        self.arena_size = arena_size
        self.color = self.random_color()

    def move(self):
        """
        Moves the robot in its current direction at its current speed,
        checking for and responding to collisions with arena boundaries.
        """
        self.position += (np.array([np.cos(self.direction),
                                    np.sin(self.direction)]) * self.speed)
        self.check_collision()

    def check_collision(self):
        """
        Checks for collisions with arena boundaries. On collision, the
        robot's direction is changed randomly using a uniform distribution,
        and its color is changed to a new random color.
        """
        if not 0 <= self.position[0] < self.arena_size[0] or \
           not 0 <= self.position[1] < self.arena_size[1]:
            self.position = np.clip(self.position, [0, 0], self.arena_size)
            self.direction += np.random.uniform(0, 2*np.pi)
            self.color = self.random_color()  # Update color on collision.

    def random_color(self):
        """
        Generates a random, sufficiently bright color to ensure visibility
        against the simulation's black background.

        Returns:
            tuple: Random color represented as (R, G, B).
        """
        min_brightness = 100
        return (np.random.randint(min_brightness, 256),
                np.random.randint(min_brightness, 256),
                np.random.randint(min_brightness, 256))


class Simulation:
    """
    Manages and runs the robot simulation, displaying Brownian Motion within
    a predefined 2D arena.
    """
    def __init__(self, arena_size, fps=70):
        """
        Initializes the simulation environment and robot.

        Args:
            arena_size (tuple): Dimensions of the arena (width, height).
            fps (int): The frames per second at which the simulation runs.
        """
        pygame.init()
        self.arena_size = arena_size
        self.screen = pygame.display.set_mode(arena_size)
        self.robot = Robot(arena_size, mean_speed=5.0, speed_std=0.5)
        self.clock = pygame.time.Clock()
        self.fps = fps

    def run(self):
        """Runs the main simulation loop."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            self.robot.move()
            self.draw()
            pygame.display.flip()
            self.clock.tick(self.fps)

    def draw(self):
        """
        Draws the current state of the simulation, including the robot
        and arena boundaries.
        """
        self.screen.fill((0, 0, 0))
        pygame.draw.circle(self.screen, self.robot.color,
                           self.robot.position.astype(int), 10)
        pygame.draw.rect(self.screen, (255, 255, 255),
                         ((0, 0), self.arena_size), 1)
