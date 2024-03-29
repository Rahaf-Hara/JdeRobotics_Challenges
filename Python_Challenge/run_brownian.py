"""
Sample Application for Brownian Motion Simulation

This script demonstrates how to use the Brownian Motion simulation module to
run a simulation of a robot moving within a 2D arena. It showcases the basic
setup and execution of the simulation, including setting the arena size and
frames per second (fps) for the simulation.
"""

from brownian_motion.brownian_sim import Simulation


def run_sample_application():
    """
    Initializes and runs the Brownian Motion simulation.

    This function sets up the simulation with a predefined arena size and
    fps, creating an instance of the Simulation class and invoking its run
    method to start the simulation.
    """
    arena_size = (600, 600)  # Define the size of the simulation arena
    fps = 70  # Define the frames per second at which the simulation runs
    simulation = Simulation(arena_size, fps)  # Create a simulation instance
    simulation.run()  # Start the simulation


if __name__ == "__main__":
    run_sample_application()
