import sys
import pygame
import numpy as np

from game_controls import GameStateManager, GameEvent
from models.robot_dynamics import SimpleRobotDynamics
from controllers.i_controller import ControllerInterface
from controllers.robot_cbf import RobotCBF
from renderer import GameRenderer, UIConfig
from utils import draw_robot


WINDOW_WIDTH = 340
WINDOW_HEIGHT = 400


def run():
    # init game systems
    game_manager = GameStateManager()
    renderer = GameRenderer(UIConfig())

    # Set up key repeat (delay in ms, interval in ms)
    pygame.key.set_repeat(10)

    # init robots
    ego_robot: ControllerInterface = RobotCBF(SimpleRobotDynamics(x0=np.array([50, 350])), (0, 255, 0), vel=3)
    static_robot: ControllerInterface = RobotCBF(SimpleRobotDynamics(x0=np.array([120, 200])), (0, 0, 255), vel=0)
    patrol_robot1: ControllerInterface = RobotCBF(SimpleRobotDynamics(x0=np.array([230, 300])), (0, 0, 255), vel=1)
    patrol_robot2: ControllerInterface = RobotCBF(SimpleRobotDynamics(x0=np.array([300, 70])), (0, 0, 255), vel=1)
    patrol_robots = [patrol_robot1, patrol_robot2]
    collision_objects = [static_robot, patrol_robot1, patrol_robot2]

    # Register reset handler
    def reset_simulation():
        ego_robot.x, ego_robot.y = 50, 350
        static_robot.x, static_robot.y = 120, 200
        patrol_robot1.x, patrol_robot1.y = 230, 300
        patrol_robot2.x, patrol_robot2.y = 300, 70

    game_manager.input_handler.register_event_handler(GameEvent.RESET_SIMULATION, reset_simulation)

    # init control configs
    count = 0
    direction_patrol_robot1 = pygame.K_UP
    direction_patrol_robot2 = pygame.K_LEFT

    # main loop
    while True:
        count += 1

        # flip moving direction of patrol robots
        if count % 150 == 0:
            direction_patrol_robot1 = pygame.K_DOWN if (direction_patrol_robot1 == pygame.K_UP) else pygame.K_UP
            direction_patrol_robot2 = pygame.K_RIGHT if (direction_patrol_robot2 == pygame.K_LEFT) else pygame.K_LEFT
        count = 0 if count >= 1e10 else count

        # Process input
        pressed_key = game_manager.process_input()
        game_state = game_manager.state

        # move robots
        static_robot.control(None)
        ego_robot.control(
            pressed_key,
            use_cbf=game_state.use_cbf,
            cbf_alpha=game_state.cbf_alphas[0],
            collision_objects=collision_objects,
            force_direction_unchanged=game_state.cbf_force_direction_unchanged,
        )

        # Update patrolling robots
        for robot, direction in zip(patrol_robots, [direction_patrol_robot1, direction_patrol_robot2]):
            collision_list = [ego_robot, static_robot] + [r for r in patrol_robots if r != robot]
            robot.control(
                direction,
                use_cbf=game_state.use_cbf_patrol_robots,
                collision_objects=(collision_list if game_state.use_cbf_patrol_robots else []),
                force_direction_unchanged=game_state.cbf_force_direction_unchanged,
            )

        # detect collision
        ego_robot.detect_collision(collision_objects=collision_objects)

        # Render frame
        renderer.clear_screen()
        renderer.draw_robots(ego_robot, static_robot, patrol_robots, game_state)
        renderer.draw_collision_warning(ego_robot.is_collided)
        renderer.draw_cbf_status(game_state)
        renderer.update_display()


if __name__ == "__main__":
    run()
