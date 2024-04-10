import sys
import pygame
import numpy as np

from models.robot_dynamics import SimpleRobotDynamics
from controllers.i_controller import ControllerInterface
from controllers.robot_cbf import RobotCBF
from utils import draw_robot


def run():
    # init pygame
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((340, 400))
    pygame.display.set_caption("Control barrier function demo")
    pygame.key.set_repeat(10)

    # init robots
    ego_robot: ControllerInterface = RobotCBF(
        SimpleRobotDynamics(x0=np.array([50, 350])), (0, 255, 0), vel=3
    )
    static_robot: ControllerInterface = RobotCBF(
        SimpleRobotDynamics(x0=np.array([120, 200])), (0, 0, 255), vel=0
    )
    patrol_robot1: ControllerInterface = RobotCBF(
        SimpleRobotDynamics(x0=np.array([230, 300])), (0, 0, 255), vel=1
    )
    patrol_robot2: ControllerInterface = RobotCBF(
        SimpleRobotDynamics(x0=np.array([300, 70])), (0, 0, 255), vel=1
    )
    collision_objects = [static_robot, patrol_robot1, patrol_robot2]

    # init control configs
    count = 0
    use_cbf = False
    cbf_force_direction_unchanged = False
    use_cbf_patrol_robots = False
    cbf_alphas = [1e-1, 1e-2, 1]
    direction_patrol_robot1 = pygame.K_UP
    direction_patrol_robot2 = pygame.K_LEFT

    # init fonts
    default_font = pygame.font.get_fonts()[0]
    font = pygame.font.SysFont(default_font, 30)  #
    text_surface = font.render("Collide!", False, (255, 255, 255))
    cbf_font = pygame.font.SysFont(default_font, 15)
    cbf_on_text_surface = cbf_font.render("CBF ON (press x to turn off)", False, (0, 255, 125))
    cbf_off_text_surface = cbf_font.render("CBF OFF (press x to turn on)", False, (255, 0, 125))

    # main loop
    while True:
        count += 1

        # flip moving direction of patrol robots
        if count % 150 == 0:
            direction_patrol_robot1 = (
                pygame.K_DOWN if (direction_patrol_robot1 == pygame.K_UP) else pygame.K_UP
            )
            direction_patrol_robot2 = (
                pygame.K_RIGHT if (direction_patrol_robot2 == pygame.K_LEFT) else pygame.K_LEFT
            )
        count = 0 if count >= 1e10 else count

        # get key press event
        pressed_key = None
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                sys.exit()
            if e.type == pygame.KEYDOWN:
                pressed_key = e.key
            if e.type == pygame.KEYUP:
                if e.key == pygame.K_x:
                    use_cbf = not use_cbf
                if e.key == pygame.K_c:
                    cbf_force_direction_unchanged = not cbf_force_direction_unchanged
                if e.key == pygame.K_v:
                    use_cbf_patrol_robots = not use_cbf_patrol_robots
                if e.key == pygame.K_z:
                    cbf_alphas.append(cbf_alphas.pop(0))  # roll left
                if e.key == pygame.K_r:
                    # reset
                    ego_robot.x, ego_robot.y = 50, 350
                    static_robot.x, static_robot.y = 120, 200
                    patrol_robot1.x, patrol_robot1.y = 230, 300
                    patrol_robot2.x, patrol_robot2.y = 300, 70
                    direction_patrol_robot1 = pygame.K_UP
                    direction_patrol_robot2 = pygame.K_LEFT
                    count = 0

        # move robots
        static_robot.control(None)
        ego_robot.control(
            pressed_key,
            use_cbf=use_cbf,
            cbf_alpha=cbf_alphas[0],
            collision_objects=collision_objects,
            force_direction_unchanged=cbf_force_direction_unchanged,
        )
        patrol_robot1.control(
            direction_patrol_robot1,
            use_cbf=use_cbf_patrol_robots,
            collision_objects=[ego_robot, static_robot, patrol_robot2]
            if use_cbf_patrol_robots
            else [],
            force_direction_unchanged=cbf_force_direction_unchanged,
        )
        patrol_robot2.control(
            direction_patrol_robot2,
            use_cbf=use_cbf_patrol_robots,
            collision_objects=[ego_robot, static_robot, patrol_robot1]
            if use_cbf_patrol_robots
            else [],
            force_direction_unchanged=cbf_force_direction_unchanged,
        )

        # detect collision
        ego_robot.detect_collision(collision_objects=collision_objects)

        # draw robots
        screen.fill((0, 0, 0))
        draw_robot(screen, static_robot)
        draw_robot(screen, patrol_robot1, draw_filtered_command=use_cbf_patrol_robots)
        draw_robot(screen, patrol_robot2, draw_filtered_command=use_cbf_patrol_robots)
        draw_robot(screen, ego_robot, draw_filtered_command=use_cbf)

        # draw texts
        if ego_robot.is_collided:
            screen.blit(text_surface, (230, 350))
        if use_cbf:
            screen.blit(cbf_on_text_surface, (0, 0))
            tmp_render = cbf_font.render(
                f"CBF alpha: {cbf_alphas[0]} (press z to change)",
                False,
                (0, 255, 125),
            )
            screen.blit(
                tmp_render,
                (0, 15),
            )
            tmp_render = cbf_font.render(
                f"Fixed dir. {cbf_force_direction_unchanged}, CBF patrols {use_cbf_patrol_robots} (c/v)",
                False,
                (0, 255, 125),
            )
            screen.blit(
                tmp_render,
                (0, 30),
            )
        else:
            screen.blit(cbf_off_text_surface, (0, 0))

        # render and wait
        pygame.display.update()
        pygame.time.wait(10)


if __name__ == "__main__":
    run()
