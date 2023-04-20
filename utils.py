import pygame
from robot import Robot


def draw_robot(
    screen: pygame.Surface,
    robot: Robot,
    draw_filtered_command: bool = False,
    color1: tuple = (175, 175, 175),
    color2: tuple = (255, 0, 0),
    arrow_size: int = 10,
):
    pygame.draw.circle(screen, robot.color, (robot.x, robot.y), robot.size)

    # draw arrows
    if robot.ux != 0:
        pygame.draw.line(
            screen,
            color1,
            (robot.x, robot.y),
            (robot.x + 20 * robot.nominal_ux, robot.y),
            width=10,
        )
        pygame.draw.polygon(
            screen,
            color1,
            [
                (robot.x + 20 * robot.nominal_ux, robot.y + arrow_size),
                (robot.x + 20 * robot.nominal_ux, robot.y - arrow_size),
                (robot.x + (arrow_size + 20) * robot.nominal_ux, robot.y),
            ],
        )
    if robot.uy != 0:
        pygame.draw.line(
            screen,
            color1,
            (robot.x, robot.y),
            (robot.x, robot.y + 20 * robot.nominal_uy),
            width=10,
        )
        pygame.draw.polygon(
            screen,
            color1,
            [
                (robot.x + arrow_size, robot.y + 20 * robot.nominal_uy),
                (robot.x - arrow_size, robot.y + 20 * robot.nominal_uy),
                (robot.x, robot.y + (arrow_size + 20) * robot.nominal_uy),
            ],
        )

    # draw output of safety filter
    if draw_filtered_command:
        if robot.ux != 0:
            pygame.draw.line(
                screen,
                color2,
                (robot.x, robot.y),
                (robot.x + 20 * robot.ux, robot.y),
                width=10,
            )
            pygame.draw.polygon(
                screen,
                color2,
                [
                    (robot.x + 20 * robot.ux, robot.y + arrow_size),
                    (robot.x + 20 * robot.ux, robot.y - arrow_size),
                    (robot.x + (arrow_size + 20) * robot.ux, robot.y),
                ],
            )
        if robot.uy != 0:
            pygame.draw.line(
                screen,
                color2,
                (robot.x, robot.y),
                (robot.x, robot.y + 20 * robot.uy),
                width=10,
            )
            pygame.draw.polygon(
                screen,
                color2,
                [
                    (robot.x + arrow_size, robot.y + 20 * robot.uy),
                    (robot.x - arrow_size, robot.y + 20 * robot.uy),
                    (robot.x, robot.y + (arrow_size + 20) * robot.uy),
                ],
            )
