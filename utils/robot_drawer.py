from dataclasses import dataclass
from typing import Tuple, NamedTuple
import pygame
from controllers.i_controller import ControllerInterface


class RobotCommand(NamedTuple):
    """Represents a robot's movement command with x and y components."""

    x: float
    y: float
    color: tuple = (175, 175, 175)  # Default color for nominal commands


@dataclass
class ArrowStyle:
    """Configuration for arrow rendering."""

    size: int = 10
    scale: int = 20  # Scale factor for arrow length
    width: int = 10
    nominal_color: tuple = (175, 175, 175)
    filtered_color: tuple = (255, 0, 0)


class RobotDrawer:
    """Handles all robot drawing operations."""

    def __init__(self, screen: pygame.Surface, arrow_style: ArrowStyle = ArrowStyle()):
        self.screen = screen
        self.style = arrow_style

    def draw_robot(
        self,
        robot: ControllerInterface,
        draw_filtered_command: bool = False,
    ) -> None:
        """Main method to draw a robot and its command arrows."""
        self._draw_robot_body(robot)

        nominal_command = RobotCommand(robot.nominal_ux, robot.nominal_uy)
        self._draw_command_arrows(robot, nominal_command, self.style.nominal_color)

        if draw_filtered_command:
            filtered_command = RobotCommand(robot.ux, robot.uy)
            self._draw_command_arrows(robot, filtered_command, self.style.filtered_color)

    def _draw_robot_body(self, robot: ControllerInterface) -> None:
        """Draw the main robot circle."""
        pygame.draw.circle(self.screen, robot.color, (robot.x, robot.y), robot.size)

    def _draw_command_arrows(self, robot: ControllerInterface, command: RobotCommand, color: tuple) -> None:
        """Draw command arrows for both x and y components."""
        if command.x != 0:
            self._draw_single_arrow(start_pos=(robot.x, robot.y), direction="x", magnitude=command.x, color=color)

        if command.y != 0:
            self._draw_single_arrow(start_pos=(robot.x, robot.y), direction="y", magnitude=command.y, color=color)

    def _draw_single_arrow(
        self, start_pos: Tuple[float, float], direction: str, magnitude: float, color: tuple
    ) -> None:
        """Draw a single arrow in either x or y direction."""
        x, y = start_pos
        scaled_magnitude = self.style.scale * magnitude

        # Calculate end points based on direction
        if direction == "x":
            end_pos = (x + scaled_magnitude, y)
            arrow_points = [
                (end_pos[0], end_pos[1] + self.style.size),
                (end_pos[0], end_pos[1] - self.style.size),
                (x + (self.style.size + self.style.scale) * magnitude, y),
            ]
        else:  # y direction
            end_pos = (x, y + scaled_magnitude)
            arrow_points = [
                (end_pos[0] + self.style.size, end_pos[1]),
                (end_pos[0] - self.style.size, end_pos[1]),
                (x, y + (self.style.size + self.style.scale) * magnitude),
            ]

        # Draw arrow line
        pygame.draw.line(self.screen, color, start_pos, end_pos, width=self.style.width)

        # Draw arrow head
        pygame.draw.polygon(self.screen, color, arrow_points)
