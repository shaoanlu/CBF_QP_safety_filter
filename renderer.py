from dataclasses import dataclass
from typing import Dict, Tuple, List
import pygame
from controllers.i_controller import ControllerInterface


@dataclass(kw_only=True)
class UIConfig:
    fps: int = 100
    window_width: int = 340
    window_height: int = 400
    default_font_size: int = 30
    small_font_size: int = 15
    collision_text_pos: Tuple[int, int] = (230, 350)
    status_text_positions: Dict[str, Tuple[int, int]] = None

    def __post_init__(self):
        if self.status_text_positions is None:
            self.status_text_positions = {"cbf_status": (0, 0), "cbf_alpha": (0, 15), "cbf_config": (0, 30)}


class GameRenderer:
    def __init__(self, config: UIConfig):
        self.config = config
        self._init_pygame()
        self._init_fonts()
        self._init_text_surfaces()
        self._last_frame_time = pygame.time.get_ticks()

    def _init_pygame(self) -> None:
        """Initialize pygame display and font system."""
        pygame.init()  # Initialize all pygame modules
        pygame.font.init()  # Explicitly initialize the font module
        self.screen = pygame.display.set_mode((self.config.window_width, self.config.window_height))
        pygame.display.set_caption("Control barrier function demo")

    def _init_fonts(self) -> None:
        """Initialize font objects."""
        default_font = pygame.font.get_fonts()[0]
        self.main_font = pygame.font.SysFont(default_font, self.config.default_font_size)
        self.small_font = pygame.font.SysFont(default_font, self.config.small_font_size)

    def _init_text_surfaces(self) -> None:
        """Initialize static text surfaces."""
        self.collision_text = self.main_font.render("Collide!", False, (255, 255, 255))
        self.cbf_on_text = self.small_font.render("CBF ON (press x to turn off)", False, (0, 255, 125))
        self.cbf_off_text = self.small_font.render("CBF OFF (press x to turn on)", False, (255, 0, 125))

    def clear_screen(self) -> None:
        """Clear the screen with black background."""
        self.screen.fill((0, 0, 0))

    def draw_robot(self, robot: ControllerInterface, draw_filtered_command: bool = False) -> None:
        """Draw a single robot."""
        from utils import draw_robot  # Import here to avoid circular import

        draw_robot(self.screen, robot, draw_filtered_command)

    def draw_robots(
        self,
        ego_robot: ControllerInterface,
        static_robot: ControllerInterface,
        patrol_robots: List[ControllerInterface],
        game_state,
    ) -> None:
        """Draw all robots in the scene."""
        self.draw_robot(static_robot)

        for robot in patrol_robots:
            self.draw_robot(robot, draw_filtered_command=game_state.use_cbf_patrol_robots)

        self.draw_robot(ego_robot, draw_filtered_command=game_state.use_cbf)

    def draw_collision_warning(self, is_collided: bool) -> None:
        """Draw collision warning if collision detected."""
        if is_collided:
            self.screen.blit(self.collision_text, self.config.collision_text_pos)

    def draw_cbf_status(self, game_state) -> None:
        """Draw CBF status information."""
        if game_state.use_cbf:
            self._draw_cbf_enabled_status(game_state)
        else:
            self.screen.blit(self.cbf_off_text, self.config.status_text_positions["cbf_status"])

    def _draw_cbf_enabled_status(self, game_state) -> None:
        """Draw detailed CBF status when enabled."""
        # Draw CBF ON status
        self.screen.blit(self.cbf_on_text, self.config.status_text_positions["cbf_status"])

        # Draw CBF alpha value
        alpha_text = self.small_font.render(
            f"CBF alpha: {game_state.current_alpha} (press z to change)", False, (0, 255, 125)
        )
        self.screen.blit(alpha_text, self.config.status_text_positions["cbf_alpha"])

        # Draw CBF configuration
        config_text = self.small_font.render(
            f"Fixed dir. {game_state.cbf_force_direction_unchanged}, "
            f"CBF patrols {game_state.use_cbf_patrol_robots} (c/v)",
            False,
            (0, 255, 125),
        )
        self.screen.blit(config_text, self.config.status_text_positions["cbf_config"])

    def update_display(self) -> None:
        """Update the display and maintain a fixed frame rate of 100 FPS."""
        pygame.display.update()

        # Target frame duration (10ms for 100 FPS)
        frame_duration = 1000 // self.config.fps  # milliseconds

        # Calculate how much time has passed since last frame
        current_time = pygame.time.get_ticks()
        time_passed = current_time - self._last_frame_time

        # Calculate how long to wait to maintain target frame rate
        wait_time = max(0, frame_duration - time_passed)
        pygame.time.wait(wait_time)

        # Update last frame time for next calculation
        self._last_frame_time = pygame.time.get_ticks()
