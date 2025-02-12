from dataclasses import dataclass
from typing import Any, Dict, Optional, List, Callable
import pygame
import sys
from enum import Enum, auto


class GameEvent(Enum):
    TOGGLE_CBF = auto()
    TOGGLE_CBF_DIRECTION = auto()
    TOGGLE_CBF_PATROL = auto()
    TOGGLE_LIDAR_SIMULATION = auto()
    CYCLE_CBF_ALPHA = auto()
    RESET_SIMULATION = auto()
    QUIT = auto()


@dataclass
class GameState:
    use_cbf: bool = False
    use_lidar_sensor: bool = False
    cbf_force_direction_unchanged: bool = False
    use_cbf_patrol_robots: bool = False
    cbf_alphas: List[float] = None
    current_alpha_idx: int = 0
    count: int = 0

    def __post_init__(self):
        if self.cbf_alphas is None:
            self.cbf_alphas = [1e-1, 1e-2, 1]

    @property
    def current_alpha(self) -> float:
        return self.cbf_alphas[self.current_alpha_idx]


class InputHandler:
    def __init__(self):
        self._key_event_map = {
            pygame.K_x: GameEvent.TOGGLE_CBF,
            pygame.K_c: GameEvent.TOGGLE_CBF_DIRECTION,
            pygame.K_v: GameEvent.TOGGLE_CBF_PATROL,
            pygame.K_z: GameEvent.CYCLE_CBF_ALPHA,
            pygame.K_r: GameEvent.RESET_SIMULATION,
            pygame.K_s: GameEvent.TOGGLE_LIDAR_SIMULATION,
        }
        self._event_handlers: Dict[GameEvent, Callable] = {}
        self._pressed_movement_key: Optional[int] = None

    def register_event_handler(self, event: GameEvent, handler: Callable) -> None:
        """Register a callback function for a specific game event."""
        self._event_handlers[event] = handler

    def get_movement_key(self) -> Optional[int]:
        """Get the currently pressed movement key."""
        return self._pressed_movement_key

    def process_events(self) -> None:
        """Process all pending pygame events."""
        self._pressed_movement_key = None

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._handle_event(GameEvent.QUIT)

            elif event.type == pygame.KEYDOWN:
                self._pressed_movement_key = event.key

            elif event.type == pygame.KEYUP:
                if event.key in self._key_event_map:
                    game_event = self._key_event_map[event.key]
                    self._handle_event(game_event)

    def _handle_event(self, event: GameEvent) -> None:
        """Handle a game event by calling its registered handler."""
        if event == GameEvent.QUIT:
            sys.exit()

        if event in self._event_handlers:
            self._event_handlers[event]()


class GameStateManager:
    def __init__(self):
        self.state = GameState()
        self.input_handler = InputHandler()
        self._setup_event_handlers()

    def _setup_event_handlers(self) -> None:
        """Set up handlers for all game events."""
        self.input_handler.register_event_handler(
            GameEvent.TOGGLE_CBF, lambda: setattr(self.state, "use_cbf", not self.state.use_cbf)
        )
        self.input_handler.register_event_handler(
            GameEvent.TOGGLE_CBF_DIRECTION,
            lambda: setattr(self.state, "cbf_force_direction_unchanged", not self.state.cbf_force_direction_unchanged),
        )
        self.input_handler.register_event_handler(
            GameEvent.TOGGLE_CBF_PATROL,
            lambda: setattr(self.state, "use_cbf_patrol_robots", not self.state.use_cbf_patrol_robots),
        )
        self.input_handler.register_event_handler(
            GameEvent.TOGGLE_LIDAR_SIMULATION,
            lambda: setattr(self.state, "use_lidar_sensor", not self.state.use_lidar_sensor),
        )
        self.input_handler.register_event_handler(GameEvent.CYCLE_CBF_ALPHA, self._cycle_cbf_alpha)

    def _cycle_cbf_alpha(self) -> None:
        """Cycle through available CBF alpha values."""
        self.state.current_alpha_idx = (self.state.current_alpha_idx + 1) % len(self.state.cbf_alphas)

    def process_input(self) -> Optional[int]:
        """Process input and return current movement key if any."""
        self.input_handler.process_events()
        return self.input_handler.get_movement_key()
