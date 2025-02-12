# Refactoring notes (ChatGPT/Claude generated)

## Before game state & contorls refactoring
```python
    use_cbf = False
    cbf_force_direction_unchanged = False
    use_cbf_patrol_robots = False
    cbf_alphas = [1e-1, 1e-2, 1]
    ...

    while True:
        ...
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
        ...
```

## ## Step-by-Step Refactoring Process (Claude)
1. First, Identify Problems in Original Code
    - The original code mixes input handling, state management, and game logic in one big loop
    - State variables are scattered (use_cbf, cbf_force_direction_unchanged, etc.)
    - Event handling is done through direct key checks
    - No clear separation of responsibilities

2. Design New Architecture
```bash
Original Structure:
run() function
├── Input handling (pygame.event.get())
├── State variables
└── Game loop with everything mixed

New Structure:
├── GameEvent (Enum) - Defines all possible game events
├── GameState (Dataclass) - Holds all game state
├── InputHandler - Handles keyboard input
└── GameStateManager - Orchestrates everything
```
3. Create GameEvent Enum
```python
class GameEvent(Enum):
    TOGGLE_CBF = auto()
    TOGGLE_CBF_DIRECTION = auto()
    TOGGLE_CBF_PATROL = auto()
    CYCLE_CBF_ALPHA = auto()
    RESET_SIMULATION = auto()
    QUIT = auto()
```
This replaces direct key checks with semantic events, making the code more maintainable.

4. Create GameState Class
```python
@dataclass
class GameState:
    use_cbf: bool = False
    cbf_force_direction_unchanged: bool = False
    use_cbf_patrol_robots: bool = False
    cbf_alphas: List[float] = None
    current_alpha_idx: int = 0
```
- Groups all related state variables together
- Uses dataclass for clean initialization and type hints
- Adds property for current_alpha to encapsulate logic

5. Create InputHandler Class
```python
class InputHandler:
    def __init__(self):
        self._key_event_map = {
            pygame.K_x: GameEvent.TOGGLE_CBF,
            pygame.K_c: GameEvent.TOGGLE_CBF_DIRECTION,
            # ...
        }
```
- Maps keyboard keys to GameEvents
- Provides clean interface for event handling
- Separates input handling from game logic

6. Create GameStateManager Class
```python
class GameStateManager:
    def __init__(self):
        self.state = GameState()
        self.input_handler = InputHandler()
        self._setup_event_handlers()
```
- Acts as the orchestrator
- Connects input events to state changes
- Provides clean interface for main game loop

7. Benefits of Each Change:
- Event Enum: Makes it clear what events exist in the system
- GameState: Groups related state, enforces type safety
- InputHandler: Separates input handling logic
- GameStateManager: Provides clean interface to rest of game
- Main Loop: Now focuses on game logic rather than implementation details

## Step-by-Step Refactoring Process (ChatGPT)
The original code is refactored for better encapsulation, modularity, and maintainability. Below is a step-by-step walkthrough of the changes.

### Step 1: Encapsulate State in a Class (`GameState`)
#### Before Refactoring
- The game state variables (`use_cbf`, `cbf_force_direction_unchanged`, etc.) are **global variables**, making the code hard to maintain.
- State updates are scattered inside the event loop.
#### Refactoring
- Move these variables into a `GameState` class.
- Use `@dataclass` to provide defaults and automatic initialization.
#### After Refactoring
```python
from dataclasses import dataclass
from typing import List

@dataclass(kw_only=True)
class GameState:
    use_cbf: bool = False
    cbf_force_direction_unchanged: bool = False
    use_cbf_patrol_robots: bool = False
    cbf_alphas: List[float] = None
    current_alpha_idx: int = 0

    def __post_init__(self):
        if self.cbf_alphas is None:
            self.cbf_alphas = [1e-1, 1e-2, 1]

    @property
    def current_alpha(self) -> float:
        return self.cbf_alphas[self.current_alpha_idx]
```
#### Why?
✅ **Encapsulation** – Groups related state variables into a single class.
✅ **Avoids Global Variables** – Keeps state management centralized.
✅ **Provides Computed Properties** – `current_alpha` dynamically retrieves the active alpha value.

### Step 2: Extract Input Handling Logic (`InputHandler`)
#### Before Refactoring
- Input handling is embedded directly in the main game loop.
- Checking key presses and updating state is tightly coupled.
#### Refactoring
- Create an `InputHandler` class.
- Use a dictionary (`_key_event_map`) to map keys to game events.
- Use an event-driven approach by allowing event handlers to be registered dynamically.
#### After Refactoring
```python
import pygame
import sys
from enum import Enum, auto
from typing import Optional, Callable

class GameEvent(Enum):
    TOGGLE_CBF = auto()
    TOGGLE_CBF_DIRECTION = auto()
    TOGGLE_CBF_PATROL = auto()
    CYCLE_CBF_ALPHA = auto()
    RESET_SIMULATION = auto()
    QUIT = auto()

class InputHandler:
    def __init__(self):
        self._key_event_map = {
            pygame.K_x: GameEvent.TOGGLE_CBF,
            pygame.K_c: GameEvent.TOGGLE_CBF_DIRECTION,
            pygame.K_v: GameEvent.TOGGLE_CBF_PATROL,
            pygame.K_z: GameEvent.CYCLE_CBF_ALPHA,
            pygame.K_r: GameEvent.RESET_SIMULATION,
        }
        self._event_handlers = {}
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
            elif event.type == pygame.KEYUP and event.key in self._key_event_map:
                self._handle_event(self._key_event_map[event.key])

    def _handle_event(self, event: GameEvent) -> None:
        """Handle a game event by calling its registered handler."""
        if event == GameEvent.QUIT:
            sys.exit()
        if event in self._event_handlers:
            self._event_handlers[event]()
```
#### Why?
✅ **Decouples** Input from Game Logic – Now `InputHandler` only processes input, while another class manages state updates.
✅ **Extensible** – New key mappings or event handlers can be added dynamically.
✅ **More Maintainable** – Each function has a _single responsibility_.

###Step 3: Create a GameState Manager (GameStateManager)
#### Before Refactoring
- Input handling and game state modifications are interleaved inside the loop.
- The reset logic is inline, making it hard to reuse.
#### Refactoring
- Create a controller class (`GameStateManager`) that manages game state transitions.
- Use lambda functions to register state-changing handlers.
- Extract reset logic into its own method.
#### After Refactoring
```python
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
        self.input_handler.register_event_handler(GameEvent.CYCLE_CBF_ALPHA, self._cycle_cbf_alpha)
        self.input_handler.register_event_handler(GameEvent.RESET_SIMULATION, self._reset_simulation)

    def _cycle_cbf_alpha(self) -> None:
        """Cycle through available CBF alpha values."""
        self.state.current_alpha_idx = (self.state.current_alpha_idx + 1) % len(self.state.cbf_alphas)

    def _reset_simulation(self) -> None:
        """Reset the simulation state."""
        self.state = GameState()  # Reinitialize state

    def process_input(self) -> Optional[int]:
        """Process input and return current movement key if any."""
        self.input_handler.process_events()
        return self.input_handler.get_movement_key()
```
#### Why?
✅ **Centralized State Management** – `GameStateManager` ensures a single source of truth for state updates.
✅ **Reusable Reset Logic** – `_reset_simulation()` reinitializes `GameState` cleanly.
✅ **Encapsulation** – Input handling (`InputHandler`) and state transitions (`GameStateManager`) are clearly separated.

### Step 4: Refactor the Main Loop
#### Before Refactoring
The main loop:

  1. Processes events.
  2. Modifies global state directly.
  3. Resets robots inline.
  4. After Refactoring
The loop now:

  1. Calls `process_input()` to handle events.
  2. Uses `GameStateManager` to update state.
  3. Keeps movement handling separate.
```python
game_manager = GameStateManager()

while True:
    pressed_key = game_manager.process_input()
    # Movement logic can use `pressed_key` if needed
```
#### Why?
✅ Simpler Main Loop – Focuses only on high-level operations.
✅ Separation of Concerns – Delegates event handling and state changes to dedicated classes.
✅ Better Readability – Clearly structured execution flow.

