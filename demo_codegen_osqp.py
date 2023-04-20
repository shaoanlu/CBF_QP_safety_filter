import sys
import pygame
import numpy as np
from scipy import sparse
import osqp


class Robot:
    def __init__(self, x0=0, y0=0, color=(0, 255, 0), vel=3, size=30):
        self.x = x0  # pos
        self.y = y0  # pos
        self.dx = 0  # control
        self.dy = 0  # control
        self.nominal_dx = 0  # user control
        self.nominal_dy = 0  # user control
        self.vel = vel
        self.color = color
        self.size = size
        self.arrow_size = 10
        self.is_collided = False

    def control(
        self, key=None, use_cbf=False, cbf_alpha=1e-1, penalty_slack=10, collision_objects=[]
    ):
        dx, dy = 0, 0
        self.nominal_dx, self.nominal_dy = 0, 0
        if key is not None:
            if key == pygame.K_LEFT:
                dx = -self.vel
            elif key == pygame.K_RIGHT:
                dx = self.vel
            elif key == pygame.K_UP:
                dy = -self.vel
            elif key == pygame.K_DOWN:
                dy = self.vel
            self.nominal_dx, self.nominal_dy = dx, dy

        if use_cbf:
            h = []
            coeffs = []
            for obj in collision_objects:
                h.append((self.x - obj.x) ** 2 + (self.y - obj.y) ** 2 - (self.size * 2.3) ** 2)
                coeffs.append([2 * self.x - 2 * obj.x, 2 * self.y - 2 * obj.y, penalty_slack])
            # Define problem data
            # P = sparse.csc_matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            q = np.array([-dx, -dy, 0])
            A = sparse.csc_matrix([c for c in coeffs] + [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            l = np.array([-cbf_alpha * h_ for h_ in h] + [-self.vel, -self.vel, -np.inf])
            u = np.array([np.inf for _ in h] + [self.vel, self.vel, np.inf])

            emosqp.update_lower_bound(l)
            emosqp.update_upper_bound(u)
            emosqp.update_lin_cost(q)
            emosqp.update_A(A.data, None, 0)

            # Solve problem
            x, y, status_val, iter, run_time = emosqp.solve()
            dx, dy, _ = x

        # update xy positions
        self.x += dx
        self.y += dy
        self.dx, self.dy = dx, dy

    def detect_collision(self, collision_objects=[]):
        self.is_collided = False
        if len(collision_objects) > 0:
            for obj in collision_objects:
                dist = np.linalg.norm(np.array([self.x, self.y] - np.array([obj.x, obj.y])))
                if dist <= (self.size * 2):
                    self.is_collided = True

    def draw(
        self,
        screen,
        draw_filtered_command=False,
        color1=(175, 175, 175),
        color2=(255, 0, 0),
    ):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.size)

        # draw arrows
        if self.dx != 0:
            pygame.draw.line(
                screen,
                color1,
                (self.x, self.y),
                (self.x + 20 * self.nominal_dx, self.y),
                width=10,
            )
            pygame.draw.polygon(
                screen,
                color1,
                [
                    (self.x + 20 * self.nominal_dx, self.y + self.arrow_size),
                    (self.x + 20 * self.nominal_dx, self.y - self.arrow_size),
                    (self.x + (self.arrow_size + 20) * self.nominal_dx, self.y),
                ],
            )
        if self.dy != 0:
            pygame.draw.line(
                screen,
                color1,
                (self.x, self.y),
                (self.x, self.y + 20 * self.nominal_dy),
                width=10,
            )
            pygame.draw.polygon(
                screen,
                color1,
                [
                    (self.x + self.arrow_size, self.y + 20 * self.nominal_dy),
                    (self.x - self.arrow_size, self.y + 20 * self.nominal_dy),
                    (self.x, self.y + (self.arrow_size + 20) * self.nominal_dy),
                ],
            )

        # draw output of safety filter
        if draw_filtered_command:
            if self.dx != 0:
                pygame.draw.line(
                    screen,
                    color2,
                    (self.x, self.y),
                    (self.x + 20 * self.dx, self.y),
                    width=10,
                )
                pygame.draw.polygon(
                    screen,
                    color2,
                    [
                        (self.x + 20 * self.dx, self.y + self.arrow_size),
                        (self.x + 20 * self.dx, self.y - self.arrow_size),
                        (self.x + (self.arrow_size + 20) * self.dx, self.y),
                    ],
                )
            if self.dy != 0:
                pygame.draw.line(
                    screen,
                    color2,
                    (self.x, self.y),
                    (self.x, self.y + 20 * self.dy),
                    width=10,
                )
                pygame.draw.polygon(
                    screen,
                    color2,
                    [
                        (self.x + self.arrow_size, self.y + 20 * self.dy),
                        (self.x - self.arrow_size, self.y + 20 * self.dy),
                        (self.x, self.y + (self.arrow_size + 20) * self.dy),
                    ],
                )


def run():
    # inits
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((340, 400))
    pygame.display.set_caption("Control barrier function demo")
    pygame.key.set_repeat(10)

    # init robots
    auto_robot = Robot(50, 350, (0, 255, 0), vel=3)
    static_robot = Robot(120, 200, (0, 0, 255))
    patrol_robot1 = Robot(230, 300, (0, 0, 255), vel=1)
    patrol_robot2 = Robot(300, 70, (0, 0, 255), vel=1)
    collision_objects = [static_robot, patrol_robot1, patrol_robot2]

    # control configs
    count = 0
    use_cbf = False
    cbf_alphas = [1e-1, 1e-2, 1]
    direction_patrol_robot1 = pygame.K_UP
    direction_patrol_robot2 = pygame.K_LEFT

    # set fonts
    default_font = pygame.font.get_fonts()[0]
    font = pygame.font.SysFont(default_font, 30)  #
    text_surface = font.render("Collide!", False, (255, 255, 255))
    cbf_font = pygame.font.SysFont(default_font, 20)
    cbf_on_text_surface = cbf_font.render("CBF ON (press x to turn off)", False, (0, 255, 125))
    cbf_off_text_surface = cbf_font.render("CBF OFF (press x to turn on)", False, (255, 0, 125))

    # main loop
    while True:
        count += 1

        # flip moving direction of patrol robot
        if count % 150 == 0:
            direction_patrol_robot1 = (
                pygame.K_DOWN if (direction_patrol_robot1 == pygame.K_UP) else pygame.K_UP
            )
            direction_patrol_robot2 = (
                pygame.K_RIGHT if (direction_patrol_robot2 == pygame.K_LEFT) else pygame.K_LEFT
            )
        count = 0 if count >= 1e10 else count

        # get user input
        pressed_key = None
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                sys.exit()
            if e.type == pygame.KEYDOWN:
                pressed_key = e.key
            if e.type == pygame.KEYUP:
                if e.key == pygame.K_x:
                    use_cbf = not use_cbf
                if e.key == pygame.K_z:
                    cbf_alphas.append(cbf_alphas.pop(0))  # roll left

        # move robots
        auto_robot.control(
            pressed_key,
            use_cbf=use_cbf,
            cbf_alpha=cbf_alphas[0],
            collision_objects=collision_objects,
        )
        static_robot.control()
        patrol_robot1.control(direction_patrol_robot1)
        patrol_robot2.control(direction_patrol_robot2)

        # detect collision
        auto_robot.detect_collision(collision_objects=collision_objects)

        # draw robots
        screen.fill((0, 0, 0))
        static_robot.draw(screen)
        patrol_robot1.draw(screen)
        patrol_robot2.draw(screen)
        auto_robot.draw(screen, draw_filtered_command=use_cbf)

        # draw texts
        if auto_robot.is_collided:
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
                (0, 20),
            )
        else:
            screen.blit(cbf_off_text_surface, (0, 0))

        # render and wait
        pygame.display.update()
        pygame.time.wait(10)


def code_gen():
    h = []
    coeffs = []
    for obj in range(3):
        h.append(0)
        coeffs.append([1, 1, 10])
    # Define problem data
    P = sparse.csc_matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    q = np.array([0, 0, 0])
    A = sparse.csc_matrix([c for c in coeffs] + [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    l = np.array([-2.2639e3, -6.1128e3, -8.7328e3] + [-3, -3, -np.inf])
    u = np.array([np.inf for _ in h] + [3, 3, np.inf])

    # Create an OSQP object
    prob = osqp.OSQP()
    prob.setup(P, q, A, l, u, verbose=False, time_limit=0)

    prob.codegen(
        "code", parameters="matrices", python_ext_name="emosqp"  # , project_type="MinGW Makefiles"
    )


if __name__ == "__main__":
    code_gen()
    import emosqp

    run()
