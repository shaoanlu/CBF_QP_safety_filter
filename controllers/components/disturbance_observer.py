"""
Memo
- Reduce grid resolution to speed up
- Reduce max iter to speed up
- increase poisson solving tol to speed up
"""
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_erosion
from scipy.sparse import diags
from scipy.sparse.linalg import spsolve
from scipy.ndimage import map_coordinates
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import imageio
from numba import njit, prange


class DisturbanceObserver:
    """
    Implementation of a basic disturbance observer using auxiliary states.

    Reference:
        Alan, Anil, et al. "Disturbance observers for robust safety-critical control with control barrier functions."
        IEEE Control Systems Letters 7 (2022): 1123-1128.
    """

    def __init__(self, gain: float = 1.0):
        self.gain = gain
        self.aux_state = 0.0
        self.est_state = 0.0

    def estimate(
        self,
        h: list[float],
        coeffs_dhdx: list[list[float]],
        control: np.ndarray,
        f_x: np.ndarray = np.zeros((2,)),
        g_x: np.ndarray = np.eye(2),
        velocity: float = 0.5,
        **kwargs,
    ) -> float:
        Lfh = np.array([coeffs_dhdx[0]]) @ f_x
        Lgh = np.array([coeffs_dhdx[0]]) @ g_x

        # Update auxiliary state
        self.aux_state += float((self.gain * (Lfh + Lgh @ control + self.est_state)).squeeze())

        # Update estimation state
        self.est_state = float(self.gain * h[0] - self.aux_state)

        # Clip the disturbance estimate
        max_disturbance = abs(coeffs_dhdx[0][0]) * velocity + abs(coeffs_dhdx[0][1]) * velocity
        self.est_state = np.clip(self.est_state, -max_disturbance, max_disturbance)

        return self.est_state


@njit(parallel=True)
def poisson_jacobi(f, boundary_mask, max_iter=1000, tol=1e-6):
    """
    Matrix-free Jacobi iteration for Poisson equation
    """
    ny, nx = f.shape
    h = f.copy()
    h_new = np.zeros_like(h)
    
    for iteration in range(max_iter):
        max_diff = 0.0
        
        for i in prange(ny):
            for j in range(nx):
                if boundary_mask[i, j]:
                    h_new[i, j] = 0.0
                else:
                    neighbors = 0.0
                    count = 0
                    if i > 0:
                        neighbors += h[i-1, j]
                        count += 1
                    if i < ny - 1:
                        neighbors += h[i+1, j]
                        count += 1
                    if j > 0:
                        neighbors += h[i, j-1]
                        count += 1
                    if j < nx - 1:
                        neighbors += h[i, j+1]
                        count += 1
                    
                    h_new[i, j] = (neighbors - f[i, j]) / 4.0
                    max_diff = max(max_diff, abs(h_new[i, j] - h[i, j]))
        
        h, h_new = h_new, h
        
        if max_diff < tol:
            break
    
    return h

def poisson_cbf_numba(occ_grid):
    """
    Numba-accelerated matrix-free version
    """
    ny, nx = occ_grid.shape
    
    dist_out = distance_transform_edt(occ_grid == 0)
    dist_in = distance_transform_edt(occ_grid == 1)
    signed_dist = dist_out - dist_in
    
    obs_interior = binary_erosion(occ_grid)
    obs_boundary = (occ_grid == 1) & (~obs_interior)
    
    f = -signed_dist / (np.max(np.abs(signed_dist)) + 1e-10)
    
    h = poisson_jacobi(f, obs_boundary)
    return h


def cbfqp_poisson(state, nominal_control, h_field, alpha=1.0, grid_res=0.05, do: DisturbanceObserver=None):
    """
    CF solution using the poisson field as h
    Args:
        state: np.array([x, y]) in meters
        nominal_control: np.array([vx, vy])
        h_field: dict with keys 'h', 'dx', 'dy'
        alpha: relaxation gain
        grid_res: meter per grid cell
    """
    # Map state to grid coordinates
    x, y = state
    ny, nx = h_field['h'].shape
    gx = x / grid_res
    gy = y / grid_res

    # Interpolate h and gradients
    h_val = map_coordinates(h_field['h'], [[gy], [gx]], order=1)[0]
    dhx = map_coordinates(h_field['dhx'], [[gy], [gx]], order=1)[0]
    dhy = map_coordinates(h_field['dhy'], [[gy], [gx]], order=1)[0]
    grad_h = np.array([dhx, dhy]).flatten()

    if do is not None:
        dist = do.estimate(h=[h_val], coeffs_dhdx=[[grad_h[0], grad_h[1]]], control=nominal_control.squeeze().reshape((-1, 1)), velocity=0.3)
    else:
        dist = 0.0

    # CBF constraint
    print(f"{dist=}")
    cbf_constr = (grad_h @ nominal_control + alpha * h_val - dist).squeeze()
    if cbf_constr >= 0:
        return nominal_control.squeeze()
    else:
        correction = (cbf_constr / (grad_h @ grad_h.T).squeeze()) * grad_h.T
        return (nominal_control - correction).squeeze()


def clfqp(state, target_state, alpha=1.0):
    """
    Args:
        state: numpy array of shape (2,) for XY position
        target_state: numpy array of shape (2, ) for goal XY position
        alpha: tuning parameter of the CLF, the larger the more aggressive
    """
    nominal_control = np.zeros((2,))
    v = np.linalg.norm(state - target_state) ** 2
    Lfv = np.zeros((1,))
    Lgv = 2 * (state - target_state)
    clf_constr = (Lfv + Lgv @ nominal_control + alpha * v).squeeze()
    if clf_constr <= 0:
        return nominal_control.squeeze()
    else:
        return (nominal_control - clf_constr / (Lgv @ Lgv.T).squeeze() * Lgv.T).squeeze()


def create_occupancy_grid(grid_res, nx, ny, obstacle_pos):
    """
    Create occupancy grid with one static and one moving obstacle
    Args:
        grid_res: resolution in meters per cell
        nx, ny: grid dimensions
        obstacle_pos: [x, y] position of moving obstacle center in meters
    """
    occ = np.zeros((ny, nx))
    
    # Static obstacle 1
    occ[int(2/grid_res):int(5/grid_res), int(3/grid_res):int(6/grid_res)] = 1
    
    # Moving obstacle 2 - circular shape
    obs_x, obs_y = obstacle_pos
    obs_radius = 1.0  # meters
    obs_radius_cells = int(obs_radius / grid_res)
    
    obs_gx = int(obs_x / grid_res)
    obs_gy = int(obs_y / grid_res)
    
    # Create circular obstacle
    # for i in range(max(0, obs_gy - obs_radius_cells), min(ny, obs_gy + obs_radius_cells + 1)):
    #     for j in range(max(0, obs_gx - obs_radius_cells), min(nx, obs_gx + obs_radius_cells + 1)):
    #         if np.sqrt((i - obs_gy)**2 + (j - obs_gx)**2) <= obs_radius_cells:
    #             occ[i, j] = 1
    # create retangular obstacle
    occ[obs_gy-obs_radius_cells: obs_gy+obs_radius_cells, obs_gx-obs_radius_cells:obs_gx+obs_radius_cells] = 1
    
    occ = np.flipud(occ)
    return occ, (obs_gx, obs_gy)


# Define the simulator
class OmniDirRobotSim:
    def __init__(self, world_width, world_height, robot_size):
        self.world_width = world_width
        self.world_height = world_height
        self.robot_size = robot_size
        self.position = np.array([0.5, 0.5])
        self.trajectory = []
        self.frames = []

    def move(self, vel_x, vel_y, dt=0.1):
        new_position = self.position + np.array([vel_x, vel_y]) * dt
        if 0 <= new_position[0] <= self.world_width and 0 <= new_position[1] <= self.world_height:
            self.position = new_position
        self.trajectory.append(tuple(self.position))

    def visualize(self, control_input, nominal_control, goal_position, occ_map, h_field, grid_res, obstacle_pos, save_frame=True):
        clear_output(wait=True)
        fig, ax = plt.subplots(figsize=(6, 6))

        # Background: show Poisson-CBF field h(x,y)
        extent = [0, occ_map.shape[1]*grid_res, 0, occ_map.shape[0]*grid_res]
        im = ax.imshow(h_field['h'], cmap='RdBu_r', origin='lower', extent=extent, alpha=0.7)
        ax.contour(
            np.linspace(0, extent[1], occ_map.shape[1]),
            np.linspace(0, extent[3], occ_map.shape[0]),
            h_field['h'],
            levels=[0],
            colors='black',
            linewidths=1.5
        )

        ax.set_xlim(0, self.world_width)
        ax.set_ylim(0, self.world_height)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.4)

        # Plot trajectory
        if self.trajectory:
            traj_x, traj_y = zip(*self.trajectory)
            ax.plot(traj_x, traj_y, 'b-', alpha=0.7, linewidth=1.5, label='Trajectory')

        # Draw robot and goal
        rx, ry = self.position
        robot = plt.Circle((rx, ry), self.robot_size, color='blue', alpha=0.8, label='Robot')
        ax.add_patch(robot)

        gx, gy = goal_position
        goal = plt.Circle((gx, gy), 0.25, color='orange', alpha=1.0, label='Goal')
        ax.add_patch(goal)
        
        # Draw moving obstacle
        obs_x, obs_y = obstacle_pos
        obs_x, obs_y = obs_x * grid_res, obs_y * grid_res
        # moving_obs = plt.Circle((obs_x, obs_y), 1.0, color='purple', alpha=0.6, label='Moving Obstacle')
        moving_obs = plt.Rectangle((obs_x-1, obs_y-1), 2.0, 2.0, color='purple', alpha=0.6, label='Moving Obstacle')
        ax.add_patch(moving_obs)

        # Control arrows
        vx_nom, vy_nom = nominal_control
        ax.arrow(rx, ry, vx_nom, vy_nom, head_width=0.2, fc='r', ec='r', label='Nominal', alpha=0.8)

        vx_safe, vy_safe = control_input
        ax.arrow(rx, ry, vx_safe, vy_safe, head_width=0.2, fc='g', ec='g', label='Safe', alpha=0.8)

        ax.legend(loc='lower right', fontsize=8)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("CLF-CBF Poisson Field with Moving Obstacle")

        if save_frame:
            fig.canvas.draw()
            frame = np.array(fig.canvas.renderer.buffer_rgba())
            self.frames.append(frame)

        display(fig)
        plt.close(fig)


    def save_gif(self, filename="robot_simulation.gif", fps=20):
        imageio.mimsave(filename, self.frames, fps=fps, loop=0)


# Setup parameters
grid_res = 0.05  # 5 cm per cell
nx, ny = int(10 / grid_res), int(10 / grid_res)

# Run simulation
robot = OmniDirRobotSim(10, 10, 0.2)
goal = np.array([8.0, 8.0])

# Moving obstacle parameters
obstacle_start = np.array([8.0, 5.5])
obstacle_velocity = np.array([-0.025, 0.04])  # moves upward slowly

num_steps = 175
update_h_every = 1  # Recompute h field every N steps

do = DisturbanceObserver()

for step in range(num_steps):
    # Update obstacle position
    obstacle_pos = obstacle_start + obstacle_velocity * step
    
    # Keep obstacle within bounds
    obstacle_pos[0] = np.clip(obstacle_pos[0], 1.0, 9.0)
    obstacle_pos[1] = np.clip(obstacle_pos[1], 1.0, 9.0)
    
    # Recompute h field periodically (computationally expensive)
    if step % update_h_every == 0:
        occ, obs_grid_pos = create_occupancy_grid(grid_res, nx, ny, obstacle_pos)
        h_im = poisson_cbf_numba(occ)
        h_world = np.flipud(h_im)
        dhy_world, dhx_world = np.gradient(h_world, grid_res, grid_res)
        h_field = {'h': h_world, 'dhx': dhx_world, 'dhy': dhy_world}
    
    # Compute control
    nominal_control = clfqp(robot.position, goal, alpha=1.0)
    assert nominal_control.shape[0] == 2
    control_input = cbfqp_poisson(robot.position, nominal_control, h_field, alpha=0.3, grid_res=grid_res, do=do)
    
    # Update robot
    robot.move(*control_input)
    robot.visualize(control_input, nominal_control, goal, occ, h_field, grid_res, obs_grid_pos)

robot.save_gif("poisson_cbf_moving_obstacle.gif")
