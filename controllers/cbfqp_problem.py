import numpy as np
from scipy import sparse
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class QPProblemData:
    """Data structure for Quadratic Programming problem matrices and vectors."""

    P: sparse.csc_matrix  # Quadratic cost matrix
    q: np.ndarray  # Linear cost vector
    A: sparse.csc_matrix  # Constraint matrix
    l: np.ndarray  # Lower bounds vector
    u: np.ndarray  # Upper bounds vector


class CBFQPFormulation:
    """
    Handles the formulation of Quadratic Programming (QP) problems for Control Barrier Functions (CBF).

                minimize  || u - u_nom ||^2 + k * δ
                    u, δ
                s.t.
                        h'(x) ≥ -𝛼 * h(x) - δ
                        u_min ≤ u ≤ u_max
                            0 ≤ δ ≤ inf
    where
        u = [ux, uy] is the control input in x and y axis respectively.
        δ is the slack variable
        h(x) is the control barrier function and h'(x) its derivative

    The problem above can be formulated as QP (ref: https://osqp.org/docs/solver/index.html)

                minimize 1/2 * x^T * Px + q^T x
                    x
                s.t.
                            l ≤ Ax ≤ u
    where
        x = [ux, uy, δ]
    """

    def __init__(self, nx: int, nh: int):
        """
        Initialize the QP problem formulation.

        Args:
            nx (int): Number of state variables (control dimensions)
            nh (int): Number of control barrier functions
        """
        self.nx = nx  # Number of states (control dimensions)
        self.nh = nh  # Number of barrier functions

    def create_matrices(
        self,
        h: List[float],
        coeffs_dhdx: List[List[float]],
        nominal_control: np.ndarray,
        max_control: np.ndarray,
        min_control: np.ndarray,
        cbf_alpha: float,
        slack_penalty: float,
        disturbance_h_dot: Optional[List[float]] = None,
    ) -> QPProblemData:
        """
        Create all matrices and vectors needed for the QP problem.
        P: shape (nx, nx)
        q: shape (nx,)
        A: shape (nx+nh, nx)
        l: shape (nh+nx,)
        u: shape (nh+nx,)
        nx: number of state (control dim)
        nh: number of control barrier functions

        Args:
            h (List[float]): Barrier function values
            coeffs_dhdx (List[List[float]]): Coefficients of barrier function derivatives
            nominal_control (np.ndarray): Nominal control inputs
            max_control (np.ndarray): Maximum allowed control values
            min_control (np.ndarray): Minimum allowed control values
            cbf_alpha (float): CBF constraint parameter
            slack_penalty (float): Penalty coefficient for slack variables
            disturbance_h_dot (Optional[List[float]]): Disturbance terms for barrier functions

        Returns:
            QPProblemData: Container with all matrices and vectors for the QP problem
        """
        if disturbance_h_dot is None:
            disturbance_h_dot = [0.0] * self.nh

        P = self._create_cost_matrix()
        q = self._create_cost_vector(nominal_control, slack_penalty)
        A = self._create_constraint_matrix(coeffs_dhdx)
        l, u = self._create_bound_vectors(h, disturbance_h_dot, cbf_alpha, max_control, min_control)

        return QPProblemData(P=P, q=q, A=A, l=l, u=u)

    def _create_cost_matrix(self) -> sparse.csc_matrix:
        """
        Create the quadratic cost matrix P.

        The matrix P defines the quadratic term in the objective function:
        1/2 x^T P x, where x = [u_x, u_y, δ]

        Returns:
            sparse.csc_matrix: Sparse matrix P of shape (nx + nh, nx + nh)
        """
        P = np.eye(self.nx + self.nh)
        P[self.nx :, self.nx :] = 0  # No quadratic penalty on slack variables
        return sparse.csc_matrix(P)

    def _create_cost_vector(self, nominal_control: np.ndarray, slack_penalty: float) -> np.ndarray:
        """
        Create the linear cost vector q.

        Args:
            nominal_control (np.ndarray): Nominal control inputs
            slack_penalty (float): Penalty coefficient for slack variables

        Returns:
            np.ndarray: Vector q of shape (nx + nh,)
        """
        return np.hstack(
            [
                -nominal_control,  # Minimize deviation from nominal control
                np.ones(self.nh) * slack_penalty,  # Penalize slack variables
            ]
        )

    def _create_constraint_matrix(self, coeffs_dhdx: List[List[float]]) -> sparse.csc_matrix:
        """
        Create the constraint matrix A.

        Args:
            coeffs_dhdx (List[List[float]]): Coefficients of barrier function derivatives

        Returns:
            sparse.csc_matrix: Matrix A of shape (nx + nh, nx + nh)
        """
        A = np.vstack(
            [np.array(coeffs_dhdx), np.eye(self.nx + self.nh)]  # CBF constraints  # Control and slack variable bounds
        )
        return sparse.csc_matrix(A)

    def _create_bound_vectors(
        self,
        h: List[float],
        disturbance_h_dot: List[float],
        cbf_alpha: float,
        max_control: np.ndarray,
        min_control: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Create the lower and upper bound vectors l and u.

        Args:
            h (List[float]): Barrier function values
            disturbance_h_dot (List[float]): Disturbance terms for barrier functions
            cbf_alpha (float): CBF constraint parameter
            max_control (np.ndarray): Maximum allowed control values
            min_control (np.ndarray): Minimum allowed control values

        Returns:
            Tuple[np.ndarray, np.ndarray]: Lower and upper bound vectors
        """
        # Lower bounds: CBF constraints and control/slack bounds
        l = np.concatenate(
            [[-cbf_alpha * h_ - d_ for h_, d_ in zip(h, disturbance_h_dot)], min_control, np.zeros(self.nh)]
        )

        # Upper bounds: Infinity for CBF constraints, control bounds, and slack bounds
        u = np.concatenate([np.full(self.nh, np.inf), max_control, np.full(self.nh, np.inf)])

        return l, u
