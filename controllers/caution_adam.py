import numpy as np


class CautionAdam:
    """
    @misc{liang2024online,
        title={Cautious Optimizers: Improving Training with One Line of Code},
        author={Kaizhao Liang and Lizhang Chen and Bo Liu and Qiang Liu},
        year={2024},
        eprint={https://arxiv.org/abs/2411.16085},
        archivePrefix={arXiv},
        primaryClass={cs.LG}
    }
    """

    def __init__(self, lr=0.1, beta1=0.9, beta2=0.999, epsilon=1e-8):
        self.lr = lr
        self.beta1 = beta1
        self.beta2 = beta2
        self.epsilon = epsilon
        self.m = {}  # First moment vector
        self.v = {}  # Second moment vector
        self.t = 0  # Time step

    def update(self, params, grads):
        self.t += 1
        updates = {}

        for key in params:
            if key not in self.m:
                self.m[key] = np.zeros_like(params[key])
                self.v[key] = np.zeros_like(params[key])

            # Standard Adam updates
            self.m[key] = self.beta1 * self.m[key] + (1 - self.beta1) * grads[key]
            self.v[key] = self.beta2 * self.v[key] + (1 - self.beta2) * (grads[key] ** 2)

            # Bias correction
            m_hat = self.m[key] / (1 - self.beta1**self.t)
            v_hat = self.v[key] / (1 - self.beta2**self.t)

            # Compute the Adam update direction
            u = m_hat / (np.sqrt(v_hat) + self.epsilon)

            # Caution step: filter updates using the provided rule
            m_caution = (u * grads[key] > 0).astype(grads[key].dtype)  # Select only updates aligned with gradients
            cautious_update = u * m_caution / (np.mean(m_caution) + self.epsilon)

            # Apply cautious update
            updates[key] = params[key] - self.lr * cautious_update

        return updates


"""
# Example usage
x1 = np.array([1.0, 2.0, 3.0])
x2 = np.array([4.0, 5.0, 6.0])
y0 = np.array([10.0, 12.0, 14.0])

# Initialize parameters
params = {"a": np.random.randn(), "b": np.random.randn()}

# CautAdam optimizer
optimizer = CautionAdam(lr=0.1)

# Training loop
num_epochs = 1000
for epoch in range(num_epochs):
    # Compute predictions
    y = params["a"] * x1 + params["b"] * x2

    # Compute loss
    loss = np.mean((y0 - y) ** 2)

    # Compute gradients
    grad_a = -2 * np.mean((y0 - y) * x1)
    grad_b = -2 * np.mean((y0 - y) * x2)

    # Update parameters using CautAdam
    grads = {"a": grad_a, "b": grad_b}
    params = optimizer.update(params, grads)

    if epoch % 100 == 0:
        print(f"Epoch {epoch}, Loss: {loss:.6f}, a: {params['a']:.4f}, b: {params['b']:.4f}")

# Final results
print(f"Final parameters: a = {params['a']:.4f}, b = {params['b']:.4f}")
"""
