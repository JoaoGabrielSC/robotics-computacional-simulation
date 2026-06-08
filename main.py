"""
Autor: João Gabriel Santos Custodio
Matricula: 2019107750
Disciplina: Robótica Móvel
Atividade: Simulação de um robô omnidirecional com controle cinemático para seguir uma trajetória de
referência enquanto evita um obstáculo usando um campo potencial.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass


@dataclass
class SimulationConfig:
    dt: float = 0.05
    t_end: float = 60.0
    k_att: float = 2.0
    k_rep: float = 15.0
    rho_0: float = 2.0


class OmnidirectionalRobot:
    def __init__(self, x: float, y: float):
        self.position = np.array([x, y], dtype=float)
        self.history = [self.position.copy()]

    def update(self, u: np.ndarray, dt: float):
        self.position += u * dt
        self.history.append(self.position.copy())


class TrajectoryGenerator:
    @staticmethod
    def get_desired_state(t: float) -> tuple[np.ndarray, np.ndarray]:
        omega1 = 2 * np.pi / 20.0
        omega2 = 4 * np.pi / 20.0

        x_d = 8.0 + 5.0 * np.cos(omega1 * t)
        y_d = 5.0 * np.sin(omega2 * t)

        vx_d = -5.0 * omega1 * np.sin(omega1 * t)
        vy_d = 5.0 * omega2 * np.cos(omega2 * t)

        return np.array([x_d, y_d]), np.array([vx_d, vy_d])


class KinematicController:
    def __init__(self, config: SimulationConfig, obs_pos: np.ndarray):
        self.config = config
        self.obs_pos = obs_pos

    def compute_control(self, current_pos: np.ndarray, t: float) -> np.ndarray:
        p_d, v_d = TrajectoryGenerator.get_desired_state(t)

        error = p_d - current_pos
        u_att = v_d + self.config.k_att * error

        distance_to_obs = np.linalg.norm(current_pos - self.obs_pos)
        u_rep = np.zeros(2)

        if 0 < distance_to_obs < self.config.rho_0:
            direction = (current_pos - self.obs_pos) / distance_to_obs
            magnitude = (
                self.config.k_rep
                * (1.0 / distance_to_obs - 1.0 / self.config.rho_0)
                * (1.0 / (distance_to_obs**2))
            )
            u_rep = magnitude * direction

        return u_att + u_rep


def run_simulation():
    config = SimulationConfig()
    robot = OmnidirectionalRobot(11.0, 0.0)
    obs_pos = np.array([10.0, 3.75])
    controller = KinematicController(config, obs_pos)

    time_steps = np.arange(0, config.t_end, config.dt)

    x_d_hist = []
    y_d_hist = []

    for t in time_steps:
        u = controller.compute_control(robot.position, t)
        robot.update(u, config.dt)
        p_d, _ = TrajectoryGenerator.get_desired_state(t)
        x_d_hist.append(p_d[0])
        y_d_hist.append(p_d[1])

    animate_results(
        time_steps, np.array(robot.history), x_d_hist, y_d_hist, obs_pos, config.dt
    )


def animate_results(time_steps, history, x_d_hist, y_d_hist, obs_pos, dt):
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(x_d_hist, y_d_hist, "r--", label="Trajetoria de Referencia")
    ax.plot(obs_pos[0], obs_pos[1], "ko", markersize=10, label="Obstaculo")
    ax.plot(history[0, 0], history[0, 1], "g^", markersize=10, label="Posicao Inicial")

    (robot_line,) = ax.plot([], [], "b-", linewidth=2, label="Trajetoria do Robo")
    (robot_point,) = ax.plot([], [], "bo", markersize=8)

    ax.set_xlabel("Eixo X [m]")
    ax.set_ylabel("Eixo Y [m]")
    ax.set_title("Simulacao com Animacao - Evasao de Obstaculo")
    ax.legend()
    ax.grid(True)
    ax.axis("equal")

    def init():
        robot_line.set_data([], [])
        robot_point.set_data([], [])
        return robot_line, robot_point

    def update(frame):
        robot_line.set_data(history[:frame, 0], history[:frame, 1])
        robot_point.set_data([history[frame, 0]], [history[frame, 1]])
        return robot_line, robot_point

    frames = range(len(time_steps))

    _ = animation.FuncAnimation(
        fig,
        update,
        frames=frames,
        init_func=init,
        blit=True,
        interval=int(dt * 1000),  # 50 ms
    )

    plt.show()


if __name__ == "__main__":
    run_simulation()
