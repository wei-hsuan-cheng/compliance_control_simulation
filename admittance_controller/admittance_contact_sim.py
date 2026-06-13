"""
1-DoF Admittance Control Simulation With Stiff Environment Contact

Architecture
------------
x_r(t) -> outer admittance -> x_cmd(t) -> perfect inner position servo -> x(t)

The admittance loop generates a deviation dx_adm from the nominal reference x_r:

    x_cmd = x_r + dx_adm

With perfect low-level tracking:

    x = x_cmd = x_r + dx_adm

The outer admittance dynamics are acceleration-driven:

    M_a * dx_adm_ddot + D_a * dx_adm_dot + K_a * dx_adm = F_env

so:

    dx_adm_ddot = (F_env - D_a * dx_adm_dot - K_a * dx_adm) / M_a

The state is integrated using semi-implicit Euler:

    dx_adm_dot[k+1] = dx_adm_dot[k] + dx_adm_ddot[k] * dt
    dx_adm[k+1]     = dx_adm[k]     + dx_adm_dot[k+1] * dt

Sign convention
---------------
+x points into the wall.

Wall position: x_w

Penetration:

    delta = x - x_w

Contact if:

    delta > 0

Environment force:

    F_env = -K_e * delta - D_e * delta_dot

Since contact is unilateral, this simulator clamps the force so that the wall
does not pull the robot into contact:

    F_env = min(0, F_env_raw)

Author: generated as a reference simulation script.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Callable, Dict, List, Optional, Tuple

import math
import csv


@dataclass
class AdmittanceParams:
    """Outer admittance parameters."""
    M_a: float = 2.0       # virtual inertia [kg]
    D_a: float = 80.0      # virtual damping [N*s/m]
    K_a: float = 800.0     # virtual stiffness [N/m]

    def validate(self) -> None:
        if self.M_a <= 0.0:
            raise ValueError("M_a must be positive.")
        if self.D_a < 0.0:
            raise ValueError("D_a must be non-negative.")
        if self.K_a < 0.0:
            raise ValueError("K_a must be non-negative.")


@dataclass
class EnvironmentParams:
    """Kelvin-Voigt unilateral wall parameters."""
    x_w: float = 0.10       # wall position [m]
    K_e: float = 50_000.0   # environment stiffness [N/m]
    D_e: float = 100.0      # environment damping [N*s/m]
    clamp_tensile_force: bool = True

    def validate(self) -> None:
        if self.K_e < 0.0:
            raise ValueError("K_e must be non-negative.")
        if self.D_e < 0.0:
            raise ValueError("D_e must be non-negative.")


@dataclass
class SimulationParams:
    """Discrete-time simulation configuration."""
    dt: float = 0.0005      # sampling time [s]
    t_final: float = 2.0    # final time [s]

    def validate(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive.")
        if self.t_final <= 0.0:
            raise ValueError("t_final must be positive.")


@dataclass
class AdmittanceState:
    """State of the admittance deviation."""
    dx: float = 0.0         # admittance position deviation [m]
    dx_dot: float = 0.0     # admittance velocity deviation [m/s]
    dx_ddot: float = 0.0    # admittance acceleration deviation [m/s^2]


@dataclass
class StepLog:
    """One timestep of simulation data."""
    t: float
    x_r: float
    x_r_dot: float
    x_r_ddot: float

    dx: float
    dx_dot: float
    dx_ddot: float

    x: float
    x_dot: float

    x_w: float
    penetration: float
    penetration_dot: float
    F_env: float
    in_contact: bool


class TeleopReference:
    """
    Base class for nominal teleop/planner reference generators.

    A reference generator returns:

        x_r(t), x_r_dot(t), x_r_ddot(t)
    """

    def sample(self, t: float) -> Tuple[float, float, float]:
        raise NotImplementedError


class RampThenHoldReference(TeleopReference):
    """
    Move with constant velocity until a target reference position, then hold.

    This is useful for simulating:
        "teleop pushes in +x, contacts the wall, then stops at
         a small nominal penetration beyond the wall."
    """

    def __init__(
        self,
        x0: float = 0.0,
        velocity: float = 0.08,
        x_hold: float = 0.105,
    ) -> None:
        self.x0 = x0
        self.velocity = velocity
        self.x_hold = x_hold

        if velocity <= 0.0:
            raise ValueError("velocity must be positive for RampThenHoldReference.")
        if x_hold < x0:
            raise ValueError("x_hold must be greater than or equal to x0.")

        self.t_hold = (x_hold - x0) / velocity

    def sample(self, t: float) -> Tuple[float, float, float]:
        if t < self.t_hold:
            return self.x0 + self.velocity * t, self.velocity, 0.0
        return self.x_hold, 0.0, 0.0


class SmoothStepReference(TeleopReference):
    """
    Smoothly move from x0 to x1 over duration T, then hold.

    Uses the polynomial:
        s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5

    where tau = t / T.

    This gives zero velocity and acceleration at the start and end.
    """

    def __init__(
        self,
        x0: float = 0.0,
        x1: float = 0.105,
        duration: float = 1.5,
    ) -> None:
        if duration <= 0.0:
            raise ValueError("duration must be positive.")

        self.x0 = x0
        self.x1 = x1
        self.duration = duration

    def sample(self, t: float) -> Tuple[float, float, float]:
        if t <= 0.0:
            return self.x0, 0.0, 0.0
        if t >= self.duration:
            return self.x1, 0.0, 0.0

        T = self.duration
        tau = t / T
        dx = self.x1 - self.x0

        s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        s_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / T
        s_ddot = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (T * T)

        x_r = self.x0 + dx * s
        x_r_dot = dx * s_dot
        x_r_ddot = dx * s_ddot

        return x_r, x_r_dot, x_r_ddot


class FunctionReference(TeleopReference):
    """
    Wrap a custom user-defined reference function.

    The function must have signature:

        f(t) -> (x_r, x_r_dot, x_r_ddot)
    """

    def __init__(self, func: Callable[[float], Tuple[float, float, float]]) -> None:
        self.func = func

    def sample(self, t: float) -> Tuple[float, float, float]:
        return self.func(t)


class UnilateralWallEnvironment:
    """Unilateral Kelvin-Voigt wall model."""

    def __init__(self, params: EnvironmentParams) -> None:
        params.validate()
        self.params = params

    def compute_force(
        self,
        x: float,
        x_dot: float,
        x_w_dot: float = 0.0,
    ) -> Tuple[float, float, float, bool]:
        """
        Compute environment force.

        Returns:
            F_env, penetration, penetration_dot, in_contact
        """
        p = self.params
        penetration = x - p.x_w
        penetration_dot = x_dot - x_w_dot

        if penetration <= 0.0:
            return 0.0, penetration, penetration_dot, False

        F_raw = -p.K_e * penetration - p.D_e * penetration_dot

        if p.clamp_tensile_force:
            F_env = min(0.0, F_raw)
        else:
            F_env = F_raw

        return F_env, penetration, penetration_dot, True


class OuterAdmittanceController:
    """
    Acceleration-driven outer admittance controller.

    Dynamics:

        M_a * dx_ddot + D_a * dx_dot + K_a * dx = F_env
    """

    def __init__(
        self,
        params: AdmittanceParams,
        initial_state: Optional[AdmittanceState] = None,
    ) -> None:
        params.validate()
        self.params = params
        self.state = initial_state if initial_state is not None else AdmittanceState()

    def compute_acceleration(self, F_env: float) -> float:
        p = self.params
        s = self.state

        return (F_env - p.D_a * s.dx_dot - p.K_a * s.dx) / p.M_a

    def step_semi_implicit_euler(self, F_env: float, dt: float) -> AdmittanceState:
        """
        Semi-implicit Euler:
            v_{k+1} = v_k + a_k dt
            x_{k+1} = x_k + v_{k+1} dt
        """
        s = self.state

        s.dx_ddot = self.compute_acceleration(F_env)
        s.dx_dot += s.dx_ddot * dt
        s.dx += s.dx_dot * dt

        return s

    def reset(self, state: Optional[AdmittanceState] = None) -> None:
        self.state = state if state is not None else AdmittanceState()


class PerfectInnerPositionServoPlant:
    """
    Perfect inner position servo.

    Command:
        x_cmd = x_r + dx_adm

    Assumption:
        x = x_cmd
        x_dot = x_r_dot + dx_adm_dot
    """

    @staticmethod
    def output(
        x_r: float,
        x_r_dot: float,
        admittance_state: AdmittanceState,
    ) -> Tuple[float, float]:
        x = x_r + admittance_state.dx
        x_dot = x_r_dot + admittance_state.dx_dot
        return x, x_dot


class AdmittanceContactSimulator:
    """
    Full simulation wrapper.

    At each timestep:
        1. sample nominal reference x_r
        2. compute actual x = x_r + dx_adm
        3. compute environment force
        4. compute admittance acceleration
        5. integrate dx_adm twice with semi-implicit Euler
    """

    def __init__(
        self,
        admittance_params: AdmittanceParams,
        environment_params: EnvironmentParams,
        simulation_params: SimulationParams,
        reference: TeleopReference,
        initial_state: Optional[AdmittanceState] = None,
    ) -> None:
        simulation_params.validate()

        self.sim_params = simulation_params
        self.reference = reference
        self.controller = OuterAdmittanceController(
            admittance_params,
            initial_state=initial_state,
        )
        self.environment = UnilateralWallEnvironment(environment_params)
        self.plant = PerfectInnerPositionServoPlant()

    def contact_mode_eigenvalues(self) -> Tuple[complex, complex]:
        """
        Eigenvalues of the linearized closed-loop contact dynamics.

        During active contact:
            M_a dx_ddot + (D_a + D_e) dx_dot + (K_a + K_e) dx
                = -K_e (x_r - x_w) - D_e x_r_dot

        Characteristic equation:
            M_a lambda^2 + (D_a + D_e) lambda + (K_a + K_e) = 0
        """
        a = self.controller.params
        e = self.environment.params

        M = a.M_a
        D = a.D_a + e.D_e
        K = a.K_a + e.K_e

        disc = D * D - 4.0 * M * K
        sqrt_disc = complex(disc, 0.0) ** 0.5

        lam1 = (-D + sqrt_disc) / (2.0 * M)
        lam2 = (-D - sqrt_disc) / (2.0 * M)

        return lam1, lam2

    def contact_mode_omega_zeta(self) -> Tuple[float, float]:
        """Return contact natural frequency and damping ratio."""
        a = self.controller.params
        e = self.environment.params

        M = a.M_a
        D = a.D_a + e.D_e
        K = a.K_a + e.K_e

        omega_n = math.sqrt(K / M)
        zeta = D / (2.0 * math.sqrt(M * K))

        return omega_n, zeta

    def run(self) -> List[StepLog]:
        dt = self.sim_params.dt
        n_steps = int(math.floor(self.sim_params.t_final / dt)) + 1

        logs: List[StepLog] = []

        for k in range(n_steps):
            t = k * dt

            x_r, x_r_dot, x_r_ddot = self.reference.sample(t)

            # Current plant output before integrating the admittance state.
            x, x_dot = self.plant.output(x_r, x_r_dot, self.controller.state)

            F_env, penetration, penetration_dot, in_contact = (
                self.environment.compute_force(x, x_dot)
            )

            # Compute admittance acceleration from current force and current state.
            dx_ddot = self.controller.compute_acceleration(F_env)

            # Log current continuous-time values before state update.
            s = self.controller.state
            logs.append(
                StepLog(
                    t=t,
                    x_r=x_r,
                    x_r_dot=x_r_dot,
                    x_r_ddot=x_r_ddot,
                    dx=s.dx,
                    dx_dot=s.dx_dot,
                    dx_ddot=dx_ddot,
                    x=x,
                    x_dot=x_dot,
                    x_w=self.environment.params.x_w,
                    penetration=penetration,
                    penetration_dot=penetration_dot,
                    F_env=F_env,
                    in_contact=in_contact,
                )
            )

            # Integrate admittance state using semi-implicit Euler.
            self.controller.step_semi_implicit_euler(F_env, dt)

        return logs


def logs_to_dict(logs: List[StepLog]) -> Dict[str, List[float]]:
    """Convert logs to a dict of lists for plotting or analysis."""
    result: Dict[str, List[float]] = {}

    for key in asdict(logs[0]).keys():
        result[key] = []

    for row in logs:
        row_dict = asdict(row)
        for key, value in row_dict.items():
            result[key].append(value)

    return result


def save_logs_csv(logs: List[StepLog], filepath: str) -> None:
    """Save simulation logs to CSV."""
    if not logs:
        raise ValueError("No logs to save.")

    fieldnames = list(asdict(logs[0]).keys())

    with open(filepath, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in logs:
            writer.writerow(asdict(row))


def print_summary(sim: AdmittanceContactSimulator, logs: List[StepLog]) -> None:
    """Print useful simulation summary."""
    omega_n, zeta = sim.contact_mode_omega_zeta()
    lam1, lam2 = sim.contact_mode_eigenvalues()

    max_pen = max(max(row.penetration, 0.0) for row in logs)
    min_force = min(row.F_env for row in logs)

    final = logs[-1]

    print("=== Contact Mode Analysis ===")
    print(f"omega_n: {omega_n:.3f} rad/s")
    print(f"zeta:    {zeta:.3f}")
    print(f"lambda1: {lam1:.3f}")
    print(f"lambda2: {lam2:.3f}")
    print()
    print("=== Simulation Summary ===")
    print(f"max penetration: {max_pen:.6f} m")
    print(f"max contact force magnitude: {abs(min_force):.3f} N")
    print(f"final x_r: {final.x_r:.6f} m")
    print(f"final dx:  {final.dx:.6f} m")
    print(f"final x:   {final.x:.6f} m")
    print(f"final penetration: {max(final.penetration, 0.0):.6f} m")
    print(f"final F_env: {final.F_env:.3f} N")


def run_example() -> None:
    """
    Example scenario:

    - Wall at x_w = 0.10 m.
    - Teleop reference ramps from 0 to 0.105 m.
    - So the nominal reference stops 5 mm beyond the wall.
    - The admittance loop yields backward under the wall contact force.
    """

    admittance_params = AdmittanceParams(
        M_a=2.0,
        D_a=80.0,
        K_a=800.0,
    )

    environment_params = EnvironmentParams(
        x_w=0.10,
        K_e=50_000.0,
        D_e=100.0,
        clamp_tensile_force=True,
    )

    simulation_params = SimulationParams(
        dt=0.0005,
        t_final=2.0,
    )

    reference = RampThenHoldReference(
        x0=0.0,
        velocity=0.08,
        x_hold=0.105,
    )

    sim = AdmittanceContactSimulator(
        admittance_params=admittance_params,
        environment_params=environment_params,
        simulation_params=simulation_params,
        reference=reference,
    )

    logs = sim.run()
    print_summary(sim, logs)

    csv_path = "admittance_contact_sim_log.csv"
    save_logs_csv(logs, csv_path)
    print(f"\nSaved log to: {csv_path}")

    # Optional plotting. The script still works if matplotlib is unavailable.
    try:
        import matplotlib.pyplot as plt

        data = logs_to_dict(logs)

        plt.figure()
        plt.plot(data["t"], data["x_r"], label="x_r nominal")
        plt.plot(data["t"], data["x"], label="x actual / x_cmd")
        plt.axhline(environment_params.x_w, linestyle="--", label="wall x_w")
        plt.xlabel("time [s]")
        plt.ylabel("position [m]")
        plt.title("Reference and Actual Position")
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(data["t"], data["dx"], label="admittance deviation dx")
        plt.xlabel("time [s]")
        plt.ylabel("dx [m]")
        plt.title("Admittance Deviation")
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(data["t"], data["F_env"], label="F_env")
        plt.xlabel("time [s]")
        plt.ylabel("force [N]")
        plt.title("Environment Contact Force")
        plt.legend()
        plt.grid(True)

        plt.figure()
        penetration_positive = [max(p, 0.0) for p in data["penetration"]]
        plt.plot(data["t"], penetration_positive, label="penetration")
        plt.xlabel("time [s]")
        plt.ylabel("penetration [m]")
        plt.title("Wall Penetration")
        plt.legend()
        plt.grid(True)

        plt.show()

    except ImportError:
        print("matplotlib is not installed; skipping plots.")


if __name__ == "__main__":
    run_example()
