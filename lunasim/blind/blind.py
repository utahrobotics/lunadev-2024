from multiprocessing import Pool
from typing import Callable, Iterable, Tuple, Union
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
import numpy as np
import time
import matplotlib.pyplot as plt
from math import sqrt, acos, pi


def fx(x, dt):
    # state transition function - predicts next state
    F = np.array([[1, dt, dt ** 2 / 2, 0, 0, 0],
                  [0, 1, dt, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 1, dt, dt ** 2 / 2],
                  [0, 0, 0, 0, 1, dt],
                  [0, 0, 0, 0, 0, 1]], dtype=float)
    return np.dot(F, x)


def imu_hx(x):
    # Acceleration
    return np.array([x[2], x[5]])


def wheels_hx(x):
    # Velocity
    return np.array([x[1], x[4]])


DELTA = 0.01
MAX_DRIFT = 10.0


def gen_ukf():
    points = MerweScaledSigmaPoints(6, alpha=.1, beta=2., kappa=-1)
    kf = UnscentedKalmanFilter(dim_x=6, dim_z=2, dt=DELTA, fx=fx, hx=imu_hx, points=points)
    kf.x = np.zeros(6)   # initial state
    kf.P *= 0.2
    kf.Q = Q_discrete_white_noise(dim=3, dt=DELTA, var=0.01**2, block_size=2)
    return kf


def run_once(plan: np.ndarray, gen_kf, imu0_std_dev: float, wheels0_std_dev: float, name: str):
    kf = gen_kf()
    kf.x[0] = plan[0][0][0]
    kf.x[3] = plan[0][0][1]
    imu0_cov = np.diag([imu0_std_dev ** 2, imu0_std_dev ** 2])    # 1 standard
    wheels0_cov = np.diag([wheels0_std_dev ** 2, wheels0_std_dev ** 2])

    for i, (pos, vel, accel) in enumerate(plan):
        kf.predict()
        kf.hx = imu_hx
        kf.update(
            accel + np.random.randn(2) * imu0_std_dev,
            imu0_cov
        )
        kf.predict()
        kf.hx = wheels_hx
        kf.update(
            vel + np.random.randn(2) * wheels0_std_dev,
            wheels0_cov
        )
        drift = np.linalg.norm(np.array([kf.x[0], kf.x[3]]) - pos)
        if i >= len(plan) // 2 and drift >= MAX_DRIFT:
            return (name, None)

    return (name, drift)


def run_args(args):
    return run_once(*args)


def plot_val(plan: np.ndarray, idx: int):
    _, ax = plt.subplots()

    runtime = 0
    xs = []
    ys = []

    for values in plan:
        xs.append(runtime)
        ys.append(np.linalg.norm(values[idx]))
        runtime += DELTA

    ax.plot(xs, ys)
    plt.show()


class TravelSection:
    def __init__(
        self,
        from_point: np.ndarray,
        to_point: np.ndarray,
        top_speed=1.5,
        acceleration=3.0
    ):
        assert type(from_point) == type(to_point) == np.ndarray
        self.from_point = from_point
        self.to_point = to_point
        self.travel = to_point - from_point
        distance = np.linalg.norm(self.travel)
        self.travel /= distance
        self.accel_time = top_speed / acceleration
        accel_distance = 0.5 * acceleration * self.accel_time ** 2
        self.acceleration = acceleration
        self.top_speed = top_speed

        if accel_distance >= distance / 2:
            self.duration = 2 * sqrt(distance / acceleration)
            self.at_max_speed = False
        else:
            self.duration = distance / top_speed + self.accel_time
            self.at_max_speed = True

    def get_acceleration(self, at: float) -> np.ndarray:
        assert 0 <= at <= self.duration
        if self.at_max_speed:
            if at <= self.accel_time:
                return self.acceleration * self.travel
            elif at <= self.duration - self.accel_time:
                return np.zeros(2)
            else:
                return - self.acceleration * self.travel
        else:
            if at <= self.duration / 2:
                return self.acceleration * self.travel
            else:
                return - self.acceleration * self.travel

    def get_velocity(self, at: float) -> np.ndarray:
        assert 0 <= at <= self.duration
        if self.at_max_speed:
            if at <= self.accel_time:
                return at * self.acceleration * self.travel
            elif at <= self.duration - self.accel_time:
                return self.travel * self.top_speed
            else:
                return self.travel * (self.top_speed - (self.accel_time - self.duration + at) * self.acceleration)
        else:
            if at <= self.duration / 2:
                return at * self.acceleration * self.travel
            else:
                return self.travel * (self.top_speed - (self.accel_time - self.duration + at) * self.acceleration)

    def get_position(self, at: float) -> np.ndarray:
        assert 0 <= at <= self.duration
        if self.at_max_speed:
            if at <= self.accel_time:
                return 0.5 * at * self.get_velocity(at) + self.from_point
            elif at <= self.duration - self.accel_time:
                return self.get_position(self.accel_time) + self.travel * self.top_speed * (at - self.accel_time)
            else:
                decel_time = self.accel_time - self.duration + at
                return self.get_position(self.duration - self.accel_time) + self.travel * 0.5 * (2 * self.top_speed - self.acceleration * decel_time) * decel_time
        else:
            if at <= self.duration / 2:
                return 0.5 * at * self.get_velocity(at) + self.from_point
            else:
                decel_time = self.accel_time - self.duration + at
                return self.get_position(self.duration - self.accel_time) + self.travel * 0.5 * (2 * self.top_speed - self.acceleration * decel_time) * decel_time


def rot_matrix(theta: float) -> np.ndarray:
    return np.array(((np.cos(theta), -np.sin(theta)),
                     (np.sin(theta),  np.cos(theta))))


class TravelPlan:
    def __init__(
        self,
        setup_plan: Iterable[Union[np.ndarray, float]],
        loop_plan: Iterable[Union[np.ndarray, float]],
        max_runtime=1800.0,
        top_speed=1.5,
        acceleration=3.0,
        turn_time=10.0
    ):
        assert len(setup_plan) > 0
        assert type(setup_plan[0]) == np.ndarray
        self.origin = setup_plan[0]
        self.setup_plan = []
        self.loop_plan = []
        self.max_runtime = max_runtime
        turn_time /= pi

        i = 0
        last_loc = self.origin
        last_vector = rot_matrix(np.random.randn() * pi * 2) * np.matrix([[1.], [0.]])
        last_vector = np.array(last_vector.transpose())
        while i < len(setup_plan) - 1:
            j = i + 1
            delays = 0.0
            found = False
            while j < len(setup_plan):
                if type(setup_plan[j]) != np.ndarray:
                    j += 1
                    delays += setup_plan[j]
                    continue
                found = True
                break
            if delays > 0:
                self.setup_plan.append(delays)
                delays = 0.0
            if found:
                section = TravelSection(setup_plan[i], setup_plan[j], top_speed, acceleration)
                assert -1 <= np.dot(last_vector, section.travel) <= 1
                angle_to_turn = acos(np.dot(last_vector, section.travel)[0])
                self.setup_plan.append(angle_to_turn * turn_time)
                last_vector = section.travel
                self.setup_plan.append(section)
                max_runtime -= section.duration
                last_loc = setup_plan[j]
                i = j

        i = 0
        delays = 0.0
        while max_runtime > 0:
            if type(loop_plan[i]) != np.ndarray:
                delays += loop_plan[i]
                i += 1
                if i >= len(loop_plan):
                    i = 0
                continue
            if delays > 0:
                self.loop_plan.append(delays)
                max_runtime -= delays
                if max_runtime <= 0:
                    break
                delays = 0.0
            section = TravelSection(last_loc, loop_plan[i], top_speed, acceleration)
            angle_to_turn = acos(np.dot(last_vector, section.travel))
            delays = angle_to_turn * turn_time
            self.loop_plan.append(delays)
            max_runtime -= delays
            if max_runtime <= 0:
                break
            delays = 0.0
            last_vector = section.travel
            last_loc = loop_plan[i]
            self.loop_plan.append(section)
            max_runtime -= section.duration
            if max_runtime <= 0:
                break
            i += 1
            if i >= len(loop_plan):
                i = 0

    def to_test_plan(self) -> np.ndarray:
        runtime = 0.0
        section_runtime = 0.0
        i = 0
        last_loc = self.origin
        plan = []
        while runtime <= self.max_runtime and i < len(self.setup_plan):
            if type(self.setup_plan[i]) == float:
                plan.append((last_loc, np.zeros(2), np.zeros(2)))
            else:
                travel_section: TravelSection = self.setup_plan[i]
                last_loc = travel_section.get_position(section_runtime)
                plan.append(
                    (
                        last_loc,
                        travel_section.get_velocity(section_runtime),
                        travel_section.get_acceleration(section_runtime),
                    )
                )
            section_runtime += DELTA
            if type(self.setup_plan[i]) == float:
                if section_runtime > self.setup_plan[i]:
                    section_runtime = 0
                    i += 1
            elif section_runtime > self.setup_plan[i].duration:
                section_runtime = 0
                i += 1

            runtime += DELTA

        section_runtime = 0.0
        i = 0
        while runtime <= self.max_runtime:
            if type(self.loop_plan[i]) == float:
                plan.append((last_loc, np.zeros(2), np.zeros(2)))
            else:
                travel_section: TravelSection = self.loop_plan[i]
                last_loc = travel_section.get_position(section_runtime)
                plan.append(
                    (
                        last_loc,
                        travel_section.get_velocity(section_runtime),
                        travel_section.get_acceleration(section_runtime),
                    )
                )
            section_runtime += DELTA
            if type(self.loop_plan[i]) == float:
                if section_runtime > self.loop_plan[i]:
                    section_runtime = 0
                    i += 1
            elif section_runtime > self.loop_plan[i].duration:
                section_runtime = 0
                i += 1

            if i >= len(self.loop_plan):
                i = 0
            runtime += DELTA

        return np.array(plan)


if __name__ == "__main__":
    travel_plan = TravelPlan(
        [np.array([1., 1.]), np.array([5.6, 1.])],
        [np.array([5.6, 3.]), 30, np.array([5.6, 1.]), 30],
        360
    )
    plan = travel_plan.to_test_plan()
    # plot_val(plan, 1)

    start_time = time.time()
    with Pool() as p:
        results = p.map(
            run_args,
            [
                (plan, gen_ukf, 0.001, 0.001, "0.001 z_std")
            ] * 40
        )
    sim_time = time.time() - start_time
    results_sorted = {}
    for name, drift in results:
        if name not in results_sorted:
            results_sorted[name] = [0, 0, 0]
        if drift is None:
            results_sorted[name][1] += 1
            results_sorted[name][2] += 1
            continue
        results_sorted[name][0] += drift
        results_sorted[name][1] += 1

    print("Sim time:", sim_time)
    for name, (sum, count, fails) in results_sorted.items():
        print(f"\"{name}\": {round(sum / count, 2)}m drift, {round(fails / count * 100, 2)}% failed")
