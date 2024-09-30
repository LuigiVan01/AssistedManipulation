import dataclasses
import enum
import matplotlib.pyplot as plt
import pandas
import pathlib
import numpy as np

@dataclasses.dataclass
class PidResults:
    control: pandas.DataFrame | None = None
    cumulative_error: pandas.DataFrame | None = None
    error: pandas.DataFrame | None = None
    reference: pandas.DataFrame | None = None
    saturation: pandas.DataFrame | None = None

@dataclasses.dataclass
class MppiResults:
    costs: pandas.DataFrame | None = None
    gradient: pandas.DataFrame | None = None
    optimal_cost: pandas.DataFrame | None = None
    optimal_rollout: pandas.DataFrame | None = None
    update: pandas.DataFrame | None = None
    weights: pandas.DataFrame | None = None

@dataclasses.dataclass
class DynamicsResults:
    end_effector_angular_acceleration: pandas.DataFrame | None = None
    end_effector_angular_velocity: pandas.DataFrame | None = None
    end_effector_linear_acceleration: pandas.DataFrame | None = None
    end_effector_linear_velocity: pandas.DataFrame | None = None
    end_effector_orientation: pandas.DataFrame | None = None
    end_effector_position: pandas.DataFrame | None = None
    power: pandas.DataFrame | None = None
    tank_energy: pandas.DataFrame | None = None

class ResultsType(enum.Enum):
    MPPI = 0
    PID = 1
    DYNAMICS = 2

def read_results(result_type: ResultsType, folder: str) -> PidResults | MppiResults | DynamicsResults:
    directory = pathlib.Path(folder)
    if not directory.exists():
        raise FileNotFoundError(f'mppi directory {folder} not found')

    results = None
    match result_type:
        case ResultsType.MPPI:
            results = MppiResults()
        case ResultsType.PID:
            results = PidResults()
        case ResultsType.DYNAMICS:
            results = DynamicsResults()
        case _:
            raise RuntimeError(f'unknown result type {result_type}')

    for field in dataclasses.fields(results):
        filename = f'{field.name}.csv'
        path = directory / filename
        if not path.exists():
            raise RuntimeError(f'file {filename} does not exist')
        setattr(results, field.name, pandas.read_csv(path, skipinitialspace = True))

    return results

def plot_time_series(axis, df):
    axis.plot(df['time'], np.asarray(df.loc[:, df.columns != 'time']))

def plot_time_norm(axis, df):
    time = df['time']
    other = df.loc[:, df.columns != 'time']
    norm = np.sqrt(np.square(other).sum(axis = 1))
    axis.plot(time, norm)

def plot_2d(axis, values_df):
    pass

def plot_3d(axis: plt.Axes, values_df):
    axis.set_

def plot_dynamics(dynamics: DynamicsResults):
    plt.figure(sharex = True, figsize = (10, 16))

    fig, axes = plt.subplots(8, 1, sharex = True, figsize = (10, 16))
    it = iter(axes)

    time_series = [
        dynamics.power,
        dynamics.tank_energy
    ]

    for axis, df in zip(it, time_series):
        plot_time_series(axis, df)

    time_norm = [
        dynamics.end_effector_linear_velocity,
        dynamics.end_effector_angular_velocity,
        dynamics.end_effector_linear_acceleration,
        dynamics.end_effector_angular_acceleration
    ]

    for axis, df in zip(it, time_norm):
        plot_time_norm(axis, df)

    # plt.figlegend([field.name for field in fields])
    # plt.subplots_adjust(right=0.8, bottom = 0.07)

    fig.suptitle('Dynamics of f{}')
    plt.show()

def plot_nice(dynamics: DynamicsResults, mppi: MppiResults, pid: PidResults):
    plt.figure(figsize = (10, 10))

    if mppi.costs is not None:
        axis = plt.subplot2grid((3, 3), (0, 0))
        plot_time_series(axis, mppi.costs)
        axis.set_title('Cost')

    if dynamics.power is not None:
        axis = plt.subplot2grid((3, 3), (0, 1))
        plot_time_series(axis, dynamics.power)
        axis.set_title('Power [J/s]')

    if dynamics.tank_energy is not None:
        axis = plt.subplot2grid((3, 3), (0, 2))
        plot_time_series(axis, dynamics.tank_energy)
        axis.set_title('Tank Energy [J]')

    if dynamics.end_effector_linear_velocity is not None:
        axis = plt.subplot2grid((3, 3), (1, 0))
        plot_time_norm(axis, dynamics.end_effector_linear_velocity)
        axis.set_title('Linear Velocity')

    if dynamics.end_effector_linear_acceleration is not None:
        axis = plt.subplot2grid((3, 3), (1, 1))
        plot_time_norm(axis, dynamics.end_effector_linear_acceleration)
        axis.set_title('Linear Velocity')

    if pid.reference and dynamics.end_effector_position is not None:
        axis = plt.subplot2grid((3, 3), (1, 2), rowspan = 2, projection = '3d')
        axis.plot(pid.reference[pid.reference.columns != 'time'])
        plot_time_norm(axis, dynamics.end_effector_linear_acceleration)
        axis.set_title('Trajectory')

    if pid.control is not None:
        axis = plt.subplot2grid((3, 3), (2, 0))
        plot_time_norm(axis, pid.control)
        axis.set_title('External Force')

    if pid.error is not None:
        axis = plt.subplot2grid((3, 3), (2, 1))
        plot_time_norm(axis, pid.error)
        axis.set_title('Reference Trajectory Error')

    plt.tight_layout()
    plt.show()

mppi = MppiResults()
pid = PidResults()
dynamics = read_results(ResultsType.DYNAMICS, 'results/reach_2024-09-28_16-03-27/dynamics')
plot_nice(dynamics, mppi, pid)
