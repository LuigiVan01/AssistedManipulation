import dataclasses
import enum
import sys
import matplotlib.pyplot as plt
import pandas
import pathlib
import numpy as np

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "Helvetica"
})

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

@dataclasses.dataclass
class PlotData:
    filename: str
    pid: PidResults
    mppi: MppiResults
    dynamics: DynamicsResults

class ResultsType(enum.Enum):
    MPPI = 0
    PID = 1
    DYNAMICS = 2

def read_results(result_type: ResultsType, folder: str) -> PidResults | MppiResults | DynamicsResults:

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

    directory = pathlib.Path(folder)
    if not directory.exists():
        return results

    for field in dataclasses.fields(results):
        filename = f'{field.name}.csv'
        path = directory / filename
        if not path.exists():
            raise RuntimeError(f'file {filename} does not exist')
        setattr(results, field.name, pandas.read_csv(path, skipinitialspace = True))

    return results

def plot_optimal_cost(axis: plt.Axes, optimal_cost: pandas.DataFrame | None):
    axis.set_title('Optimal Cost')
    axis.set_xlabel('Time [$s$]')
    axis.set_ylabel('Cost')

    if optimal_cost is not None:
        axis.plot(optimal_cost['time'], np.asarray(optimal_cost['cost']))

    axis.set_xlim(xmin = 0.0, xmax = max(optimal_cost['time']))
    axis.set_ylim(ymin = 0.0)

def plot_tank_energy(axis: plt.Axes, tank_energy: pandas.DataFrame | None):
    axis.set_title('Energy Tank Evolution')
    axis.set_xlabel('Time [$s$]')
    axis.set_ylabel('Energy [$J$]')

    if tank_energy is not None:
        axis.plot(tank_energy['time'], tank_energy['energy'])

    axis.set_xlim(xmin = 0.0, xmax = max(tank_energy['time']))
    axis.set_ylim(ymin = 0.0)

def plot_pid_error(axis: plt.Axes, pid_error: pandas.DataFrame | None):
    axis.set_title('External Wrench PID Error')
    axis.set_xlabel('Time [$s$]')
    axis.set_ylabel('Error [$m$]')

    if pid_error is not None:
        axis.plot(pid_error['time'], pid_error['error'])

    axis.set_xlim(xmin = 0.0)
    axis.set_ylim(ymin = 0.0)

def plot_end_effector_trajectory(
        axis: plt.Axes,
        position: pandas.DataFrame | None,
        reference: pandas.DataFrame | None
    ):

    if position is not None:
        axis.plot(position['time'], position['x'], position['y'], p)

    if reference is not None:
        pass

def plot_useful(data: PlotData):
    plt.figure(figsize = (10, 8))

    plot_optimal_cost(plt.subplot2grid((3, 1), (0, 0)), data.mppi.optimal_cost)
    plot_tank_energy(plt.subplot2grid((3, 1), (1, 0)), data.dynamics.tank_energy)
    plot_pid_error(plt.subplot2grid((3, 1), (2, 0)), data.pid.error)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    if not sys.argv[1:]:
        raise RuntimeError('No result directory provided.')

    path = pathlib.Path(sys.argv[1])
    print(f'analysing results of {path}')

    data = PlotData(
        filename = path,
        mppi = read_results(ResultsType.MPPI, path / 'mppi'),
        dynamics = read_results(ResultsType.DYNAMICS, path / 'dynamics'),
        pid = read_results(ResultsType.PID, path / 'pid'),
    )

    plot_useful(data)
