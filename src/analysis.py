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
    control: pandas.DataFrame | None = None
    end_effector_angular_acceleration: pandas.DataFrame | None = None
    end_effector_angular_velocity: pandas.DataFrame | None = None
    end_effector_linear_acceleration: pandas.DataFrame | None = None
    end_effector_linear_velocity: pandas.DataFrame | None = None
    end_effector_orientation: pandas.DataFrame | None = None
    end_effector_position: pandas.DataFrame | None = None
    joints: pandas.DataFrame | None = None
    power: pandas.DataFrame | None = None
    tank_energy: pandas.DataFrame | None = None

@dataclasses.dataclass
class ForecastResults:
    end_effector_angular_acceleration: pandas.DataFrame | None = None
    end_effector_angular_velocity: pandas.DataFrame | None = None
    end_effector_linear_acceleration: pandas.DataFrame | None = None
    end_effector_linear_velocity: pandas.DataFrame | None = None
    end_effector_orientation: pandas.DataFrame | None = None
    end_effector_position: pandas.DataFrame | None = None
    joints: pandas.DataFrame | None = None
    power: pandas.DataFrame | None = None
    tank_energy: pandas.DataFrame | None = None
    wrench: pandas.DataFrame | None = None

@dataclasses.dataclass
class ObjectiveResults:
    assisted_manipulation: pandas.DataFrame | None = None

@dataclasses.dataclass
class PlotData:
    filename: str
    force_pid: PidResults
    torque_pid: PidResults
    mppi: MppiResults
    dynamics: DynamicsResults
    forecast: ForecastResults
    objective: ObjectiveResults

class ResultsType(enum.Enum):
    MPPI = 0
    PID = 1
    DYNAMICS = 2
    FORECAST = 3
    OBJECTIVE = 4

def read_results(result_type: ResultsType, folder: str) -> PidResults | MppiResults | DynamicsResults:

    results = None
    match result_type:
        case ResultsType.MPPI:
            results = MppiResults()
        case ResultsType.PID:
            results = PidResults()
        case ResultsType.DYNAMICS:
            results = DynamicsResults()
        case ResultsType.FORECAST:
            results = ForecastResults()
        case ResultsType.OBJECTIVE:
            results = ObjectiveResults()
        case _:
            raise RuntimeError(f'unknown result type {result_type}')

    directory = pathlib.Path(folder)
    if not directory.exists():
        return results

    for field in dataclasses.fields(results):
        filename = f'{field.name}.csv'
        path = directory / filename
        if not path.exists():
            continue

        try:
            setattr(results, field.name, pandas.read_csv(path, skipinitialspace = True))
        except pandas.errors.EmptyDataError:
            pass

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

def plot_time_norm(axis: plt.Axes, df: pandas.DataFrame | None):
    if df is None:
        return

    to_norm = df.columns[df.columns != 'time']
    error = np.sqrt(np.square(df[to_norm]).sum(axis = 1))

    axis.plot(df['time'], error)
    axis.set_xlim(xmin = 0.0, xmax = max(df['time']))
    axis.set_ylim(ymin = 0.0)

def plot_reference_error(axis: plt.Axes, pid_error: pandas.DataFrame | None):
    axis.set_title('Reference Position Error')
    axis.set_xlabel('Time [$s$]')
    axis.set_ylabel('Error [$m$]')
    plot_time_norm(axis, pid_error)

def plot_force_control(axis: plt.Axes, pid_control: pandas.DataFrame | None):
    axis.set_title('Observed End Effector Force')
    axis.set_xlabel('Time [$s$]')
    axis.set_ylabel('Force [$N$]')
    plot_time_norm(axis, pid_control)

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
    figure = plt.figure(figsize = (10, 3))

    plot_optimal_cost(plt.subplot2grid((2, 1), (0, 0)), data.mppi.optimal_cost)
    plot_force_control(plt.subplot2grid((2, 1), (1, 0)), data.force_pid.control)
    plot_tank_energy(plt.subplot2grid((4, 1), (1, 0)), data.dynamics.tank_energy)
    # plot_reference_error(plt.subplot2grid((4, 1), (2, 0)), data.force_pid.error)

    figure.tight_layout()
    return figure

def plot_timeseries(
        df: PlotData | None,
        units: dict[str, str],
        title: str,
        /, *,
        dependent = 'time',
        y_scale = 'min_max'
    ):
    """Plot a time series graph:

    Args:
        df: The data to plot.
        units: Mapping of columns to their data type.
        title: Title of the plot.
        dependent: The dependent column name.
        y_scale: One of 'around_zero', 'from_zero' or 'min_max'
        y_log: Log scale on y axis.
    """

    if df is None:
        plt.subplots(len(units), 1, figsize = (10, 12))
        return

    columns = df.columns[df.columns != dependent]

    all_axes: list[plt.Axes]
    figure, all_axes = plt.subplots(
        len(columns), 1,
        figsize = (10, len(columns)),
        layout = 'constrained'
    ) 
    for column, axes in zip(columns, all_axes):
        axes: plt.Axes
        unit = units[column]

        axes.plot(df[dependent], df[column])
        axes.grid(True, color = 'lightgrey')

        x_min = 0.0
        x_max = max(df[dependent])
        y_min = min(df[column])
        y_max = max(df[column])
        y_ticks = []

        match y_scale:
            case 'around_zero':
                limit = max(abs(y_min), abs(y_max), 0.05)
                limit += limit / 10
                y_min, y_max = -limit, limit
                y_ticks = [y_min, 0.0, y_max]
            case 'from_zero':
                y_min = 0.0
                if abs(y_max - y_min) < 1e-3:
                    y_max = 1.0
                y_ticks = [0.0, y_max]
            case 'min_max':
                if abs(y_max - y_min) < 1e-3:
                    y_min -= 1.0
                    y_max += 1.0
            case _:
                raise RuntimeError(f'Unknown y scaling {y_scale}')

        axes.set_yticks(y_ticks)
        axes.set_ylim(ymin = y_min, ymax = y_max)
        axes.yaxis.get_majorticklabels()[0].set_verticalalignment('bottom')
        axes.yaxis.get_majorticklabels()[-1].set_verticalalignment('top')

        axes.set_xlim(xmin = x_min, xmax = x_max)
        if column != columns[-1]:
            axes.set_xticklabels([])

        axes.text(
            -x_max * 1/12,
            (y_min + y_max) / 2,
            f"{column.replace('_', ' ').capitalize()} [${unit}$]",
            ha = 'right',
            va = 'center'
        )

    all_axes[-1].set_xlabel(f'{dependent.capitalize()} [$s$]')

    plt.suptitle(title)
    return figure

def plot_control(data: PlotData):
    return plot_timeseries(
        data.dynamics.control if data.dynamics is not None else None,
        {
            'x': 'm/s',
            'y': 'm/s',
            'yaw': 'rad/s',
            'arm1': 'Nm',
            'arm2': 'Nm',
            'arm3': 'Nm',
            'arm4': 'Nm',
            'arm5': 'Nm',
            'arm6': 'Nm',
            'arm7': 'Nm',
            'gripper_x': 'm',
            'gripper_y': 'm',
        },
        'Controls',
        y_scale = 'around_zero'
    )

def plot_assisted_manipulation_objective(data: PlotData):
    objectives = (
        'joint_limit',
        'minimise_velocity',
        'self_collision',
        'trajectory',
        'workspace',
        'power',
        'energy_tank',
        'manipulability',
        'variable_damping',
        'total'
    )
    return plot_timeseries(
        data.objective.assisted_manipulation if data.objective is not None else None,
        {key: '\#' for key in objectives},
        'Objective Function Cost Composition Over Time',
        y_scale = 'from_zero'
    )

def plot_error(data: PlotData):
    figure = plt.figure(figsize = (8, 4), layout = 'tight')

    if data.force_pid is None:
        return figure

    df = data.force_pid.error
    if df is None:
        return figure

    axis = plt.gca()

    to_norm = df.columns[df.columns != 'time']
    error = np.sqrt(np.square(df[to_norm]).sum(axis = 1))
    time = df['time']

    axis.plot(time, error)
    axis.set_xlim(xmin = 0.0, xmax = max(time))
    axis.set_ylim(ymin = 0.0)
    axis.set_title('Reference Error of User Model over Time')
    axis.set_xlabel('Time [$s$]')
    axis.set_ylabel('Error [$m$]')

    return figure

if __name__ == '__main__':
    if not sys.argv[1:]:
        raise RuntimeError('No result directory provided.')

    path = pathlib.Path(sys.argv[1])
    print(f'analysing results of {path}')

    data = PlotData(
        filename = path,
        mppi = read_results(ResultsType.MPPI, path / 'mppi'),
        dynamics = read_results(ResultsType.DYNAMICS, path / 'dynamics'),
        force_pid = read_results(ResultsType.PID, path / 'pid' / 'force'),
        torque_pid = read_results(ResultsType.PID, path / 'pid' / 'torque'),
        forecast = read_results(ResultsType.FORECAST, path / 'forecast'),
        objective = read_results(ResultsType.OBJECTIVE, path / 'objective')
    )

    fig = plot_error(data).savefig(path / 'error.png', dpi = 300)

    plot_useful(data).savefig(path / 'overview.png', dpi = 300)
    plot_control(data).savefig(path / 'control.png', dpi = 300)

    if data.objective.assisted_manipulation is not None:
        plot_assisted_manipulation_objective(data).savefig(path / 'objective.png', dpi = 300)
