import dataclasses
import enum
import sys
import matplotlib.pyplot as plt
import pandas
import pathlib
import numpy as np

plt.rcParams.update({
    "text.usetex": True,
    "text.latex.preamble": [r'\usepackage{amsmath}'],
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
    name: str
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

def read_data(path: pathlib.Path):
    name = ' '.join(path.stem.split('_')[1:])
    name = name[0].capitalize() + name[1:]
    return PlotData(
        filename = path,
        name = name,
        mppi = read_results(ResultsType.MPPI, path / 'mppi'),
        dynamics = read_results(ResultsType.DYNAMICS, path / 'dynamics'),
        force_pid = read_results(ResultsType.PID, path / 'pid' / 'force'),
        torque_pid = read_results(ResultsType.PID, path / 'pid' / 'torque'),
        forecast = read_results(ResultsType.FORECAST, path / 'forecast'),
        objective = read_results(ResultsType.OBJECTIVE, path / 'objective')
    )

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
    figure = plt.figure(figsize = (10, 10), layout = 'constrained')

    plot_optimal_cost(plt.subplot2grid((4, 1), (0, 0)), data.mppi.optimal_cost)
    plot_force_control(plt.subplot2grid((4, 1), (1, 0)), data.force_pid.control)
    plot_tank_energy(plt.subplot2grid((4, 1), (2, 0)), data.dynamics.tank_energy)
    plot_reference_error(plt.subplot2grid((4, 1), (3, 0)), data.force_pid.error)

    # figure.tight_layout()
    return figure

def plot_timeseries(
        df: PlotData | None,
        units: dict[str, str],
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
        y_scale = 'around_zero'
    )

def plot_assisted_manipulation_objective(data: PlotData):
    objectives = (
        'joint_limit',
        'self_collision',
        'workspace',
        'energy_tank',
        'minimise_velocity',
        'trajectory',
        'manipulability',
        'total'
    )
    return plot_timeseries(
        data.objective.assisted_manipulation if data.objective is not None else None,
        {key: '\#' for key in objectives},
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

def analyse_single(path: pathlib.Path):
    print(f'analysing results of {path}')

    data = read_data(path)

    plot_error(data).savefig(path / 'error.png', dpi = 300)
    plot_useful(data).savefig(path / 'overview.png', dpi = 300)
    plot_control(data).savefig(path / 'control.png', dpi = 300)

    if data.objective.assisted_manipulation is not None:
        plot_assisted_manipulation_objective(data).savefig(path / 'objective.png', dpi = 300)

def plot_force_multi(data: list[PlotData]):

    if all(d.force_pid is None or d.force_pid.control is None for d in data):
        return

    figure = plt.figure(figsize = (7, 4), layout='tight')
    axes = figure.gca()

    # names = (
    #     r'$c_\mathrm{manipulability} = 500$',
    #     r'$c_\mathrm{manipulability} = 1000$',
    #     r'$c_\mathrm{manipulability} = 1500$',
    #     r'$c_\mathrm{manipulability} = 2000$',
    #     r'$c_\mathrm{manipulability} = 2500$',
    # )

    for d in data:
        df = d.force_pid.control
        to_norm = df.columns[df.columns != 'time']
        error = np.sqrt(np.square(df[to_norm]).sum(axis = 1))
        time = df['time']
        axes.plot(time, error, label = d.name)

    axes.grid()
    axes.set_xticks(np.arange(0, 15, 1.0))
    # axes.set_yticks(np.arange(0, 160, 10))
    axes.set_xlim(xmin = 0.0, xmax = 15)
    axes.set_ylim(ymin = 0.05)
    axes.set_xlabel('Time [$s$]')
    axes.set_ylabel('Force [$N$]')
    axes.legend()
    # for i in range(30):
    #     axes.axvline(0.5 * i, color = 'black', linewidth = 0.1)
    return figure

def plot_reference_error_multi(data: list[PlotData]):

    if all(d.force_pid is None or d.force_pid.error is None for d in data):
        return

    figure = plt.figure(figsize = (7, 4), layout='tight')
    axes = figure.gca()

    for d in data:
        df = d.force_pid.error
        to_norm = df.columns[df.columns != 'time']
        error = np.sqrt(np.square(df[to_norm]).sum(axis = 1))
        time = df['time']
        axes.plot(time, error, label = d.name)

    axes.grid()
    axes.set_xticks(np.arange(0, 15, 1.0))
    # axes.set_yticks(np.arange(0, 160, 10))
    axes.set_xlim(xmin = 0.0, xmax = 15)
    axes.set_ylim(ymin = 0.0)
    axes.set_xlabel('Time [$s$]')
    axes.set_ylabel('User Trajectory Error [$m$]')
    axes.legend()
    # for i in range(30):
    #     axes.axvline(0.5 * i, color = 'black', linewidth = 0.1)
    return figure

def plot_velocity_multi(data: list[PlotData]):

    if all(d.dynamics is None or d.dynamics.end_effector_angular_velocity is None for d in data):
        return

    figure = plt.figure(figsize = (7, 4), layout='tight')
    axes = figure.gca()

    for d in data:
        df = d.dynamics.end_effector_angular_velocity
        to_norm = df.columns[df.columns != 'time']
        error = np.sqrt(np.square(df[to_norm]).sum(axis = 1))
        time = df['time']
        axes.plot(time, error, label = d.name)

    axes.grid()
    axes.set_xticks(np.arange(0, 15, 1.0))
    # axes.set_yticks(np.arange(0, 160, 10))
    axes.set_xlim(xmin = 0.0, xmax = 15)
    axes.set_ylim(ymin = 0.0)
    axes.set_xlabel('Time [$s$]')
    axes.set_ylabel('End-Effector Velocity [$m/s$]')
    axes.legend()
    # for i in range(30):
    #     axes.axvline(0.5 * i, color = 'black', linewidth = 0.1)
    return figure

def barchart():

    # names = ('Unassisted', 'Average', 'LOCF', 'Kalman 1', 'Kalman 2')
    # data = {
    #     'Pose': (0.001088024655113938, 0.0009079267291307213, 0.0006694987803991998, 0.0008538404377074903, 0.0008506949491946322),
    #     'Circle': (0.09064071362734615, 0.0431296615009239, 0.04404907289861029, 0.044768773110300385, 0.04524059656960858),
    #     'Figure Eight': (0.20720194775837483, 0.07970206752669437, 0.07022175970041583, 0.06036570606721701, 0.06520889663731642),
    #     'Rectangle': (0.1348200430917584, 0.0544936457417769, 0.0544421654218168, 0.05403947848822745, 0.057392673314378334),
    # }
    # ylabel = 'RMSE [$m$]'
    # title = 'Trajectory Error [$m$] of User Effort by Trajectory and Wrench Forecast'

    names = ('Pose', 'Circle', 'Figure Eight', 'Rectangle')
    data = {
        'Unassisted': (0.00,   24.81, 46.32, 34.17),
        'Average':    (0.22,  11.94, 21.94, 15.50),
        'LOCF':       (0.09,  12.29, 19.18, 15.90),
        'Kalman 1':   (0.04,  12.59, 16.52, 15.75),
        'Kalman 2':   (0.07,  12.73, 17.90, 16.70),
    }
    ylabel = 'Mean User Force [$N$]'
    title = 'Mean User Effort by Trajectory and Wrench Forecast'

    x = np.arange(len(names))
    width = 0.25  # the width of the bars

    figure, ax = plt.subplots(layout = 'constrained')
    for i, (attribute, value) in enumerate(data.items()):
        rects = ax.bar(x + width * i, value, width, label = attribute)
        ax.bar_label(rects, padding = 3)

    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xticks(x + width, names)
    ax.legend(loc = 'upper left')
    plt.show()

def analyse_multiple(path: pathlib.Path):

    # Read all experimental data.
    data = [read_data(p) for p in path.iterdir() if p.is_dir()]

    plot_velocity_multi(data).savefig(path / f'{path.stem}_velocity.png', dpi = 300)
    plot_force_multi(data).savefig(path / f'{path.stem}_effort.png', dpi = 300)
    plot_reference_error_multi(data).savefig(path / f'{path.stem}_reference_error.png', dpi = 300)

    with open(path / 'pid_force_summary.txt', 'w') as file:
        file.write(f'name, mean, std, min, max\n')
        for d in data:
            df = d.force_pid.control
            force = df[df['time'] > 0.01]
            force = force[force.columns[force.columns != 'time']]
            force = np.sqrt(np.square(force).sum(axis = 1))

            file.write(f'"{d.name}", {force.mean()}, {force.std()}, {force.min()}, {force.max()}\n')

    with open(path / 'pid_reference_summary.txt', 'w') as file:
        file.write(f'name, rmse, mean, std, min, max\n')
        for d in data:
            df = d.force_pid.error
            error = df[df['time'] > 0.01]
            error = error[error.columns[error.columns != 'time']]
            error = np.sqrt(np.square(error).sum(axis = 1))
            rmse = np.sqrt(np.square(error).mean())

            file.write(f'"{d.name}", {rmse}, {error.mean()}, {error.std()}, {error.min()}, {error.max()}\n')

if __name__ == '__main__':
    args = sys.argv[1:]

    if not args:
        raise RuntimeError('usage: <"single" | "multi"> <path>')

    if len(args) < 2:
        raise RuntimeError('usage: <"single" | "multi"> <path>')

    todo = args[0]
    path = pathlib.Path(args[1])

    if todo == 'single':
        analyse_single(path)
    elif todo == 'multi':
        analyse_multiple(path)
    else:
        raise RuntimeError(f'unknown operation {todo}')
