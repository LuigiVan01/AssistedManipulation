import pandas
import pathlib
import dataclasses

def read_results(folder: str, *files: str) -> list[pandas.DataFrame]:
    directory = pathlib.Path(folder)
    if directory.exists():
        raise FileNotFoundError(f'mppi directory {folder} not found')

    data = []

    for file in files:
        path = directory / f'{file}.csv'
        if not path.exists():
            raise FileNotFoundError(f'mppi {file} file not found at {path}')
        data.append(pandas.read_csv(path))

    return data

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

def read_pid_results(folder: str) -> PidResults:
    return PidResults(*read_results(
        folder, 'control', 'cumulative_error', 'error', 'reference', 'saturation'
    ))

def read_mppi_results(folder: str) -> MppiResults:
    return MppiResults(*read_results(
        folder,
        'costs', 'gradient', 'optimal_cost', 'optimal_rollout', 'update', 'weights'
    ))

def generate_mppi_graphs(results: MppiResults, /):
    pass
