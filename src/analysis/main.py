import json
import subprocess
import typer
import pandas
import pathlib
import dataclasses

app = typer.Typer(
    add_completion = False,
    pretty_exceptions_show_locals = False
)

def run_test(name: str, configuration: dict | None = None) -> bool:
    """Run a test.

    Args:
        name: The name of the test to run.
        configuration: The optional configuration of the test.

    Returns:
        If the test was successful.
    """
    args = ['test', '--test', f'"{name}"']

    if configuration is not None:
        args.extend(['--configuration', f'"{json.dump(configuration)}"'])

    status: subprocess.CompletedProcess = subprocess.run(
        ' '.join(args),
        stdout = subprocess.STDOUT,
        stderr = subprocess.STDOUT
    )

    return status.returncode == 0

class ResultsReader:

    def __init__(self, *files):
        self._files = files

    def __init_subclass__(cls, *files):
        cls._files = files
        for file in files:
            setattr(cls, file, None)

    @classmethod
    def read(folder: str) -> 'ResultsReader':
        directory = pathlib.Path(folder)
        if directory.exists():
            raise FileNotFoundError(f'mppi directory {folder} not found')

        mppi = MPPI()

        files = [
            'costs', 'gradient', 'optimal_cost', 'optimal_rollout', 'update',
            'weights'
        ]

        for file in files:
            path = directory / f'{file}.csv'
            if not path.exists():
                raise FileNotFoundError(f'mppi {file} file not found at {path}')
            setattr(mppi, file, pandas.read_csv(path))

        return mppi

class MppiReader(ResultsReader, 'costs', 'gradient', 'optimal_cost', 'optimal_rollout',
        'update', 'weights'
    ):
    pass

@dataclasses
class MPPI:
    costs: pandas.DataFrame | None = None
    gradient: pandas.DataFrame | None = None
    optimal_cost: pandas.DataFrame | None = None
    optimal_rollout: pandas.DataFrame | None = None
    update: pandas.DataFrame | None = None
    weights: pandas.DataFrame | None = None

@app.callback()
def main():
    raise NotImplementedError()

if __name__ == '__main__':
    app()
