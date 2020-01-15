# Seamonsters Robot Code Template

Template for robot code. Features include:

- Coroutine-based robot commands to replace the wpilib Command framework. [Read more!](https://seamonsters-2605.github.io/docs/seamonsters-generators/)
- A universal drivetrain controller that can be configured for any type/configuration of wheels
- Robot position tracking and path following
- Integrates with [REMI](https://github.com/dddomodossola/remi/) for building web-based dashboards

## Directory Structure

- `.vscode/`: Settings for Visual Studio Code, including launch configurations
- `seamonsters/`: The seamonsters library
- `tests/`: Created and used by pyfrc for tests.
- `physics.py` and `sim/`: Used for pyfrc robot simulation. See `sim/README` for details.

## How to update the seamonsters library documentation

The built sphinx documentation is published using GitHub pages on the `gh-pages` branch. Pull from master to update `gh-pages` and in `seamonsters/docs`, run `make.bat clean`, then `make.bat html` on that branch and push to update.

You can view the documentation [here](https://seamonsters-2605.github.io/SeamonstersTemplate/seamonsters/docs/_build/html/index.html).
