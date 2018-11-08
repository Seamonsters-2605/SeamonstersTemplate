# Seamonsters Robot Code Template

Template for robot code.

## Directory Structure

- `.vscode/`: Settings for Visual Studio Code, including launch configurations
- `seamonsters/`: The seamonsters library
- `tests/`: Created and used by pyfrc for tests.
- `physics.py` and `sim/`: Used for pyfrc robot simulation. See `sim/README` for details.

## How to update the seamonsters library documentation

The built sphinx documentation is published using GitHub pages on the `gh-pages` branch. In `seamonsters/docs`, run `make clean`, then `make html` on that branch and push to update.

You can view the documentation [here](https://seamonsters-2605.github.io/SeamonstersTemplate/seamonsters/docs/_build/html/index.html).