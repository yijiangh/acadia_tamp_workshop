# acadia_tamp_workshop

## Installation

### Conda environment (required!)

The following steps will create a new Conda environment with python 3.11 and install this package in the new environment.
The new environment is named `acadia_tamp` and can be activated by `conda activate acadia_tamp`.
Whenever you open a new terminal, you need to activate the environment again.
These commands are best executed in the Anaconda Prompt or Anaconda Powershell Prompt.

```bash
conda create --name acadia_tamp  -c conda-forge  --yes numpy networkx scipy matplotlib-base pillow schema sympy typing_extensions watchdog "jsonschema>=4.17,<4.18" imageio pyserial pybullet roslibpy pycollada colorama python=3.7
```

Activate the environment

```
conda activate acadia_tamp
```

### To Delete the environment

```bash
conda env remove --name acadia_tamp
```

### Clone and update submodules

Install this library from source by cloning this repo to a local "working directory" and install it.
The `--recursive` flag is needed when cloning to initialize all the git submodules. You can learn more about submodules [here](https://git-scm.com/book/en/v2/Git-Tools-Submodules).


```bash
cd <your working directory>
git clone --recursive https://github.com/yijiangh/acadia_tamp_workshop.git
cd acadia_tamp_workshop
```

Later in the development, whenever you need to update the submodules, issue the following:

```bash
git submodule update --init --recursive
```

### Install libraries

Run this the following in terminal from the root folder of this repo. All libraries are installed from source (in the [editable mode](https://pip.pypa.io/en/stable/reference/pip_install/#install-editable)).

```bash
pip install compas --no-deps
# Install the submodule libraries from source
pip install --no-deps -e .\external\compas_fab
pip install --no-deps -e .\external\compas_fab_pychoreo
pip install 'pybullet_planning>=0.6.0'
pip install -e .\external\ikfast_pybind # Only if you want to experiment IKFast later
python -m compas_rhino.install
```

If the installer complain about missing cython or cmake, install the following tool chain:

```bash
conda install cython numpy==1.21.5 mkl-devel cmake
```

If the installation process ask for Visual Studio Build Tools. Download the installer (Build Tools for Visual Studio 2022) from [here](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2022) and install it. This may take a while

TODO: to be debated if we should make this repo as a package


<!-- # install `acadia_tamp_workshop` from source  -->
<!-- pip install -e . -->
<!-- # Run the following code add the python library paths to Rhino / Grasshopper:
python -m compas_rhino.install -p compas compas_fab compas_ghpython integral_timber_joints -->



## Validate the installation

Activate the environment

```bash
conda activate acadia_tamp
```

Go to the workshop folder

```bash
cd <your working directory>
```
Run the follwing example file:

```bash
python examples\03_pybullet_single_query\plan_free_motion.py
```

You should see the result:
```bash
pybullet build time: Oct 14 2023 15:51:40
Found a trajectory!
Planning took 3.998746395111084 seconds
```