# acadia_tamp_workshop

## Installation

### Conda environment (required!)

Create a new environment with python 3.7 and install this package in the new environment

```bash
conda create --name acadia_tamp python=3.7
conda activate acadia_tamp
```

```bash
conda install cython numpy==1.21.5 mkl-devel cmake
```

### Clone and update submodules

Install this library from source by cloning this repo to local and install from source.

```bash
git clone --recursive https://github.com/yijiangh/acadia_tamp_workshop.git
cd acadia_tamp_workshop
```

The `--recursive` flag when cloning above is used for initializing all the git submodules. You can learn more about submodules [here](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

Later in the development, whenever you need to update the submodules, issue the following:

```bash
git submodule update --init --recursive
```

### Install libraries

Run this the following in terminal from the root folder of this repo. All libraries are installed from source (in the [editable mode](https://pip.pypa.io/en/stable/reference/pip_install/#install-editable)).

```bash
# Install the submodule libraries from source
pip install -e .\external\compas_fab
pip install -e .\external\compas_fab_pychoreo
pip install -e .\external\ikfast_pybind

```

TODO: to be debated if we should make this repo as a package


<!-- # install `acadia_tamp_workshop` from source  -->
<!-- pip install -e . -->
<!-- # Run the following code add the python library paths to Rhino / Grasshopper:
python -m compas_rhino.install -p compas compas_fab compas_ghpython integral_timber_joints -->
