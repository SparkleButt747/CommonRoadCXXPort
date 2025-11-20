# Python Vehicle Models of CommonRoad

This package contains the dynamic single-track, single-track drift, and multi-body vehicle models of the [CommonRoad benchmarks](https://commonroad.in.tum.de/).

We provide implementations of the vehicle dynamics, routines to convert initial states, and vehicle parameters.

## Documentation

For a detailed explanation of the vehicle models, please have a look at the [documentation](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf).

## Installation

To use vehicle models and parameters, run
```
pip install commonroad-vehicle-models
```

## Code examples

For an extended simulation example demonstrating the advantages of more complicated models, we refer to our [gitlab repository](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/tree/master/PYTHON/scripts). A simple simulation example for using the dynamic single-track model in combination with an odeint solver would be

```python3
from scipy.integrate import odeint
import numpy

from vehiclemodels.init_st import init_st
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st


def func_ST(x, t, u, p):
    f = vehicle_dynamics_st(x, u, p)
    return f


tStart = 0  # start time
tFinal = 1  # start time

# load vehicle parameters
p = parameters_vehicle2()

# initial state for simulation
delta0 = 0
vel0 = 15
Psi0 = 0
sy0 = 0
initialState = [0, sy0, delta0, vel0, Psi0, 0, 0]
x0_ST = init_st(initialState)

t = numpy.arange(0, tFinal, 0.01)
u = [0, 5]
x = odeint(func_ST, x0_ST, t, args=(u, p))

```



## Contribute

If you want to contribute new vehicle models, you can create a merge request in our [repository](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/), or contact via our [forum](https://commonroad.in.tum.de/forum/).


## Changelog
Compared to version 2.0.0 the following features were added/changed:
* Kinematic single-track variants and related linearized models have been removed from the Python examples in favor of dynamic single-track, single-track drift, and multi-body models
* Vehicle parameters are stored in YAML-files
* Parameter configuration of vehicles are generated from YAML-files using [OmegaConf](https://omegaconf.readthedocs.io/en/2.2_branch/) (backwards compatible)


## Referencing

If you use CommonRoad for your research, please cite [our paper](http://mediatum.ub.tum.de/doc/1379638/776321.pdf):

```
@inproceedings{Althoff2017a,
        author = {Althoff, Matthias and Koschi, Markus and Manzinger, Stefanie},
        title = {CommonRoad: Composable benchmarks for motion planning on roads},
        booktitle = {Proc. of the IEEE Intelligent Vehicles Symposium},
        year = {2017},
}
```
