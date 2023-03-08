# Q-Learning Based System for Path Planning with UAV Swarms in Obstacle Environments

System for coordinating UAV swarms for Path Planning in fields with fixed obstacles. made with open source libraries and Python 3. 

With this system, reinforcement learning techniques are employed. It makes use of the Deep Q-Learning method to be more exact.

## Installation

To install this computer you must download this repository:

```bash
$ git clone https://github.com/TheMVS/UAV_SWARMS_RL_FIXED_OBSTACLES_MAPS
```

Once downloaded, it is necessary to enter the project's root folder and install the necessary libraries with [pip](https://pip.pypa.io/en/stable/):

 * [numpy](https://numpy.org)
 * [scipy](https://www.scipy.org)
 * [Shapely](https://shapely.readthedocs.io/en/latest/)
 * [Keras](https://keras.io)
 * [Matplotlib](https://matplotlib.org)

## Usage

### Run system

To run the system you must be in the root folder of the project and execute the file [Program.py](https://github.com/TheMVS/UAV_SWARMS_RL_FIXED_OBSTACLES_MAPS/blob/main/Program.py):

```bash
$ cd UAV_SWARMS_RL_FIXED_OBSTACLES_MAPS
$ python Program.py
```

### Configuration

All necessary data for experimented should be added to [data.json](https://github.com/TheMVS/UAV_SWARMS_RL_FIXED_OBSTACLES_MAPS/blob/main/data.json) and [Config.py](https://github.com/TheMVS/UAV_SWARMS_RL_FIXED_OBSTACLES_MAPS/blob/main/Config.py).

## Authors

* [Alejandro Puente-Castro](https://orcid.org/0000-0002-0134-6877)
* [Daniel Rivero](https://orcid.org/0000-0001-8245-3094)
* [Eurico Pedrosa](https://orcid.org/0000-0002-7954-9922)
* [Artur Pereira](https://orcid.org/0000-0002-7099-1247)
* [Nuno Lau](https://orcid.org/0000-0003-0513-158X)
* [Enrique Fernandez-Blanco](https://orcid.org/0000-0003-3260-8734)

## License
[MIT](https://choosealicense.com/licenses/mit/)
