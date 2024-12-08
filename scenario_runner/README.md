## Environment setup
### Package installation
+ carla: 0.9.12
+ Autoware: https://github.com/carla-simulator/carla-autoware
+ Clone this project
+ Clone the project [LAV](https://github.com/dotchen/LAV) from its github

Envronment variable setting:
1. Set carla environment variable as the carla installation path
2. Set scenario runner environment variable as the path of ``srunner`` in this project

Code setting
1. Change the paths of ADS in 
Lines 387-393 of ``scenario_parser.py``.

## Run the code
1. Launch carla
2. Open terminal and set the current work directory as the porject path
3. Run the command ``python scenario_parser.py -f <> -m <> -t <>`` where ``f`` is the path of the scenario yaml in the folder ``generated_yaml``; ``m`` is model except for autoware; ``t`` is the town. 
4. In another terminal, run ``python manual_control.py`` to visualize the scenario.
5. TODO: update the running of autoware 
