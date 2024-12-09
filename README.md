# TARGET
This is the repository for the work "TARGET: Traffic Rule-based Test Generation for Autonomous Driving Systems".


### Prerequisites
1. Download Carla 0.9.12 and install Carla PythonAPI
2. follow `env_variables.txt` to set up environment variables in your `bashrc`
4. Install dependencies `pip install -r requirements.txt`
5. Create a new OpenAI API key and save it to `openai_api_key.txt`
6. (Optional) If you want to use Llama to generate test scenario descriptions, download Ollma and your desired open source model such as llama3.1:70b.
7. (Optional) Install Carla-ROS-Bridge and Autoware following [ros-bridge](https://github.com/carla-simulator/ros-bridge) and [Autoware](https://github.com/carla-simulator/carla-autoware).

### Run
1. Run Carla in a terminal `./CarlaUE4.sh`
2. Run existing test scenarios in another terminal `python scenario_runner/scenario_parser.py -f <scenarios/rule_1.yaml> -t <10> -m <auto>` where `-f` is the path to the scenario file and `-t` is the number of test map such as town10. `-m` is the model to use. Two models `auto` and `mmfn` are provided in `carla-expert/`, which is modified from [carla-expert](https://github.com/Kin-Zhang/carla-expert) to integrate with TARGET. `LAV` is the model from [LAV](https://github.com/Tsinghua-MARS-Lab/LAV). You can download the model and configure its path in `scenario_runner/scenario_parser.py`.
3. In another terminal, run `python scenario_vis.py` to visualize the test scenario.
4. To test models implemented in ROS such as Carla-AD in carla-ros-pkg and Autoware, follow the below steps:
    1. Run Carla in a terminal `./CarlaUE4.sh`
    2. In another terminal, run `roslaunch carla_ad_demo carla_ad_demo_scenario_runner.launch town:=<town_name>`
    3. Run Carla-AD in another terminal `roslaunch carla_ad_demo carla_ad_demo_agent.launch` or run Autoware in Docker following its instructions.
    4. Run `python scenario_runner/scenario_parser.py -f <scenarios/rule_1.yaml> -m <carla_ad> -r 0` to load the scenario.
    5. Run `roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch`.

### Generate new test scenarios
1. Run `python rule_parse.py` to generate scenario descriptions for rules in `rule_parser/rules.txt`, which is corresponding to the provided scenario files in `scenarios/`.
2. If you want to generate scenario descriptions using Llama, run your LLM model with `ollama run <llama_model>` (e.g. `ollama run llama3.1:70b`) and run `python rule_parse.py --model llama`.

### Scenario Description DSL
In the work, we propose a scenario description DSL to describe functional test scenario derived from traffic rules. The DSL in code is implemented in YAML format. The syntax and an example is as follows:
```yaml
"Actors":
  "Ego vehicle":
    "Behavior": "turn left"
    "Position":
      "Position reference": "intersection"
      "Position relation": "behind"
    "Type": "car"
  "Other actor 1":
    "Behavior": "go forward"
    "Position":
      "Position reference": "intersection"
      "Position relation": "opposite"
    "Type": "car"
"Environment":
  "Time": "daytime"
  "Weather": "sunny"
"Oracle":
  "lateral": "None"
  "longitudinal": "yield"
"Road network":
  "Road marker": "None"
  "Road type": "intersection"
  "Traffic signs": "None"
```
The above scenario description describes a scenario where the ego vehicle turns left at an intersection and the other vehicle goes forward at the same time, which is derived from the traffic rule "when turning left, always yield the right-of-way to any vehicle coming straight through from the other direction". In the functional test scenario, no parameter values need to be specified. The scenario parser in `scenario_runner` will identify the parameter values based on the scenario description and the map information.

Scenarios for testing ADSs are provided in `scenarios/`, and the corresponding traffic rules are provided in `rule_parser/rules.txt`. The full list of traffic rules to evaluate the effectiveness of LLM-based rule parser is provided in the [supplementary material](https://docs.google.com/document/d/1Xyr6hDOQb-ooNnwY71yE5w3pOiZpmD-7n17DfqyzAy8/edit?tab=t.0).




