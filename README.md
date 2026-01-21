# optimizing-crowd-evacuation-during-emergencies

Crowd evacuation during emergency situations poses significant challenges due to complex interactions among pedestrians and with the surrounding environment. This work investigates the effectiveness of bio-inspired swarm intelligence techniques for optimizing evacuation dynamics in enclosed spaces. Three approaches are analyzed and compared: Reynolds’ Boids model, Particle Swarm Optimization (PSO), and Ant Colony Optimization (ACO).

While the Boids framework relies on local behavioral rules, PSO and ACO are integrated within the Social Force Model (SFM) to better capture physical interactions. A custom simulation engine and visualizer were developed to evaluate the algorithms under controlled conditions. 

The comparative analysis highlights the strengths and limitations of each approach and provides insights into the suitability of swarm-based optimization methods for emergency evacuation modeling. 

## Run the code

### Prepare the python environment
To run the code, a python virtual environment is suggested.

```bash
python -m venv <name_env>
source <name_env>/bin/activate
```

Some packages are required for the program to work. These are listed inside the `requirements.txt` file.

```bash
pip install -r requirements.txt
```

### Change configuration parameters
All the parameters that may change the simulation can be found inside the `resources/config.yaml` file. Change them as needed before running the program.

### Start the simulation
To see the simulation evolving, run the program. Notice that the config file is automatically read from the `resources` directory.

```bash
python3 main.py
```

