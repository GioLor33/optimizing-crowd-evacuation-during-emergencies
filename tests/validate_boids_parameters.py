import numpy as np
import copy
import time
from environments.scenarios import get_scenario_by_name
from parser.config import Config
from boids_algorithm.crowdSimulator import CrowdSimulator
SCENARIOS_TO_TEST = ["empty", "two_doors", "slalom"]

POPULATION_SIZE = 50
GENERATIONS = 30
MAX_STEPS = 2000
PARAM_RANGES = {
    'seek': [1.0, 10.0],
    'avoid': [1.0, 10.0],
    'separation': [0.5, 5.0],
    'alignment': [0.1, 3.0],
    'cohesion': [0.1, 3.0]
}
def run_simulation(weights, base_config, scenario_name):
    env = get_scenario_by_name(scenario_name)
    run_config = copy.copy(base_config)

    run_config.w_seek = weights[0]
    run_config.W_SEEK = weights[0]

    run_config.w_avoid = weights[1]
    run_config.W_AVOID = weights[1]

    run_config.w_separate = weights[2]
    run_config.W_SEPARATE = weights[2]

    run_config.w_align = weights[3]
    run_config.W_ALIGN = weights[3]

    run_config.w_cohere = weights[4]
    run_config.W_COHERE = weights[4]

    run_config.visualization = False

    sim = CrowdSimulator(env, config=run_config)
    steps = 0
    dt = run_config.dt
    total_agents = run_config.num_agents

    while steps < MAX_STEPS:
        sim.update(dt)
        steps += 1

        if len(sim.agents_escaped) == total_agents:
            break

    if steps >= MAX_STEPS:
        remaining = total_agents - len(sim.agents_escaped)
        steps = MAX_STEPS + (remaining * 100)
    return steps

def generate_random_individual():
    return [np.random.uniform(r[0], r[1]) for r in PARAM_RANGES.values()]


def mutate(individual, mutation_rate=0.6, strength=1.5):
    new_ind = individual[:]
    for i, (key, bounds) in enumerate(PARAM_RANGES.items()):
        if np.random.random() < mutation_rate:
            change = np.random.uniform(-strength, strength)
            new_ind[i] = np.clip(new_ind[i] + change, bounds[0], bounds[1])
    return new_ind


def crossover(parent1, parent2):
    split = np.random.randint(1, len(parent1) - 1)
    child = parent1[:split] + parent2[split:]
    return child

if __name__ == "__main__":

    try:
        base_config = Config("resources/config.yaml")
    except Exception as e:
        print(f"Error loading config: {e}")
        exit(1)

    for scenario_name in SCENARIOS_TO_TEST:
        population = [generate_random_individual() for _ in range(POPULATION_SIZE)]
        best_overall_score = float('inf')
        best_overall_weights = None

        for gen in range(GENERATIONS):
            scores = []
            for i, ind in enumerate(population):
                score = run_simulation(ind, base_config, scenario_name)
                scores.append((score, ind))

            scores.sort(key=lambda x: x[0])

            best_gen_score = scores[0][0]
            best_gen_weights = scores[0][1]

            if best_gen_score < best_overall_score:
                best_overall_score = best_gen_score
                best_overall_weights = best_gen_weights

            num_elites = max(2, int(POPULATION_SIZE * 0.2))
            survivors = [s[1] for s in scores[:num_elites]]
            next_gen = survivors[:]

            while len(next_gen) < POPULATION_SIZE:
                p1 = survivors[np.random.randint(len(survivors))]
                p2 = survivors[np.random.randint(len(survivors))]
                child = crossover(p1, p2)
                child = mutate(child)
                next_gen.append(child)

            population = next_gen

        print(f"\n>>> FINAL RESULTS FOR {scenario_name} <<<")
        keys = list(PARAM_RANGES.keys())
        for i, w in enumerate(best_overall_weights):
            print(f"  {keys[i]}: {w:.4f}")