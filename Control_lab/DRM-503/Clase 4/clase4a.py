import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

distance = ctrl.Antecedent(np.arange(0, 101, 1), 'distance')

turn = ctrl.Consequent(np.arange(-100, 101, 1), 'turn')

# Distancia
distance['near']   = fuzz.trimf(distance.universe, [0, 0, 40])
distance['medium'] = fuzz.trimf(distance.universe, [20, 50, 80])
distance['far']    = fuzz.trimf(distance.universe, [60, 100, 100])

# Giro
turn['sharp_right'] = fuzz.trimf(turn.universe, [0, 40, 80])
turn['none']        = fuzz.trimf(turn.universe, [-10, 0, 10])
turn['sharp_left']  = fuzz.trimf(turn.universe, [-80, -40, 0])

rule1 = ctrl.Rule(distance['near'],   turn['sharp_right'])
rule2 = ctrl.Rule(distance['medium'], turn['none'])
rule3 = ctrl.Rule(distance['far'],    turn['sharp_left'])

fuzzy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
sim = ctrl.ControlSystemSimulation(fuzzy_ctrl)

input_distance = 35

sim.input['distance'] = input_distance
sim.compute()

output_turn = sim.output['turn']
print(f"Distancia = {input_distance} cm → giro = {output_turn:.2f}")

distance.view(sim=sim)
turn.view(sim=sim)
plt.show()
