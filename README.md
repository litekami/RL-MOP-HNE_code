# RL-MOP-HNE: Reinforcement-Learning-Guided Multi-Objective Trajectory Planning

MATLAB implementation of the RL-MOP-HNE algorithm for multi-objective trajectory planning of high-DOF robotic manipulators in obstacle environments.

## Dependencies

- MATLAB R2022b or later
- Peter Corke Robotics Toolbox (SerialLink, fkine, Jacobian)
  - Install from: https://petercorke.com/toolboxes/robotics-toolbox/

## Files

| File | Description |
|------|-------------|
| `src/generate_all_plots.m` | Main entry point. Runs all 6 algorithms on 3 test cases, generates figures and metrics. Contains 85+ functions. |
| `src/main_q_nsga_plan.m` | Standalone NSGA-II planner with RL guidance. |
| `src/main_rl_select.m` | RL training module (Tabular Q-learning / DQN). |
| `src/generate_setup_figure.m` | UR5 simulation setup figure generator. |
| `src/focused_experiment_runner.m` | Single-case experiment runner with all 6 algorithms. |
| `src/quick_revision_runner.m` | Quick test runner with reduced generations. |
| `ablation/main.m` | Ablation study main script. |
| `ablation/runPlanner.m` | Single planner runner for ablations. |
| `ablation/radarPlot.m` | Radar plot generator. |

## Quick Start

```matlab
% 1. Add Robotics Toolbox to path
% 2. Run main experiment:
>> cd src/
>> generate_all_plots

% Or for a quick test:
>> quick_revision_runner
```

## Algorithm

RL-MOP-HNE integrates three key components:
1. **NSGA-II Core**: Multi-objective evolutionary optimization
2. **SARSA-based RL Guidance**: State-aware search control
3. **Hierarchical Neighborhood Evolution (HNE)**: Multi-resolution search mechanism

## Citation


## DOI
10.5281/zenodo.21194284
