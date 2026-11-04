# Experiments

`scripts/run_experiments.sh` runs the deterministic test backend across scenario names, seeds and `original`, `nearest`, `2opt` methods. It writes aggregate CSV and JSON. Each run gets a fresh `MarkStore` and independent deposition and failure random streams. It is not a fresh Isaac physics simulation. The backend honors obstacle geometry, deposition sigma and failure probability. Scenarios with localization or navigation fault fields fail explicitly and are recorded as failed runs; they need a future ROS/Isaac runtime.
