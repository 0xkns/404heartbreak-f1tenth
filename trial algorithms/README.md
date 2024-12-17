# There are various algorithms we experimented with to get a Suitable Track Time. 

## Bayesian filtering

Bayesian filtering is a probabilistic method used to estimate the state of a dynamic system. For autonomous racing, it can be applied to optimize the racing line, ensuring the vehicle follows the fastest and most efficient path around the track while accounting for uncertainties in sensor data, vehicle dynamics, and environmental conditions.

### Example workflow for the Bayesian Filtering 

1. **Initialization:**

i. Start with a predefined or rough racing line.
ii. Define a probabilistic model for the track and vehicle dynamics.

2. **Prediction Step:**

i. Predict the next state of the racing line based on the current model and dynamics.

3. **Update Step:**

i. Use sensor measurements to update the state estimate, correcting for errors or uncertainties.

4. **Optimization:**

i. Iteratively adjust the racing line using the refined estimates to minimize the lap time.

5. **Execution:**

i. Apply the optimized racing line to control algorithms for real-time implementation on the autonomous vehicle.

**This is rather complex, so it was skipped.**

## Levine Opitimisations 

The Levine Method in autonomous racing refers to a strategy that optimizes the vehicle's trajectory for minimum lap time while considering vehicle dynamics constraints. This method uses a blend of optimal control theory and nonlinear optimization to compute the racing line (trajectory) that balances speed, safety, and physical limits. Levin uses the **Cost System** method which could be good but is rather jaggery and uneven, as it can have various fluctuations. 

## Wall Tracing
