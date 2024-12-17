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

This is rather complex, so it was skipped. 

## 
