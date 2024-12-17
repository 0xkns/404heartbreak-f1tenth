# 404s & Heartbreak F1 Tenth

# Team Details
1. Shanthakumar Karan 
2. Harsh Vivekanand Pandey

## Algorithm

The algorithm our team is using is **Wall Tracing** algorithm 

### Explanation 

This code implements a wall-following algorithm using ROS2. The key task is to steer the vehicle to maintain a fixed distance from the wall while navigating curves. The control relies on **PID (Proportional-Integral-Derivative)** logic.

**The code layout is in form of:**

1. Laser Scan Implementation
2. Error Calculation
3. PID Control
4. Dynamic Speed Control

### Equations used within the Code

**Angle of Deviation** 

$$\
\alpha = \arctan\left(\frac{a \cdot \cos(\theta) - b}{a \cdot \sin(\theta)}\right)
\$$
 
**Dynamic Speed Control**

$$\
\text{speed} =
\begin{cases} 
v_{\text{turn}} & \text{if } |\text{steeringAngle}| > \text{turnThreshold} \\
v_{\text{default}} & \text{otherwise}
\end{cases}
\$$

**PID Control**

$$\
\text{steeringAngle} = k_p \cdot \text{error} + k_d \cdot (\text{error} - \text{prevError}) + k_i \cdot \text{accumulatedError}
\$$

**Error Calculation**

$$\
\text{error} = \text{setpoint} - \text{distance}
\$$

**Distance to the WALL**

$$\
\text{distance} = b \cdot \cos(\alpha)
\$$
