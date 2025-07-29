# ⚖️ Interactive PID Controller Demo

An interactive web application that demonstrates PID (Proportional-Integral-Derivative) control theory through a real-time simulation of balancing a ball on a beam.

## 🎯 Overview

This application simulates a classic control theory problem: balancing a ball on a pivoting beam. The system uses a PID controller to maintain the ball at the center of the beam by adjusting the beam's angle in real-time.

## ✨ Features

- **Real-time Simulation**: Watch the ball balance on the beam with physics-based motion
- **Interactive PID Controls**: Adjust Kp, Ki, and Kd gains using sliders to see immediate effects
- **Visual Feedback**: Real-time plotting shows the beam, ball position, and target setpoint
- **Disturbance Testing**: Push the ball button to test controller robustness
- **Reset Functionality**: Easily restart the simulation with different parameters

## 🚀 Installation

1. **Clone or download this repository**

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the application**:
   ```bash
   streamlit run pid_viewer.py
   ```

## 🎮 Usage

1. **Open your browser** to the URL shown in the terminal (typically `http://localhost:8501`)

2. **Try the preset values** for quick experimentation:
   - **🎯 Perfect Balance**: Optimized settings for stable control
   - **Beginner**: Safe starting point with moderate response
   - **Aggressive**: Fast response but may overshoot
   - **Conservative**: Stable but slower response

3. **Adjust the PID gains** in the sidebar:
   - **Kp (Proportional)**: Controls how aggressively the system responds to error
   - **Ki (Integral)**: Eliminates steady-state error over time
   - **Kd (Derivative)**: Reduces overshoot and improves stability

4. **Test the system**:
   - Use the "Push the Ball" button to apply disturbances
   - Watch how different PID values affect the response
   - Reset the simulation when the ball falls off
   - Observe the real-time analysis in the sidebar

## 🧮 PID Controller Theory

The PID controller calculates the beam angle using three terms:

- **Proportional (P)**: `Kp × error` - Responds to current error
- **Integral (I)**: `Ki × ∫error dt` - Accumulates past errors to eliminate steady-state error
- **Derivative (D)**: `Kd × d(error)/dt` - Responds to rate of change of error

## 🎛️ Recommended Starting Values

### 🎯 Perfect Balance (Recommended)
- **Kp = 25.0**: Strong proportional response for quick correction
- **Ki = 10.0**: Good integral action to eliminate steady-state error
- **Kd = 30.0**: Strong derivative action for stability and damping

### Beginner-Friendly Settings
- **Kp = 20.0**: Moderate proportional response - good starting point
- **Ki = 8.0**: Light integral action to reduce steady-state error
- **Kd = 25.0**: Moderate derivative action for stability

### Aggressive Settings
- **Kp = 35.0**: Strong proportional response - faster but may overshoot
- **Ki = 15.0**: Strong integral action - eliminates error quickly
- **Kd = 40.0**: Strong derivative action - very stable but may be sluggish

### Conservative Settings
- **Kp = 12.0**: Gentle proportional response - slower but stable
- **Ki = 4.0**: Very light integral action - minimal steady-state correction
- **Kd = 15.0**: Light derivative action - less damping

## 🧮 PID Control Theory Explained

### What is PID Control?

PID control is a feedback control mechanism that continuously calculates an error value (the difference between a desired setpoint and a measured process variable) and applies a correction based on proportional, integral, and derivative terms.

### The Three Components:

#### 1. Proportional (P) - "Present Error"
- **Formula**: `P_output = Kp × error`
- **What it does**: Responds immediately to the current error
- **Effect**: 
  - Higher Kp = faster response, but may cause overshoot
  - Lower Kp = slower response, but more stable
- **In our simulation**: Controls how quickly the beam tilts to correct ball position

#### 2. Integral (I) - "Past Errors"
- **Formula**: `I_output = Ki × ∫error dt`
- **What it does**: Accumulates past errors to eliminate steady-state error
- **Effect**:
  - Higher Ki = eliminates steady-state error faster, but may cause oscillation
  - Lower Ki = slower elimination of steady-state error, but more stable
- **In our simulation**: Gradually corrects for any persistent offset from the center

#### 3. Derivative (D) - "Future Error Prediction"
- **Formula**: `D_output = Kd × d(error)/dt`
- **What it does**: Predicts future error based on the rate of change
- **Effect**:
  - Higher Kd = more damping, reduces overshoot, but may slow response
  - Lower Kd = less damping, faster response, but may overshoot
- **In our simulation**: Dampens oscillations and improves stability

### Tuning Guidelines:

1. **Start with P only** (Ki = 0, Kd = 0):
   - Increase Kp until you get a reasonable response
   - Look for about 20-30% overshoot

2. **Add D** to reduce overshoot:
   - Increase Kd until overshoot is acceptable
   - Be careful not to make it too sluggish

3. **Add I** to eliminate steady-state error:
   - Start with small Ki values
   - Increase gradually to eliminate any persistent offset

### Common Issues:

- **Oscillations**: Usually too much Kp or Ki, or not enough Kd
- **Slow response**: Too much Kd or not enough Kp
- **Steady-state error**: Need more Ki
- **Instability**: Too much Kp or Ki, reduce gains

## 🔧 System Parameters

- **Beam Length**: 1.0 meters
- **Ball Radius**: 0.05 meters
- **Time Step**: 0.02 seconds
- **Gravity**: 9.81 m/s²
- **Max Beam Angle**: ±30 degrees
- **Setpoint**: 0.0 (center of beam)
- **Friction Coefficient**: 0.5 (damping for stability)
- **Output Filter**: 0.8 (smoothing coefficient)

## 🛠️ Technical Details

- Built with **Streamlit** for the web interface
- Uses **NumPy** for numerical computations
- **Matplotlib** for real-time plotting
- **Real-time physics simulation** with gravity, friction, and boundary conditions
- **Enhanced PID controller** with output filtering and anti-windup protection
- **Interactive presets** for easy experimentation
- **Real-time analysis** of PID parameter effects

## 🎓 Learning Objectives

This demo helps understand:
- How PID controllers work in practice
- The effects of different gain values
- System stability and response characteristics
- Real-time control system behavior
- Physics-based control system simulation
- Tuning strategies for different performance objectives

## 📝 License

This project is open source and available under the MIT License.

## 🤝 Contributing

Feel free to submit issues, feature requests, or pull requests to improve this educational tool! 