import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import time

# --- App Configuration ---
st.set_page_config(
    page_title="PID Controller Demo",
    page_icon="⚖️",
    layout="wide"
)

# --- Simulation Constants ---
G = 9.81  # Acceleration due to gravity (m/s^2)
BEAM_LENGTH = 1.0  # Length of the beam (m)
BALL_RADIUS = 0.05 # Radius of the ball for visualization
DT = 0.02  # Time step for simulation (s)
SETPOINT = 0.0 # Target position for the ball (center of the beam)

# --- PID Controller Class ---
class PIDController:
    """A simple PID controller."""
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.max_integral = 2.0 # Anti-windup limit
        self.output_filter = 0.0 # For smoothing the output

    def update(self, current_value, dt):
        """Calculate the PID output."""
        error = self.setpoint - current_value

        # Proportional term
        p_term = self.Kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        i_term = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.Kd * derivative

        # Update previous error for the next iteration
        self.previous_error = error

        # The output is the desired angle of the beam
        output_angle = p_term + i_term + d_term
        
        # Apply output filtering for smoother control
        alpha = 0.8  # Filter coefficient
        self.output_filter = alpha * self.output_filter + (1 - alpha) * output_angle
        
        return self.output_filter

# --- State Initialization ---
def initialize_state():
    """Resets the simulation to its initial state."""
    st.session_state.ball_pos = -0.4  # Start position of the ball
    st.session_state.ball_vel = 0.0    # Initial velocity of the ball
    st.session_state.beam_angle = 0.0  # Initial angle of the beam
    st.session_state.pid_controller = PIDController(
        st.session_state.Kp,
        st.session_state.Ki,
        st.session_state.Kd,
        SETPOINT
    )

# --- Main App UI ---
st.title("⚖️ Interactive PID Controller Demo")
st.markdown("This simulation demonstrates how a PID controller can balance a ball on a beam. Adjust the `Kp`, `Ki`, and `Kd` gains in the sidebar to see how they affect the system's performance.")

# --- Educational Content ---
with st.expander("📚 PID Control Theory", expanded=False):
    st.markdown("""
    ### What is PID Control?
    PID control is a feedback mechanism that continuously calculates an error value and applies corrections based on three terms:
    
    **P (Proportional)** - Responds to current error: `Kp × error`
    - Higher Kp = faster response, but may overshoot
    - Lower Kp = slower response, but more stable
    
    **I (Integral)** - Eliminates steady-state error: `Ki × ∫error dt`
    - Higher Ki = eliminates error faster, but may oscillate
    - Lower Ki = slower error elimination, but more stable
    
    **D (Derivative)** - Dampens oscillations: `Kd × d(error)/dt`
    - Higher Kd = more damping, reduces overshoot
    - Lower Kd = less damping, faster response
    
    ### Tuning Tips:
    1. **Start with P only** (set Ki=0, Kd=0)
    2. **Add D** to reduce overshoot
    3. **Add I** to eliminate steady-state error
    """)

# --- Preset Values ---
with st.expander("🎛️ Quick Preset Values", expanded=False):
    col1, col2, col3 = st.columns(3)
    
    with col1:
        if st.button("Beginner", use_container_width=True):
            st.session_state.Kp = 20.0
            st.session_state.Ki = 8.0
            st.session_state.Kd = 25.0
            st.success("Loaded beginner-friendly settings!")
    
    with col2:
        if st.button("Aggressive", use_container_width=True):
            st.session_state.Kp = 35.0
            st.session_state.Ki = 15.0
            st.session_state.Kd = 40.0
            st.success("Loaded aggressive settings!")
    
    with col3:
        if st.button("Conservative", use_container_width=True):
            st.session_state.Kp = 12.0
            st.session_state.Ki = 4.0
            st.session_state.Kd = 15.0
            st.success("Loaded conservative settings!")
    
    st.markdown("""
    **Beginner**: Safe starting point with moderate response
    **Aggressive**: Fast response but may overshoot
    **Conservative**: Stable but slower response
    """)
    
    # Add a perfect balance preset
    if st.button("🎯 Perfect Balance", use_container_width=True):
        st.session_state.Kp = 25.0
        st.session_state.Ki = 10.0
        st.session_state.Kd = 30.0
        st.success("Loaded perfect balance settings!")

# --- Sidebar for Controls ---
with st.sidebar:
    st.header("Controller Gains")
    # Sliders for PID gains, stored in session state
    st.session_state.Kp = st.slider("Proportional Gain (Kp)", 0.0, 50.0, 20.0, 0.5)
    st.session_state.Ki = st.slider("Integral Gain (Ki)", 0.0, 30.0, 8.0, 0.5)
    st.session_state.Kd = st.slider("Derivative Gain (Kd)", 0.0, 50.0, 25.0, 0.5)

    st.header("Simulation Controls")
    if st.button("🔄 Reset Simulation", use_container_width=True):
        initialize_state()

    if st.button("💥 Push the Ball", use_container_width=True):
        # Apply a disturbance by adding to the ball's velocity
        if 'ball_vel' in st.session_state:
            st.session_state.ball_vel += 2.0
    
    # --- Real-time PID Analysis ---
    st.header("📊 Current Analysis")
    
    # Get current values
    kp = st.session_state.Kp
    ki = st.session_state.Ki
    kd = st.session_state.Kd
    
    # Analyze current settings
    if kp > 15:
        p_status = "🟡 High - May cause overshoot"
    elif kp > 8:
        p_status = "🟢 Good - Balanced response"
    else:
        p_status = "🔵 Low - Stable but slow"
    
    if ki > 8:
        i_status = "🟡 High - May oscillate"
    elif ki > 3:
        i_status = "🟢 Good - Eliminates steady-state error"
    else:
        i_status = "🔵 Low - May have persistent error"
    
    if kd > 20:
        d_status = "🟡 High - May be sluggish"
    elif kd > 10:
        d_status = "🟢 Good - Good damping"
    else:
        d_status = "🔵 Low - May overshoot"
    
    st.markdown(f"""
    **Proportional (Kp = {kp:.1f})**: {p_status}
    **Integral (Ki = {ki:.1f})**: {i_status}
    **Derivative (Kd = {kd:.1f})**: {d_status}
    """)
    
    # Overall assessment
    if kp > 12 and ki > 6 and kd < 10:
        overall = "⚠️ **Aggressive** - Fast but may be unstable"
    elif kp < 6 and ki < 2 and kd < 8:
        overall = "🐌 **Conservative** - Stable but slow"
    else:
        overall = "✅ **Balanced** - Good compromise"
    
    st.markdown(f"**Overall**: {overall}")

# --- Initialize session state if it doesn't exist ---
if 'ball_pos' not in st.session_state:
    initialize_state()

# Create a placeholder for the plot
plot_placeholder = st.empty()

# --- Main Simulation Loop ---
while True:
    # Get the current state
    ball_pos = st.session_state.ball_pos
    ball_vel = st.session_state.ball_vel
    pid = st.session_state.pid_controller

    # Update PID controller gains from sliders
    pid.Kp, pid.Ki, pid.Kd = st.session_state.Kp, st.session_state.Ki, st.session_state.Kd

    # 1. Calculate the control output from the PID controller
    # The output is the desired angle for the beam
    target_angle = pid.update(ball_pos, DT)

    # Clamp the beam angle to a maximum of +/- 30 degrees
    st.session_state.beam_angle = np.clip(target_angle, -np.pi/6, np.pi/6)

    # 2. Update the physics of the ball
    # Acceleration is caused by gravity component along the beam
    # The ball accelerates down the slope of the beam
    ball_acc = G * np.sin(st.session_state.beam_angle)
    
    # Add some friction to make the system more stable
    friction = -0.5 * ball_vel  # Friction coefficient
    ball_acc += friction
    
    ball_vel += ball_acc * DT
    ball_pos += ball_vel * DT

    # 3. Handle boundary conditions (ball falls off the beam)
    half_beam = BEAM_LENGTH / 2
    if ball_pos > half_beam or ball_pos < -half_beam:
        st.warning("The ball fell off! Try adjusting the PID gains or reset the simulation.")
        initialize_state() # Reset on failure
        time.sleep(2) # Pause to show the message
        continue

    # Update session state for the next iteration
    st.session_state.ball_pos = ball_pos
    st.session_state.ball_vel = ball_vel

    # 4. Create the plot for visualization
    fig, ax = plt.subplots(figsize=(12, 3))

    # Beam coordinates
    beam_x = [-half_beam, half_beam]
    beam_y = [0, 0]
    # Rotate the beam
    angle = st.session_state.beam_angle
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    rotated_beam = np.dot(rotation_matrix, [beam_x, beam_y])

    # Draw the beam
    ax.plot(rotated_beam[0], rotated_beam[1], 'k-', lw=5, solid_capstyle='round')

    # Draw the ball
    ball_x = ball_pos * np.cos(angle)
    ball_y = ball_pos * np.sin(angle) + BALL_RADIUS
    ball = plt.Circle((ball_x, ball_y), BALL_RADIUS, color='dodgerblue', zorder=10)
    ax.add_artist(ball)

    # Draw the setpoint (target)
    ax.vlines(SETPOINT, -0.2, 0.5, colors='r', linestyles='--', lw=2, label='Setpoint')

    # Plot styling
    ax.set_xlim(-half_beam - 0.1, half_beam + 0.1)
    ax.set_ylim(-0.5, 0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    # Only add legend if there are labeled elements
    if len(ax.get_legend_handles_labels()[0]) > 0:
        plt.legend()

    # Display the plot in the placeholder
    with plot_placeholder.container():
        st.pyplot(fig)
        # Close the figure to free up memory
        plt.close(fig)

    # Control the loop speed
    time.sleep(DT)