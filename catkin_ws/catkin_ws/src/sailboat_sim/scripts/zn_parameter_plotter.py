#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from collections import deque
import threading

class ZNParameterPlotter:
    def __init__(self):
        rospy.init_node('zn_parameter_plotter')
        
        # Data storage
        self.time_window = 30  # seconds
        self.sample_rate = 50  # Hz
        buffer_size = self.time_window * self.sample_rate
        
        # Store absolute start time
        self.start_time = None
        
        # Separate buffers for each controller
        self.times = deque(maxlen=buffer_size)
        self.setpoints = {
            'speed': deque(maxlen=buffer_size),
            'heading': deque(maxlen=buffer_size),
            'turn_rate': deque(maxlen=buffer_size)
        }
        self.states = {
            'speed': deque(maxlen=buffer_size),
            'heading': deque(maxlen=buffer_size),
            'turn_rate': deque(maxlen=buffer_size)
        }
        
        # Store oscillation data
        self.oscillations = {
            'speed': [],
            'heading': [],
            'turn_rate': []
        }
        
        # Store parameters
        self.Ku = {
            'speed': 0.0,
            'heading': 0.0,
            'turn_rate': 0.0
        }
        self.Tu = {
            'speed': 0.0,
            'heading': 0.0,
            'turn_rate': 0.0
        }
        
        # Store initial values
        self.initial_values = {
            'speed': None,
            'heading': None,
            'turn_rate': None
        }
        
        # Store mean values for oscillation detection
        self.mean_values = {
            'speed': None,
            'heading': None,
            'turn_rate': None
        }
        
        # Store first zero crossing time
        self.first_crossing_time = {
            'speed': None,
            'heading': None,
            'turn_rate': None
        }
        
        # Store last values for zero crossing detection
        self.last_values = {
            'speed': deque(maxlen=3),
            'heading': deque(maxlen=3),
            'turn_rate': deque(maxlen=3)
        }
        
        # Subscribe to topics
        for control_type in ['speed', 'heading', 'turn_rate']:
            rospy.Subscriber(f'/{control_type}/setpoint', Float64, 
                           lambda msg, ct=control_type: self.setpoint_callback(msg, ct))
            rospy.Subscriber(f'/{control_type}/state', Float64,
                           lambda msg, ct=control_type: self.state_callback(msg, ct))
        
        # Start plotting thread
        self.plotting_thread = threading.Thread(target=self.plot_loop)
        self.plotting_thread.daemon = True
        self.plotting_thread.start()
        
        rospy.loginfo("ZN Parameter Plotter initialized")
        
    def setpoint_callback(self, msg, control_type):
        if self.start_time is None:
            self.start_time = rospy.get_time()
        
        curr_time = rospy.get_time() - self.start_time
        self.times.append(curr_time)
        self.setpoints[control_type].append(msg.data)
        
    def state_callback(self, msg, control_type):
        # Store initial value
        if self.initial_values[control_type] is None:
            self.initial_values[control_type] = msg.data
            rospy.loginfo(f"Initial {control_type} value: {msg.data}")
            
        if self.start_time is None:
            self.start_time = rospy.get_time()
            
        self.states[control_type].append(msg.data)
        self.last_values[control_type].append(msg.data)
        
        # Update mean value
        if len(self.states[control_type]) > 10:
            self.mean_values[control_type] = np.mean(list(self.states[control_type])[-10:])
        
        self.detect_oscillations(control_type)
        
    def detect_oscillations(self, control_type):
        if len(self.last_values[control_type]) < 3:
            return
            
        # Detect oscillations using improved method
        if len(self.states[control_type]) > 50:  # Wait for enough data
            recent_values = list(self.states[control_type])[-50:]
            setpoint = list(self.setpoints[control_type])[-1]
            
            # Calculate oscillation characteristics
            max_val = max(recent_values)
            min_val = min(recent_values)
            amplitude = (max_val - min_val) / 2
            mean = (max_val + min_val) / 2
            
            # Update mean value for oscillation detection
            self.mean_values[control_type] = mean
            
            # Zero crossing detection with hysteresis
            v1, v2, v3 = list(self.last_values[control_type])
            if (v2 - mean) * (v3 - mean) < 0:  # Zero crossing around mean
                curr_time = rospy.get_time() - self.start_time
                
                # Record first zero crossing
                if self.first_crossing_time[control_type] is None:
                    self.first_crossing_time[control_type] = curr_time
                    rospy.loginfo(f"First {control_type} oscillation at: {curr_time:.3f}s")
                
                self.oscillations[control_type].append(curr_time)
                
                if len(self.oscillations[control_type]) >= 3:
                    # Calculate Tu from last three zero crossings
                    Tu = (self.oscillations[control_type][-1] - 
                         self.oscillations[control_type][-3]) / 2
                    
                    # Calculate Ku only if there are real oscillations
                    # Adjust these thresholds based on your system
                    amplitude_thresholds = {
                        'speed': 0.1,
                        'heading': 0.05,
                        'turn_rate': 0.02
                    }
                    
                    if amplitude > amplitude_thresholds[control_type] and abs(setpoint) > 0.001:
                        self.Ku[control_type] = 4 * amplitude / (np.pi * abs(setpoint))
                        self.Tu[control_type] = Tu
                        
                        rospy.loginfo(f"\nOscillation detected for {control_type}:")
                        rospy.loginfo(f"Time: {curr_time:.3f}s")
                        rospy.loginfo(f"Amplitude: {amplitude:.3f}")
                        rospy.loginfo(f"Period (Tu): {Tu:.3f}s")
                        rospy.loginfo(f"Ultimate Gain (Ku): {self.Ku[control_type]:.3f}")
                        
                        self.calculate_zn_params(control_type)
    
    def calculate_zn_params(self, control_type):
        if self.Ku[control_type] > 0 and self.Tu[control_type] > 0:
            # Classic Ziegler-Nichols formulas with more conservative gains
            Kp = 0.45 * self.Ku[control_type]  # More conservative than standard 0.6
            Ti = 0.5 * self.Tu[control_type]
            Td = 0.125 * self.Tu[control_type]
            
            Ki = Kp / Ti if Ti > 0 else 0
            Kd = Kp * Td
            
            rospy.loginfo(f"\nCalculated PID gains for {control_type}:")
            rospy.loginfo(f"Kp: {Kp:.3f}")
            rospy.loginfo(f"Ki: {Ki:.3f}")
            rospy.loginfo(f"Kd: {Kd:.3f}")
            rospy.loginfo(f"Ti: {Ti:.3f}")
            rospy.loginfo(f"Td: {Td:.3f}")

    def plot_loop(self):
        plt.ion()
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
        
        while not rospy.is_shutdown():
            try:
                if len(self.times) > 0:
                    times_array = np.array(self.times)
                    
                    for ax, (control_type, label) in zip(
                        [ax1, ax2, ax3], 
                        [('speed', 'Speed (m/s)'), 
                         ('heading', 'Heading (rad)'),
                         ('turn_rate', 'Turn Rate (rad/s)')]):
                        
                        ax.clear()
                        if len(self.states[control_type]) > 0:
                            states_array = np.array(list(self.states[control_type]))
                            setpoints_array = np.array(list(self.setpoints[control_type]))
                            
                            # Ensure all arrays have same length
                            min_len = min(len(times_array), len(states_array), len(setpoints_array))
                            
                            ax.plot(times_array[-min_len:], setpoints_array[-min_len:], 
                                  'r--', label='Setpoint')
                            ax.plot(times_array[-min_len:], states_array[-min_len:], 
                                  'b-', label='State')
                            
                            # Add mean value line if oscillating
                            if self.mean_values[control_type] is not None:
                                ax.axhline(y=self.mean_values[control_type], color='g', 
                                         linestyle=':', alpha=0.5)
                            
                            ax.set_xlabel('Time (s)')
                            ax.set_ylabel(label)
                            ax.grid(True, which='both', linestyle='--', alpha=0.6)
                            ax.minorticks_on()
                            
                            # Dynamic y-axis limits
                            if control_type == 'speed':
                                ax.set_ylim(-0.5, 4.0)
                            elif control_type == 'heading':
                                ax.set_ylim(-3.5, 3.5)
                            elif control_type == 'turn_rate':
                                ax.set_ylim(-1.0, 1.0)
                            
                            # Always show from time 0
                            ax.set_xlim(0, max(times_array))
                            
                            # Add parameters to plot
                            param_text = (f'Ku: {self.Ku[control_type]:.3f}\n'
                                        f'Tu: {self.Tu[control_type]:.3f}\n'
                                        f'Ti: {self.Tu[control_type]/2 if self.Tu[control_type] > 0 else 0:.3f}')
                            ax.text(0.02, 0.98, param_text,
                                  transform=ax.transAxes, 
                                  verticalalignment='top',
                                  bbox=dict(facecolor='white', alpha=0.8))
                            
                            ax.legend()
                    
                    plt.tight_layout()
                    plt.pause(0.1)
                    
            except Exception as e:
                rospy.logwarn(f"Error in plotting loop: {str(e)}")
                plt.pause(0.1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        plotter = ZNParameterPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
