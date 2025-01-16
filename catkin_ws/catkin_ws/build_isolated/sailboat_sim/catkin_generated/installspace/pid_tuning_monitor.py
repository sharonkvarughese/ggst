#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np

class PIDTuningMonitor:
    def __init__(self):
        rospy.init_node('pid_tuning_monitor')
        
        # Store oscillation data for each controller
        self.controllers = {
            'speed': {'data': [], 'period': None, 'amplitude': None},
            'heading': {'data': [], 'period': None, 'amplitude': None},
            'turn_rate': {'data': [], 'period': None, 'amplitude': None}
        }
        
        # Subscribe to controller states
        rospy.Subscriber('speed_controller/state', Float64, lambda msg: self.state_callback(msg, 'speed'))
        rospy.Subscriber('heading_controller/state', Float64, lambda msg: self.state_callback(msg, 'heading'))
        rospy.Subscriber('turn_rate_controller/state', Float64, lambda msg: self.state_callback(msg, 'turn_rate'))
        
        # Store PID gains
        self.tuned_gains = {}
        
        rospy.loginfo("PID Tuning Monitor initialized")
        rospy.loginfo("Collecting oscillation data for Ziegler-Nichols tuning...")

    def state_callback(self, msg, controller_name):
        self.controllers[controller_name]['data'].append(msg.data)
        
        # When enough data collected, calculate parameters
        if len(self.controllers[controller_name]['data']) >= 200:  # 20 seconds at 10Hz
            self.calculate_zn_params(controller_name)

    def calculate_zn_params(self, controller_name):
        data = np.array(self.controllers[controller_name]['data'])
        
        # Find peaks for oscillation analysis
        peaks = []
        for i in range(1, len(data)-1):
            if data[i] > data[i-1] and data[i] > data[i+1]:
                peaks.append((i, data[i]))
        
        if len(peaks) >= 2:
            # Calculate period and amplitude
            periods = [peaks[i+1][0] - peaks[i][0] for i in range(len(peaks)-1)]
            period = np.mean(periods) * 0.1  # Convert to seconds (10Hz sampling)
            amplitude = np.mean([peak[1] for peak in peaks])
            
            # Store oscillation characteristics
            self.controllers[controller_name]['period'] = period
            self.controllers[controller_name]['amplitude'] = amplitude
            
            # Calculate Ziegler-Nichols parameters
            ku = 4 * amplitude / (np.pi * 0.1)  # Ultimate gain
            tu = period  # Ultimate period
            
            # Calculate PID gains using Ziegler-Nichols formulas
            kp = 0.6 * ku
            ki = 1.2 * ku / tu
            kd = 0.075 * ku * tu
            
            self.tuned_gains[controller_name] = {
                'Kp': kp,
                'Ki': ki,
                'Kd': kd
            }
            
            rospy.loginfo(f"\n=== Auto-tuned PID Gains for {controller_name} ===")
            rospy.loginfo(f"Kp: {kp:.3f}")
            rospy.loginfo(f"Ki: {ki:.3f}")
            rospy.loginfo(f"Kd: {kd:.3f}")
            rospy.loginfo(f"Period: {period:.2f}s")
            rospy.loginfo(f"Ultimate Gain (Ku): {ku:.3f}")
            rospy.loginfo("==========================================")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = PIDTuningMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
