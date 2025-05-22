# traffic_signal_controller.py
import time
import threading
import numpy as np
from enum import Enum

class SignalState(Enum):
    """Enum for traffic signal states."""
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"

class TrafficSignal:
    """Class representing a single traffic signal."""
    
    def __init__(self, signal_id, initial_state=SignalState.RED, default_durations=None):
        self.signal_id = signal_id
        self.state = initial_state
        
        # Default durations for each state in seconds
        self.default_durations = default_durations or {
            SignalState.RED: 30,
            SignalState.YELLOW: 5,
            SignalState.GREEN: 20
        }
        
        self.current_duration = self.default_durations[initial_state]
        self.time_in_state = 0
        self.is_emergency_mode = False
        
    def update(self, dt):
        """Update the signal state based on time elapsed."""
        if self.is_emergency_mode:
            return  # Don't change state in emergency mode
            
        self.time_in_state += dt
        
        if self.time_in_state >= self.current_duration:
            self.time_in_state = 0
            self.change_state()
            
    def change_state(self):
        """Change to the next state in the cycle."""
        if self.state == SignalState.RED:
            self.state = SignalState.GREEN
        elif self.state == SignalState.GREEN:
            self.state = SignalState.YELLOW
        elif self.state == SignalState.YELLOW:
            self.state = SignalState.RED
            
        self.current_duration = self.default_durations[self.state]
        
    def set_emergency_mode(self, active):
        """Activate or deactivate emergency mode."""
        self.is_emergency_mode = active
        if active:
            self.state = SignalState.GREEN
            print(f"Signal {self.signal_id} set to EMERGENCY mode (GREEN)")
        else:
            print(f"Signal {self.signal_id} returned to normal operation")
            
    def __str__(self):
        return f"Signal {self.signal_id}: {self.state.value} ({self.time_in_state:.1f}/{self.current_duration}s)"

class TrafficJunction:
    """Class representing a traffic junction with multiple signals."""
    
    def __init__(self, junction_id, num_signals=4):
        self.junction_id = junction_id
        self.signals = []
        
        # Create signals for the junction
        for i in range(num_signals):
            # Initialize signals with different starting states
            if i == 0:
                self.signals.append(TrafficSignal(f"{junction_id}_{i}", SignalState.GREEN))
            else:
                self.signals.append(TrafficSignal(f"{junction_id}_{i}", SignalState.RED))
                
        self.active_emergency_route = None
        
    def update(self, dt):
        """Update all signals at the junction."""
        for signal in self.signals:
            signal.update(dt)
            
    def set_emergency_route(self, from_signal_idx, to_signal_idx):
        """
        Set an emergency route through the junction.
        
        Args:
            from_signal_idx: Index of the signal where emergency vehicle is coming from
            to_signal_idx: Index of the signal where emergency vehicle is going to
        """
        # Reset all signals to red except the from and to signals
        for i, signal in enumerate(self.signals):
            if i == from_signal_idx or i == to_signal_idx:
                signal.set_emergency_mode(True)
            else:
                signal.state = SignalState.RED
                signal.set_emergency_mode(False)
                
        self.active_emergency_route = (from_signal_idx, to_signal_idx)
        print(f"Emergency route set at junction {self.junction_id}: {from_signal_idx} -> {to_signal_idx}")
        
    def clear_emergency_route(self):
        """Clear the emergency route and return to normal operation."""
        if self.active_emergency_route:
            print(f"Clearing emergency route at junction {self.junction_id}")
            for signal in self.signals:
                signal.set_emergency_mode(False)
            self.active_emergency_route = None

class TrafficSignalController:
    """Controller class for managing multiple traffic junctions."""
    
    def __init__(self):
        self.junctions = {}
        self.running = False
        self.update_thread = None
        self.incident_zones = {}  # Track areas with active incidents
        
    def add_junction(self, junction_id, num_signals=4):
        """Add a new junction to the controller."""
        self.junctions[junction_id] = TrafficJunction(junction_id, num_signals)
        return self.junctions[junction_id]
        
    def start(self):
        """Start the traffic signal control loop."""
        if self.running:
            return
            
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
        print("Traffic Signal Controller started")
        
    def stop(self):
        """Stop the traffic signal control loop."""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=1.0)
        print("Traffic Signal Controller stopped")
        
    def _update_loop(self):
        """Main update loop for the traffic signals."""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            for junction in self.junctions.values():
                junction.update(dt)
                
            time.sleep(0.1)  # Sleep to prevent high CPU usage
            
    def handle_emergency_vehicle(self, junction_id, approach_direction, target_direction):
        """
        Handle an emergency vehicle approaching a junction.
        
        Args:
            junction_id: ID of the junction
            approach_direction: Direction from which the vehicle is approaching (0-3)
            target_direction: Direction to which the vehicle is heading (0-3)
        """
        if junction_id not in self.junctions:
            print(f"Error: Junction {junction_id} not found")
            return False
            
        junction = self.junctions[junction_id]
        junction.set_emergency_route(approach_direction, target_direction)
        
        # Schedule clearing the emergency route after a delay
        threading.Timer(15.0, self._clear_emergency_route, args=[junction_id]).start()
        
        return True
        
    def _clear_emergency_route(self, junction_id):
        """Clear the emergency route for the specified junction."""
        if junction_id in self.junctions:
            self.junctions[junction_id].clear_emergency_route()
            
    def handle_incident(self, incident_info):
        """
        Adjust traffic patterns in response to an incident.
        
        Args:
            incident_info: Dictionary containing incident details
                - location: (x, y) coordinates
                - type: Type of incident
                - severity: Severity rating
                - affected_junctions: List of affected junction IDs
        """
        incident_id = f"inc_{int(time.time())}"
        self.incident_zones[incident_id] = incident_info
        
        print(f"Handling incident: {incident_info['type']} at {incident_info['location']}")
        print(f"Severity: {incident_info['severity']}")
        
        # Adjust signals at affected junctions based on incident severity
        for junction_id in incident_info['affected_junctions']:
            if junction_id not in self.junctions:
                continue
                
            junction = self.junctions[junction_id]
            
            # Simple strategy: If severe incident, prioritize outbound directions 
            # from the incident location
            if incident_info['severity'] == "HIGH":
                # Determine which signals should be prioritized (simplified)
                priority_signals = [0, 2]  # Example: prioritize north and south
                for i, signal in enumerate(junction.signals):
                    if i in priority_signals:
                        signal.default_durations[SignalState.GREEN] = 40  # Longer green
                        signal.default_durations[SignalState.RED] = 20    # Shorter red
                    else:
                        signal.default_durations[SignalState.GREEN] = 15  # Shorter green
                        signal.default_durations[SignalState.RED] = 40    # Longer red
                        
                print(f"Modified signal timings at junction {junction_id} due to severe incident")
                
        # Set a timer to clear the incident after a delay (simulating incident resolution)
        cleanup_delay = {
            "LOW": 60,    # 1 minute for low severity
            "MEDIUM": 180, # 3 minutes for medium severity
            "HIGH": 300    # 5 minutes for high severity
        }.get(incident_info['severity'], 120)
        
        threading.Timer(cleanup_delay, self._clear_incident, args=[incident_id]).start()
        
        return incident_id
        
    def _clear_incident(self, incident_id):
        """Clear an incident and restore normal traffic patterns."""
        if incident_id not in self.incident_zones:
            return
            
        incident_info = self.incident_zones[incident_id]
        print(f"Clearing incident: {incident_id}")
        
        # Restore default signal timings
        for junction_id in incident_info['affected_junctions']:
            if junction_id not in self.junctions:
                continue
                
            junction = self.junctions[junction_id]
            for signal in junction.signals:
                signal.default_durations = {
                    SignalState.RED: 30,
                    SignalState.YELLOW: 5,
                    SignalState.GREEN: 20
                }
                
        # Remove incident from tracking
        del self.incident_zones[incident_id]