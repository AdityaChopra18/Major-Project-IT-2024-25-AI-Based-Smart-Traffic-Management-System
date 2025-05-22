# enhanced_simulator_improved.py
import time
import numpy as np
import cv2
import threading
import queue  # Using queue for BFS
from enum import Enum

# Assume traffic_signal_controller.py exists with SignalState Enum
# from traffic_signal_controller import SignalState

# Mock SignalState and Controller for self-contained example
class SignalState(Enum):
    """Enum for traffic signal states."""
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"

class MockSignal:
    def __init__(self):
        self.state = SignalState.RED
        self.default_durations = {
            SignalState.RED: 30,
            SignalState.YELLOW: 5,
            SignalState.GREEN: 20
        }
        self._time_in_state = 0.0 # Track time in current state

    def update(self, dt):
        """Update the signal state based on time."""
        self._time_in_state += dt

        if self.state == SignalState.RED:
            if self._time_in_state >= self.default_durations[SignalState.RED]:
                self.state = SignalState.GREEN
                self._time_in_state = 0.0
        elif self.state == SignalState.GREEN:
            if self._time_in_state >= self.default_durations[SignalState.GREEN]:
                self.state = SignalState.YELLOW
                self._time_in_state = 0.0
        elif self.state == SignalState.YELLOW:
            if self._time_in_state >= self.default_durations[SignalState.YELLOW]:
                self.state = SignalState.RED
                self._time_in_state = 0.0

    def set_state(self, state):
        """Force set the signal state."""
        if state in SignalState:
            self.state = state
            self._time_in_state = 0.0

class MockJunction:
    def __init__(self, junction_id):
        self.id = junction_id
        # Signals for each approach: 0=North, 1=East, 2=South, 3=West
        self.signals = [MockSignal(), MockSignal(), MockSignal(), MockSignal()]
        self._current_phase = 0 # Simple phase cycling
        self._phase_duration = 30 # Duration for each phase in static mode
        self._emergency_active = False # Flag to indicate if emergency priority is active
        self._emergency_approach = -1 # The approach with emergency priority
        self._emergency_vehicle_id = None # ID of the emergency vehicle that triggered priority

    def set_emergency_priority(self, approach_dir, emergency_vehicle_id):
        """Set priority for an emergency vehicle approaching from a specific direction."""
        if not self._emergency_active: # Only grant priority if not already in emergency mode
            print(f"Junction {self.id}: Setting emergency priority for approach {approach_dir} for vehicle {emergency_vehicle_id}")
            self._emergency_active = True
            self._emergency_approach = approach_dir
            self._emergency_vehicle_id = emergency_vehicle_id
            # Force green for the emergency vehicle's approach and red for others
            for i, signal in enumerate(self.signals):
                if i == approach_dir:
                    signal.set_state(SignalState.GREEN)
                else:
                    signal.set_state(SignalState.RED)
        # else:
            # print(f"Junction {self.id}: Emergency priority already active.")


    def reset_emergency_priority(self, emergency_vehicle_id):
        """Reset emergency priority if the specified vehicle was the one that triggered it."""
        if self._emergency_active and self._emergency_vehicle_id == emergency_vehicle_id:
            print(f"Junction {self.id}: Resetting emergency priority triggered by vehicle {emergency_vehicle_id}.")
            self._emergency_active = False
            self._emergency_approach = -1
            self._emergency_vehicle_id = None
            # Signals will resume normal operation based on the global mode in the next update cycle


    def update(self, dt, mode, traffic_density=None):
        """Update junction logic based on simulation mode."""
        # If emergency priority is active, signals are controlled by set_emergency_priority
        # and remain in that state until reset.
        if self._emergency_active:
            # No updates needed here, signals are held in emergency state.
            pass

        elif mode == SimulationMode.STATIC:
            # Simple static cycling
            for signal in self.signals:
                signal.update(dt) # Let individual signals cycle

        elif mode == SimulationMode.DYNAMIC and traffic_density:
            # Dynamic adjustment based on density (simplified)
            # Find the busiest approach
            busiest_approach = max(traffic_density, key=traffic_density.get)
            max_density = traffic_density[busiest_approach]

            # Adjust green times (simplified logic)
            for i, signal in enumerate(self.signals):
                if i == busiest_approach:
                    # Longer green for busiest
                    signal.default_durations[SignalState.GREEN] = 20 + min(max_density * 2, 40) # Max 60s green
                    signal.default_durations[SignalState.RED] = 20 # Shorter red
                else:
                    # Shorter green for others
                    signal.default_durations[SignalState.GREEN] = max(5, 20 - max_density) # Min 5s green
                    signal.default_durations[SignalState.RED] = 40 # Longer red

                signal.update(dt) # Update signal state based on new durations

        # If global mode is EMERGENCY but _emergency_active is False,
        # it means the global mode was set but no specific vehicle triggered priority.
        # In a real scenario, this might mean all signals go red, or a specific emergency plan is activated.
        # For this mock, we'll just let signals continue based on the last non-emergency logic
        # until a vehicle triggers priority.

class MockController:
    def __init__(self):
        self.junctions = {}

    def add_junction(self, junction_id):
        """Add a junction to the controller."""
        self.junctions[junction_id] = MockJunction(junction_id)
        return self.junctions[junction_id]

    def update(self, dt, simulation_mode, junction_traffic_data):
        """Update all controlled junctions."""
        for j_id, junction in self.junctions.items():
            density_data = junction_traffic_data.get(j_id)
            # Pass the global simulation mode. The junction's internal _emergency_active flag
            # will override normal logic when set by an emergency vehicle.
            junction.update(dt, simulation_mode, density_data)

# End Mock Controller

class SimulationMode(Enum):
    """Enum for simulation operation modes."""
    STATIC = "STATIC"     # Fixed timing for signals
    DYNAMIC = "DYNAMIC"   # Adaptive timing based on traffic conditions
    EMERGENCY = "EMERGENCY"  # Override mode for emergency situations

class TrafficDensity(Enum):
    """Enum for traffic density levels."""
    LOW = "LOW"           # Light traffic
    MEDIUM = "MEDIUM"     # Moderate traffic
    HIGH = "HIGH"         # Heavy traffic
    CRITICAL = "CRITICAL" # Gridlock conditions

class TrafficSimulator:
    """An enhanced simulator for realistic traffic management scenarios."""

    def __init__(self, width=1024, height=768):
        self.width = width
        self.height = height
        self.frame = np.zeros((height, width, 3), dtype=np.uint8)
        self.junctions = {}
        self.roads = {}
        self.emergency_vehicles = []
        self.regular_vehicles = []
        self.incidents = []
        self.running = False
        self.sim_thread = None
        self.simulation_time = 0
        self.simulation_speed = 1.0  # 1.0 = real-time, 2.0 = 2x speed, etc.

        # Simulation mode
        self.mode = SimulationMode.STATIC

        # Time of day simulation
        self.time_of_day = 12  # 24-hour format
        self.rush_hours = [(7, 10), (16, 19)]  # Morning and evening rush hours
        self.is_rush_hour = False

        # Weather conditions affecting traffic (0-1, 1 is worst)
        self.weather_impact = 0

        # Stats tracking
        self.stats = {
            "emergency_response_times": [],
            "average_wait_times": {}, # Per junction
            "congestion_levels": {}, # Per road
            "incidents_resolved": 0,
            "total_vehicles_completed": 0
        }

        # Traffic density heatmap
        self.traffic_density_map = np.zeros((height, width), dtype=np.float32)

        # Visualization options
        self.show_density_map = False
        self.show_stats = True
        self.show_routes = False # Option to visualize vehicle routes (can be noisy)

        # Reference to the traffic signal controller
        self.traffic_controller = None

    def add_junction(self, junction_id, position, num_lanes=2):
        """
        Add a junction to the simulation.

        Args:
            junction_id: Unique identifier for the junction
            position: (x, y) coordinates
            num_lanes: Number of lanes per approach (influences road width)
        """
        self.junctions[junction_id] = {
            "position": position,
            "controller": None, # Will be linked to a controller junction
            "num_lanes": num_lanes,
            "traffic_density": {0: 0, 1: 0, 2: 0, 3: 0},  # Density for each approach (vehicles approaching)
            "waiting_vehicles": {0: [], 1: [], 2: [], 3: []},  # Vehicles waiting at each approach
            "current_mode": self.mode # Junction's current operational mode
        }

        # Initialize stats tracking for this junction
        self.stats["average_wait_times"][junction_id] = 0
        # Congestion level is tracked per road, not per junction directly in this version
        # self.stats["congestion_levels"][junction_id] = 0

    def add_road(self, road_id, start_junction_id, end_junction_id, num_lanes=2, max_speed=60):
        """
        Add a road connecting two junctions. Roads are bidirectional.

        Args:
            road_id: Unique identifier for the road
            start_junction_id: ID of the starting junction
            end_junction_id: ID of the ending junction
            num_lanes: Number of lanes in each direction
            max_speed: Maximum speed limit in pixels per second (assuming 1 pixel ~ 1 meter)
        """
        if start_junction_id not in self.junctions or end_junction_id not in self.junctions:
            print(f"Error: Junction not found when adding road {road_id}")
            return

        start_pos = self.junctions[start_junction_id]["position"]
        end_pos = self.junctions[end_junction_id]["position"]

        # Calculate road length and direction
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        length = np.sqrt(dx*dx + dy*dy)
        direction = np.arctan2(dy, dx) # Angle in radians

        self.roads[road_id] = {
            "start_junction": start_junction_id,
            "end_junction": end_junction_id,
            "start_pos": start_pos,
            "end_pos": end_pos,
            "length": length,
            "direction": direction, # Angle from start to end
            "num_lanes": num_lanes,
            "max_speed": max_speed,
            "vehicles": [], # Vehicles currently on this road
            "congestion_level": 0.0  # 0-1 scale
        }

        # Initialize congestion tracking for this road
        self.stats["congestion_levels"][road_id] = 0.0

    def set_traffic_controller(self, controller):
        """
        Set the traffic signal controller for the simulation.
        Links simulator junctions to controller junctions.
        """
        self.traffic_controller = controller
        for j_id, junction_data in self.junctions.items():
            if j_id in controller.junctions:
                junction_data["controller"] = controller.junctions[j_id]
                print(f"Linked simulator junction {j_id} to controller junction.")
            else:
                print(f"Warning: No controller junction found for simulator junction {j_id}")


    def add_regular_vehicle(self, start_junction_id, route, vehicle_type="car"):
        """
        Add a regular vehicle to the simulation.

        Args:
            start_junction_id: ID of the starting junction
            route: List of junction IDs defining the vehicle's route
            vehicle_type: Type of vehicle (car, bus, truck)
        """
        if start_junction_id not in self.junctions:
            print(f"Error: Starting junction {start_junction_id} not found")
            return None

        # Check if route is valid and find the first road segment
        if not all(j_id in self.junctions for j_id in route):
            print("Error: Invalid route - some junctions don't exist")
            return None

        if len(route) < 2:
             print("Error: Route must contain at least two junctions.")
             return None

        # Find the first road segment for the vehicle
        first_road_id = self._find_road_id(route[0], route[1])
        if not first_road_id:
             print(f"Error: No road found between {route[0]} and {route[1]} for vehicle route.")
             return None

        # Create vehicle
        speed_factors = {"car": 1.0, "bus": 0.8, "truck": 0.7}
        speed_factor = speed_factors.get(vehicle_type, 1.0)

        vehicle = {
            "id": f"v_{len(self.regular_vehicles)}",
            "type": vehicle_type,
            "position": list(self.junctions[start_junction_id]["position"]), # Use list for mutability
            "current_junction": start_junction_id, # The last junction the vehicle passed
            "next_junction_index": 1, # Index in the route of the next junction to reach
            "route": route,
            "speed": 0, # Start stationary
            "max_speed": 70 * speed_factor, # Pixels per second
            "acceleration": 20 * speed_factor, # Pixels per second^2
            "spawned_time": self.simulation_time,
            "waiting_time": 0, # Time waiting at junctions
            "total_travel_time": 0,
            "active": True,
            "color": self._get_vehicle_color(vehicle_type),
            "size": self._get_vehicle_size(vehicle_type),
            "current_road": first_road_id, # The road segment the vehicle is currently on
            "distance_on_road": 0.0 # Distance traveled along the current road segment
        }

        self.regular_vehicles.append(vehicle)
        # Add vehicle to the road's vehicle list
        self.roads[first_road_id]["vehicles"].append(vehicle["id"])
        return vehicle["id"]

    def add_emergency_vehicle(self, vehicle_type, start_junction_id, route):
        """
        Add an emergency vehicle to the simulation.

        Args:
            vehicle_type: Type of emergency vehicle (ambulance, police_car, fire_truck)
            start_junction_id: ID of the starting junction
            route: List of junction IDs defining the vehicle's route
        """
        if start_junction_id not in self.junctions:
            print(f"Error: Starting junction {start_junction_id} not found")
            return None

        # Check if route is valid and find the first road segment
        if not all(j_id in self.junctions for j_id in route):
            print("Error: Invalid route - some junctions don't exist")
            return None

        if len(route) < 2:
             print("Error: Route must contain at least two junctions.")
             return None

        # Find the first road segment for the vehicle
        first_road_id = self._find_road_id(route[0], route[1])
        if not first_road_id:
             print(f"Error: No road found between {route[0]} and {route[1]} for emergency vehicle route.")
             return None

        # Emergency vehicles are faster than regular ones
        speed_factors = {"ambulance": 1.3, "police_car": 1.2, "fire_truck": 1.1}
        speed_factor = speed_factors.get(vehicle_type, 1.2)

        vehicle = {
            "id": f"em_{len(self.emergency_vehicles)}",
            "type": vehicle_type,
            "position": list(self.junctions[start_junction_id]["position"]), # Use list for mutability
            "current_junction": start_junction_id, # The last junction the vehicle passed
            "next_junction_index": 1, # Index in the route of the next junction to reach
            "route": route,
            "speed": 0, # Start stationary
            "max_speed": 100 * speed_factor, # Pixels per second
            "acceleration": 30 * speed_factor, # Pixels per second^2
            "spawned_time": self.simulation_time,
            "response_time": 0, # Time to reach destination
            "priority_requested": False,
            "active": True,
            "current_road": first_road_id, # The road segment the vehicle is currently on
            "distance_on_road": 0.0 # Distance traveled along the current road segment
        }

        self.emergency_vehicles.append(vehicle)
        # Add vehicle to the road's vehicle list
        self.roads[first_road_id]["vehicles"].append(vehicle["id"])
        return vehicle["id"]

    def add_incident(self, incident_type, position=None, junction_id=None, severity="MEDIUM"):
        """
        Add an incident to the simulation.

        Args:
            incident_type: Type of incident (accident, roadblock, etc.)
            position: (x, y) coordinates or None if using junction_id
            junction_id: Junction ID where incident occurs, or None if using position
            severity: LOW, MEDIUM, HIGH impact on traffic
        """
        if position is None and junction_id is None:
            print("Error: Either position or junction_id must be provided")
            return None

        if junction_id and junction_id not in self.junctions:
            print(f"Error: Junction {junction_id} not found")
            return None

        # Calculate position if junction is provided
        if junction_id:
            position = self.junctions[junction_id]["position"]

        # Create incident
        incident = {
            "id": f"inc_{len(self.incidents)}",
            "type": incident_type,
            "position": position,
            "junction_id": junction_id,
            "severity": severity,
            "time": self.simulation_time,
            "duration": self._get_incident_duration(incident_type, severity),
            "affected_radius": self._get_incident_radius(severity),
            "resolved": False,
            "impacted_roads": [] # Roads affected by this incident
        }

        self.incidents.append(incident)

        # Determine affected roads and junctions
        for road_id, road in self.roads.items():
            # Check if the incident position is near the road
            start_pos = np.array(road["start_pos"])
            end_pos = np.array(road["end_pos"])
            inc_pos = np.array(position)

            # Calculate distance from incident to the road segment
            if road["length"] > 0:
                l2 = road["length"]**2
                t = max(0, min(1, np.dot(inc_pos - start_pos, end_pos - start_pos) / l2))
                closest_point_on_road = start_pos + t * (end_pos - start_pos)
                dist_to_road = np.linalg.norm(inc_pos - closest_point_on_road)
            else: # Handle zero length roads (shouldn't happen with junctions)
                 dist_to_road = np.linalg.norm(inc_pos - start_pos)


            if dist_to_road < incident["affected_radius"]:
                incident["impacted_roads"].append(road_id)
                print(f"Incident {incident['id']} impacts road {road_id}")

        # Update traffic controller about affected junctions/roads (if applicable)
        if self.traffic_controller:
            # In a real scenario, the controller would adjust signal timings, reroute traffic, etc.
            print(f"Notifying controller about incident {incident['id']}")


        return incident["id"]

    def set_simulation_mode(self, mode):
        """
        Set the global simulation mode.

        Args:
            mode: SimulationMode Enum value
        """
        if mode not in SimulationMode:
            print(f"Error: Invalid simulation mode {mode}")
            return

        self.mode = mode
        print(f"Simulation mode set to {mode.value}")

        # Update all junctions' operational mode
        for j_data in self.junctions.values():
            j_data["current_mode"] = mode

    def set_time_of_day(self, hour):
        """
        Set the simulated time of day.

        Args:
            hour: Hour in 24-hour format (0-23)
        """
        if 0 <= hour <= 23:
            self.time_of_day = hour
            print(f"Time of day set to {hour}:00")

            # Check if it's rush hour
            self.is_rush_hour = any(start <= hour < end for start, end in self.rush_hours)

            if self.is_rush_hour:
                print("Rush hour traffic conditions in effect")
            else:
                print("Normal traffic conditions in effect")

    def set_weather_conditions(self, condition):
        """
        Set weather conditions affecting traffic.

        Args:
            condition: String describing weather (clear, rain, snow, fog)
        """
        weather_impacts = {
            "clear": 0.0,
            "cloudy": 0.1,
            "rain": 0.3,
            "heavy_rain": 0.5,
            "snow": 0.6,
            "fog": 0.4,
            "storm": 0.7
        }

        self.weather_impact = weather_impacts.get(condition.lower(), 0.0)
        print(f"Weather set to {condition}, impact factor: {self.weather_impact}")

    def start(self):
        """Start the simulation."""
        if self.running:
            return

        self.running = True
        self.sim_thread = threading.Thread(target=self._simulation_loop)
        self.sim_thread.daemon = True # Allow thread to exit when main program exits
        self.sim_thread.start()
        print("Enhanced Traffic Simulator started")

    def stop(self):
        """Stop the simulation."""
        self.running = False
        if self.sim_thread and self.sim_thread.is_alive():
            self.sim_thread.join(timeout=2.0) # Give thread a moment to finish
            if self.sim_thread.is_alive():
                 print("Warning: Simulation thread did not terminate gracefully.")
        print("Enhanced Traffic Simulator stopped")

        # Final statistics
        self._print_simulation_stats()

    def _simulation_loop(self):
        """Main simulation loop."""
        last_time = time.time()
        frame_count = 0

        while self.running:
            current_time = time.time()
            dt = (current_time - last_time) * self.simulation_speed
            last_time = current_time

            # Update simulation time
            self.simulation_time += dt

            # Update all components
            self._update_incidents(dt)
            self._update_junctions(dt) # Updates junction traffic density data
            if self.traffic_controller:
                 # Update the controller based on the latest junction data
                 controller_traffic_data = {
                     j_id: j_data["traffic_density"] for j_id, j_data in self.junctions.items()
                 }
                 self.traffic_controller.update(dt, self.mode, controller_traffic_data)

            self._update_regular_vehicles(dt)
            self._update_emergency_vehicles(dt)
            self._update_traffic_density_map() # Update heatmap data

            # Generate new vehicles occasionally
            if frame_count % 50 == 0: # Adjust frequency as needed
                self._generate_new_traffic()

            # Update statistics
            if frame_count % 100 == 0: # Adjust frequency as needed
                self._update_statistics()

            # Render the simulation
            self._render_frame()

            # Show the frame
            cv2.imshow("Enhanced Traffic Simulation", self.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running = False # Set running flag to False to stop the loop
            elif key == ord('d'):
                # Toggle density map
                self.show_density_map = not self.show_density_map
            elif key == ord('s'):
                # Toggle statistics
                self.show_stats = not self.show_stats
            elif key == ord('m'):
                # Cycle through modes
                if self.mode == SimulationMode.STATIC:
                    self.set_simulation_mode(SimulationMode.DYNAMIC)
                elif self.mode == SimulationMode.DYNAMIC:
                    self.set_simulation_mode(SimulationMode.STATIC) # Can add EMERGENCY cycle if needed
            elif key == ord('+'):
                # Increase simulation speed
                self.simulation_speed = min(5.0, self.simulation_speed + 0.1) # Smaller increments
                print(f"Simulation speed: {self.simulation_speed:.1f}x")
            elif key == ord('-'):
                # Decrease simulation speed
                self.simulation_speed = max(0.1, self.simulation_speed - 0.1) # Smaller increments, min 0.1x
                print(f"Simulation speed: {self.simulation_speed:.1f}x")
            elif key == ord('r'):
                 # Toggle route visualization
                 self.show_routes = not self.show_routes


            frame_count += 1
            # Control frame rate - aim for roughly 30 FPS
            sleep_time = max(0.001, (1.0 / 30.0) - (time.time() - current_time))
            time.sleep(sleep_time)

        cv2.destroyAllWindows()

    def _update_junctions(self, dt):
        """
        Update traffic junctions.
        Calculates traffic density approaching each junction.
        Signal updates are handled by the traffic_controller.
        """
        for j_id, junction in self.junctions.items():
            junction_pos = np.array(junction["position"])

            # Reset traffic density counts for vehicles *approaching* the junction
            # These are vehicles on a road segment leading to this junction
            for direction in range(4):
                junction["traffic_density"][direction] = 0
                junction["waiting_vehicles"][direction] = []

            # Count vehicles on roads leading to this junction
            for road_id, road in self.roads.items():
                # Check if this road leads to the current junction
                is_approaching_from_start = (road["end_junction"] == j_id)
                is_approaching_from_end = (road["start_junction"] == j_id)

                if is_approaching_from_start or is_approaching_from_end:
                    # Determine the approach direction to the junction
                    if is_approaching_from_start:
                        # Vehicles on this road are moving from road["start_junction"] towards j_id
                        approach_pos = np.array(road["start_pos"])
                    else: # is_approaching_from_end
                        # Vehicles on this road are moving from road["end_junction"] towards j_id
                        approach_pos = np.array(road["end_pos"])

                    # Calculate the direction from the approach position to the junction center
                    # This gives the angle of the road segment relative to the junction
                    dx = junction_pos[0] - approach_pos[0]
                    dy = junction_pos[1] - approach_pos[1]
                    angle_to_junction = np.degrees(np.arctan2(dy, dx)) % 360

                    # Map angle to approach direction (0=North, 1=East, 2=South, 3=West)
                    # This needs to be the direction the *vehicle* is coming *from* relative to the junction center
                    # If the road goes from A to B, and B is the junction, vehicles on this road are approaching B from the direction of A.
                    # The angle calculated above is from A to B. The approach direction at B is the opposite of this angle.
                    approach_dir = int(((angle_to_junction + 180 + 45) % 360) // 90) % 4


                    # Count vehicles on this road segment that are close to the junction
                    for vehicle_id in road["vehicles"]:
                        vehicle = self._get_vehicle_by_id(vehicle_id)
                        if vehicle and vehicle["active"]:
                            # Check if this vehicle is heading towards this junction
                            if vehicle["next_junction_index"] < len(vehicle["route"]) and \
                               vehicle["route"][vehicle["next_junction_index"]] == j_id:

                                # Check if the vehicle is on the road segment leading to this junction
                                # and close to the junction
                                dist_to_junction = np.linalg.norm(np.array(vehicle["position"]) - junction_pos)
                                if dist_to_junction < 100: # Within a certain radius of the junction
                                    junction["traffic_density"][approach_dir] += 1
                                    # Add vehicle to waiting list if it's stopped near the junction
                                    if vehicle["speed"] < 5: # Arbitrary threshold for 'stopped'
                                         junction["waiting_vehicles"][approach_dir].append(vehicle_id)


    def _update_regular_vehicles(self, dt):
        """Update regular vehicles."""
        vehicles_to_remove = []

        for vehicle in self.regular_vehicles:
            if not vehicle["active"]:
                continue

            # Update total travel time
            vehicle["total_travel_time"] += dt

            # Get the road the vehicle is currently on
            current_road_id = vehicle["current_road"]
            if not current_road_id or current_road_id not in self.roads:
                print(f"Error: Vehicle {vehicle['id']} is not on a valid road: {current_road_id}")
                vehicle["active"] = False # Deactivate vehicle if not on a road
                vehicles_to_remove.append(vehicle)
                continue

            current_road = self.roads[current_road_id]

            # Determine the direction of travel on the road (start->end or end->start)
            travel_direction = 1 # Moving from start to end
            if vehicle["current_junction"] == current_road["end_junction"]:
                 # Vehicle is moving from the road's end junction towards its start junction
                 travel_direction = -1
                 # Need to adjust distance_on_road calculation if moving backwards
                 # For simplicity, let's assume distance_on_road is always from the start_pos
                 # This requires careful handling of position updates.

            # Get the next junction the vehicle is heading towards
            next_junction_index = vehicle["next_junction_index"]
            if next_junction_index >= len(vehicle["route"]):
                # Vehicle has reached its final destination
                vehicle["active"] = False
                vehicles_to_remove.append(vehicle)
                self.stats["total_vehicles_completed"] += 1
                # Remove vehicle from the road's vehicle list
                if vehicle["id"] in current_road["vehicles"]:
                    current_road["vehicles"].remove(vehicle["id"])
                continue

            next_junction_id = vehicle["route"][next_junction_index]
            next_junction_pos = np.array(self.junctions[next_junction_id]["position"])
            current_pos = np.array(vehicle["position"])

            # Check for incidents on the current road
            incident_factor = 0.0
            for incident in self.incidents:
                if not incident["resolved"] and current_road_id in incident["impacted_roads"]:
                    # Simple impact: reduce speed
                    incident_factor = max(incident_factor, 0.5) # Reduce speed by up to 50%

            # Calculate desired speed (influenced by max speed, weather, traffic, incidents)
            traffic_factor = self._get_traffic_factor(current_pos) # Local density impact
            max_safe_speed = vehicle["max_speed"] * (1 - self.weather_impact) * (1 - 0.5 * traffic_factor) * (1 - incident_factor)

            # Check traffic signal at the next junction if applicable
            at_junction = np.linalg.norm(current_pos - next_junction_pos) < 50 # Close to junction
            signal_state = SignalState.GREEN # Assume green if no controller or not at junction

            if at_junction and self.junctions[next_junction_id]["controller"]:
                 controller_junction = self.junctions[next_junction_id]["controller"]
                 # Determine which signal applies to this vehicle's approach
                 approach_dir = self._calculate_approach_direction(current_pos, next_junction_pos)
                 signal_state = controller_junction.signals[approach_dir].state

                 if signal_state == SignalState.RED:
                      # Stop at red light
                      vehicle["speed"] = 0
                      vehicle["waiting_time"] += dt
                      # Don't move
                      continue
                 elif signal_state == SignalState.YELLOW:
                      # Slow down at yellow light
                      max_safe_speed = min(max_safe_speed, vehicle["max_speed"] * 0.3) # Significantly reduce speed


            # Accelerate towards max safe speed if not stopped by a signal
            if vehicle["speed"] < max_safe_speed:
                vehicle["speed"] = min(max_safe_speed, vehicle["speed"] + vehicle["acceleration"] * dt)
            elif vehicle["speed"] > max_safe_speed:
                 vehicle["speed"] = max(max_safe_speed, vehicle["speed"] - vehicle["acceleration"] * dt) # Decelerate if needed

            # Calculate movement distance
            move_distance = vehicle["speed"] * dt

            # Update distance along the current road
            # Need to handle both directions of travel on the road
            if vehicle["current_junction"] == current_road["start_junction"]:
                 # Moving towards end_junction
                 vehicle["distance_on_road"] += move_distance
                 # Ensure distance doesn't exceed road length
                 vehicle["distance_on_road"] = min(vehicle["distance_on_road"], current_road["length"])
            else: # Moving towards start_junction
                 vehicle["distance_on_road"] -= move_distance
                 # Ensure distance doesn't go below 0
                 vehicle["distance_on_road"] = max(vehicle["distance_on_road"], 0.0)


            # Update vehicle position based on distance along the road
            # Calculate position as a point along the line segment
            road_vec = np.array(current_road["end_pos"]) - np.array(current_road["start_pos"])
            if current_road["length"] > 0:
                 normalized_road_vec = road_vec / current_road["length"]
                 # If moving start->end, position is start_pos + distance * normalized_vec
                 # If moving end->start, this is more complex if distance_on_road is from start_pos
                 # Let's stick to distance_on_road being distance from start_pos for simplicity
                 # If moving end->start, the actual distance from start_pos is length - distance_on_road
                 current_dist_from_start = vehicle["distance_on_road"]
                 if vehicle["current_junction"] == current_road["end_junction"]:
                      current_dist_from_start = current_road["length"] - vehicle["distance_on_road"]

                 vehicle["position"] = list(np.array(current_road["start_pos"]) + current_dist_from_start * normalized_road_vec)
            else:
                 # Road has zero length, vehicle stays at the start junction position
                 vehicle["position"] = list(current_road["start_pos"])


            # Check if vehicle has reached the next junction
            # This happens when distance_on_road is close to road length (if moving start->end)
            # or close to 0 (if moving end->start)
            reached_next_junction = False
            if vehicle["current_junction"] == current_road["start_junction"] and vehicle["distance_on_road"] >= current_road["length"] - 5: # Within 5 pixels of end
                 reached_next_junction = True
            elif vehicle["current_junction"] == current_road["end_junction"] and vehicle["distance_on_road"] <= 5: # Within 5 pixels of start
                 reached_next_junction = True


            if reached_next_junction:
                # Move to the next junction in the route
                vehicle["current_junction"] = next_junction_id
                vehicle["next_junction_index"] += 1
                vehicle["waiting_time"] = 0 # Reset waiting time
                vehicle["speed"] = 0 # Stop at junction initially

                # Find the next road segment if the route is not finished
                if vehicle["next_junction_index"] < len(vehicle["route"]):
                    next_next_junction_id = vehicle["route"][vehicle["next_junction_index"]]
                    next_road_id = self._find_road_id(vehicle["current_junction"], next_next_junction_id)

                    if next_road_id:
                         # Remove vehicle from old road's vehicle list
                         if current_road_id in self.roads and vehicle["id"] in self.roads[current_road_id]["vehicles"]:
                             self.roads[current_road_id]["vehicles"].remove(vehicle["id"])

                         vehicle["current_road"] = next_road_id
                         # Reset distance on the new road - set to 0 if moving start->end, or length if moving end->start
                         next_road = self.roads[next_road_id]
                         if vehicle["current_junction"] == next_road["start_junction"]:
                              vehicle["distance_on_road"] = 0.0
                         else: # Vehicle is entering the new road from its end
                              vehicle["distance_on_road"] = next_road["length"]

                         # Add vehicle to the new road's vehicle list
                         self.roads[next_road_id]["vehicles"].append(vehicle["id"])
                    else:
                        print(f"Warning: No road found between {vehicle['current_junction']} and {next_next_junction_id}. Vehicle {vehicle['id']} stopping.")
                        vehicle["active"] = False # Stop vehicle if no path
                        vehicles_to_remove.append(vehicle)
                        # Remove vehicle from the road's vehicle list
                        if current_road_id in self.roads and vehicle["id"] in self.roads[current_road_id]["vehicles"]:
                            self.roads[current_road_id]["vehicles"].remove(vehicle["id"])


        # Remove completed/inactive vehicles
        for vehicle in vehicles_to_remove:
            if vehicle in self.regular_vehicles:
                self.regular_vehicles.remove(vehicle)


    def _update_emergency_vehicles(self, dt):
        """Update emergency vehicles and handle priority requests."""
        vehicles_to_remove = []

        for vehicle in self.emergency_vehicles:
            if not vehicle["active"]:
                continue

            # Update response time
            vehicle["response_time"] += dt

            # Get the road the vehicle is currently on
            current_road_id = vehicle["current_road"]
            if not current_road_id or current_road_id not in self.roads:
                print(f"Error: Emergency vehicle {vehicle['id']} is not on a valid road: {current_road_id}")
                vehicle["active"] = False # Deactivate vehicle if not on a road
                vehicles_to_remove.append(vehicle)
                continue

            current_road = self.roads[current_road_id]

            # Get the next junction the vehicle is heading towards
            next_junction_index = vehicle["next_junction_index"]
            if next_junction_index >= len(vehicle["route"]):
                # Vehicle has reached its final destination
                vehicle["active"] = False
                vehicles_to_remove.append(vehicle)
                self.stats["emergency_response_times"].append(vehicle["response_time"])
                 # Remove vehicle from the road's vehicle list
                if vehicle["id"] in current_road["vehicles"]:
                    current_road["vehicles"].remove(vehicle["id"])
                # Reset emergency priority at the last junction it passed
                last_junction_id = vehicle["route"][next_junction_index - 1]
                if last_junction_id in self.junctions and self.junctions[last_junction_id]["controller"]:
                    self.junctions[last_junction_id]["controller"].reset_emergency_priority(vehicle["id"])

                continue

            next_junction_id = vehicle["route"][next_junction_index]
            next_junction_pos = np.array(self.junctions[next_junction_id]["position"])
            current_pos = np.array(vehicle["position"])

            # Request priority if approaching a junction and not already requested
            dist_to_next_junction = np.linalg.norm(current_pos - next_junction_pos)
            if dist_to_next_junction < 200 and not vehicle["priority_requested"] and self.junctions[next_junction_id]["controller"]: # Within 200 pixels
                # Calculate approach direction to the next junction
                approach_dir = self._calculate_approach_direction(current_pos, next_junction_pos)

                # Calculate target direction (the direction the vehicle will leave the junction)
                target_dir = None
                if vehicle["next_junction_index"] + 1 < len(vehicle["route"]):
                    next_next_junction_id = vehicle["route"][vehicle["next_junction_index"] + 1]
                    next_next_junction_pos = np.array(self.junctions[next_next_junction_id]["position"])
                    target_dir = self._calculate_approach_direction(next_junction_pos, next_next_junction_pos)
                else:
                    # If this is the last junction, the target direction doesn't matter for priority
                    pass # Or set a default/invalid target_dir

                # Request priority from the controller for this junction
                controller_junction = self.junctions[next_junction_id]["controller"]
                if controller_junction:
                     controller_junction.set_emergency_priority(approach_dir, vehicle["id"]) # Pass vehicle ID
                     vehicle["priority_requested"] = True


            # Emergency vehicles try to maintain max speed, only slowing for obstacles
            max_desired_speed = vehicle["max_speed"]

            # Check for vehicles directly in front on the same road segment
            # This is a simplified collision avoidance/following
            vehicle_in_front = None
            min_dist_ahead = float('inf')
            lookahead_distance = 100 # Pixels to look ahead

            # Determine the vehicle's position along the road from start_pos
            current_dist_from_start = vehicle["distance_on_road"]
            if vehicle["current_junction"] == current_road["end_junction"]:
                 current_dist_from_start = current_road["length"] - vehicle["distance_on_road"]

            for other_vehicle_id in current_road["vehicles"]:
                if other_vehicle_id != vehicle["id"]:
                    other_vehicle = self._get_vehicle_by_id(other_vehicle_id)
                    if other_vehicle and other_vehicle["active"]:
                        # Determine the other vehicle's position along the road from start_pos
                        other_dist_from_start = other_vehicle["distance_on_road"]
                        if other_vehicle["current_junction"] == current_road["end_junction"]:
                             other_dist_from_start = current_road["length"] - other_vehicle["distance_on_road"]

                        # Check if the other vehicle is ahead and on the same "lane" (simplified)
                        # Assuming vehicles on the same road segment are effectively in one line for this check
                        if (vehicle["current_junction"] == current_road["start_junction"] and other_dist_from_start > current_dist_from_start) or \
                           (vehicle["current_junction"] == current_road["end_junction"] and other_dist_from_start < current_dist_from_start):

                            dist_ahead = abs(other_dist_from_start - current_dist_from_start)
                            if dist_ahead < min_dist_ahead:
                                min_dist_ahead = dist_ahead
                                vehicle_in_front = other_vehicle

            if vehicle_in_front and min_dist_ahead < lookahead_distance:
                 # Slow down to match vehicle in front or stop if too close
                 max_desired_speed = min(max_desired_speed, vehicle_in_front["speed"])
                 if min_dist_ahead < 20: # Dangerously close
                      max_desired_speed = 0


            # Adjust speed based on desired speed
            if vehicle["speed"] < max_desired_speed:
                vehicle["speed"] = min(max_desired_speed, vehicle["speed"] + vehicle["acceleration"] * dt)
            elif vehicle["speed"] > max_desired_speed:
                 vehicle["speed"] = max(0, vehicle["speed"] - vehicle["acceleration"] * dt) # Don't go below 0


            # Calculate movement distance
            move_distance = vehicle["speed"] * dt

             # Update distance along the current road (same logic as regular vehicles)
            if vehicle["current_junction"] == current_road["start_junction"]:
                 vehicle["distance_on_road"] += move_distance
                 vehicle["distance_on_road"] = min(vehicle["distance_on_road"], current_road["length"])
            else: # Moving towards start_junction
                 vehicle["distance_on_road"] -= move_distance
                 vehicle["distance_on_road"] = max(vehicle["distance_on_road"], 0.0)

            # Update vehicle position based on distance along the road (same logic as regular vehicles)
            road_vec = np.array(current_road["end_pos"]) - np.array(current_road["start_pos"])
            if current_road["length"] > 0:
                 normalized_road_vec = road_vec / current_road["length"]
                 current_dist_from_start = vehicle["distance_on_road"]
                 if vehicle["current_junction"] == current_road["end_junction"]:
                      current_dist_from_start = current_road["length"] - vehicle["distance_on_road"]

                 vehicle["position"] = list(np.array(current_road["start_pos"]) + current_dist_from_start * normalized_road_vec)
            else:
                 vehicle["position"] = list(current_road["start_pos"])


            # Check if vehicle has reached the next junction (same logic as regular vehicles)
            reached_next_junction = False
            if vehicle["current_junction"] == current_road["start_junction"] and vehicle["distance_on_road"] >= current_road["length"] - 5:
                 reached_next_junction = True
            elif vehicle["current_junction"] == current_road["end_junction"] and vehicle["distance_on_road"] <= 5:
                 reached_next_junction = True


            if reached_next_junction:
                # Move to the next junction in the route
                last_junction_id = vehicle["current_junction"] # Store current junction before updating
                vehicle["current_junction"] = next_junction_id
                vehicle["next_junction_index"] += 1
                vehicle["speed"] = 0 # Stop at junction briefly

                # Reset priority request flag once the junction is reached
                vehicle["priority_requested"] = False

                # Reset emergency priority at the junction it just passed
                if last_junction_id in self.junctions and self.junctions[last_junction_id]["controller"]:
                    self.junctions[last_junction_id]["controller"].reset_emergency_priority(vehicle["id"])


                # Find the next road segment if the route is not finished
                if vehicle["next_junction_index"] < len(vehicle["route"]):
                    next_next_junction_id = vehicle["route"][vehicle["next_junction_index"]]
                    next_road_id = self._find_road_id(vehicle["current_junction"], next_next_junction_id)

                    if next_road_id:
                         # Remove vehicle from old road's vehicle list
                         if current_road_id in self.roads and vehicle["id"] in self.roads[current_road_id]["vehicles"]:
                             self.roads[current_road_id]["vehicles"].remove(vehicle["id"])

                         vehicle["current_road"] = next_road_id
                         # Reset distance on the new road
                         next_road = self.roads[next_road_id]
                         if vehicle["current_junction"] == next_road["start_junction"]:
                              vehicle["distance_on_road"] = 0.0
                         else:
                              vehicle["distance_on_road"] = next_road["length"]

                         # Add vehicle to the new road's vehicle list
                         self.roads[next_road_id]["vehicles"].append(vehicle["id"])
                    else:
                        print(f"Warning: No road found between {vehicle['current_junction']} and {next_next_junction_id} for emergency vehicle {vehicle['id']}. Stopping.")
                        vehicle["active"] = False # Stop vehicle if no path
                        vehicles_to_remove.append(vehicle)
                         # Remove vehicle from the road's vehicle list
                        if current_road_id in self.roads and vehicle["id"] in self.roads[current_road_id]["vehicles"]:
                            self.roads[current_road_id]["vehicles"].remove(vehicle["id"])


        # Remove completed/inactive vehicles
        for vehicle in vehicles_to_remove:
            if vehicle in self.emergency_vehicles:
                self.emergency_vehicles.remove(vehicle)


    def _update_incidents(self, dt):
        """Update incidents and check for resolution."""
        for incident in self.incidents:
            if incident["resolved"]:
                continue

            # Check if incident duration has elapsed
            if self.simulation_time - incident["time"] > incident["duration"]:
                incident["resolved"] = True
                self.stats["incidents_resolved"] += 1
                print(f"Incident {incident['id']} resolved after {incident['duration']:.1f} seconds")

                # Notify affected roads/junctions to return to normal operation
                # In a real scenario, this would involve the controller
                print(f"Incident {incident['id']} resolved. Affected roads: {incident['impacted_roads']}")


    def _update_traffic_density_map(self):
        """Update the traffic density heatmap based on vehicle positions."""
        # Reset density map
        self.traffic_density_map.fill(0)

        # Add regular vehicles to density map
        for vehicle in self.regular_vehicles:
            if not vehicle["active"]:
                continue

            x, y = int(vehicle["position"][0]), int(vehicle["position"][1])

            # Make sure coordinates are within bounds
            if 0 <= x < self.width and 0 <= y < self.height:
                # Add density value with Gaussian distribution
                radius = 20  # Influence radius
                intensity = 0.7

                # Apply Gaussian density around vehicle position
                y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
                mask = x_indices**2 + y_indices**2 <= radius**2

                # Get boundaries for the area to update
                x_min = max(0, x - radius)
                x_max = min(self.width - 1, x + radius)
                y_min = max(0, y - radius)
                y_max = min(self.height - 1, y + radius)

                # Calculate local mask indices
                x_local_min = max(0, radius - x)
                x_local_max = min(2*radius + 1, self.width - x + radius)
                y_local_min = max(0, radius - y)
                y_local_max = min(2*radius + 1, self.height - y + radius)

                local_mask = mask[y_local_min:y_local_max, x_local_min:x_local_max]

                # Update density map in the valid region
                self.traffic_density_map[y_min:y_max+1, x_min:x_max+1][local_mask[:y_max-y_min+1, :x_max-x_min+1]] += intensity

        # Also add emergency vehicles with higher intensity
        for vehicle in self.emergency_vehicles:
            if not vehicle["active"]:
                continue

            x, y = int(vehicle["position"][0]), int(vehicle["position"][1])

            if 0 <= x < self.width and 0 <= y < self.height:
                # Add with higher intensity
                radius = 30  # Larger influence radius
                intensity = 1.0  # Higher intensity

                # Apply Gaussian density around vehicle position (similar to above)
                y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
                mask = x_indices**2 + y_indices**2 <= radius**2

                # Get boundaries for the area to update
                x_min = max(0, x - radius)
                x_max = min(self.width - 1, x + radius)
                y_min = max(0, y - radius)
                y_max = min(self.height - 1, y + radius)

                # Calculate local mask indices
                x_local_min = max(0, radius - x)
                x_local_max = min(2*radius + 1, self.width - x + radius)
                y_local_min = max(0, radius - y)
                y_local_max = min(2*radius + 1, self.height - y + radius)

                local_mask = mask[y_local_min:y_local_max, x_local_min:x_max-x_min+y_local_max-y_local_min] # Corrected indexing

                # Update density map in the valid region
                # Ensure slicing matches mask and target array dimensions
                target_slice = self.traffic_density_map[y_min:y_max+1, x_min:x_max+1]
                mask_slice = local_mask[:target_slice.shape[0], :target_slice.shape[1]]
                target_slice[mask_slice] += intensity


    def _get_traffic_factor(self, position):
        """
        Get traffic congestion factor at a specific position from the density map.

        Args:
            position: (x, y) position

        Returns:
            Congestion factor between 0-1, where 1 is completely congested
        """
        x, y = int(position[0]), int(position[1])

        if 0 <= x < self.width and 0 <= y < self.height:
            # Normalize density map value to a 0-1 factor
            # Max density could be higher than 1, so clamp and scale
            max_possible_density = 10 # Arbitrary max density for scaling
            return min(1.0, self.traffic_density_map[y, x] / max_possible_density)
        return 0.0

    def _find_road_id(self, start_junction_id, end_junction_id):
        """
        Find the ID of the road connecting two junctions.
        Assumes only one road connects any two junctions directly.

        Args:
            start_junction_id: ID of the starting junction
            end_junction_id: ID of the ending junction

        Returns:
            Road ID if found, otherwise None.
        """
        for road_id, road in self.roads.items():
            if (road["start_junction"] == start_junction_id and road["end_junction"] == end_junction_id) or \
               (road["end_junction"] == start_junction_id and road["start_junction"] == end_junction_id):
                return road_id
        return None

    def _generate_new_traffic(self):
        """Generate new vehicles based on time of day and traffic patterns."""
        # Skip if there are too many vehicles
        if len(self.regular_vehicles) > 150: # Increased limit slightly
            return

        # Probability based on time of day and weather
        spawn_probability = 0.2 # Base probability
        if self.is_rush_hour:
            spawn_probability *= 2.0 # Double during rush hour
        spawn_probability *= (1 - self.weather_impact * 0.7) # Reduce with bad weather

        # Decide whether to spawn a new vehicle
        if np.random.random() > spawn_probability:
            return

        # Pick random start and end junctions
        if len(self.junctions) < 2:
            return

        junction_ids = list(self.junctions.keys())

        # Try a few times to find a valid route
        for _ in range(5): # Attempt up to 5 times
            start_id = np.random.choice(junction_ids)
            end_id = np.random.choice(junction_ids)

            if start_id == end_id:
                continue # Need different start and end

            route = self._generate_route(start_id, end_id)

            if route and len(route) >= 2:
                # Found a valid route, now pick vehicle type and add
                vehicle_types = ["car", "car", "car", "bus", "truck"]
                vehicle_type = np.random.choice(vehicle_types)
                self.add_regular_vehicle(start_id, route, vehicle_type)
                return # Added a vehicle, exit function

        # If loop finishes without adding a vehicle, no valid route was found after attempts
        # print("Could not generate a valid route for new vehicle.")


    def _generate_route(self, start_id, end_id):
        """
        Generate a route from start to end junction using Breadth-First Search (BFS).

        Args:
            start_id: Starting junction ID
            end_id: Ending junction ID

        Returns:
            List of junction IDs forming a route, or None if no route found.
        """
        if start_id not in self.junctions or end_id not in self.junctions:
            return None

        q = queue.Queue()
        q.put((start_id, [start_id])) # Store (current_junction, path_so_far)

        visited = set([start_id])

        while not q.empty():
            (current_junction_id, path) = q.get()

            # If destination reached, return the path
            if current_junction_id == end_id:
                return path

            # Find neighbors (connected junctions via roads)
            neighbors = []
            for road_id, road in self.roads.items():
                if road["start_junction"] == current_junction_id:
                    neighbors.append(road["end_junction"])
                elif road["end_junction"] == current_junction_id:
                    neighbors.append(road["start_junction"])

            # Add unvisited neighbors to the queue
            for neighbor_id in neighbors:
                if neighbor_id not in visited:
                    visited.add(neighbor_id)
                    q.put((neighbor_id, path + [neighbor_id]))

        # If queue is empty and destination not reached, no path exists
        return None

    def _update_statistics(self):
        """Update simulation statistics."""
        # Update average wait times per junction
        for j_id in self.junctions:
            total_wait = 0
            waiting_count = 0
            # Count vehicles currently waiting at this junction
            for direction in range(4):
                 for vehicle_id in self.junctions[j_id]["waiting_vehicles"][direction]:
                      vehicle = self._get_vehicle_by_id(vehicle_id)
                      if vehicle and vehicle["active"]:
                           total_wait += vehicle["waiting_time"]
                           waiting_count += 1

            if waiting_count > 0:
                self.stats["average_wait_times"][j_id] = total_wait / waiting_count
            else:
                self.stats["average_wait_times"][j_id] = 0

        # Calculate congestion for each road
        for road_id, road in self.roads.items():
            vehicle_count_on_road = len(road["vehicles"])

            # Update congestion level (0-1 scale)
            # A simple model: congestion increases with number of vehicles relative to lanes/length
            # Could be more sophisticated (e.g., based on average speed on the road)
            max_capacity_factor = road["num_lanes"] * road["length"] / 100 # Arbitrary scaling
            if max_capacity_factor > 0:
                 road["congestion_level"] = min(1.0, vehicle_count_on_road / max_capacity_factor)
            else:
                 road["congestion_level"] = 0.0

            self.stats["congestion_levels"][road_id] = road["congestion_level"]


    def _render_frame(self):
        """Render the simulation frame using OpenCV."""
        # Clear frame
        self.frame.fill(0) # Black background

        # Draw density map if enabled
        if self.show_density_map:
            # Normalize density map to 0-255
            normalized_map = np.minimum(self.traffic_density_map * 255, 255).astype(np.uint8)
            # Create a heatmap (blue -> green -> red)
            heatmap = cv2.applyColorMap(normalized_map, cv2.COLORMAP_JET)

            # Blend heatmap with the background
            alpha = 0.5 # Transparency
            self.frame = cv2.addWeighted(heatmap, alpha, self.frame, 1 - alpha, 0)

        # Draw roads
        for road_id, road in self.roads.items():
            # Determine color based on congestion
            congestion = road["congestion_level"]
            # Interpolate between green (0, 255, 0) and red (0, 0, 255) based on congestion
            color = (
                int(255 * congestion),      # Blue component (increases with congestion)
                int(255 * (1 - congestion)), # Green component (decreases with congestion)
                0                           # Red component (constant 0 for BGR format)
            )

            # Draw road with variable width based on number of lanes
            width = max(2, road["num_lanes"] * 3) # Minimum width 2, scales with lanes
            cv2.line(
                self.frame,
                (int(road["start_pos"][0]), int(road["start_pos"][1])),
                (int(road["end_pos"][0]), int(road["end_pos"][1])),
                color,
                width
            )

        # Draw junctions
        for j_id, junction in self.junctions.items():
            # Determine color based on mode
            if junction["current_mode"] == SimulationMode.STATIC:
                color = (128, 128, 128)  # Gray
            elif junction["current_mode"] == SimulationMode.DYNAMIC:
                color = (0, 255, 0)  # Green
            else:  # Emergency
                color = (0, 0, 255)  # Red

            # Draw junction as a circle
            cv2.circle(
                self.frame,
                (int(junction["position"][0]), int(junction["position"][1])),
                15, # Slightly larger radius
                color,
                -1  # Filled
            )

            # Draw junction ID text
            cv2.putText(self.frame, j_id, (int(junction["position"][0]) + 10, int(junction["position"][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


            # Draw traffic signals if controller is available
            if junction["controller"]:
                for i, signal in enumerate(junction["controller"].signals):
                    # Calculate signal position (N, E, S, W relative to junction)
                    # Positions are offset from the junction center
                    offset = 25 # Distance from junction center
                    angles_deg = [270, 0, 90, 180] # Angles for N, E, S, W approaches
                    angle_rad = np.radians(angles_deg[i])
                    signal_x = int(junction["position"][0] + offset * np.cos(angle_rad))
                    signal_y = int(junction["position"][1] + offset * np.sin(angle_rad))

                    # Determine signal color
                    if signal.state == SignalState.RED:
                        signal_color = (0, 0, 255)  # Red (BGR)
                    elif signal.state == SignalState.YELLOW:
                        signal_color = (0, 255, 255)  # Yellow (BGR)
                    else:  # GREEN
                        signal_color = (0, 255, 0)  # Green (BGR)

                    # Draw signal as a small circle
                    cv2.circle(
                        self.frame,
                        (signal_x, signal_y),
                        5, # Signal size
                        signal_color,
                        -1  # Filled
                    )

        # Draw regular vehicles
        for vehicle in self.regular_vehicles:
            if not vehicle["active"]:
                continue

            # Draw vehicle as a circle
            cv2.circle(
                self.frame,
                (int(vehicle["position"][0]), int(vehicle["position"][1])),
                vehicle.get("size", 5),
                vehicle.get("color", (255, 255, 255)), # Default white
                -1  # Filled
            )

            # Optionally draw vehicle route
            if self.show_routes and len(vehicle["route"]) > vehicle["next_junction_index"]:
                 route_color = (255, 255, 0) # Cyan
                 current_pos = vehicle["position"]
                 for i in range(vehicle["next_junction_index"], len(vehicle["route"])):
                      next_j_id = vehicle["route"][i]
                      next_j_pos = self.junctions[next_j_id]["position"]
                      cv2.line(self.frame, (int(current_pos[0]), int(current_pos[1])),
                               (int(next_j_pos[0]), int(next_j_pos[1])), route_color, 1)
                      current_pos = next_j_pos


        # Draw emergency vehicles with blinking effect
        for vehicle in self.emergency_vehicles:
            if not vehicle["active"]:
                continue

            # Blinking effect (alternate colors)
            blink_interval = 0.5 # Seconds per blink state
            blink_state = int(self.simulation_time / blink_interval) % 2

            # Alternate between red and blue
            color = (0, 0, 255) if blink_state == 0 else (255, 0, 0) # Red/Blue (BGR)

            # Draw vehicle as a larger circle
            cv2.circle(
                self.frame,
                (int(vehicle["position"][0]), int(vehicle["position"][1])),
                10,  # Larger size
                color,
                -1  # Filled
            )

             # Optionally draw emergency vehicle route
            if self.show_routes and len(vehicle["route"]) > vehicle["next_junction_index"]:
                 route_color = (255, 0, 255) # Magenta
                 current_pos = vehicle["position"]
                 for i in range(vehicle["next_junction_index"], len(vehicle["route"])):
                      next_j_id = vehicle["route"][i]
                      next_j_pos = self.junctions[next_j_id]["position"]
                      cv2.line(self.frame, (int(current_pos[0]), int(current_pos[1])),
                               (int(next_j_pos[0]), int(next_j_pos[1])), route_color, 1)
                      current_pos = next_j_pos


        # Draw incidents
        for incident in self.incidents:
            if incident["resolved"]:
                continue

            # Draw incident location
            severity_colors = {
                "LOW": (0, 255, 255),     # Yellow
                "MEDIUM": (0, 165, 255),   # Orange
                "HIGH": (0, 0, 255),       # Red
                "CRITICAL": (0, 0, 128)    # Dark red
            }

            color = severity_colors.get(incident["severity"], (0, 0, 255)) # Default red

            # Draw incident marker (e.g., a cross or square)
            pos = (int(incident["position"][0]), int(incident["position"][1]))
            marker_size = 10
            cv2.drawMarker(self.frame, pos, color, cv2.MARKER_CROSS, marker_size, 2)


            # Draw affected radius (optional, can be visually noisy)
            # cv2.circle(
            #     self.frame,
            #     pos,
            #     int(incident["affected_radius"]),
            #     color,
            #     1  # Thin outline
            # )

            # Draw incident ID text
            cv2.putText(self.frame, incident["id"], (pos[0] + 15, pos[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


        # Draw statistics if enabled
        if self.show_stats:
            # Count active vehicles and incidents
            active_regular = sum(1 for v in self.regular_vehicles if v["active"])
            active_emergency = sum(1 for v in self.emergency_vehicles if v["active"])
            active_incidents = sum(1 for i in self.incidents if not i["resolved"])

            # Calculate average wait time across all junctions
            total_avg_wait = sum(self.stats["average_wait_times"].values())
            num_junctions_with_wait = sum(1 for wait_time in self.stats["average_wait_times"].values() if wait_time > 0)
            overall_avg_wait = total_avg_wait / num_junctions_with_wait if num_junctions_with_wait > 0 else 0

            # Calculate average response time for completed emergency vehicles
            avg_response = 0
            if self.stats["emergency_response_times"]:
                avg_response = sum(self.stats["emergency_response_times"]) / len(self.stats["emergency_response_times"])

            # Calculate overall average road congestion
            avg_road_congestion = sum(self.stats["congestion_levels"].values()) / len(self.stats["congestion_levels"]) if self.stats["congestion_levels"] else 0


            # Draw text statistics
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1
            text_color = (255, 255, 255) # White
            line_height = 20
            x_pos = 10
            y_pos = 30

            cv2.putText(self.frame, f"Sim Time: {self.simulation_time:.1f}s", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Mode: {self.mode.value}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Time of Day: {self.time_of_day:02d}:00 ({'Rush Hour' if self.is_rush_hour else 'Normal'})", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Weather Impact: {self.weather_impact:.1f}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Speed: {self.simulation_speed:.1f}x", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height * 2 # Add extra space

            cv2.putText(self.frame, f"Regular Vehicles: {active_regular}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Emergency Vehicles: {active_emergency}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Active Incidents: {active_incidents}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Incidents Resolved: {self.stats['incidents_resolved']}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Vehicles Completed: {self.stats['total_vehicles_completed']}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height * 2 # Add extra space

            cv2.putText(self.frame, f"Overall Avg Wait: {overall_avg_wait:.1f}s", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Avg Emergency Response: {avg_response:.1f}s", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height

            cv2.putText(self.frame, f"Overall Avg Congestion: {avg_road_congestion:.2f}", (x_pos, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height


            # Controls help text
            y_pos = self.height - 100
            x_pos_controls = self.width - 250 # Position controls on the right
            cv2.putText(self.frame, "Controls:", (x_pos_controls, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height
            cv2.putText(self.frame, "Q: Quit", (x_pos_controls, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height
            cv2.putText(self.frame, "D: Toggle density map", (x_pos_controls, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height
            cv2.putText(self.frame, "S: Toggle stats", (x_pos_controls, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height
            cv2.putText(self.frame, "M: Toggle mode (Static/Dynamic)", (x_pos_controls, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height
            cv2.putText(self.frame, "+/-: Adjust Speed", (x_pos_controls, y_pos), font, font_scale, text_color, font_thickness)
            y_pos += line_height
            cv2.putText(self.frame, "R: Toggle Routes (Experimental)", (x_pos_controls, y_pos), font, font_scale, text_color, font_thickness)


    def _print_simulation_stats(self):
        """Print final simulation statistics to console."""
        print("\n=== Final Simulation Statistics ===")
        print(f"Total simulation time: {self.simulation_time:.1f} seconds")
        print(f"Total regular vehicles simulated: {len(self.regular_vehicles)}")
        print(f"Total emergency vehicles simulated: {len(self.emergency_vehicles)}")
        print(f"Total incidents created: {len(self.incidents)}")
        print(f"Incidents resolved: {self.stats['incidents_resolved']}")
        print(f"Total vehicles completed route: {self.stats['total_vehicles_completed']}")


        # Calculate average response time for completed emergency vehicles
        avg_response = 0
        if self.stats["emergency_response_times"]:
            avg_response = sum(self.stats["emergency_response_times"]) / len(self.stats["emergency_response_times"])
            print(f"Average emergency response time for completed routes: {avg_response:.1f} seconds")
        else:
             print("No emergency vehicles completed their routes.")

        # Calculate overall average congestion
        avg_road_congestion = sum(self.stats["congestion_levels"].values()) / len(self.stats["congestion_levels"]) if self.stats["congestion_levels"] else 0
        print(f"Overall average road congestion level: {avg_road_congestion:.2f}")

        # Report average wait times per junction
        print("Average wait times per junction:")
        for j_id, wait_time in self.stats["average_wait_times"].items():
             print(f"  {j_id}: {wait_time:.1f} seconds")

        # Report congestion levels per road
        print("Congestion levels per road:")
        for road_id, congestion in self.stats["congestion_levels"].items():
             print(f"  {road_id}: {congestion:.2f}")


        print("===================================")

    def _get_vehicle_by_id(self, vehicle_id):
        """Helper to find a vehicle by its ID."""
        for vehicle_list in [self.regular_vehicles, self.emergency_vehicles]:
            for vehicle in vehicle_list:
                if vehicle["id"] == vehicle_id:
                    return vehicle
        return None

    def _calculate_approach_direction(self, from_pos, to_pos):
        """
        Calculate the approach direction at 'to_pos' from 'from_pos' (0=North, 1=East, 2=South, 3=West).

        Args:
            from_pos: Starting position (x, y)
            to_pos: Target position (x, y)

        Returns:
            Direction index (0-3)
        """
        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]
        # Angle from 'from_pos' to 'to_pos'
        angle_rad = np.arctan2(dy, dx)
        angle_deg = np.degrees(angle_rad) % 360

        # The approach direction at 'to_pos' is the direction *from* 'from_pos'.
        # So, if the angle from from_pos to to_pos is 0 degrees (East), the approach at to_pos is from the West (180 degrees).
        # We need to map the angle of the incoming road segment to the approach direction at the junction.
        # Angle 0-90 (Quadrant 1) -> Approach from SW (dir 3)
        # Angle 90-180 (Quadrant 2) -> Approach from NW (dir 0)
        # Angle 180-270 (Quadrant 3) -> Approach from NE (dir 1)
        # Angle 270-360 (Quadrant 4) -> Approach from SE (dir 2)

        # A simpler way: The angle from the junction center to the vehicle's position gives the direction *to* the vehicle.
        # The approach direction is the opposite of this.
        # Calculate angle from to_pos (junction center) to from_pos (vehicle position)
        dx_rev = from_pos[0] - to_pos[0]
        dy_rev = from_pos[1] - to_pos[1]
        angle_rev_rad = np.arctan2(dy_rev, dx_rev)
        angle_rev_deg = np.degrees(angle_rev_rad) % 360

        # Map angle from junction to vehicle to approach direction
        # Angle 0 (East) -> Approach from East (dir 1)
        # Angle 90 (South) -> Approach from South (dir 2)
        # Angle 180 (West) -> Approach from West (dir 3)
        # Angle 270 (North) -> Approach from North (dir 0)
        # This mapping seems more direct.
        # We can use a threshold around the cardinal directions
        if 45 <= angle_rev_deg < 135:
            return 2 # South approach
        elif 135 <= angle_rev_deg < 225:
            return 3 # West approach
        elif 225 <= angle_rev_deg < 315:
            return 0 # North approach
        else: # 315 <= angle_rev_deg < 360 or 0 <= angle_rev_deg < 45
            return 1 # East approach


    def _get_vehicle_color(self, vehicle_type):
        """Get color based on vehicle type (BGR format for OpenCV)."""
        colors = {
            "car": (255, 0, 0),       # Blue
            "bus": (0, 200, 0),       # Green
            "truck": (128, 128, 0),   # Teal (approx)
            "ambulance": (0, 0, 255), # Red
            "police_car": (255, 0, 0),# Blue
            "fire_truck": (0, 0, 255) # Red
        }
        return colors.get(vehicle_type, (128, 128, 128))  # Default gray

    def _get_vehicle_size(self, vehicle_type):
        """Get size based on vehicle type."""
        sizes = {
            "car": 6,
            "bus": 10,
            "truck": 12,
            "ambulance": 8,
            "police_car": 8,
            "fire_truck": 12
        }
        return sizes.get(vehicle_type, 6)  # Default size

    def _get_incident_duration(self, incident_type, severity):
        """Get incident duration based on type and severity (in seconds)."""
        base_durations = {
            "accident": 300,    # 5 minutes
            "roadblock": 180,   # 3 minutes
            "construction": 600 # 10 minutes
        }

        severity_factors = {
            "LOW": 0.5,
            "MEDIUM": 1.0,
            "HIGH": 2.0,
            "CRITICAL": 3.0
        }

        base = base_durations.get(incident_type, 300) # Default 5 min
        factor = severity_factors.get(severity, 1.0) # Default 1.0

        return base * factor

    def _get_incident_radius(self, severity):
        """Get incident radius of effect based on severity (in pixels)."""
        radius = {
            "LOW": 50,
            "MEDIUM": 100,
            "HIGH": 200,
            "CRITICAL": 300
        }
        return radius.get(severity, 100) # Default 100 pixels


def main():
    """Main function to run the simulation."""
    # Create the simulator
    simulator = TrafficSimulator(width=1200, height=800)

    # Create a mock controller for the simulation
    controller = MockController()

    # Define junction positions
    junctions_data = {
        "J1": (200, 200),
        "J2": (500, 200),
        "J3": (800, 200),
        "J4": (200, 500),
        "J5": (500, 500),
        "J6": (800, 500),
        "J7": (500, 80), # Added a junction
        "J8": (500, 720) # Added a junction
    }

    # Add junctions to controller and simulator
    for j_id, pos in junctions_data.items():
        controller.add_junction(j_id) # Add to controller first
        simulator.add_junction(j_id, pos, num_lanes=2) # Add to simulator

    # Link the controller to the simulator
    simulator.set_traffic_controller(controller)

    # Add roads between junctions
    roads_data = [
        ("R1", "J1", "J2"),
        ("R2", "J2", "J3"),
        ("R3", "J4", "J5"),
        ("R4", "J5", "J6"),
        ("R5", "J1", "J4"),
        ("R6", "J2", "J5"),
        ("R7", "J3", "J6"),
        ("R8", "J2", "J7"), # New road
        ("R9", "J5", "J8"), # New road
        ("R10", "J7", "J8"), # New road
        ("R11", "J1", "J7"), # New road
        ("R12", "J3", "J7"), # New road
        ("R13", "J4", "J8"), # New road
        ("R14", "J6", "J8")  # New road
    ]

    for r_id, start, end in roads_data:
        simulator.add_road(r_id, start, end, num_lanes=2, max_speed=70) # Increased max speed slightly

    # Set simulation parameters
    simulator.set_simulation_mode(SimulationMode.DYNAMIC) # Start in dynamic mode
    simulator.set_time_of_day(8)  # Morning rush hour
    simulator.set_weather_conditions("rain") # Add some weather impact

    try:
        # Start the simulation
        simulator.start()

        # Add sample vehicles
        # Generate routes between various junctions
        all_junction_ids = list(junctions_data.keys())
        for i in range(30): # Add more initial vehicles
            start = np.random.choice(all_junction_ids)
            end = np.random.choice(all_junction_ids)

            if start == end: continue

            route = simulator._generate_route(start, end)

            if route:
                vehicle_type = np.random.choice(["car", "car", "car", "bus", "truck"])
                simulator.add_regular_vehicle(start, route, vehicle_type)
            # else:
                # print(f"Could not generate route from {start} to {end}")


        # Add an emergency vehicle with a specific route
        emergency_route = simulator._generate_route("J1", "J8") # Example route
        if emergency_route:
             simulator.add_emergency_vehicle("ambulance", "J1", emergency_route)
        else:
             print("Could not generate route for emergency vehicle.")


        # Add an incident
        # Add incident at a junction or on a road
        simulator.add_incident("accident", junction_id="J5", severity="HIGH")
        simulator.add_incident("roadblock", position=(600, 350), severity="MEDIUM") # Incident on a road segment

        # Keep running until user stops with 'q' or Ctrl+C
        while simulator.running:
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Stop simulation gracefully
        print("\nStopping simulation...")
    finally:
        simulator.stop()

if __name__ == "__main__":
    main()
