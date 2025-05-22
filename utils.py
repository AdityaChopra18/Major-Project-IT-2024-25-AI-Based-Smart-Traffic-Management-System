# utils.py
import numpy as np
import os
import time

def calculate_nearest_junctions(location, junctions, max_distance=200):
    """
    Calculate the nearest junctions to a given location.
    
    Args:
        location: (x, y) coordinates
        junctions: Dictionary of junctions with positions
        max_distance: Maximum distance to consider junctions as affected
        
    Returns:
        List of junction IDs sorted by distance
    """
    distances = {}
    location_array = np.array(location)
    
    for j_id, junction in junctions.items():
        pos = np.array(junction["position"])
        distance = np.linalg.norm(location_array - pos)
        if distance <= max_distance:
            distances[j_id] = distance
            
    # Sort by distance
    sorted_junctions = sorted(distances.items(), key=lambda x: x[1])
    return [j_id for j_id, _ in sorted_junctions]

def get_random_incident():
    """Generate a random incident for testing."""
    incident_types = ["accident", "roadblock", "construction", "fallen_tree", "flooding"]
    severity_levels = ["LOW", "MEDIUM", "HIGH"]
    
    return {
        "type": np.random.choice(incident_types),
        "position": (
            np.random.randint(100, 700),
            np.random.randint(100, 500)
        ),
        "severity": np.random.choice(severity_levels)
    }

def get_random_emergency_vehicle():
    """Generate a random emergency vehicle for testing."""
    vehicle_types = ["ambulance", "police_car", "fire_truck"]
    
    # Create a route with 3-5 waypoints
    num_waypoints = np.random.randint(3, 6)
    route = []
    
    for _ in range(num_waypoints):
        route.append((
            np.random.randint(100, 700),
            np.random.randint(100, 500)
        ))
        
    return {
        "type": np.random.choice(vehicle_types),
        "start_position": (np.random.randint(50, 750), np.random.randint(50, 550)),
        "route": route
    }

def ensure_directory_exists(path):
    """Ensure that a directory exists, creating it if necessary."""
    if not os.path.exists(path):
        os.makedirs(path)
        
def generate_report(incidents, emergency_vehicles, duration):
    """Generate a simple report of traffic events."""
    report = "=== Traffic Management System Report ===\n"
    report += f"Time period: {duration:.1f} seconds\n\n"
    
    report += "Incidents Detected:\n"
    for i, incident in enumerate(incidents):
        report += f"  {i+1}. {incident['type']} at {incident['position']} (Severity: {incident['severity']})\n"
    
    report += "\nEmergency Vehicles Processed:\n"
    for i, vehicle in enumerate(emergency_vehicles):
        report += f"  {i+1}. {vehicle['type']} - {'Completed route' if not vehicle['active'] else 'Still active'}\n"
        
    return report