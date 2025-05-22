# main.py
# command to run the video demo "python main.py --mode demo --video road3.mp4 --duration 30 --report"
# simulation can be run by "python simulation.py" or run simulation.py
import argparse
import cv2
import time
import os
import threading
import numpy as np
from emergency_vehicle_detection import EmergencyVehicleDetector
from incident_detection import IncidentDetector
from traffic_signal_controller import TrafficSignalController
from simulator import TrafficSimulator
from utils import calculate_nearest_junctions, get_random_incident, get_random_emergency_vehicle
from utils import ensure_directory_exists, generate_report

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="AI-Based Smart Traffic Management System")
    
    parser.add_argument("--mode", type=str, default="simulation",
                        choices=["simulation", "demo"],
                        help="Operation mode: simulation or demo with provided videos")
                        
    parser.add_argument("--video", type=str, default=None,
                        help="Path to video file for processing (required in demo mode)")
                        
    parser.add_argument("--duration", type=int, default=60,
                        help="Duration of simulation in seconds")
                        
    parser.add_argument("--junctions", type=int, default=4,
                        help="Number of junctions to simulate")
                        
    parser.add_argument("--incidents", type=int, default=2,
                        help="Number of random incidents to simulate")
                        
    parser.add_argument("--emergency", type=int, default=3,
                        help="Number of emergency vehicles to simulate")
                        
    parser.add_argument("--report", action="store_true",
                        help="Generate a report after simulation")
                        
    return parser.parse_args()

def setup_simulation(args, controller):
    """Set up the traffic simulation."""
    simulator = TrafficSimulator()
    
    # Add junctions
    for i in range(args.junctions):
        x = 150 + (i % 2) * 500
        y = 150 + (i // 2) * 300
        
        junction_id = f"J{i}"
        controller.add_junction(junction_id)
        simulator.add_junction(junction_id, (x, y), controller)
        
    # Add random incidents
    incidents = []
    for _ in range(args.incidents):
        incident = get_random_incident()
        incidents.append(incident)
        
        # Find affected junctions
        affected_junctions = []
        for j_id, junction in simulator.junctions.items():
            dist = np.linalg.norm(np.array(incident["position"]) - np.array(junction["position"]))
            if dist < 200:  # Affect junctions within 200 pixels
                affected_junctions.append(j_id)
                
        # Handle the incident in the controller
        controller.handle_incident({
            "type": incident["type"],
            "location": incident["position"],
            "severity": incident["severity"],
            "affected_junctions": affected_junctions
        })
        
        # Add to simulator
        simulator.add_incident(
            incident["type"],
            incident["position"],
            incident["severity"]
        )
        
    # Add emergency vehicles with random routes
    # main.py (continued)
    # Add emergency vehicles with random routes
    emergency_vehicles = []
    for _ in range(args.emergency):
        vehicle = get_random_emergency_vehicle()
        emergency_vehicles.append(vehicle)
        
        # Add to simulator
        simulator.add_emergency_vehicle(
            vehicle["type"],
            vehicle["start_position"],
            vehicle["route"]
        )
        
    return simulator, incidents, emergency_vehicles

def run_simulation(args):
    """Run the traffic simulation."""
    print("Starting traffic simulation...")
    
    # Create controller
    controller = TrafficSignalController()
    
    # Set up simulation
    simulator, incidents, emergency_vehicles = setup_simulation(args, controller)
    
    # Start controller and simulator
    controller.start()
    simulator.start()
    
    # Run for specified duration
    start_time = time.time()
    try:
        while time.time() - start_time < args.duration:
            # Monitor for keyboard input to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Simulation interrupted by user")
    finally:
        # Stop everything
        simulator.stop()
        controller.stop()
        
    # Generate report if requested
    if args.report:
        report = generate_report(incidents, emergency_vehicles, time.time() - start_time)
        
        # Ensure reports directory exists
        report_dir = "reports"
        ensure_directory_exists(report_dir)
        
        # Save report
        report_path = os.path.join(report_dir, f"report_{int(time.time())}.txt")
        with open(report_path, "w") as f:
            f.write(report)
            
        print(f"Report saved to {report_path}")
        print("\nReport Summary:")
        print(report)

def run_demo(args):
    """Run demonstration using real video input."""
    if not args.video:
        print("Error: Video path must be specified in demo mode")
        return
        
    if not os.path.exists(args.video):
        print(f"Error: Video file not found: {args.video}")
        return
        
    print(f"Starting demo with video: {args.video}")
    
    # Initialize detectors
    emergency_detector = EmergencyVehicleDetector()
    incident_detector = IncidentDetector()
    
    # Initialize controller
    controller = TrafficSignalController()
    
    # Add some junctions
    for i in range(4):
        controller.add_junction(f"J{i}")
        
    controller.start()
    
    # Process video
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print("Error: Could not open video")
        return
        
    frame_count = 0
    detected_incidents = []
    detected_emergency = []
    
    # For demonstration purposes, create a window
    cv2.namedWindow("Demo - AI Traffic Management", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Demo - AI Traffic Management", 1024, 768)
    
    start_time = time.time()
    try:
        while cap.isOpened() and (time.time() - start_time < args.duration):
            ret, frame = cap.read()
            if not ret:
                # End of video, loop back
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
                
            # Process every 5th frame to simulate real-time
            if frame_count % 5 == 0:
                # Detect emergency vehicles
                emergency_vehicles = emergency_detector.detect_emergency_vehicles(frame)
                for vehicle in emergency_vehicles:
                    print(f"Detected {vehicle['type']} with confidence {vehicle['confidence']:.2f}")
                    detected_emergency.append(vehicle)
                    
                    # Handle emergency vehicle (random junction and direction for demo)
                    junction_id = f"J{np.random.randint(0, 4)}"
                    approach = np.random.randint(0, 4)
                    target = (approach + 2) % 4  # Opposite direction
                    controller.handle_emergency_vehicle(junction_id, approach, target)
                    
                    # Draw bounding box on frame
                    x1, y1, x2, y2 = vehicle["bbox"]
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                    cv2.putText(frame, f"{vehicle['type']} {vehicle['confidence']:.2f}",
                               (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Detect incidents
                incidents = incident_detector.detect_incidents(frame)
                for incident in incidents:
                    severity = incident_detector.analyze_incident_severity(incident)
                    print(f"Detected {incident['type']} with severity {severity}")
                    detected_incidents.append(incident)
                    
                    # Handle incident (random affected junctions for demo)
                    affected = [f"J{np.random.randint(0, 4)}" for _ in range(np.random.randint(1, 3))]
                    controller.handle_incident({
                        "type": incident["type"],
                        "location": incident["location"],
                        "severity": severity,
                        "affected_junctions": affected
                    })
                    
                    # Draw incident on frame
                    x, y = incident["location"]
                    cv2.circle(frame, (int(x), int(y)), 30, (0, 0, 255), 2)
                    cv2.putText(frame, f"{incident['type']} ({severity})",
                               (int(x) - 40, int(y) - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
            # Add simulation time
            elapsed = time.time() - start_time
            cv2.putText(frame, f"Time: {elapsed:.1f}s", (30, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                       
            # Add stats
            cv2.putText(frame, f"Emergency Vehicles: {len(detected_emergency)}", (30, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Incidents: {len(detected_incidents)}", (30, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                       
            # Show the frame
            cv2.imshow("Demo - AI Traffic Management", frame)
            
            # Press Q to quit
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
                
            frame_count += 1
            
    except KeyboardInterrupt:
        print("Demo interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        controller.stop()
        
    # Generate report if requested
    if args.report:
        report = f"=== AI Traffic Management Demo Report ===\n"
        report += f"Duration: {time.time() - start_time:.1f} seconds\n"
        report += f"Video processed: {args.video}\n\n"
        
        report += f"Emergency Vehicles Detected: {len(detected_emergency)}\n"
        for i, vehicle in enumerate(detected_emergency[:10]):  # Show max 10
            report += f"  {i+1}. {vehicle['type']} (Confidence: {vehicle['confidence']:.2f})\n"
            
        report += f"\nIncidents Detected: {len(detected_incidents)}\n"
        for i, incident in enumerate(detected_incidents[:10]):  # Show max 10
            severity = incident_detector.analyze_incident_severity(incident)
            report += f"  {i+1}. {incident['type']} (Severity: {severity})\n"
            
        # Ensure reports directory exists
        report_dir = "reports"
        ensure_directory_exists(report_dir)
        
        # Save report
        report_path = os.path.join(report_dir, f"demo_report_{int(time.time())}.txt")
        with open(report_path, "w") as f:
            f.write(report)
            
        print(f"Report saved to {report_path}")
        print("\nReport Summary:")
        print(report)

def main():
    """Main entry point of the application."""
    args = parse_arguments()
    
    print("=" * 50)
    print("AI-Based Smart Traffic Management System")
    print("=" * 50)
    
    if args.mode == "simulation":
        run_simulation(args)
    else:
        run_demo(args)
        
if __name__ == "__main__":
    main()