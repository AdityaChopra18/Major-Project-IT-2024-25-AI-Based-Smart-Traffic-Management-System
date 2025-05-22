# emergency_vehicle_detection.py
import cv2
import numpy as np
import time


class EmergencyVehicleDetector:
    """Module for detecting emergency vehicles in video streams using computer vision."""
    
    def __init__(self, model_path="emergency_vehicle_detection.h5", detection_threshold=0.85):
        self.detection_threshold = detection_threshold
        # In a real implementation, load a pre-trained model
        print(f"Initializing Emergency Vehicle Detector with model: {model_path}")
        # This is a placeholder for model initialization
        self.classes = ["car", "bus", "truck", "ambulance", "police_car", "fire_truck"]
        
        # For simulation purposes, set a lower random detection rate
        self.detection_rate = 0.05  # Reduced from 0.2 to 0.05 (5% chance)
        
    def preprocess_frame(self, frame):
        """Preprocess frame for model input."""
        # Resize the frame
        resized = cv2.resize(frame, (416, 416))
        # Normalize
        normalized = resized / 255.0
        return normalized
    
    def detect_emergency_vehicles(self, frame):
        """
        Detect emergency vehicles in the given frame.
        
        Args:
            frame: Image frame from video stream
            
        Returns:
            List of dictionaries containing detected emergency vehicles with bounding boxes and confidence
        """
        # Preprocess the frame
        processed_frame = self.preprocess_frame(frame)
        
        # In a real implementation, this would use the loaded model for inference
        # This is a simulated detection with improved filtering
        emergency_vehicles = []
        
        # First phase: get candidate detections (random for demonstration)
        candidates = []
        # Lower probability of generating a detection at all
        if np.random.random() < self.detection_rate:
            # Generate 1-3 candidate detections per frame
            num_candidates = np.random.randint(1, 4)
            for _ in range(num_candidates):
                vehicle_type = np.random.choice(self.classes)
                # Generate a wider range of confidence scores
                confidence = np.random.uniform(0.3, 0.99)
                
                # Create a bounding box
                h, w = frame.shape[:2]
                x1 = int(np.random.uniform(0, w/2))
                y1 = int(np.random.uniform(0, h/2))
                x2 = int(x1 + np.random.uniform(100, w/2))
                y2 = int(y1 + np.random.uniform(100, h/2))
                
                candidates.append({
                    "type": vehicle_type,
                    "confidence": confidence,
                    "bbox": (x1, y1, x2, y2)
                })
        
        # Second phase: filter candidates by confidence and class
        for candidate in candidates:
            # Only keep detections above threshold
            if candidate["confidence"] < self.detection_threshold:
                continue
                
            # Only keep emergency vehicle classes
            if candidate["type"] not in ["ambulance", "police_car", "fire_truck"]:
                continue
                
            emergency_vehicles.append(candidate)
        
        # Apply non-maximum suppression if there are multiple detections
        # (simplified version for this example)
        if len(emergency_vehicles) > 1:
            emergency_vehicles.sort(key=lambda x: x["confidence"], reverse=True)
            # Keep only the highest confidence detection
            emergency_vehicles = emergency_vehicles[:1]
            
        return emergency_vehicles
    
    def process_video_stream(self, video_source):
        """Process video stream and detect emergency vehicles."""
        cap = cv2.VideoCapture(video_source)
        if not cap.isOpened():
            print(f"Error: Could not open video source {video_source}")
            return []
            
        detections = []
        frame_count = 0
        detection_count = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            frame_count += 1
            vehicles = self.detect_emergency_vehicles(frame)
            
            if vehicles:
                detection_count += 1
                detections.extend(vehicles)
                print(f"Detected emergency vehicle: {vehicles[0]['type']} with {vehicles[0]['confidence']:.2f} confidence")
                
                # Draw bounding box on frame (for visualization)
                for vehicle in vehicles:
                    x1, y1, x2, y2 = vehicle["bbox"]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{vehicle['type']}: {vehicle['confidence']:.2f}"
                    cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Optional: Save or display frame with detection
                # cv2.imwrite(f"detection_{detection_count}.jpg", frame)
                
            # Break after processing a reasonable number of frames
            if frame_count >= 100:
                break
                
            time.sleep(0.1)  # Simulate processing time
            
        print(f"Processed {frame_count} frames, found {detection_count} emergency vehicles")
        cap.release()
        return detections
    
    def adjust_detection_threshold(self, new_threshold):
        """
        Adjust the detection threshold to filter out more false positives.
        
        Args:
            new_threshold: Float between 0 and 1, higher values mean fewer false positives
        """
        if 0 <= new_threshold <= 1:
            self.detection_threshold = new_threshold
            print(f"Detection threshold updated to {new_threshold}")
        else:
            print("Threshold must be between 0 and 1")
