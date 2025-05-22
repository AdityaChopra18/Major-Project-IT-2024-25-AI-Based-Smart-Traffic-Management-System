# incident_detection.py
import cv2
import numpy as np
import time

class IncidentDetector:
    """Module for detecting incidents like accidents or road blockages using computer vision."""
    
    def __init__(self, model_path="incident_detection_model.h5", detection_threshold=0.85):
        self.detection_threshold = detection_threshold
        # In a real implementation, load a pre-trained model
        print(f"Initializing Incident Detector with model: {model_path}")
        
        # Incident types that can be detected
        self.incident_types = ["accident", "roadblock", "construction", "flooding", "fallen_tree"]
        
        # For simulation purposes, set a lower random detection rate
        self.detection_rate = 0.05  # Reduced from 0.15 to 0.05 (5% chance)
        
        # Keep track of previously detected incidents to avoid duplicates
        self.previous_detections = []
        self.duplicate_distance_threshold = 50  # pixels
        self.incident_timeout = 10  # seconds before a similar incident can be detected again
        
    def preprocess_frame(self, frame):
        """Preprocess frame for model input."""
        # Resize the frame
        resized = cv2.resize(frame, (416, 416))
        # Normalize
        normalized = resized / 255.0
        return normalized
    
    def detect_incidents(self, frame):
        """
        Detect incidents in the given frame.
        
        Args:
            frame: Image frame from video stream
            
        Returns:
            List of dictionaries containing detected incidents with locations and confidence
        """
        # Preprocess the frame
        processed_frame = self.preprocess_frame(frame)
        
        # In a real implementation, this would use the loaded model for inference
        # This is a simulated detection with improved filtering
        incident_candidates = []
        
        # First phase: get candidate detections (random for demonstration)
        if np.random.random() < self.detection_rate:
            # Generate 1-2 candidate detections per frame
            num_candidates = np.random.randint(1, 3)
            for _ in range(num_candidates):
                incident_type = np.random.choice(self.incident_types)
                # Generate a wider range of confidence scores
                confidence = np.random.uniform(0.4, 0.95)
                
                # Create a location
                h, w = frame.shape[:2]
                x = int(np.random.uniform(0, w))
                y = int(np.random.uniform(0, h))
                
                incident_candidates.append({
                    "type": incident_type,
                    "confidence": confidence,
                    "location": (x, y),
                    "timestamp": time.time()
                })
        
        # Second phase: filter candidates by confidence threshold
        filtered_incidents = []
        for incident in incident_candidates:
            # Only keep detections above threshold
            if incident["confidence"] < self.detection_threshold:
                continue
                
            filtered_incidents.append(incident)
        
        # Third phase: remove possible duplicates by comparing with previous detections
        current_time = time.time()
        unique_incidents = []
        
        for incident in filtered_incidents:
            is_duplicate = False
            
            # Check if this incident is too similar to a previous one
            for prev in self.previous_detections:
                # Check if the previous detection is still valid (not timed out)
                if current_time - prev["timestamp"] > self.incident_timeout:
                    continue
                    
                # Check if same incident type
                if incident["type"] != prev["type"]:
                    continue
                    
                # Check if at a similar location
                x1, y1 = incident["location"]
                x2, y2 = prev["location"]
                distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                
                if distance < self.duplicate_distance_threshold:
                    # It's a duplicate, possibly just detected again
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique_incidents.append(incident)
                
        # Update the previous detections list with new unique incidents
        self.previous_detections.extend(unique_incidents)
        
        # Clean up old detections from the list
        self.previous_detections = [
            prev for prev in self.previous_detections
            if current_time - prev["timestamp"] <= self.incident_timeout
        ]
            
        return unique_incidents
    
    def process_video_stream(self, video_source):
        """Process video stream and detect incidents."""
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
                
            # Only process every 5th frame to simulate real-time processing
            if frame_count % 5 == 0:
                incidents = self.detect_incidents(frame)
                if incidents:
                    detection_count += len(incidents)
                    detections.extend(incidents)
                    for incident in incidents:
                        print(f"Detected incident: {incident['type']} with {incident['confidence']:.2f} confidence")
                        
                        # Optional: Visualize the detection
                        x, y = incident["location"]
                        cv2.circle(frame, (x, y), 20, (0, 0, 255), 2)
                        severity = self.analyze_incident_severity(incident)
                        label = f"{incident['type']} ({severity}): {incident['confidence']:.2f}"
                        cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        
                        # Optional: Save or display frame with detection
                        # cv2.imwrite(f"incident_{detection_count}.jpg", frame)
            
            frame_count += 1
                
            # Break after processing enough frames for demonstration
            if frame_count > 100:
                break
                
            time.sleep(0.05)  # Simulate processing time
            
        print(f"Processed {frame_count} frames, found {detection_count} incidents")
        cap.release()
        return detections
    
    def analyze_incident_severity(self, incident):
        """Analyze the severity of a detected incident."""
        severity_levels = {
            "accident": 0.9,
            "roadblock": 0.7,
            "construction": 0.5,
            "flooding": 0.8,
            "fallen_tree": 0.6
        }
        
        base_severity = severity_levels.get(incident["type"], 0.5)
        # Adjust severity based on confidence
        adjusted_severity = base_severity * incident["confidence"]
        
        if adjusted_severity > 0.8:
            return "HIGH"
        elif adjusted_severity > 0.5:
            return "MEDIUM"
        else:
            return "LOW"
            
    def adjust_detection_parameters(self, threshold=None, timeout=None, distance_threshold=None):
        """
        Adjust detection parameters to fine-tune false positive filtering.
        
        Args:
            threshold: Confidence threshold (0-1)
            timeout: Time in seconds before a similar incident can be detected again
            distance_threshold: Pixel distance to consider incidents as duplicates
        """
        if threshold is not None and 0 <= threshold <= 1:
            self.detection_threshold = threshold
            print(f"Detection threshold updated to {threshold}")
            
        if timeout is not None and timeout > 0:
            self.incident_timeout = timeout
            print(f"Incident timeout updated to {timeout} seconds")
            
        if distance_threshold is not None and distance_threshold > 0:
            self.duplicate_distance_threshold = distance_threshold
            print(f"Duplicate distance threshold updated to {distance_threshold} pixels")
