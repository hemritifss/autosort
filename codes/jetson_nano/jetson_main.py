# ═══════════════════════════════════════════════════
# JETSON NANO - MAIN CONTROLLER
# ═══════════════════════════════════════════════════

import cv2
import numpy as np
import serial
import threading
import time
import json
from ultralytics import YOLO
from queue import Queue
import RPi.GPIO as GPIO  # jetson.GPIO

class WasteSortingSystem:
    
    def __init__(self):
        # ── AI MODEL ──────────────────────────────
        self.model = YOLO('waste_model.pt')
        self.classes = [
            'PET_clear', 'PET_colored', 'HDPE',
            'PP', 'PS', 'paper', 'cardboard',
            'aluminum', 'steel', 'glass',
            'film_plastic', 'tetra_pak',
            'battery', 'hazardous', 'organic',
            'unknown'
        ]
        
        # ── CAMERAS ───────────────────────────────
        self.cam_top = cv2.VideoCapture(0)    # CSI
        self.cam_top.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cam_top.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cam_top.set(cv2.CAP_PROP_FPS, 30)
        
        # ── SERIAL COMMUNICATIONS ─────────────────
        self.stm32 = serial.Serial(
            '/dev/ttyUSB0', 
            baudrate=115200,
            timeout=0.1
        )
        self.esp32_sensors = serial.Serial(
            '/dev/ttyUSB1',
            baudrate=115200, 
            timeout=0.1
        )
        self.esp32_safety = serial.Serial(
            '/dev/ttyUSB2',
            baudrate=115200,
            timeout=0.1
        )
        
        # ── QUEUES ────────────────────────────────
        self.detection_queue = Queue(maxsize=20)
        self.ejection_queue = Queue(maxsize=20)
        self.sensor_data = {}
        
        # ── CONVEYOR PARAMS ───────────────────────
        self.belt_speed_cms = 20.0    # cm/s
        self.cam_to_jet_cm = 25.0     # cm
        self.ejection_delay_ms = int(
            (self.cam_to_jet_cm / self.belt_speed_cms) * 1000
        )
        
        # ── STATS ─────────────────────────────────
        self.stats = {cls: 0 for cls in self.classes}
        self.total_processed = 0
        self.hazards_detected = 0
        
        print(f"✅ System initialized")
        print(f"📏 Ejection delay: {self.ejection_delay_ms}ms")
    
    # ─────────────────────────────────────────────
    # VISION THREAD - يشتغل باستمرار
    # ─────────────────────────────────────────────
    def vision_thread(self):
        
        while True:
            ret, frame = self.cam_top.read()
            if not ret:
                continue
            
            # Get sensor data for this frame
            sensor = self.sensor_data.copy()
            
            # ── AI INFERENCE ──────────────────────
            results = self.model(
                frame,
                conf=0.5,           # confidence threshold
                iou=0.45,           # NMS threshold
                imgsz=640,          # inference size
                verbose=False
            )
            
            # ── PROCESS DETECTIONS ────────────────
            for result in results:
                boxes = result.boxes
                
                for box in boxes:
                    # Basic detection
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    bbox = box.xyxy[0].tolist()
                    
                    # Fuse with sensor data
                    final_class, final_conf = self.fuse_classification(
                        cls_id, confidence, sensor
                    )
                    
                    # Create detection event
                    detection = {
                        'class': final_class,
                        'confidence': final_conf,
                        'bbox': bbox,
                        'timestamp': time.time(),
                        'sensor_data': sensor.copy()
                    }
                    
                    # Queue for ejection
                    if not self.detection_queue.full():
                        self.detection_queue.put(detection)
                    
                    # Update stats
                    self.stats[final_class] = \
                        self.stats.get(final_class, 0) + 1
                    self.total_processed += 1
                    
                    # ⚠️ HAZARD IMMEDIATE RESPONSE
                    if final_class in ['battery', 'hazardous']:
                        self.emergency_eject(final_class)
                        self.hazards_detected += 1
            
            # ── DISPLAY (debug) ───────────────────
            self.draw_debug(frame, results)
            cv2.imshow('Waste Sorter', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    # ─────────────────────────────────────────────
    # SENSOR FUSION
    # ─────────────────────────────────────────────
    def fuse_classification(self, cls_id, confidence, sensor):
        
        base_class = self.classes[cls_id]
        
        # Metal detection override
        if sensor.get('metal_detected', False):
            metal_strength = sensor.get('metal_strength', 0)
            
            if metal_strength > 0.8:
                # Strong metal signal
                if sensor.get('is_ferrous', True):
                    return 'steel', 0.95
                else:
                    return 'aluminum', 0.90
            
            elif metal_strength > 0.4:
                # Medium metal - might be battery!
                if self.is_battery_shape(sensor):
                    return 'battery', 0.85
                # Composite object
                return base_class + '_metal_composite', 0.70
        
        # Temperature override (hot = possible battery)
        temp = sensor.get('temperature', 20)
        if temp > 45:
            return 'battery', 0.80
        
        # Gas detection override
        if sensor.get('gas_level', 0) > 400:
            return 'hazardous', 0.90
        
        # Spectral data enhancement
        if 'spectrum' in sensor:
            spectral_class = self.classify_by_spectrum(
                sensor['spectrum']
            )
            if spectral_class and confidence < 0.75:
                # Low vision confidence → trust spectrum more
                return spectral_class, 0.80
        
        # Weight-based check
        weight = sensor.get('weight_g', 0)
        if weight > 500 and 'glass' not in base_class:
            # Very heavy, probably glass or ceramic
            if confidence < 0.80:
                return 'glass', 0.65
        
        return base_class, confidence
    
    # ─────────────────────────────────────────────
    # SPECTRAL CLASSIFICATION
    # ─────────────────────────────────────────────
    def classify_by_spectrum(self, spectrum):
        """
        AS7265x gives us 18 channels (410-940nm)
        Simple signature matching for plastics
        """
        # Normalized spectrum
        total = sum(spectrum.values()) + 1e-6
        norm = {k: v/total for k, v in spectrum.items()}
        
        # PET signature: strong absorption at 900-940nm
        pet_score = norm.get('900nm', 0) + norm.get('940nm', 0)
        
        # HDPE signature: different NIR pattern
        hdpe_score = norm.get('760nm', 0) + norm.get('810nm', 0)
        
        # PP signature
        pp_score = norm.get('730nm', 0)
        
        # Paper: more reflective in visible, less in NIR
        paper_score = (norm.get('535nm', 0) + 
                       norm.get('610nm', 0))
        
        scores = {
            'PET_clear': pet_score,
            'HDPE': hdpe_score,
            'PP': pp_score,
            'paper': paper_score
        }
        
        best = max(scores, key=scores.get)
        if scores[best] > 0.35:  # threshold
            return best
        return None
    
    # ─────────────────────────────────────────────
    # EJECTION THREAD
    # ─────────────────────────────────────────────
    def ejection_thread(self):
        
        EJECTION_MAP = {
            'PET_clear':    1,
            'PET_colored':  1,
            'HDPE':         2,
            'PP':           2,
            'PS':           2,
            'paper':        3,
            'cardboard':    3,
            'aluminum':     4,
            'steel':        4,
            'glass':        5,
            'battery':      6,  # HAZARD!
            'hazardous':    6,  # HAZARD!
            'organic':      0,  # falls through
            'unknown':      0   # falls through
        }
        
        while True:
            if not self.detection_queue.empty():
                detection = self.detection_queue.get()
                
                jet_id = EJECTION_MAP.get(
                    detection['class'], 0
                )
                
                if jet_id > 0:
                    # Calculate timing
                    elapsed = (time.time() - 
                               detection['timestamp']) * 1000
                    remaining_delay = max(
                        0, 
                        self.ejection_delay_ms - elapsed
                    )
                    
                    # Wait for item to reach jet
                    time.sleep(remaining_delay / 1000)
                    
                    # Send ejection command to ESP32 Safety
                    cmd = json.dumps({
                        'cmd': 'eject',
                        'jet': jet_id,
                        'duration_ms': 80,
                        'class': detection['class']
                    }) + '\n'
                    
                    self.esp32_safety.write(cmd.encode())
                    
                    print(f"🚀 Ejected: {detection['class']} "
                          f"→ Jet {jet_id} "
                          f"(conf: {detection['confidence']:.2f})")
    
    # ─────────────────────────────────────────────
    # EMERGENCY EJECT (immediate, no delay)
    # ─────────────────────────────────────────────
    def emergency_eject(self, hazard_type):
        cmd = json.dumps({
            'cmd': 'emergency_eject',
            'jet': 6,
            'duration_ms': 200,
            'class': hazard_type,
            'alarm': True
        }) + '\n'
        self.esp32_safety.write(cmd.encode())
        print(f"🚨 EMERGENCY EJECT: {hazard_type}")
    
    # ─────────────────────────────────────────────
    # SENSOR READ THREAD
    # ─────────────────────────────────────────────
    def sensor_read_thread(self):
        
        while True:
            # Read from ESP32 Sensors
            if self.esp32_sensors.in_waiting:
                line = self.esp32_sensors.readline()
                try:
                    data = json.loads(line.decode().strip())
                    self.sensor_data.update(data)
                except:
                    pass
            
            # Read from ESP32 Safety
            if self.esp32_safety.in_waiting:
                line = self.esp32_safety.readline()
                try:
                    data = json.loads(line.decode().strip())
                    if data.get('emergency_stop'):
                        self.handle_emergency_stop()
                    self.sensor_data.update(data)
                except:
                    pass
            
            time.sleep(0.01)  # 100Hz sensor read rate
    
    # ─────────────────────────────────────────────
    # MOTOR CONTROL - send to STM32
    # ─────────────────────────────────────────────
    def set_belt_speed(self, speed_percent):
        cmd = json.dumps({
            'cmd': 'set_speed',
            'motor': 'main_belt',
            'speed': speed_percent
        }) + '\n'
        self.stm32.write(cmd.encode())
        
        # Recalculate ejection delay
        self.belt_speed_cms = speed_percent * 0.4  # calibrate!
        self.ejection_delay_ms = int(
            (self.cam_to_jet_cm / self.belt_speed_cms) * 1000
        )
    
    # ─────────────────────────────────────────────
    # STATS & DASHBOARD
    # ─────────────────────────────────────────────
    def print_stats(self):
        print("\n" + "="*50)
        print("📊 SORTING STATISTICS")
        print("="*50)
        print(f"Total processed: {self.total_processed}")
        print(f"Hazards detected: {self.hazards_detected} ⚠️")
        print("-"*50)
        for cls, count in sorted(
            self.stats.items(), 
            key=lambda x: x[1], 
            reverse=True
        ):
            if count > 0:
                pct = count / max(self.total_processed, 1) * 100
                bar = "█" * int(pct/5)
                print(f"{cls:20s}: {count:4d} ({pct:.1f}%) {bar}")
        print("="*50)
    
    # ─────────────────────────────────────────────
    # MAIN RUN
    # ─────────────────────────────────────────────
    def run(self):
        
        print("🚀 Starting Waste Sorting System...")
        
        # Start all threads
        threads = [
            threading.Thread(target=self.vision_thread, 
                           daemon=True),
            threading.Thread(target=self.ejection_thread,
                           daemon=True),
            threading.Thread(target=self.sensor_read_thread,
                           daemon=True),
        ]
        
        for t in threads:
            t.start()
        
        # Main loop - stats display
        try:
            while True:
                time.sleep(10)
                self.print_stats()
        except KeyboardInterrupt:
            print("\n🛑 System stopped")
            self.stm32.write(b'{"cmd":"stop_all"}\n')


# ─────────────────────────────────────────────────
if __name__ == "__main__":
    system = WasteSortingSystem()
    system.run()
