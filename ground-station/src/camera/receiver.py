"""
Camera Receiver Module for Image/Video Telemetry
"""

import asyncio
import logging
import os
from datetime import datetime
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class CameraReceiver:
    """Handles camera data reception and storage"""
    
    def __init__(self):
        self.running = False
        self.image_count = 0
        self.video_active = False
        self.storage_dir = "reports/camera"
        os.makedirs(self.storage_dir, exist_ok=True)
        
        logger.info("Camera receiver initialized")
        
    async def start(self):
        """Start camera receiver"""
        self.running = True
        logger.info("Camera receiver started")
        
        # Start receiving loop
        asyncio.create_task(self._receive_loop())
        
    async def stop(self):
        """Stop camera receiver"""
        self.running = False
        self.video_active = False
        logger.info("Camera receiver stopped")
        
    async def _receive_loop(self):
        """Main reception loop for camera data"""
        while self.running:
            try:
                # Simulate camera data reception
                await asyncio.sleep(5)  # Receive image every 5 seconds
                
                if self.running:
                    await self._process_image()
                    
            except Exception as e:
                logger.error(f"Error in camera receive loop: {e}")
                
    async def _process_image(self):
        """Process received image"""
        self.image_count += 1
        
        # For demo, just log the reception
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"image_{timestamp}_{self.image_count}.jpg"
        filepath = os.path.join(self.storage_dir, filename)
        
        # In production, this would save actual image data
        logger.info(f"Received image #{self.image_count}: {filename}")
        
        # Create placeholder file
        with open(filepath, 'w') as f:
            f.write(f"Placeholder for image {self.image_count}")
            
    async def start_video_recording(self):
        """Start video recording"""
        if not self.video_active:
            self.video_active = True
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            logger.info(f"Started video recording at {timestamp}")
            
    async def stop_video_recording(self):
        """Stop video recording"""
        if self.video_active:
            self.video_active = False
            logger.info("Stopped video recording")
            
    def get_stats(self) -> Dict[str, Any]:
        """Get camera receiver statistics"""
        return {
            'running': self.running,
            'images_received': self.image_count,
            'video_active': self.video_active,
            'storage_directory': self.storage_dir
        }
        
    async def capture_image(self, metadata: Optional[Dict[str, Any]] = None):
        """Trigger image capture"""
        logger.info(f"Image capture triggered with metadata: {metadata}")
        await self._process_image()
        
    def get_latest_image_path(self) -> Optional[str]:
        """Get path to most recent image"""
        try:
            files = os.listdir(self.storage_dir)
            image_files = [f for f in files if f.startswith('image_')]
            if image_files:
                latest = sorted(image_files)[-1]
                return os.path.join(self.storage_dir, latest)
        except:
            pass
        return None