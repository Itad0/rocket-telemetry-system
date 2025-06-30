"""
Telemetry Encryption and Security Module
"""

import os
import json
import hmac
import hashlib
import secrets
import logging
from typing import Dict, Any
from datetime import datetime
from dataclasses import dataclass
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend

logger = logging.getLogger(__name__)


@dataclass
class SecurityConfig:
    """Security configuration parameters"""
    encryption_algorithm: str = "AES-128-CBC"
    key_rotation_interval: int = 86400  # 24 hours
    enable_integrity_check: bool = True


class TelemetryEncryption:
    """Main telemetry encryption handler"""
    
    def __init__(self, config: SecurityConfig = None):
        self.config = config or SecurityConfig()
        self.current_key_id = "telemetry_key_v1"
        self._key = None
        logger.info("Telemetry encryption initialized")
        
    def _get_or_generate_key(self) -> bytes:
        """Get or generate encryption key"""
        if self._key is None:
            # In production, this would come from secure storage/HSM
            # For demo, generate a key from environment or default
            key_string = os.getenv('ENCRYPTION_KEY', 'default-demo-key-16b')
            self._key = key_string.encode()[:16].ljust(16, b'0')  # Ensure 16 bytes
        return self._key
        
    def encrypt_telemetry(self, telemetry_data: Dict[str, Any]) -> Dict[str, Any]:
        """Encrypt telemetry data packet"""
        
        # Convert to JSON bytes
        plaintext = json.dumps(telemetry_data).encode('utf-8')
        
        # Generate IV
        iv = secrets.token_bytes(16)
        
        # Get key
        key = self._get_or_generate_key()
        
        # Create cipher
        cipher = Cipher(
            algorithms.AES(key),
            modes.CBC(iv),
            backend=default_backend()
        )
        encryptor = cipher.encryptor()
        
        # Pad the plaintext
        padder = padding.PKCS7(128).padder()
        padded_data = padder.update(plaintext) + padder.finalize()
        
        # Encrypt
        ciphertext = encryptor.update(padded_data) + encryptor.finalize()
        
        # Create MAC if enabled
        mac = b''
        if self.config.enable_integrity_check:
            h = hmac.new(key, digestmod=hashlib.sha256)
            h.update(iv)
            h.update(ciphertext)
            mac = h.digest()
        
        # Return encrypted packet
        return {
            "version": 1,
            "key_id": self.current_key_id,
            "iv": iv.hex(),
            "ciphertext": ciphertext.hex(),
            "mac": mac.hex(),
            "timestamp": datetime.now().isoformat()
        }
        
    def decrypt_telemetry(self, encrypted_packet: Dict[str, Any]) -> Dict[str, Any]:
        """Decrypt telemetry data packet"""
        
        # Extract components
        iv = bytes.fromhex(encrypted_packet["iv"])
        ciphertext = bytes.fromhex(encrypted_packet["ciphertext"])
        mac = bytes.fromhex(encrypted_packet["mac"])
        
        # Get key
        key = self._get_or_generate_key()
        
        # Verify MAC if enabled
        if self.config.enable_integrity_check and mac:
            h = hmac.new(key, digestmod=hashlib.sha256)
            h.update(iv)
            h.update(ciphertext)
            expected_mac = h.digest()
            if not hmac.compare_digest(mac, expected_mac):
                raise ValueError("Integrity check failed")
        
        # Create cipher
        cipher = Cipher(
            algorithms.AES(key),
            modes.CBC(iv),
            backend=default_backend()
        )
        decryptor = cipher.decryptor()
        
        # Decrypt
        padded_plaintext = decryptor.update(ciphertext) + decryptor.finalize()
        
        # Unpad
        unpadder = padding.PKCS7(128).unpadder()
        plaintext = unpadder.update(padded_plaintext) + unpadder.finalize()
        
        # Parse JSON
        return json.loads(plaintext.decode('utf-8'))


class AuditLogger:
    """Security audit logger"""
    
    def __init__(self, log_file: str = "logs/security_audit.log"):
        self.log_file = log_file
        
    def log_event(self, event_type: str, details: Dict[str, Any]):
        """Log security event"""
        event = {
            "timestamp": datetime.now().isoformat(),
            "event_type": event_type,
            "details": details
        }
        logger.info(f"Security event: {event_type}")