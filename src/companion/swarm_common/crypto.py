# Crypto utilities for companion computer (Python)

import hashlib
import hmac
import os
from typing import Tuple, Optional
from pathlib import Path

try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey
    from cryptography.hazmat.primitives import serialization
    from cryptography.exceptions import InvalidSignature
    CRYPTOGRAPHY_AVAILABLE = True
except ImportError:
    CRYPTOGRAPHY_AVAILABLE = False

try:
    import nacl.signing
    import nacl.encoding
    NACL_AVAILABLE = True
except ImportError:
    NACL_AVAILABLE = False


class Crc32:
    """CRC32C (Castagnoli) - compatible with MAVLink 2.0"""
    
    POLYNOMIAL = 0x1EDC6F41
    INIT_VALUE = 0xFFFFFFFF
    XOR_OUT = 0xFFFFFFFF
    
    # Precomputed table for performance
    _table = None
    
    @classmethod
    def _init_table(cls):
        if cls._table is None:
            cls._table = [0] * 256
            for i in range(256):
                crc = i
                for _ in range(8):
                    crc = (crc >> 1) ^ cls.POLYNOMIAL if (crc & 1) else (crc >> 1)
                cls._table[i] = crc
    
    @classmethod
    def compute(cls, data: bytes, crc: int = INIT_VALUE) -> int:
        cls._init_table()
        crc ^= cls.INIT_VALUE
        for byte in data:
            crc = cls._table[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc ^ cls.XOR_OUT
    
    @classmethod
    def compute_bytearray(cls, data: bytearray, crc: int = INIT_VALUE) -> int:
        return cls.compute(bytes(data), crc)


class Ed25519:
    """ED25519 signature operations"""
    
    PUBLIC_KEY_SIZE = 32
    PRIVATE_KEY_SIZE = 32  # Seed only (private key derives public)
    SIGNATURE_SIZE = 64
    
    @classmethod
    def generate_keypair(cls) -> Tuple[bytes, bytes]:
        """Generate (private_key_seed, public_key)"""
        if CRYPTOGRAPHY_AVAILABLE:
            private_key = Ed25519PrivateKey.generate()
            public_key = private_key.public_key()
            private_seed = private_key.private_bytes(
                encoding=serialization.Encoding.Raw,
                format=serialization.PrivateFormat.Raw,
                encryption_algorithm=serialization.NoEncryption()
            )
            public_bytes = public_key.public_bytes(
                encoding=serialization.Encoding.Raw,
                format=serialization.PublicFormat.Raw
            )
            return private_seed, public_bytes
        elif NACL_AVAILABLE:
            signing_key = nacl.signing.SigningKey.generate()
            return bytes(signing_key), bytes(signing_key.verify_key)
        else:
            # Fallback: use os.urandom (NOT cryptographically secure for production)
            private_seed = os.urandom(32)
            # In real implementation, would derive public from private
            raise NotImplementedError("cryptography or pynacl required for ED25519")
    
    @classmethod
    def sign(cls, message: bytes, private_seed: bytes) -> bytes:
        if CRYPTOGRAPHY_AVAILABLE:
            private_key = Ed25519PrivateKey.from_private_bytes(private_seed)
            return private_key.sign(message)
        elif NACL_AVAILABLE:
            signing_key = nacl.signing.SigningKey(private_seed)
            signed = signing_key.sign(message)
            return signed.signature
        else:
            raise NotImplementedError("cryptography or pynacl required")
    
    @classmethod
    def verify(cls, message: bytes, signature: bytes, public_key: bytes) -> bool:
        if CRYPTOGRAPHY_AVAILABLE:
            public_key_obj = Ed25519PublicKey.from_public_bytes(public_key)
            try:
                public_key_obj.verify(signature, message)
                return True
            except InvalidSignature:
                return False
        elif NACL_AVAILABLE:
            verify_key = nacl.signing.VerifyKey(public_key)
            try:
                verify_key.verify(message, signature)
                return True
            except nacl.exceptions.BadSignatureError:
                return False
        else:
            raise NotImplementedError("cryptography or pynacl required")
    
    @classmethod
    def verify_firmware(cls, image: bytes, signature: bytes, public_key: bytes) -> bool:
        return cls.verify(image, signature, public_key)
    
    @classmethod
    def verify_ota_bundle(cls, bundle: bytes, signature: bytes, public_key: bytes) -> bool:
        return cls.verify(bundle, signature, public_key)


class KeyStore:
    """Key management for companion computer"""
    
    def __init__(self, key_dir: str = "/etc/drone-swarm/keys"):
        self._key_dir = Path(key_dir)
        self._key_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    
    def get_ota_public_key(self) -> Optional[bytes]:
        """Get OTA verification public key"""
        key_file = self._key_dir / "ota_public.key"
        if key_file.exists():
            return key_file.read_bytes()
        return None
    
    def set_ota_public_key(self, public_key: bytes) -> bool:
        """Store OTA public key (ground station only)"""
        if len(public_key) != Ed25519.PUBLIC_KEY_SIZE:
            return False
        key_file = self._key_dir / "ota_public.key"
        key_file.write_bytes(public_key)
        key_file.chmod(0o600)
        return True
    
    def get_identity_keypair(self) -> Optional[Tuple[bytes, bytes]]:
        """Get device identity keypair for MAVLink signing"""
        priv_file = self._key_dir / "identity_private.key"
        pub_file = self._key_dir / "identity_public.key"
        if priv_file.exists() and pub_file.exists():
            return priv_file.read_bytes(), pub_file.read_bytes()
        return None
    
    def generate_identity_keypair(self) -> Tuple[bytes, bytes]:
        """Generate new identity keypair"""
        private_seed, public_key = Ed25519.generate_keypair()
        priv_file = self._key_dir / "identity_private.key"
        pub_file = self._key_dir / "identity_public.key"
        priv_file.write_bytes(private_seed)
        pub_file.write_bytes(public_key)
        priv_file.chmod(0o600)
        pub_file.chmod(0o644)
        return private_seed, public_key


def secure_compare(a: bytes, b: bytes) -> bool:
    """Constant-time comparison"""
    return hmac.compare_digest(a, b)