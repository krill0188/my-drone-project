#ifndef SWARM_CRYPTO_HPP
#define SWARM_CRYPTO_HPP

#include <cstdint>
#include <array>
#include <span>

namespace swarm::crypto {

// CRC32 (Castagnoli polynomial 0x1EDC6F41) - used in MAVLink 2.0, OTA
class Crc32 {
public:
    static constexpr uint32_t POLYNOMIAL = 0x1EDC6F41;
    static constexpr uint32_t INIT_VALUE = 0xFFFFFFFF;
    static constexpr uint32_t XOR_OUT = 0xFFFFFFFF;

    static uint32_t compute(const uint8_t* data, size_t len, uint32_t crc = INIT_VALUE) {
        crc ^= INIT_VALUE;
        for (size_t i = 0; i < len; ++i) {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                crc = (crc & 1) ? (crc >> 1) ^ POLYNOMIAL : (crc >> 1);
            }
        }
        return crc ^ XOR_OUT;
    }

    static uint32_t compute(std::span<const uint8_t> data, uint32_t crc = INIT_VALUE) {
        return compute(data.data(), data.size(), crc);
    }

    template<size_t N>
    static uint32_t compute(const std::array<uint8_t, N>& data, uint32_t crc = INIT_VALUE) {
        return compute(data.data(), N, crc);
    }
};

// ED25519 signature verification (for OTA, firmware signing)
// Minimal implementation using TweetNaCl or similar
// In production, use libsodium or mbedTLS

class Ed25519 {
public:
    static constexpr size_t PUBLIC_KEY_SIZE = 32;
    static constexpr size_t PRIVATE_KEY_SIZE = 64;  // seed + public
    static constexpr size_t SIGNATURE_SIZE = 64;

    // Verify signature: returns true if valid
    // msg: message to verify
    // sig: 64-byte signature
    // pub: 32-byte public key
    static bool verify(const uint8_t* msg, size_t msg_len,
                       const uint8_t* sig,
                       const uint8_t* pub);

    // Sign message (requires private key - only for ground station)
    static void sign(const uint8_t* msg, size_t msg_len,
                     const uint8_t* priv,
                     uint8_t* sig);

    // Generate keypair (for ground station)
    static void generate_keypair(uint8_t* pub, uint8_t* priv);

    // Verify firmware image signature
    static bool verify_firmware(const uint8_t* image, size_t len,
                                const uint8_t* sig,
                                const uint8_t* pub) {
        return verify(image, len, sig, pub);
    }

    // Verify OTA bundle signature
    static bool verify_ota_bundle(const uint8_t* bundle, size_t len,
                                  const uint8_t* sig,
                                  const uint8_t* pub) {
        return verify(bundle, len, sig, pub);
    }
};

// Key storage (platform-specific)
// In firmware: keys stored in secure flash region
// In companion: keys stored in filesystem with permissions

class KeyStore {
public:
    // Get public key for verification (firmware: from secure flash)
    static bool get_ota_public_key(uint8_t* pub_out);

    // Get device identity key (for MAVLink signing)
    static bool get_identity_key(uint8_t* pub_out, uint8_t* priv_out);

    // Store new OTA public key (ground station only)
    static bool set_ota_public_key(const uint8_t* pub);
};

} // namespace swarm::crypto

#endif // SWARM_CRYPTO_HPP