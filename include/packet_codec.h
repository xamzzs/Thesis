#ifndef PACKET_CODEC_H
#define PACKET_CODEC_H

// Binary LoRa packet layout (Section 3.5.2 / 4.4.3).
// Everything here is pure byte handling so it can be unit-tested on a PC.
//
// DATA frame (SOS / general emergency):
//   [0]      header = (k << 6) | (TTL << 2) | type
//   [1]      sourceNode   (node that transmitted this hop)
//   [2]      nextHopNode  (node that must receive / ACK this hop)
//   [3]      destinationNode
//   [4]      originNode
//   [5..6]   message ID (big-endian), unique per origin node
//   [7]      MAC busy retries for this transmission
//   [8..9]   relay path (big-endian): relays in order, 3 bits each, slot 0 in the
//            lowest bits, 0 = empty. Each relay appends itself before forwarding,
//            so the hub knows the route the packet ACTUALLY took. Bit 15 = overflow.
//   [10..]   encrypted body = 16-byte IV + AES-128-CBC ciphertext
//   [last16] frame tag = HMAC-SHA-256 over bytes [0 .. end of ciphertext], truncated to 16 bytes
//
// ACK frame: the first 8 prefix bytes (no relay path) + 16-byte frame tag (24 bytes total).
//
// HEARTBEAT frame (1-hop broadcast, never forwarded, never ACKed), 29 bytes:
//   [0]      header = type HEARTBEAT (priority 0, TTL 0)
//   [1]      sourceNode
//   [2..3]   heartbeat sequence number (big-endian)
//   [4..5]   battery level in 0.1 % units, 0-1000 (big-endian); 0xFFFF = unknown
//   [6]      traffic load: data-transmission backoffs (TBEB) since the previous heartbeat (saturates at 255)
//   [7]      reserved; transmitted as 0 (queue occupancy is not broadcast)
//   --- Link-State Update (Section 3.1) ---
//   [8..12]  sender's OWN one-hop row: edge weight to every other node, in node-ID
//            order skipping the sender (1..254 = cost, 255 = no edge)
//   [13]     relayed row owner (0 = none): one row learned from another node,
//            rotated each heartbeat so every row spreads across the mesh
//   [14..15] relayed row's sequence number (the owner's heartbeat sequence)
//   [16..20] relayed row weights (node-ID order skipping the row owner)
//   [21..28] frame tag = HMAC-SHA-256 over bytes [0..20], truncated to 8 bytes
// 29 bytes is the largest heartbeat with the same SF12 airtime (~1.65 s).
//
// Plaintext message body (encrypted inside a DATA frame):
//   [0..3]   timestamp_tx (uint32, big-endian)
//   [4]      location length L
//   [5..5+L) location bytes (UTF-8, no terminator)
//   [rest]   message bytes  (UTF-8, no terminator)
// Location + message together are limited to MAX_TEXT_BYTES (200 bytes), which
// keeps the worst case frame at 10 + 16 + 208 + 16 = 250 bytes = the LoRa MTU.

#include <stdint.h>
#include <stddef.h>
#include <string.h>

constexpr uint8_t PACKET_TYPE_SOS = 0;
constexpr uint8_t PACKET_TYPE_HEARTBEAT = 1;
constexpr uint8_t PACKET_TYPE_TELEMETRY = 2;   // used for k=2 / k=3 messages
constexpr uint8_t PACKET_TYPE_ACK = 3;

constexpr size_t MAX_LORA_PACKET_SIZE = 250;
constexpr size_t FRAME_PREFIX_SIZE = 10;        // data frames (includes the relay path)
constexpr size_t ACK_PREFIX_SIZE = 8;           // ACK frames carry no relay path
constexpr size_t FRAME_AUTH_TAG_SIZE = 16;
constexpr size_t FRAME_OVERHEAD_SIZE = FRAME_PREFIX_SIZE + FRAME_AUTH_TAG_SIZE;   // 26
constexpr size_t ACK_PACKET_SIZE = ACK_PREFIX_SIZE + FRAME_AUTH_TAG_SIZE;         // 24
constexpr size_t MAX_ENCRYPTED_BODY_SIZE = MAX_LORA_PACKET_SIZE - FRAME_OVERHEAD_SIZE; // 224

constexpr size_t AES_BLOCK_SIZE_BYTES = 16;
constexpr size_t AES_IV_SIZE = 16;

constexpr uint16_t BATTERY_UNKNOWN = 0xFFFF;
constexpr size_t LS_ROW_WIRE_SIZE = 5;          // = node count - 1 (checked in link_state.h)
constexpr size_t HEARTBEAT_BODY_SIZE = 8 + LS_ROW_WIRE_SIZE + 3 + LS_ROW_WIRE_SIZE;   // 21
constexpr size_t HEARTBEAT_TAG_SIZE = 8;
constexpr size_t HEARTBEAT_PACKET_SIZE = HEARTBEAT_BODY_SIZE + HEARTBEAT_TAG_SIZE;     // 29

// Body: Unix seconds (4) + milliseconds 0..999 (2) + location length (1) + location + message
constexpr size_t BODY_FIXED_SIZE = 7;
constexpr size_t MAX_TEXT_BYTES = 200;         // location + message
constexpr size_t MAX_PLAINTEXT_BODY_SIZE = BODY_FIXED_SIZE + MAX_TEXT_BYTES; // 207 -> 224 encrypted

// Size of the encrypted body for a given plaintext length (IV + PKCS#7-padded ciphertext).
constexpr size_t encryptedBodySize(size_t plainLen) {
    return AES_IV_SIZE + (plainLen / AES_BLOCK_SIZE_BYTES + 1) * AES_BLOCK_SIZE_BYTES;
}

static_assert(FRAME_OVERHEAD_SIZE + encryptedBodySize(MAX_PLAINTEXT_BODY_SIZE) <= MAX_LORA_PACKET_SIZE,
              "A full 200-byte text payload must fit in one 250-byte LoRa frame");

// --- Relay path (2 bytes, 5 slots x 3 bits; node IDs 1..7) ---
constexpr uint8_t RELAY_PATH_SLOTS = 5;
constexpr uint16_t RELAY_PATH_OVERFLOW = 0x8000;

inline uint8_t relayPathAt(uint16_t path, uint8_t slot) {
    return slot < RELAY_PATH_SLOTS ? (uint8_t)((path >> (3 * slot)) & 0x07) : 0;
}

// Appends a relay; if all 5 slots are used, sets the overflow bit instead.
inline uint16_t relayPathAppend(uint16_t path, uint8_t nodeId) {
    for (uint8_t slot = 0; slot < RELAY_PATH_SLOTS; slot++) {
        if (relayPathAt(path, slot) == 0) {
            return (uint16_t)(path | ((uint16_t)(nodeId & 0x07) << (3 * slot)));
        }
    }
    return (uint16_t)(path | RELAY_PATH_OVERFLOW);
}

inline uint8_t encodeHeader(uint8_t priority, uint8_t ttl, uint8_t packetType) {
    return (uint8_t)(((priority & 0x03) << 6) | ((ttl & 0x0F) << 2) | (packetType & 0x03));
}

inline uint8_t headerPacketType(uint8_t header) { return header & 0x03; }

// Builds the plaintext body. Fails (returns false) instead of truncating when
// location + message exceed MAX_TEXT_BYTES, so the sender can reject the input.
inline bool buildMessageBody(uint32_t timestamp, uint16_t timestampMs, const char *location,
                             const char *message, uint8_t *out, size_t capacity, size_t &outLen) {
    outLen = 0;
    if (!out) return false;
    const size_t locLen = location ? strlen(location) : 0;
    const size_t msgLen = message ? strlen(message) : 0;
    if (locLen > 255 || locLen + msgLen > MAX_TEXT_BYTES) return false;
    const size_t total = BODY_FIXED_SIZE + locLen + msgLen;
    if (total > capacity) return false;

    out[0] = (uint8_t)(timestamp >> 24);
    out[1] = (uint8_t)(timestamp >> 16);
    out[2] = (uint8_t)(timestamp >> 8);
    out[3] = (uint8_t)(timestamp);
    if (timestampMs > 999) timestampMs = 999;
    out[4] = (uint8_t)(timestampMs >> 8);
    out[5] = (uint8_t)(timestampMs);
    out[6] = (uint8_t)locLen;
    if (locLen) memcpy(out + BODY_FIXED_SIZE, location, locLen);
    if (msgLen) memcpy(out + BODY_FIXED_SIZE + locLen, message, msgLen);
    outLen = total;
    return true;
}

// Parses a decrypted body into NUL-terminated strings. Rejects malformed lengths.
inline bool parseMessageBody(const uint8_t *in, size_t len, uint32_t &timestamp, uint16_t &timestampMs,
                             char *location, size_t locationSize,
                             char *message, size_t messageSize) {
    if (!in || !location || !message || locationSize == 0 || messageSize == 0 ||
        len < BODY_FIXED_SIZE || len > MAX_PLAINTEXT_BODY_SIZE) return false;
    const size_t locLen = in[6];
    if (BODY_FIXED_SIZE + locLen > len) return false;
    const size_t msgLen = len - BODY_FIXED_SIZE - locLen;
    if (locLen + msgLen > MAX_TEXT_BYTES || locLen >= locationSize || msgLen >= messageSize) return false;

    timestamp = ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) |
                ((uint32_t)in[2] << 8) | (uint32_t)in[3];
    timestampMs = (uint16_t)(((uint16_t)in[4] << 8) | in[5]);
    if (timestampMs > 999) return false;
    memcpy(location, in + BODY_FIXED_SIZE, locLen);
    location[locLen] = '\0';
    memcpy(message, in + BODY_FIXED_SIZE + locLen, msgLen);
    message[msgLen] = '\0';
    return true;
}

// Heartbeat body (bytes [0..20]); the caller appends the 8-byte tag.
// ownRowWire / relayedRowWire: LS_ROW_WIRE_SIZE bytes each (see link_state.h).
inline void buildHeartbeatBody(uint8_t sourceNode, uint16_t sequence, uint16_t batteryPermille,
                               uint8_t trafficLoad, const uint8_t *ownRowWire, uint8_t relayedOwner,
                               uint16_t relayedSequence, const uint8_t *relayedRowWire,
                               uint8_t *out) {
    out[0] = encodeHeader(0, 0, PACKET_TYPE_HEARTBEAT);
    out[1] = sourceNode;
    out[2] = (uint8_t)(sequence >> 8);
    out[3] = (uint8_t)(sequence);
    out[4] = (uint8_t)(batteryPermille >> 8);
    out[5] = (uint8_t)(batteryPermille);
    out[6] = trafficLoad;
    out[7] = 0; // reserved for compatibility; do not disclose local queue state
    memcpy(out + 8, ownRowWire, LS_ROW_WIRE_SIZE);
    out[13] = relayedOwner;
    out[14] = (uint8_t)(relayedSequence >> 8);
    out[15] = (uint8_t)(relayedSequence);
    if (relayedOwner && relayedRowWire) memcpy(out + 16, relayedRowWire, LS_ROW_WIRE_SIZE);
    else memset(out + 16, 0xFF, LS_ROW_WIRE_SIZE);
}

struct HeartbeatInfo {
    uint8_t sourceNode;
    uint16_t sequence;
    uint16_t batteryPermille;   // 0.1 % units; BATTERY_UNKNOWN if not available
    uint8_t trafficLoad;
    uint8_t ownRowWire[LS_ROW_WIRE_SIZE];
    uint8_t relayedOwner;       // 0 = no relayed row
    uint16_t relayedSequence;
    uint8_t relayedRowWire[LS_ROW_WIRE_SIZE];
};

inline bool parseHeartbeatBody(const uint8_t *in, size_t len, HeartbeatInfo &info) {
    if (!in || len != HEARTBEAT_PACKET_SIZE ||
        in[0] != encodeHeader(0, 0, PACKET_TYPE_HEARTBEAT)) return false;
    info.sourceNode = in[1];
    info.sequence = ((uint16_t)in[2] << 8) | in[3];
    info.batteryPermille = ((uint16_t)in[4] << 8) | in[5];
    info.trafficLoad = in[6];
    memcpy(info.ownRowWire, in + 8, LS_ROW_WIRE_SIZE);
    info.relayedOwner = in[13];
    info.relayedSequence = ((uint16_t)in[14] << 8) | in[15];
    memcpy(info.relayedRowWire, in + 16, LS_ROW_WIRE_SIZE);
    return true;
}

// Temporal Seen-List (Section 3.3): ring buffer of recently accepted
// (originNode, messageId) pairs used to suppress duplicate forwards.
template <size_t CAPACITY>
class SeenList {
private:
    struct Entry {
        uint8_t originNode;
        uint16_t messageId;
    };
    Entry entries[CAPACITY];
    size_t count = 0;
    size_t writeIndex = 0;

public:
    bool contains(uint8_t originNode, uint16_t messageId) const {
        for (size_t i = 0; i < count; i++) {
            if (entries[i].originNode == originNode && entries[i].messageId == messageId) return true;
        }
        return false;
    }

    void add(uint8_t originNode, uint16_t messageId) {
        if (contains(originNode, messageId)) return;
        entries[writeIndex].originNode = originNode;
        entries[writeIndex].messageId = messageId;
        writeIndex = (writeIndex + 1) % CAPACITY;
        if (count < CAPACITY) count++;
    }

    void clear() { count = 0; writeIndex = 0; }
    size_t size() const { return count; }
};

#endif
