#ifndef LINK_STATE_H
#define LINK_STATE_H

// Link-state routing support (Section 3.1).
//
// 1. Each node measures its one-hop links: an averaged RSSI/SNR per neighbour,
//    updated on every authenticated frame heard from it, turned into an 8-bit
//    edge weight w(u,v) by linkWeightFromQuality().
// 2. Link-State Update: each heartbeat carries the sender's own row of weights
//    plus one row learned from another node (rotating), so every row spreads
//    through the mesh without extra transmissions.
// 3. LinkStateDb keeps the newest row from every node (by heartbeat sequence)
//    and builds the global 6x6 weighted adjacency matrix (Table 3.1) that
//    Dijkstra runs on.
//
// Pure byte/number handling: unit-testable on a PC.

#include <stdint.h>
#include <string.h>
#include "network_core.h"
#include "packet_codec.h"

static_assert(LS_ROW_WIRE_SIZE == NETWORK_NODE_COUNT - 1,
              "Heartbeat row size must match the node count");
static_assert(NETWORK_NODE_COUNT <= 7, "Relay path encoding stores node IDs in 3 bits");

// ---------------------------------------------------------------------------
// Baseline edge weight (Section 3.1: "the ESP32 extracts the physical radio
// metrics to compute the baseline edge weight w(u,v)").
//   w = 1 + max(0, SNR_GOOD - SNR) / SNR_STEP + max(0, RSSI_GOOD - RSSI) / RSSI_STEP
// A strong link (SNR >= +5 dB, RSSI >= -90 dBm) costs 1, like a plain hop.
// Each 2 dB of SNR below +5 dB and each 5 dB of RSSI below -90 dBm adds 1.
// Example: SNR -5 dB, RSSI -110 dBm -> 1 + 5 + 4 = 10.
// SF12 can still decode down to about -20 dB SNR, so weak links stay usable,
// just expensive. Tune these constants after field profiling.
// ---------------------------------------------------------------------------
constexpr float LINK_SNR_GOOD_DB = 5.0f;
constexpr float LINK_SNR_STEP_DB = 2.0f;
constexpr float LINK_RSSI_GOOD_DBM = -90.0f;
constexpr float LINK_RSSI_STEP_DB = 5.0f;

inline uint8_t linkWeightFromQuality(float rssiDbm, float snrDb) {
    float weight = 1.0f;
    if (snrDb < LINK_SNR_GOOD_DB) weight += (LINK_SNR_GOOD_DB - snrDb) / LINK_SNR_STEP_DB;
    if (rssiDbm < LINK_RSSI_GOOD_DBM) weight += (LINK_RSSI_GOOD_DBM - rssiDbm) / LINK_RSSI_STEP_DB;
    const int rounded = (int)(weight + 0.5f);
    if (rounded < 1) return 1;
    if (rounded > LINK_WEIGHT_MAX) return LINK_WEIGHT_MAX;
    return (uint8_t)rounded;
}

// Exponentially weighted average of a link's RSSI/SNR (smooths fading).
constexpr float LINK_QUALITY_ALPHA = 0.3f;

struct LinkQuality {
    bool valid;
    float rssiAvg;
    float snrAvg;
    uint32_t lastFrameMs;
};

inline void linkQualityAddSample(LinkQuality &q, float rssi, float snr, uint32_t now) {
    if (!q.valid) {
        q.rssiAvg = rssi;
        q.snrAvg = snr;
        q.valid = true;
    } else {
        q.rssiAvg += LINK_QUALITY_ALPHA * (rssi - q.rssiAvg);
        q.snrAvg += LINK_QUALITY_ALPHA * (snr - q.snrAvg);
    }
    q.lastFrameMs = now;
}

// --- Row <-> wire format: weights to all OTHER nodes, in node-ID order ---
inline void packLinkRow(uint8_t ownerNode, const uint8_t row[NETWORK_NODE_COUNT], uint8_t *wire) {
    uint8_t j = 0;
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        if (node == ownerNode) continue;
        wire[j++] = row[node - 1];
    }
}

inline void unpackLinkRow(uint8_t ownerNode, const uint8_t *wire, uint8_t row[NETWORK_NODE_COUNT]) {
    uint8_t j = 0;
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        if (node == ownerNode) { row[node - 1] = 0; continue; }
        row[node - 1] = wire[j++];
    }
}

inline bool sequenceNewer(uint16_t candidate, uint16_t current) {
    return (int16_t)(candidate - current) > 0;   // wrap-safe
}

// ---------------------------------------------------------------------------
// Link-state database: newest row per node.
// A row is "fresh" while its owner's sequence keeps advancing; relays re-sending
// an old row do not refresh it, so a dead node's row ages out.
// ---------------------------------------------------------------------------
class LinkStateDb {
public:
    struct Row {
        bool valid;
        uint16_t sequence;
        uint32_t lastAdvanceMs;     // local time the sequence last increased
        uint8_t weight[NETWORK_NODE_COUNT];
    };

private:
    Row rows[NETWORK_NODE_COUNT];
    uint8_t selfNode = 0;
    uint8_t relayCursor = 0;

public:
    void begin(uint8_t self) {
        selfNode = self;
        relayCursor = 0;
        for (uint8_t i = 0; i < NETWORK_NODE_COUNT; i++) {
            rows[i].valid = false;
            rows[i].sequence = 0;
            rows[i].lastAdvanceMs = 0;
            memset(rows[i].weight, LINK_WEIGHT_NONE, sizeof(rows[i].weight));
        }
    }

    // Stores a row if it is newer than what we have. Returns true if it changed.
    bool update(uint8_t ownerNode, uint16_t sequence, const uint8_t weight[NETWORK_NODE_COUNT], uint32_t now) {
        if (ownerNode < 1 || ownerNode > NETWORK_NODE_COUNT || ownerNode == selfNode) return false;
        Row &row = rows[ownerNode - 1];
        if (row.valid && !sequenceNewer(sequence, row.sequence)) return false;
        row.valid = true;
        row.sequence = sequence;
        row.lastAdvanceMs = now;
        memcpy(row.weight, weight, sizeof(row.weight));
        row.weight[ownerNode - 1] = 0;
        return true;
    }

    bool isFresh(uint8_t ownerNode, uint32_t now, uint32_t maxAgeMs) const {
        if (ownerNode < 1 || ownerNode > NETWORK_NODE_COUNT) return false;
        const Row &row = rows[ownerNode - 1];
        return row.valid && (uint32_t)(now - row.lastAdvanceMs) <= maxAgeMs;
    }

    const Row &row(uint8_t ownerNode) const { return rows[ownerNode - 1]; }

    // Round-robin choice of the next fresh foreign row to relay in a heartbeat.
    bool nextRowToRelay(uint32_t now, uint32_t maxAgeMs, uint8_t &ownerNode) {
        for (uint8_t tries = 0; tries < NETWORK_NODE_COUNT; tries++) {
            relayCursor = (uint8_t)((relayCursor % NETWORK_NODE_COUNT) + 1);
            if (relayCursor != selfNode && isFresh(relayCursor, now, maxAgeMs)) {
                ownerNode = relayCursor;
                return true;
            }
        }
        return false;
    }

    // Global weighted adjacency matrix (Table 3.1). For each link u-v the two
    // ends' measurements are combined: if both rows are known the larger
    // (worse) weight is used, because a hop needs both directions (data + ACK);
    // if only one end is known its weight is used. allowedLinks (0/1) limits
    // routing to the planned testbed topology when useAllowedLinks is true.
    void buildGraph(const uint8_t ownRow[NETWORK_NODE_COUNT],
                    const uint8_t allowedLinks[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT],
                    bool useAllowedLinks, uint32_t now, uint32_t maxAgeMs,
                    uint8_t graph[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT]) const {
        for (uint8_t u = 0; u < NETWORK_NODE_COUNT; u++) {
            for (uint8_t v = 0; v < NETWORK_NODE_COUNT; v++) graph[u][v] = (u == v) ? 0 : LINK_WEIGHT_NONE;
        }
        for (uint8_t u = 0; u < NETWORK_NODE_COUNT; u++) {
            for (uint8_t v = (uint8_t)(u + 1); v < NETWORK_NODE_COUNT; v++) {
                if (useAllowedLinks && !(allowedLinks[u][v] && allowedLinks[v][u])) continue;
                const bool knownU = (u + 1 == selfNode) || isFresh(u + 1, now, maxAgeMs);
                const bool knownV = (v + 1 == selfNode) || isFresh(v + 1, now, maxAgeMs);
                const uint8_t a = knownU ? (u + 1 == selfNode ? ownRow[v] : rows[u].weight[v]) : 0;
                const uint8_t b = knownV ? (v + 1 == selfNode ? ownRow[u] : rows[v].weight[u]) : 0;
                uint8_t weight = LINK_WEIGHT_NONE;
                if (knownU && knownV) weight = a > b ? a : b;
                else if (knownU) weight = a;
                else if (knownV) weight = b;
                if (weight == 0) weight = LINK_WEIGHT_NONE;
                graph[u][v] = weight;
                graph[v][u] = weight;
            }
        }
    }
};

#endif
