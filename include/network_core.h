#ifndef NETWORK_CORE_H
#define NETWORK_CORE_H

#include <Arduino.h>
#include "min_heap.h"

constexpr uint8_t NETWORK_NODE_COUNT = 6;
constexpr size_t DTN_BUFFER_MAX_BYTES = 8192;

// TTL_max (Section 3.3): how long a message stays actionable for responders.
// One value for the priority queue and the DTN buffer: 72 hours. Counted from
// when THIS node accepted the message (node clocks are not synchronized).
// 72 h = 259,200,000 ms, well inside millis()' 49.7-day wrap-safe range.
constexpr uint32_t DTN_RETENTION_MS = 72UL * 60UL * 60UL * 1000UL;

class DtnBuffer {
private:
    struct Entry {
        EmergencyMessage message;
        bool occupied;
    };

    static constexpr size_t MAX_ENTRIES =
        (DTN_BUFFER_MAX_BYTES - sizeof(size_t)) / sizeof(Entry);
    Entry entries[MAX_ENTRIES];
    size_t usedEntries;

    static bool deadlineReached(uint32_t now, uint32_t deadline) {
        return (int32_t)(now - deadline) >= 0;
    }

public:
    static constexpr size_t B_MAX_BYTES = DTN_BUFFER_MAX_BYTES;

    DtnBuffer() : usedEntries(0) {
        for (size_t i = 0; i < MAX_ENTRIES; i++) entries[i].occupied = false;
    }

    bool store(const EmergencyMessage &message, uint32_t now) {
        purgeExpired(now);
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (entries[i].occupied && entries[i].message.id == message.id &&
                entries[i].message.originNode == message.originNode) {
                return true;
            }
        }
        if (usedEntries >= MAX_ENTRIES) return false;

        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (entries[i].occupied) continue;
            entries[i].message = message;
            if (entries[i].message.expiresAtMs == 0) {
                entries[i].message.expiresAtMs = now + DTN_RETENTION_MS;
            }
            entries[i].occupied = true;
            usedEntries++;
            return true;
        }
        return false;
    }

    bool peekDue(uint32_t now, EmergencyMessage &message) {
        purgeExpired(now);
        int selected = -1;
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (!entries[i].occupied ||
                !deadlineReached(now, entries[i].message.nextRetryAtMs)) continue;
            if (selected < 0 || messageOutranks(entries[i].message, entries[selected].message)) {
                selected = (int)i;
            }
        }
        if (selected < 0) return false;
        message = entries[selected].message;
        return true;
    }

    bool defer(uint16_t id, uint8_t originNode, uint32_t nextAttemptAtMs, uint8_t retryExponent) {
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (!entries[i].occupied || entries[i].message.id != id ||
                entries[i].message.originNode != originNode) continue;
            entries[i].message.nextRetryAtMs = nextAttemptAtMs;
            entries[i].message.retryExponent = retryExponent;
            return true;
        }
        return false;
    }

    bool remove(uint16_t id, uint8_t originNode) {
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (!entries[i].occupied || entries[i].message.id != id ||
                entries[i].message.originNode != originNode) continue;
            entries[i].occupied = false;
            usedEntries--;
            return true;
        }
        return false;
    }

    size_t purgeExpired(uint32_t now) {
        size_t purged = 0;
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (entries[i].occupied && deadlineReached(now, entries[i].message.expiresAtMs)) {
                entries[i].occupied = false;
                usedEntries--;
                purged++;
            }
        }
        return purged;
    }

    // Link recovered: make every buffered message eligible for an immediate retry.
    size_t makeAllDue(uint32_t now) {
        size_t count = 0;
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (!entries[i].occupied) continue;
            entries[i].message.nextRetryAtMs = now;
            count++;
        }
        return count;
    }

    // Priority-aware store (Section 3.3): when the buffer is full, the
    // lowest-priority entry (highest k, oldest first) is evicted, but only if it
    // is strictly less urgent than the incoming message.
    bool storeWithEviction(const EmergencyMessage &message, uint32_t now,
                           bool &evicted, EmergencyMessage &evictedMessage) {
        evicted = false;
        if (store(message, now)) return true;

        int victim = -1;
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (!entries[i].occupied || entries[i].message.priority <= message.priority) continue;
            if (victim < 0 ||
                entries[i].message.priority > entries[victim].message.priority ||
                (entries[i].message.priority == entries[victim].message.priority &&
                 messageOutranks(entries[i].message, entries[victim].message))) {
                victim = (int)i;
            }
        }
        if (victim < 0) return false;

        evictedMessage = entries[victim].message;
        evicted = true;
        entries[victim].message = message;
        if (entries[victim].message.expiresAtMs == 0) {
            entries[victim].message.expiresAtMs = now + DTN_RETENTION_MS;
        }
        return true;
    }

    size_t size() const { return usedEntries; }

    // PQA support: compares every other waiting message with m.
    //  ahead    = waiting messages that should leave before m (outrank it).
    //             A correct priority order gives ahead == 0 when m is sent.
    //  bypassed = waiting messages that arrived EARLIER but are LESS urgent,
    //             i.e. the lower-priority traffic m is overtaking.
    // Only packets whose retry time has come are counted: a packet held for its
    // DTN poll (no route) cannot be sent, so it is not "waiting ahead".
    void countRelativeTo(const EmergencyMessage &m, uint32_t now, size_t &ahead, size_t &bypassed) const {
        for (size_t i = 0; i < MAX_ENTRIES; i++) {
            if (!entries[i].occupied || !deadlineReached(now, entries[i].message.nextRetryAtMs)) continue;
            const EmergencyMessage &other = entries[i].message;
            if (other.id == m.id && other.originNode == m.originNode) continue;
            if (messageOutranks(other, m)) ahead++;
            else if (other.priority > m.priority && (int32_t)(other.arrivalSeq - m.arrivalSeq) < 0) bypassed++;
        }
    }
    size_t usedBytes() const { return usedEntries * sizeof(Entry); }
};

static_assert(sizeof(DtnBuffer) <= DtnBuffer::B_MAX_BYTES,
              "DTN buffer object exceeds its configured SRAM ceiling");

// Edge weights (Section 3.1, Table 3.1): 8-bit, 1..254 = link penalty,
// LINK_WEIGHT_NONE (255) = no edge. 0 is also treated as "no edge" so the 0/1
// topology mask can be passed directly. Path costs are summed in 16-bit.
constexpr uint8_t LINK_WEIGHT_NONE = 255;
constexpr uint8_t LINK_WEIGHT_MAX = 254;

// Dijkstra (Section 3.1): d(v) = min(d(v), d(u) + w(u,v)).
// Returns the first hop of the minimum-cost path; pathCost (optional) gets the total.
inline bool dijkstraNextHop(const uint8_t graph[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT],
                            uint8_t sourceNode, uint8_t destinationNode, uint8_t &nextHopNode,
                            uint16_t *pathCost = nullptr) {
    if (sourceNode < 1 || sourceNode > NETWORK_NODE_COUNT ||
        destinationNode < 1 || destinationNode > NETWORK_NODE_COUNT) return false;
    if (sourceNode == destinationNode) {
        nextHopNode = sourceNode;
        if (pathCost) *pathCost = 0;
        return true;
    }

    constexpr uint16_t INFINITY_COST = 0xFFFF;
    const uint8_t source = sourceNode - 1;
    const uint8_t destination = destinationNode - 1;
    uint16_t distance[NETWORK_NODE_COUNT];
    uint8_t previous[NETWORK_NODE_COUNT];
    bool visited[NETWORK_NODE_COUNT] = {};
    for (uint8_t i = 0; i < NETWORK_NODE_COUNT; i++) {
        distance[i] = INFINITY_COST;
        previous[i] = 0xFF;
    }
    distance[source] = 0;

    for (uint8_t step = 0; step < NETWORK_NODE_COUNT; step++) {
        int current = -1;
        for (uint8_t i = 0; i < NETWORK_NODE_COUNT; i++) {
            if (!visited[i] && distance[i] != INFINITY_COST &&
                (current < 0 || distance[i] < distance[current])) current = i;
        }
        if (current < 0 || current == destination) break;
        visited[current] = true;

        for (uint8_t neighbor = 0; neighbor < NETWORK_NODE_COUNT; neighbor++) {
            const uint8_t edgeCost = graph[current][neighbor];
            if (neighbor == current || edgeCost == 0 || edgeCost == LINK_WEIGHT_NONE ||
                visited[neighbor]) continue;
            const uint16_t candidate = (uint16_t)(distance[current] + edgeCost);   // max 5 x 254: no overflow
            if (candidate < distance[neighbor]) {
                distance[neighbor] = candidate;
                previous[neighbor] = (uint8_t)current;
            }
        }
    }

    if (distance[destination] == INFINITY_COST) return false;
    uint8_t hop = destination;
    while (previous[hop] != source) {
        if (previous[hop] == 0xFF) return false;
        hop = previous[hop];
    }
    nextHopNode = hop + 1;
    if (pathCost) *pathCost = distance[destination];
    return true;
}

#endif