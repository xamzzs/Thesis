#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include <Arduino.h>

// Message Structure matching Thesis Section 4.4.3
struct EmergencyMessage {
    uint16_t id;             // 2-byte Message ID
    uint8_t  priority;       // k=1 (Medical SOS), k=2 (General Emergency), k=3 (Routine/Info)
    uint32_t timestamp_tx;   // Creation time at the origin, Unix seconds (carried in the packet; used for latency, not for queue order)
    uint16_t timestamp_ms;   // milliseconds part of the creation time, 0..999 (carried in the packet)
    uint8_t  originNode;     // Node ID (1=V1 Main Hub, 2..5=V2..V5 Mini Hubs, 6=V6 Edge Node)
    uint8_t  destinationNode;
    uint8_t  ttl;
    uint8_t  packetType;
    uint32_t expiresAtMs;
    uint32_t nextRetryAtMs;
    uint8_t  retryExponent;
    uint16_t relayPath;      // relays this message passed through (packet_codec.h encoding)
    char     location[84];   // Landmark/location string
    char     payload[201];   // Emergency message text (<=200 UTF-8 bytes)
    uint8_t  encryptedLen;   // Length of authenticated encrypted packet (IV + ciphertext + HMAC)
    uint8_t  encryptedData[256];
    // Local queue-order keys, set by PriorityMinHeap::push() on THIS node.
    // Both come from this node's own clock/counter, so every message in the
    // queue is ordered on one time source (phone time and origin clocks are
    // never compared against each other).
    uint32_t enqueuedAtMs;   // millis() when this node accepted the message
    uint32_t arrivalSeq;     // strictly increasing per node; final tie-breaker
    // Logging only (CSV, Section 4.5.2 / 4.7):
    uint8_t  queuePosAtEntry; // messages that would be sent before this one when it entered the queue
    uint32_t dtnStoredAtMs;   // millis() when it entered the DTN buffer (0 = never)
};

// Queue order (Section 3.2, extended): primary = priority k (1 = most urgent),
// secondary = local enqueue time, tertiary = local arrival counter.
// Strict comparisons only: equal-priority messages leave in arrival order (FIFO).
inline bool messageOutranks(const EmergencyMessage &a, const EmergencyMessage &b) {
    if (a.priority != b.priority) return a.priority < b.priority;
    const int32_t timeDelta = (int32_t)(a.enqueuedAtMs - b.enqueuedAtMs);   // millis() wrap-safe
    if (timeDelta != 0) return timeDelta < 0;
    return (int32_t)(a.arrivalSeq - b.arrivalSeq) < 0;
}

// Dual-Key Binary Min-Heap Priority Queue (Section 3.2)
// Memory-deterministic static array allocated in SRAM
class PriorityMinHeap {
private:
    static const size_t MAX_CAPACITY = 32; // Deterministic static capacity for ESP32 SRAM
    EmergencyMessage heap[MAX_CAPACITY];
    size_t currentSize;
    uint32_t nextArrivalSeq = 1;

    inline int parent(int i) const { return (i - 1) / 2; }
    inline int leftChild(int i) const { return 2 * i + 1; }
    inline int rightChild(int i) const { return 2 * i + 2; }

    // True if a must be closer to the root than b (see messageOutranks).
    bool isHigherPriority(const EmergencyMessage &a, const EmergencyMessage &b) const {
        return messageOutranks(a, b);
    }

    void swapNodes(int i, int j) {
        EmergencyMessage temp = heap[i];
        heap[i] = heap[j];
        heap[j] = temp;
    }

    void heapifyUp(int index) {
        while (index > 0 && isHigherPriority(heap[index], heap[parent(index)])) {
            swapNodes(index, parent(index));
            index = parent(index);
        }
    }

    void heapifyDown(int index) {
        int smallest = index;
        int left = leftChild(index);
        int right = rightChild(index);

        if (left < (int)currentSize && isHigherPriority(heap[left], heap[smallest])) {
            smallest = left;
        }
        if (right < (int)currentSize && isHigherPriority(heap[right], heap[smallest])) {
            smallest = right;
        }

        if (smallest != index) {
            swapNodes(index, smallest);
            heapifyDown(smallest);
        }
    }

public:
    PriorityMinHeap() : currentSize(0) {}

    size_t size() const { return currentSize; }
    bool isEmpty() const { return currentSize == 0; }
    bool isFull() const { return currentSize >= MAX_CAPACITY; }

    // Insertion: O(log n). Stamps the local ordering keys on the stored copy.
    // Returns PUSH_OK, PUSH_EVICTED (evictedOut holds the dropped message) or
    // PUSH_REJECTED (queue full of equal/more urgent messages). The caller logs it.
    enum PushResult { PUSH_OK, PUSH_EVICTED, PUSH_REJECTED };

    PushResult push(const EmergencyMessage &incoming, EmergencyMessage &evictedOut) {
        EmergencyMessage msg = incoming;
        msg.enqueuedAtMs = millis();
        msg.arrivalSeq = nextArrivalSeq++;
        size_t before = 0;                       // queue position on entry (send order)
        for (size_t i = 0; i < currentSize; i++) if (messageOutranks(heap[i], msg)) before++;
        msg.queuePosAtEntry = before > 255 ? 255 : (uint8_t)before;

        if (isFull()) {
            // Eviction policy (Section 3.3): drop the least urgent message, i.e. the
            // highest k, oldest first, but only if it is strictly less urgent than
            // the incoming one.
            int victim = -1;
            for (size_t i = 0; i < currentSize; i++) {
                if (heap[i].priority <= msg.priority) continue;
                if (victim < 0 || heap[i].priority > heap[victim].priority ||
                    (heap[i].priority == heap[victim].priority &&
                     messageOutranks(heap[i], heap[victim]))) {
                    victim = (int)i;
                }
            }
            if (victim < 0) return PUSH_REJECTED;
            evictedOut = heap[victim];
            heap[victim] = msg;
            heapifyUp(victim);
            heapifyDown(victim);
            return PUSH_EVICTED;
        }

        heap[currentSize] = msg;
        heapifyUp(currentSize);
        currentSize++;
        return PUSH_OK;
    }

    static size_t capacity() { return MAX_CAPACITY; }

    // PQA support: compares every other waiting message with m.
    //  ahead    = waiting messages that should leave before m (outrank it).
    //             A correct priority order gives ahead == 0 when m is sent.
    //  bypassed = waiting messages that arrived EARLIER but are LESS urgent,
    //             i.e. the lower-priority traffic m is overtaking.
    void countRelativeTo(const EmergencyMessage &m, size_t &ahead, size_t &bypassed) const {
        for (size_t i = 0; i < currentSize; i++) {
            const EmergencyMessage &other = heap[i];
            if (other.id == m.id && other.originNode == m.originNode) continue;
            if (messageOutranks(other, m)) ahead++;
            else if (other.priority > m.priority && (int32_t)(other.arrivalSeq - m.arrivalSeq) < 0) bypassed++;
        }
    }

    // Extraction: O(log n) - Root index 0 always contains the highest priority element
    bool pop(EmergencyMessage &outMsg) {
        if (isEmpty()) return false;

        outMsg = heap[0];
        heap[0] = heap[currentSize - 1];
        currentSize--;

        if (currentSize > 0) {
            heapifyDown(0);
        }
        return true;
    }

    // Message IDs are only unique per origin node (every node runs its own
    // counter), so a message is identified by the (originNode, id) pair.
    bool removeById(uint16_t id, uint8_t originNode) {
        for (size_t i = 0; i < currentSize; i++) {
            if (heap[i].id != id || heap[i].originNode != originNode) continue;

            currentSize--;
            if (i < currentSize) {
                heap[i] = heap[currentSize];
                if (i > 0 && isHigherPriority(heap[i], heap[parent((int)i)])) {
                    heapifyUp((int)i);
                } else {
                    heapifyDown((int)i);
                }
            }
            return true;
        }
        return false;
    }

    // Peek root without removing
    bool peek(EmergencyMessage &outMsg) const {
        if (isEmpty()) return false;
        outMsg = heap[0];
        return true;
    }

    void clear() {
        currentSize = 0;
    }
};

#endif
