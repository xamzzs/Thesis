#ifndef FIFO_QUEUE_H
#define FIFO_QUEUE_H

// Plain First-In-First-Out message queue for the Baseline configuration
// (Section 4.2.1, Config 1). Same interface as PriorityMinHeap, so the MAC loop
// can use either one (QUEUE_MODE build flag in main.cpp). Messages leave in
// arrival order whatever their priority k; a full queue drops the NEW message
// (tail drop), the conventional behaviour of a LoRa mesh without priorities.

#include <stdint.h>
#include <stddef.h>
#include "min_heap.h"   // EmergencyMessage, messageOutranks()

class FifoQueue {
private:
    static const size_t MAX_CAPACITY = 32;   // same SRAM budget as the min-heap
    EmergencyMessage items[MAX_CAPACITY];    // items[0] = oldest = next to send
    size_t currentSize = 0;
    uint32_t nextArrivalSeq = 1;

public:
    enum PushResult { PUSH_OK, PUSH_EVICTED, PUSH_REJECTED };

    size_t size() const { return currentSize; }
    bool isEmpty() const { return currentSize == 0; }
    bool isFull() const { return currentSize >= MAX_CAPACITY; }
    static size_t capacity() { return MAX_CAPACITY; }

    // O(1). Never evicts: a full FIFO rejects the newcomer.
    PushResult push(const EmergencyMessage &incoming, EmergencyMessage & /*evictedOut*/) {
        if (isFull()) return PUSH_REJECTED;
        EmergencyMessage msg = incoming;
        msg.enqueuedAtMs = millis();
        msg.arrivalSeq = nextArrivalSeq++;
        msg.queuePosAtEntry = (uint8_t)currentSize;   // everything already queued leaves first
        items[currentSize++] = msg;
        return PUSH_OK;
    }

    bool peek(EmergencyMessage &outMsg) const {
        if (isEmpty()) return false;
        outMsg = items[0];
        return true;
    }

    bool pop(EmergencyMessage &outMsg) {
        if (!peek(outMsg)) return false;
        return removeAt(0);
    }

    // A message is identified by (originNode, id): IDs are only unique per origin.
    bool removeById(uint16_t id, uint8_t originNode) {
        for (size_t i = 0; i < currentSize; i++) {
            if (items[i].id == id && items[i].originNode == originNode) return removeAt(i);
        }
        return false;
    }

    // PQA support, same definitions as PriorityMinHeap::countRelativeTo():
    //  ahead    = waiting messages that should leave before m by priority (k, then age).
    //             In a FIFO a k = 1 message only leaves from the front, so its own row
    //             shows ahead = 0; the inversion appears on the rows of the less urgent
    //             messages sent while it waited (ahead > 0 there). PQA analysis must
    //             therefore look at every data TX row, not only the k = 1 rows.
    //  bypassed = earlier, less urgent messages m overtakes (always 0 in a FIFO,
    //             because m only leaves when everything older has left).
    void countRelativeTo(const EmergencyMessage &m, size_t &ahead, size_t &bypassed) const {
        for (size_t i = 0; i < currentSize; i++) {
            const EmergencyMessage &other = items[i];
            if (other.id == m.id && other.originNode == m.originNode) continue;
            if (messageOutranks(other, m)) ahead++;
            else if (other.priority > m.priority && (int32_t)(other.arrivalSeq - m.arrivalSeq) < 0) bypassed++;
        }
    }

    void clear() { currentSize = 0; }

private:
    bool removeAt(size_t index) {
        if (index >= currentSize) return false;
        for (size_t i = index + 1; i < currentSize; i++) items[i - 1] = items[i];
        currentSize--;
        return true;
    }
};

#endif
