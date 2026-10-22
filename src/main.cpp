#include <Arduino.h>
#include <stdarg.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <RadioLib.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "captive_portal.h"
#include "admin_dashboard.h"
#include "aes_crypto.h"
#include "min_heap.h"
#include "network_core.h"
#include "packet_codec.h"
#include "link_state.h"
#include "fifo_queue.h"

// ---------------------------------------------------------------------------
// Experimental configuration (Section 4.2.1). Set per build in platformio.ini
// (environments baseline-nN, partial-nN, heuristic-nN, ablation-nN, full-nN);
// the plain nodeN environments keep the defaults below.
//   QUEUE_MODE_CFG     QUEUE_FIFO | QUEUE_HEAP           (default HEAP)
//   DTN_ENABLED_CFG    0 = drop on link failure | 1 = DTN store-and-forward (default 1)
//   RELAY_FILTER_CFG   RELAY_FILTER_NONE | _HEURISTIC | _CART   (default NONE)
//   CONFIG_NAME_CFG    label written in the boot log and the CSV CONFIG row
// ---------------------------------------------------------------------------
#define QUEUE_FIFO 0
#define QUEUE_HEAP 1
#ifndef QUEUE_MODE_CFG
#define QUEUE_MODE_CFG QUEUE_HEAP
#endif
#ifndef DTN_ENABLED_CFG
#define DTN_ENABLED_CFG 1
#endif
#ifndef CONFIG_NAME_CFG
#define CONFIG_NAME_CFG "dev"
#endif
#if QUEUE_MODE_CFG == QUEUE_FIFO
using MessageQueue = FifoQueue;              // Config 1 (Baseline)
#else
using MessageQueue = PriorityMinHeap;        // Configs 2-5
#endif
constexpr bool DTN_ENABLED = DTN_ENABLED_CFG != 0;
// Without DTN a message whose hop fails is retried right away (another route if
// one exists) at most this many times in total, then dropped ("drop on link failure").
constexpr uint8_t NO_DTN_MAX_ATTEMPTS = 3;

// ---------------------------------------------------------
// Network & Node Configuration (Barangay San Miguel Testbed)
// ---------------------------------------------------------
// Node identity comes from platformio.ini ([env:node1] ... [env:node6]).
// Pick the env for the board you are flashing; do not edit these lines.
#if !defined(NODE_ID_CFG) || !defined(AP_SSID_CFG)
#error "Build with a node environment from platformio.ini, e.g. pio run -e node3 -t upload"
#endif
const char *AP_SSID = AP_SSID_CFG;
const byte DNS_PORT = 53;
// Phones allowed on each node's Wi-Fi at once (Section 1.1 Limitations says
// "about 25"). The ESP32 Wi-Fi driver has its own hard cap, ESP_WIFI_MAX_CONN_NUM
// (10 or 15 depending on the ESP-IDF version), and a larger request is not
// honoured, so the effective cap is the smaller of the two. setup() prints the
// value the driver actually accepted.
constexpr uint8_t AP_CLIENTS_REQUESTED = 25;
#ifdef ESP_WIFI_MAX_CONN_NUM
constexpr uint8_t AP_CLIENTS_DRIVER_MAX = ESP_WIFI_MAX_CONN_NUM;
#else
constexpr uint8_t AP_CLIENTS_DRIVER_MAX = 10;
#endif
const uint8_t MAX_AP_CLIENTS = AP_CLIENTS_REQUESTED < AP_CLIENTS_DRIVER_MAX ? AP_CLIENTS_REQUESTED
                                                                           : AP_CLIENTS_DRIVER_MAX;

// /api/submit body: a JSON form of at most ~260 bytes of text plus field names.
// Phones may send it in several TCP pieces; collectSubmitBody() joins them.
constexpr size_t SUBMIT_BODY_MAX = 1024;
struct SubmitBody {
    size_t total;
    size_t received;
    char data[];
};

IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

DNSServer dnsServer;
AsyncWebServer server(80);

// Message queue: Dual-Key Binary Min-Heap (Section 3.2), or FIFO for the Baseline.
MessageQueue priorityQueue;
SemaphoreHandle_t priorityQueueMutex = nullptr;
SemaphoreHandle_t liveIncidentMutex = nullptr;
constexpr size_t LIVE_INCIDENT_CAPACITY = 20;   // data messages only (heartbeats never stored)

struct LiveIncident {
    uint16_t messageId;
    uint8_t priority;
    uint8_t packetType;
    uint8_t originNode;
    int32_t latencyMs;
    uint32_t receivedAtMs;
    uint32_t arrivalNo;      // 1, 2, 3 ... in the order this node received messages
    char path[64];
    char location[84];
    char message[201];
};

LiveIncident liveIncidents[LIVE_INCIDENT_CAPACITY] = {};
size_t liveIncidentWriteIndex = 0;
size_t liveIncidentCount = 0;
uint32_t liveIncidentArrivalCounter = 0;

volatile uint16_t globalMessageCounter = 1000;
portMUX_TYPE messageCounterMux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint8_t LORA_NSS_PIN = 8;
constexpr uint8_t LORA_DIO1_PIN = 14;
constexpr uint8_t LORA_RESET_PIN = 12;
constexpr uint8_t LORA_BUSY_PIN = 13;
constexpr uint8_t LORA_SCK_PIN = 9;
constexpr uint8_t LORA_MISO_PIN = 11;
constexpr uint8_t LORA_MOSI_PIN = 10;
// Radio regulatory settings - confirm with NTC before field deployment.
// 915.0 MHz matches the team's 915 MHz antennas and the thesis (Chapter 1, 3.5).
// Note: Meshtastic lists the Philippine band as 915.0-918.0 MHz (PH_915, 24 dBm).
// With a 125 kHz bandwidth, a 915.0 MHz centre occupies 914.9375-915.0625 MHz,
// so the lower half of the channel sits just below that band edge.
constexpr float LORA_FREQUENCY_MHZ = 915.0f;
constexpr int8_t REGULATORY_LIMIT_DBM = 24;   // treated as EIRP to stay on the safe side
constexpr int8_t ANTENNA_GAIN_DBI = 5;        // worst case of the 3-5 dBi antennas (Table 4.5.1)
constexpr int8_t SX1262_MAX_POWER_DBM = 22;
constexpr int8_t LORA_TX_POWER_DBM =
    (REGULATORY_LIMIT_DBM - ANTENNA_GAIN_DBI) < SX1262_MAX_POWER_DBM
        ? (REGULATORY_LIMIT_DBM - ANTENNA_GAIN_DBI) : SX1262_MAX_POWER_DBM;   // 19 dBm
// Frame sizes, packet types and the binary layout live in packet_codec.h.
constexpr uint8_t INITIAL_TTL = 15;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 30000UL;
constexpr uint32_t HEARTBEAT_JITTER_MS = 5000UL;        // each interval is 30 s +/- 2.5 s
constexpr uint8_t HEARTBEAT_MAX_CAD_ATTEMPTS = 3;      // skip this heartbeat if the channel stays busy
constexpr uint32_t HEARTBEAT_CAD_RETRY_MAX_MS = 2000UL;
constexpr uint8_t BROADCAST_NODE_ID = 0xFF;
constexpr unsigned long MIN_VALID_EPOCH = 1700000000UL;   // clock counts as "set" after Nov 2023
constexpr size_t SEEN_LIST_CAPACITY = 64;
constexpr uint8_t BATTERY_ADC_PIN = 1;
constexpr uint8_t BATTERY_ADC_ENABLE_PIN = 37;
constexpr float BATTERY_DIVIDER_RATIO = 4.9f;      // 390k / 100k divider on the Heltec V3
// Per-board calibration factor (multimeter / ADC reading), set with the serial
// command "battery cal <volts measured with a multimeter>" and kept in flash.
// 1.0 = uncalibrated. Allowed range 0.80-1.20.
float batteryCalibration = 1.0f;
constexpr uint32_t BATTERY_LOG_INTERVAL_MS = 60000UL;   // BATT row every minute (energy metric)
// Where the battery level comes from:
//   BATTERY_SOURCE_LIPO     - CURRENT SETUP (decision D1, Oct 17): LiPo on the JST
//       connector, measured on VBAT. Heartbeats carry the level (CART battery
//       feature) and a BATT row logs the voltage every minute (R_drop, 4.7 metric 5).
//       A node on USB power only (no LiPo) reports battery = unknown.
//   BATTERY_SOURCE_NONE     - USB power banks only: battery always unknown.
//   BATTERY_SOURCE_ESTIMATE - parked option for power banks (needs a USB power
//       meter to calibrate). The level is an
//       ESTIMATE: energy used = measured idle/TX currents x time in each state,
//       subtracted from the bank's usable capacity. See the "Energy estimate"
//       section below and the serial "energy" commands.
enum BatterySource { BATTERY_SOURCE_NONE, BATTERY_SOURCE_ESTIMATE, BATTERY_SOURCE_LIPO };
constexpr BatterySource BATTERY_SOURCE = BATTERY_SOURCE_LIPO;
// DTN_RETENTION_MS (72 h, Section 3.3 TTL_max) is defined in network_core.h.
constexpr uint32_t ACK_TIMEOUT_GUARD_MS = 2500UL;
constexpr uint32_t DTN_POLL_BASE_MS = 10000UL;      // first poll 10 s after a route is lost
constexpr uint32_t DTN_POLL_MAX_MS = 60000UL;       // polls back off to once a minute
constexpr uint8_t LINK_DOWN_AFTER_ACK_FAILURES = 2; // consecutive ACK timeouts before rerouting
constexpr uint32_t LINK_DOWN_HOLD_MS = 60000UL;     // retry a DOWN link after this even if unheard
constexpr uint8_t NODE_ID = NODE_ID_CFG;          // set per board in platformio.ini
constexpr uint8_t HUB_NODE_ID = 1;
static_assert(NODE_ID >= 1 && NODE_ID <= NETWORK_NODE_COUNT, "NODE_ID_CFG must be 1..6");

// CSMA/CA with TBEB (Section 3.5.1). SlotTime is not a fixed constant: setup()
// derives it from the radio's time-on-air for the active SF/BW (an ACK frame,
// about 1.5 s at SF12/BW125), so the backoff window scales with the airtime.
constexpr uint8_t MAC_MIN_BACKOFF_EXPONENT = 1;
constexpr uint8_t MAC_MAX_BACKOFF_EXPONENT = 4;   // c_max: Random(0, 15) slots, about 0-22 s at SF12
constexpr uint8_t MAC_MAX_CAD_RETRIES = 6;

// ---------------------------------------------------------------------------
// Link-state routing (Section 3.1)
// adjacencyMatrix is the PLANNED testbed topology (1 = these two nodes may be
// neighbours). With USE_TOPOLOGY_MASK, routing only uses these links even when
// more nodes can hear each other (e.g. all boards on one table), so multi-hop
// paths are exercised. Edge WEIGHTS come from measured RSSI/SNR via the
// heartbeat Link-State Updates (link_state.h), not from this table.
// Set USE_TOPOLOGY_MASK = false in the field to let any measured link be used.
// ---------------------------------------------------------------------------
constexpr bool USE_TOPOLOGY_MASK = true;
constexpr uint32_t LINK_STALE_MS = 95000UL;      // no frame from a neighbour for ~3 heartbeats -> no edge
constexpr uint32_t LSDB_MAX_AGE_MS = 300000UL;   // a node's row whose sequence stops advancing for 5 min ages out

uint8_t adjacencyMatrix[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT] = {
    {0, 1, 1, 1, 0, 0},
    {1, 0, 0, 1, 1, 0},
    {1, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0},
    {0, 1, 1, 0, 0, 1},
    {0, 0, 1, 0, 1, 0}
};

SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RESET_PIN, LORA_BUSY_PIN);
 DtnBuffer dtnBuffer;
bool radioReady = false;
// RadioLib maps both "packet sent" and "packet received" callbacks onto the
// single SX1262 DIO1 line (setPacketSentAction/setPacketReceivedAction both call
// setDio1Action, so the second registration silently replaced the first).
// One ISR now raises radioIrqPending; serviceRadioIrq() decides in loop()
// whether the interrupt was a TX-done or an RX-done event.
volatile bool radioIrqPending = false;
bool radioTxDone = false;
bool radioRxPending = false;
bool radioTxInFlight = false;
bool radioAckTxInFlight = false;
bool radioAwaitingAck = false;
bool radioTxFromDtn = false;
uint16_t radioTxMessageId = 0;
uint8_t radioTxOriginNode = 0;
uint8_t radioTxNextHop = 0;
size_t radioTxPacketLen = 0;
uint32_t radioAckDeadlineMs = 0;
uint32_t radioTxStartedAtMs = 0;
EmergencyMessage activeTxMessage{};
uint8_t macBackoffExponent = MAC_MIN_BACKOFF_EXPONENT;
uint8_t macBusyRetries = 0;
uint16_t macBackoffMessageId = 0;
uint8_t macBackoffOriginNode = 0;
bool macBackoffActive = false;
uint32_t nextMacAttemptAt = 0;
uint8_t radioPacket[MAX_LORA_PACKET_SIZE];
uint8_t ackPacket[ACK_PACKET_SIZE];
uint32_t nextHeartbeatAtMs = 0;
uint32_t macSlotTimeMs = 1500;        // replaced in setup() by ToA(ACK frame)
uint32_t maxFrameAirtimeMs = 9000;    // replaced in setup() by ToA(250-byte frame)

// Radio silence: replaces "disconnect the antenna" for link-failure tests. A
// silent node neither transmits nor receives, but the portal keeps queueing.
// Never transmit with the antenna removed; it can damage the SX1262.
bool radioSilent = false;
bool radioSilentRequested = false;

// Heartbeat state (1-hop broadcast, no ACK, no forwarding).
bool heartbeatPending = false;
uint32_t heartbeatPendingSinceMs = 0;   // a heartbeat is never postponed longer than one interval
bool radioTxIsHeartbeat = false;
uint8_t heartbeatCadAttempts = 0;
uint32_t nextHeartbeatTryAtMs = 0;
uint16_t heartbeatSequence = 0;
// Traffic-load feature sent in each heartbeat (Table 4.1: cumulative CSMA/CA
// backoff cycles): how many times since the previous heartbeat this node had to
// back off or defer a DATA transmission because the channel was busy (TBEB, in
// handleChannelBusy). Busy checks made while sending heartbeats are not counted.
uint8_t dataBackoffsSinceHeartbeat = 0;
uint8_t heartbeatPacket[HEARTBEAT_PACKET_SIZE];

// Latest heartbeat telemetry heard from each neighbour (feature cache for the
// relay-viability classifier, Section 3.4). Index = node ID - 1.
struct NeighborInfo {
    bool valid;
    float rssi;
    float snr;
    uint16_t batteryPermille;   // 0.1 % units, BATTERY_UNKNOWN if not reported
    uint8_t trafficLoad;
    uint16_t lastSequence;
    uint32_t lastHeardMs;
    uint8_t missesLogged;       // HB_MISS rows already written by timeout since lastHeardMs
    bool labelResync;           // next heartbeat only re-synchronises (no label row)
};
NeighborInfo neighborTable[NETWORK_NODE_COUNT] = {};

// --- CART training labels from heartbeats (receiver side) ---------------------
// Every heartbeat a neighbour is expected to send becomes one labelled row:
//   HB_OK   it arrived      HB_MISS   it did not (sequence gap or long silence)
// Features are the values this node held BEFORE that heartbeat was due (last
// heartbeat's RSSI/SNR, the neighbour's traffic, and the heartbeat age at
// the moment it should have arrived = j x interval), the same features a data
// TX row carries. Using the age measured on arrival would leak the label.
constexpr uint8_t HB_LABEL_MAX_GAP = 64;          // larger jump = neighbour rebooted: resync, no label
constexpr uint8_t HB_LABEL_MAX_TIMEOUT_MISSES = 10;   // stop after ~5 min of silence
// Wait 1.5 extra intervals: a neighbour that skips one heartbeat on a busy
// channel does not use up a sequence number, so its next one comes ~60 s later.
constexpr uint32_t HB_LABEL_TIMEOUT_GRACE_MS = HEARTBEAT_INTERVAL_MS + HEARTBEAT_INTERVAL_MS / 2;

// Measured one-hop link quality per neighbour (averaged RSSI/SNR of every
// authenticated frame heard) and the link-state database of all nodes' rows.
LinkQuality linkQuality[NETWORK_NODE_COUNT] = {};
LinkStateDb linkStateDb;

// Temporal Seen-List for duplicate suppression (Section 3.3).
SeenList<SEEN_LIST_CAPACITY> seenList;

void resetMacBackoff();

// ---------------------------------------------------------------------------
// Machine-readable event log. Every row starts with "CSV," and has the same
// columns, so the PC can keep only those lines and load them straight into pandas:
// CSV,ms,node,event,origin,id,k,type,peer,rssi,snr,peer_batt_pct,peer_traffic,
//     peer_queue (reserved/blank),peer_hb_age_ms,self_batt_pct,self_used_mah,mac_retries,bytes,queue_len,dtn_len,
//     ahead,bypassed,cart_pf,cart_class,heur_class,wait_ms,queue_index,d_buffer_ms,detail
//   ms      this node's uptime (millis)       node   node that wrote the row
//   event   NEW | TX | RX | ACK | NOACK | DELIVER | DUP | STORE | EVICT | DROP | LINK |
//           HB_OK | HB_MISS (CART training labels: expected heartbeat from peer arrived / lost;
//           id = that heartbeat's sequence, peer_hb_age_ms = age when it was due) |
//           FORWARD (a message left the DTN buffer, acknowledged by the next hop) | CONFIG | TIME |
//           BATT (LiPo voltage at boot and every minute: detail mv=...; self_batt_pct = level)
//   cart_pf,cart_class,heur_class  data TX rows: both relay-filter verdicts for the next hop
//   wait_ms      data TX rows: total TBEB backoff/defer time before this send
//   queue_index  data TX rows: messages that would be sent before this one when it
//                entered the queue (FIFO: everything already queued; heap: more urgent ones)
//   d_buffer_ms  FORWARD rows: how long the message was held in the DTN buffer
//   origin,id  message key (heartbeats: source node + heartbeat sequence)
//   peer    the other end of the radio link: previous hop for RX, next hop for TX/ACK/NOACK
//   rssi,snr   RX rows: measured on this frame. Other rows: last frame heard from peer
//   peer_*  latest heartbeat telemetry of peer (battery mV, traffic, age in ms);
//           peer_queue remains blank for compatibility with existing CSV files
//   peer_batt_pct  peer's battery level (%) from its heartbeat (estimated on power banks)
//   self_batt_pct,self_used_mah  this node's own battery % and energy used so far
//           (heartbeat TX rows, every ~30 s; energy-consumption metric)
//   queue_len,dtn_len,ahead,bypassed  data TX rows only (Priority Queuing Accuracy):
//           ahead = waiting messages that should have gone first (0 = correct order),
//           bypassed = older, less urgent messages this one overtook
// Empty field = not applicable / unknown.
// ---------------------------------------------------------------------------
struct CsvEvent {
    const char *event;
    uint8_t origin;
    uint16_t id;
    uint8_t k;
    uint8_t type;
    uint8_t peer;
    bool hasRadio;
    float rssi;
    float snr;
    bool hasSelfEnergy;
    int16_t macRetries;   // < 0 = not logged
    uint16_t bytes;
    bool hasQueueState;
    uint16_t queueLen;
    uint16_t dtnLen;
    uint16_t ahead;
    uint16_t bypassed;
    bool hasVerdict;      // data TX rows: classification of the next hop (C10)
    float cartPf;         // < 0 = no CART model
    bool cartRisky;
    bool heurRisky;
    int32_t waitMs;       // data TX rows: total TBEB backoff wait before this send, ms (< 0 = n/a)
    int16_t queueIndex;   // data TX rows: queue position when the message entered the queue (< 0 = n/a)
    int32_t dBufferMs;    // FORWARD rows: custody time in the DTN buffer, ms (< 0 = n/a)
    bool hasPeerAge;      // log peerAgeMs instead of "now - last heartbeat"
    uint32_t peerAgeMs;
    const char *detail;
};
CsvEvent csvEvent(const char *event, uint8_t origin, uint16_t id, uint8_t k, uint8_t type, uint8_t peer);
CsvEvent csvEvent(const char *event, const EmergencyMessage &message, uint8_t peer);
void logCsv(const CsvEvent &e);
uint16_t currentBatteryPermille();
// "97.5%" or "unknown", for human-readable logs.
const char *batteryText(uint16_t permille, char *buffer, size_t size) {
    if (permille == BATTERY_UNKNOWN) snprintf(buffer, size, "unknown");
    else snprintf(buffer, size, "%.1f%%", permille / 10.0f);
    return buffer;
}
double energyUsedMah();
void markRadioTx(bool transmitting);
void buildOwnLinkRow(uint32_t now, bool markDownLinks, uint8_t row[NETWORK_NODE_COUNT]);
void noteLinkQuality(uint8_t neighborNode, float rssi, float snr);
bool radioReceivingInProgress();
void recordLinkAlive(uint8_t neighborNode, const char *evidence);

uint16_t allocateMessageId() {
    portENTER_CRITICAL(&messageCounterMux);
    uint16_t id = ++globalMessageCounter;
    portEXIT_CRITICAL(&messageCounterMux);
    return id;
}

bool queuePush(const EmergencyMessage &msg) {
    if (!priorityQueueMutex || xSemaphoreTake(priorityQueueMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        CsvEvent e = csvEvent("DROP", msg, 0);
        e.detail = "queue_busy";
        logCsv(e);
        return false;
    }
    EmergencyMessage evicted{};
    const MessageQueue::PushResult result = priorityQueue.push(msg, evicted);
    xSemaphoreGive(priorityQueueMutex);

    if (result == MessageQueue::PUSH_EVICTED) {
        Serial.printf("[QUEUE] EVICT k=%u packet V%u#%04u to admit k=%u packet V%u#%04u.\n",
                      evicted.priority, evicted.originNode, evicted.id,
                      msg.priority, msg.originNode, msg.id);
        char detail[40];
        snprintf(detail, sizeof(detail), "queue;admit=V%u#%u", msg.originNode, msg.id);
        CsvEvent e = csvEvent("EVICT", evicted, 0);
        e.detail = detail;
        logCsv(e);
    } else if (result == MessageQueue::PUSH_REJECTED) {
        Serial.printf("[QUEUE] Full (%u); rejected k=%u packet V%u#%04u.\n",
                      (unsigned int)MessageQueue::capacity(), msg.priority, msg.originNode, msg.id);
        CsvEvent e = csvEvent("DROP", msg, 0);
        e.detail = "queue_full";
        logCsv(e);
    }
    return result != MessageQueue::PUSH_REJECTED;
}

bool queuePeek(EmergencyMessage &msg) {
    if (!priorityQueueMutex || xSemaphoreTake(priorityQueueMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    bool result = priorityQueue.peek(msg);
    xSemaphoreGive(priorityQueueMutex);
    return result;
}

bool queueRemoveById(uint16_t id, uint8_t originNode) {
    if (!priorityQueueMutex || xSemaphoreTake(priorityQueueMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    bool result = priorityQueue.removeById(id, originNode);
    xSemaphoreGive(priorityQueueMutex);
    return result;
}

size_t queueSize() {
    if (!priorityQueueMutex || xSemaphoreTake(priorityQueueMutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;
    size_t result = priorityQueue.size();
    xSemaphoreGive(priorityQueueMutex);
    return result;
}

void appendPathNode(char *path, size_t pathSize, uint8_t nodeId) {
    const size_t used = strlen(path);
    if (used >= pathSize) return;
    snprintf(path + used, pathSize - used, "%sV%u", used ? " -> " : "", nodeId);
}

// Route the message ACTUALLY took: origin, the relays recorded in the packet's
// relay-path field (each relay adds itself), then this node.
void buildIncidentPath(uint8_t originNode, uint16_t relayPath, char *path, size_t pathSize) {
    if (!path || pathSize == 0) return;
    path[0] = '\0';
    appendPathNode(path, pathSize, originNode);
    for (uint8_t slot = 0; slot < RELAY_PATH_SLOTS; slot++) {
        const uint8_t relay = relayPathAt(relayPath, slot);
        if (relay == 0) break;
        appendPathNode(path, pathSize, relay);
    }
    if (relayPath & RELAY_PATH_OVERFLOW) {
        const size_t used = strlen(path);
        if (used < pathSize) snprintf(path + used, pathSize - used, " -> ...");
    }
    appendPathNode(path, pathSize, NODE_ID);
}

// Same route as a compact log field, e.g. "6-3-1".
void formatRouteForLog(uint8_t originNode, uint16_t relayPath, char *out, size_t outSize) {
    size_t used = (size_t)snprintf(out, outSize, "%u", originNode);
    for (uint8_t slot = 0; slot < RELAY_PATH_SLOTS && used < outSize; slot++) {
        const uint8_t relay = relayPathAt(relayPath, slot);
        if (relay == 0) break;
        used += (size_t)snprintf(out + used, outSize - used, "-%u", relay);
    }
    if ((relayPath & RELAY_PATH_OVERFLOW) && used < outSize) {
        used += (size_t)snprintf(out + used, outSize - used, "-x");
    }
    if (used < outSize) snprintf(out + used, outSize - used, "-%u", NODE_ID);
}

// Wall-clock time in ms (Unix epoch), or 0 if this node's clock was never set.
uint64_t epochNowMs() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    if (tv.tv_sec < (time_t)MIN_VALID_EPOCH) return 0;
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

// End-to-end latency in ms (Section 4.7 metric 2): hub clock at delivery minus
// the creation time carried in the packet (seconds + milliseconds). Accurate
// only as far as the origin's clock (phone or node) agrees with the hub's.
int32_t calculateIncidentLatencyMs(uint32_t transmittedAtSeconds, uint16_t transmittedAtMs) {
    const uint64_t nowMs = epochNowMs();
    const uint64_t sentMs = (uint64_t)transmittedAtSeconds * 1000ULL + transmittedAtMs;
    if (transmittedAtSeconds < MIN_VALID_EPOCH || nowMs == 0 || nowMs < sentMs) return -1;
    const uint64_t elapsedMs = nowMs - sentMs;
    return elapsedMs <= INT32_MAX ? (int32_t)elapsedMs : -1;
}

void recordLiveIncident(uint16_t messageId, uint8_t priority, uint8_t packetType,
                        uint8_t originNode, uint16_t relayPath, uint32_t transmittedAtSeconds,
                        uint16_t transmittedAtMs, const char *location, const char *message) {
    if (!liveIncidentMutex ||
        xSemaphoreTake(liveIncidentMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    LiveIncident &incident = liveIncidents[liveIncidentWriteIndex];
    incident.messageId = messageId;
    incident.priority = priority;
    incident.packetType = packetType;
    incident.originNode = originNode;
    incident.latencyMs = calculateIncidentLatencyMs(transmittedAtSeconds, transmittedAtMs);
    incident.receivedAtMs = millis();
    incident.arrivalNo = ++liveIncidentArrivalCounter;
    buildIncidentPath(originNode, relayPath, incident.path, sizeof(incident.path));
    snprintf(incident.location, sizeof(incident.location), "%s", location ? location : "Unspecified");
    snprintf(incident.message, sizeof(incident.message), "%s", message ? message : "");

    liveIncidentWriteIndex = (liveIncidentWriteIndex + 1) % LIVE_INCIDENT_CAPACITY;
    if (liveIncidentCount < LIVE_INCIDENT_CAPACITY) liveIncidentCount++;
    xSemaphoreGive(liveIncidentMutex);
}

bool serializeLoRaPacket(const EmergencyMessage &msg, uint8_t nextHopNode,
                         uint8_t *out, size_t capacity, size_t &outLen) {
    outLen = 0;
    if (!out || msg.priority < 1 || msg.priority > 3 || msg.ttl > 15 ||
        msg.packetType == PACKET_TYPE_HEARTBEAT || msg.packetType > PACKET_TYPE_TELEMETRY ||
        nextHopNode < 1 ||
        nextHopNode > NETWORK_NODE_COUNT || msg.destinationNode < 1 ||
        msg.destinationNode > NETWORK_NODE_COUNT || msg.originNode < 1 ||
        msg.originNode > NETWORK_NODE_COUNT || msg.encryptedLen == 0 ||
        msg.encryptedLen > sizeof(msg.encryptedData) || capacity < FRAME_OVERHEAD_SIZE ||
        (size_t)msg.encryptedLen + FRAME_OVERHEAD_SIZE > capacity ||
        (size_t)msg.encryptedLen + FRAME_OVERHEAD_SIZE > MAX_LORA_PACKET_SIZE) {
        return false;
    }

    out[0] = encodeHeader(msg.priority, msg.ttl, msg.packetType);
    out[1] = NODE_ID;
    out[2] = nextHopNode;
    out[3] = msg.destinationNode;
    out[4] = msg.originNode;
    out[5] = (uint8_t)(msg.id >> 8);
    out[6] = (uint8_t)(msg.id & 0xFF);
    out[7] = macBusyRetries;
    out[8] = (uint8_t)(msg.relayPath >> 8);
    out[9] = (uint8_t)(msg.relayPath & 0xFF);
    memcpy(out + FRAME_PREFIX_SIZE, msg.encryptedData, msg.encryptedLen);
    outLen = FRAME_PREFIX_SIZE + msg.encryptedLen;
    if (!AESCrypto::authenticateFrame(out, outLen, out + outLen, FRAME_AUTH_TAG_SIZE)) {
        outLen = 0;
        return false;
    }
    outLen += FRAME_AUTH_TAG_SIZE;
    return true;
}

// Single DIO1 interrupt handler. Kept in IRAM and minimal: it only raises a flag.
void IRAM_ATTR onRadioDio1() {
    radioIrqPending = true;
}

// Classifies a pending DIO1 interrupt. Runs at the start of each loop() pass.
//  - While a data packet or ACK is being transmitted, the interrupt is TX-done.
//  - Otherwise the radio is in RX mode, and a packet is only read when the chip
//    actually reports RX_DONE (CAD-done or stale interrupts are ignored, which
//    prevents re-processing the previous packet left in the radio buffer).
void serviceRadioIrq() {
    if (!radioReady || !radioIrqPending) return;
    if (radioSilent) { radioIrqPending = false; return; }
    radioIrqPending = false;

    if (radioTxInFlight || radioAckTxInFlight) {
        const uint32_t irqFlags = radio.getIrqFlags();
        if (!(irqFlags & RADIOLIB_SX126X_IRQ_TX_DONE)) {
            Serial.printf("[LORA IRQ] DIO1 during TX without TX_DONE (flags 0x%04lX); finishing TX.\n",
                          (unsigned long)irqFlags);
        }
        radioTxDone = true;
        return;
    }

    const uint32_t irqFlags = radio.getIrqFlags();
    if (irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE) {
        radioRxPending = true;
    }
}

bool deadlineReached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

// ---------------------------------------------------------------------------
// Battery (VBAT) measurement - Heltec WiFi LoRa 32 V3
// GPIO37 (ADC_Ctrl) switches the VBAT divider on, but its polarity depends on
// the board revision:
//   V3 / V3.1: pull-up on GPIO37, divider ON when driven LOW
//   V3.2:      transistor,        divider ON when driven HIGH
// detectBatteryControlPolarity() tells them apart at boot: with GPIO37 as an
// input, the V3/V3.1 pull-up reads HIGH.
// IMPORTANT: VBAT is the LiPo connector. When a node runs from a USB power bank
// with no LiPo attached, VBAT does not show the power bank's charge, so the
// Battery Voltage Degradation metric needs LiPo-powered nodes.
// ---------------------------------------------------------------------------
uint8_t batteryCtrlActiveLevel = LOW;
bool batteryBoardIsV32 = false;

void detectBatteryControlPolarity() {
    pinMode(BATTERY_ADC_ENABLE_PIN, INPUT);
    delay(2);
    batteryBoardIsV32 = digitalRead(BATTERY_ADC_ENABLE_PIN) == LOW;
    batteryCtrlActiveLevel = batteryBoardIsV32 ? HIGH : LOW;
    pinMode(BATTERY_ADC_ENABLE_PIN, OUTPUT);
    digitalWrite(BATTERY_ADC_ENABLE_PIN, batteryCtrlActiveLevel == HIGH ? LOW : HIGH);   // divider off
}

// Returns the average ADC pin voltage in millivolts (before the divider ratio).
uint32_t readBatteryPinMilliVolts() {
    analogReadResolution(12);
    digitalWrite(BATTERY_ADC_ENABLE_PIN, batteryCtrlActiveLevel);           // divider on
    delay(10);
    uint32_t millivoltSum = 0;
    constexpr uint8_t SAMPLE_COUNT = 8;
    for (uint8_t sample = 0; sample < SAMPLE_COUNT; sample++) {
        millivoltSum += analogReadMilliVolts(BATTERY_ADC_PIN);
    }
    digitalWrite(BATTERY_ADC_ENABLE_PIN, batteryCtrlActiveLevel == HIGH ? LOW : HIGH);   // divider off
    return millivoltSum / SAMPLE_COUNT;
}

float readBatteryVoltage() {
    return (float)readBatteryPinMilliVolts() * BATTERY_DIVIDER_RATIO * batteryCalibration / 1000.0f;
}

// A single-cell LiPo is 3.0-4.25 V. Outside that range the reading is not a battery.
bool batteryReadingPlausible(float volts) {
    return volts >= 3.0f && volts <= 4.35f;
}

// ---------------------------------------------------------------------------
// Energy estimate (power-bank nodes)
// The board cannot see a power bank's charge (it always gets a regulated 5 V),
// so the level is ESTIMATED by coulomb counting with measured currents:
//   used_mAh += I_idle x (time listening)  +  I_tx x (time transmitting)
//   battery % = 100 x (1 - used_mAh / usable_capacity_mAh)
// Currents are measured once per board with a USB power meter (serial
// "energy idle <mA>" / "energy tx <mA>"; "energy txtest" helps with the TX one).
// Usable capacity default: 10,000 mAh x 3.7 V = 37 Wh; x 0.85 converter
// efficiency / 5 V = 6,290 mAh at the USB port.
// Calibration and energy used are kept in flash (NVS), so they survive resets.
// ---------------------------------------------------------------------------
constexpr float DEFAULT_IDLE_MA = 110.0f;        // placeholder until measured
constexpr float DEFAULT_TX_MA = 210.0f;          // placeholder until measured (19 dBm)
constexpr float DEFAULT_USABLE_CAPACITY_MAH = 6290.0f;
constexpr uint32_t ENERGY_SAVE_INTERVAL_MS = 60000UL;

Preferences energyStore;
float energyIdleMa = DEFAULT_IDLE_MA;
float energyTxMa = DEFAULT_TX_MA;
float energyCapacityMah = DEFAULT_USABLE_CAPACITY_MAH;
double energyUsedMahTotal = 0.0;
bool energyCalibrated = false;
bool energyTxActive = false;
uint32_t energyLastUpdateMs = 0;
uint32_t energyLastSaveMs = 0;
bool energyStoreOpen = false;

void energyUpdate() {
    const uint32_t now = millis();
    const uint32_t elapsedMs = now - energyLastUpdateMs;
    energyLastUpdateMs = now;
    const float currentMa = energyTxActive ? energyTxMa : energyIdleMa;
    energyUsedMahTotal += (double)currentMa * (double)elapsedMs / 3600000.0;
}

double energyUsedMah() {
    energyUpdate();
    return energyUsedMahTotal;
}

void energySave() {
    if (!energyStoreOpen) return;
    energyUpdate();
    energyStore.putDouble("used", energyUsedMahTotal);
    energyLastSaveMs = millis();
}

void energySaveCalibration() {
    if (!energyStoreOpen) return;
    energyStore.putFloat("idle", energyIdleMa);
    energyStore.putFloat("tx", energyTxMa);
    energyStore.putFloat("cap", energyCapacityMah);
    energyStore.putBool("cal", energyCalibrated);
    energySave();
}

void energyBegin() {
    energyStoreOpen = energyStore.begin("energy", false);
    if (energyStoreOpen) {
        energyIdleMa = energyStore.getFloat("idle", DEFAULT_IDLE_MA);
        energyTxMa = energyStore.getFloat("tx", DEFAULT_TX_MA);
        energyCapacityMah = energyStore.getFloat("cap", DEFAULT_USABLE_CAPACITY_MAH);
        energyCalibrated = energyStore.getBool("cal", false);
        energyUsedMahTotal = energyStore.getDouble("used", 0.0);
    }
    energyLastUpdateMs = millis();
    energyLastSaveMs = millis();
}

// Call at every transmit start (true) and finish (false).
void markRadioTx(bool transmitting) {
    energyUpdate();
    energyTxActive = transmitting;
}

void energyService() {
    if (BATTERY_SOURCE != BATTERY_SOURCE_ESTIMATE) return;
    if (millis() - energyLastSaveMs >= ENERGY_SAVE_INTERVAL_MS) energySave();
}

float estimatedBatteryPercent() {
    if (energyCapacityMah <= 0.0f) return 0.0f;
    const double percent = 100.0 * (1.0 - energyUsedMah() / energyCapacityMah);
    return percent < 0.0 ? 0.0f : (percent > 100.0 ? 100.0f : (float)percent);
}

// Battery level for heartbeats / CSV, in 0.1 % units, or BATTERY_UNKNOWN.
uint16_t currentBatteryPermille() {
    if (BATTERY_SOURCE == BATTERY_SOURCE_NONE) return BATTERY_UNKNOWN;
    if (BATTERY_SOURCE == BATTERY_SOURCE_ESTIMATE) {
        return (uint16_t)(estimatedBatteryPercent() * 10.0f + 0.5f);
    }
    const float volts = readBatteryVoltage();
    if (!batteryReadingPlausible(volts)) return BATTERY_UNKNOWN;
    // Simple linear LiPo map: 3.30 V = 0 %, 4.20 V = 100 %.
    float percent = (volts - 3.30f) / (4.20f - 3.30f) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return (uint16_t)(percent * 10.0f + 0.5f);
}

void printEnergyReport() {
    Serial.printf("[ENERGY] Estimated battery %.1f%%: used %.1f of %.0f mAh. Currents: idle %.1f mA, TX %.1f mA (%s).\n",
                  estimatedBatteryPercent(), energyUsedMah(), energyCapacityMah, energyIdleMa, energyTxMa,
                  energyCalibrated ? "measured" : "DEFAULTS - not yet measured on this board");
}

// Transmits 3 full-size (250-byte, ~8.9 s each) frames so a USB power meter has
// time to settle on the TX current. Neighbours drop them (no valid tag).
void energyTxTest() {
    if (!radioReady || radioTxInFlight || radioAckTxInFlight) {
        Serial.println("[ENERGY] Radio busy; try again in a few seconds.");
        return;
    }
    uint8_t testFrame[MAX_LORA_PACKET_SIZE];
    memset(testFrame, 0, sizeof(testFrame));
    Serial.println("[ENERGY] TX test: transmitting for about 27 s. Read the meter while the LED/meter is steady.");
    for (uint8_t i = 0; i < 3; i++) {
        markRadioTx(true);
        radio.transmit(testFrame, sizeof(testFrame));   // blocking
        markRadioTx(false);
        Serial.printf("[ENERGY] TX test frame %u/3 done.\n", i + 1);
    }
    radioIrqPending = false;
    radioRxPending = false;
    radio.startReceive();
    Serial.println("[ENERGY] TX test finished. Enter the reading with: energy tx <mA>");
}

void handleEnergyCommand(const char *args) {
    char word[12] = {0};
    float value = 0.0f;
    const int parsed = sscanf(args, "%11s %f", word, &value);
    if (parsed < 1) {
        printEnergyReport();
        Serial.println("[ENERGY] Commands: energy | energy idle <mA> | energy tx <mA> | energy txtest |"
                       " energy full | energy set <percent> | energy capacity <mAh>");
        return;
    }
    if (!strcmp(word, "idle") && parsed == 2 && value > 0.0f) {
        energyUpdate();
        energyIdleMa = value;
        energyCalibrated = true;
    } else if (!strcmp(word, "tx") && parsed == 2 && value > 0.0f) {
        energyUpdate();
        energyTxMa = value;
        energyCalibrated = true;
    } else if (!strcmp(word, "capacity") && parsed == 2 && value > 0.0f) {
        energyCapacityMah = value;
    } else if (!strcmp(word, "full")) {
        energyUpdate();
        energyUsedMahTotal = 0.0;            // freshly charged power bank connected
    } else if (!strcmp(word, "set") && parsed == 2 && value >= 0.0f && value <= 100.0f) {
        energyUpdate();
        energyUsedMahTotal = energyCapacityMah * (1.0 - value / 100.0);   // start a scenario at a chosen level
    } else if (!strcmp(word, "txtest")) {
        energyTxTest();
        return;
    } else {
        Serial.println("[ENERGY] Not understood. Type: energy");
        return;
    }
    energySaveCalibration();
    printEnergyReport();
    CsvEvent e = csvEvent("ENERGY", 0, 0, 0, 0, 0);
    e.hasSelfEnergy = true;
    e.detail = word;
    logCsv(e);
}

Preferences batteryStore;

void batteryLoadCalibration() {
    batteryStore.begin("battery", true);
    const float saved = batteryStore.getFloat("cal", 1.0f);
    batteryStore.end();
    batteryCalibration = (saved >= 0.80f && saved <= 1.20f) ? saved : 1.0f;
}

void printBatteryReport() {
    const uint32_t pinMv = readBatteryPinMilliVolts();
    const float volts = (float)pinMv * BATTERY_DIVIDER_RATIO * batteryCalibration / 1000.0f;
    const uint16_t level = currentBatteryPermille();
    char percent[12];
    Serial.printf("[BATTERY] Board %s (GPIO37 active %s). ADC pin %lu mV -> VBAT %.3f V = %s (calibration x%.3f%s).\n",
                  batteryBoardIsV32 ? "V3.2" : "V3/V3.1", batteryCtrlActiveLevel == HIGH ? "HIGH" : "LOW",
                  (unsigned long)pinMv, volts, batteryText(level, percent, sizeof(percent)), batteryCalibration,
                  batteryCalibration == 1.0f ? ", NOT CALIBRATED" : "");
    if (!batteryReadingPlausible(volts)) {
        Serial.println("[BATTERY] WARNING: not a LiPo voltage. Is a LiPo connected to the JST battery "
                       "connector? A USB power bank alone cannot be measured on VBAT.");
    } else if (batteryCalibration == 1.0f) {
        Serial.println("[BATTERY] To calibrate: measure the LiPo with a multimeter, then type "
                       "battery cal <volts>, e.g. battery cal 3.92");
    }
}

// "battery cal <volts>": factor = multimeter / current reading, saved in flash.
// "battery cal reset": back to 1.0.
void handleBatteryCalibration(const char *args) {
    float factor = 1.0f;
    if (strcmp(args, "reset") != 0) {
        const float measured = strtof(args, nullptr);
        const float raw = (float)readBatteryPinMilliVolts() * BATTERY_DIVIDER_RATIO / 1000.0f;
        if (measured < 3.0f || measured > 4.35f || !batteryReadingPlausible(raw)) {
            Serial.printf("[BATTERY] Calibration refused: multimeter %.3f V, board reads %.3f V uncalibrated. "
                          "Both must be LiPo voltages (3.0-4.35 V).\n", measured, raw);
            return;
        }
        factor = measured / raw;
        if (factor < 0.80f || factor > 1.20f) {
            Serial.printf("[BATTERY] Calibration refused: factor %.3f is outside 0.80-1.20. "
                          "Check the multimeter reading and the LiPo connection.\n", factor);
            return;
        }
    }
    batteryCalibration = factor;
    batteryStore.begin("battery", false);
    batteryStore.putFloat("cal", factor);
    batteryStore.end();
    Serial.printf("[BATTERY] Calibration factor x%.4f saved in flash.\n", factor);
    char detail[24];
    snprintf(detail, sizeof(detail), "cal=%.4f", factor);
    CsvEvent e = csvEvent("BATT", 0, 0, 0, 0, 0);
    e.hasSelfEnergy = true;
    e.detail = detail;
    logCsv(e);
    printBatteryReport();
}

// BATT row with the LiPo voltage: at boot and every minute (energy metric R_drop,
// Section 4.7 metric 5). detail "mv=<millivolts>;cal=<factor>"; empty mv = no LiPo.
void logBatteryVoltage(const char *why) {
    if (BATTERY_SOURCE != BATTERY_SOURCE_LIPO) return;
    const float volts = readBatteryVoltage();
    char detail[48];
    if (batteryReadingPlausible(volts))
        snprintf(detail, sizeof(detail), "mv=%u;cal=%.4f;%s", (unsigned int)(volts * 1000.0f + 0.5f),
                 batteryCalibration, why);
    else
        snprintf(detail, sizeof(detail), "mv=;no_lipo;%s", why);
    CsvEvent e = csvEvent("BATT", 0, 0, 0, 0, 0);
    e.hasSelfEnergy = true;
    e.detail = detail;
    logCsv(e);
}

void serviceBatteryLog() {
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();
    if (lastLogMs != 0 && (uint32_t)(now - lastLogMs) < BATTERY_LOG_INTERVAL_MS) return;
    const bool first = lastLogMs == 0;
    lastLogMs = now ? now : 1;
    logBatteryVoltage(first ? "boot" : "periodic");
}

CsvEvent csvEvent(const char *event, uint8_t origin, uint16_t id, uint8_t k, uint8_t type, uint8_t peer) {
    CsvEvent e;
    memset(&e, 0, sizeof(e));
    e.event = event;
    e.origin = origin;
    e.id = id;
    e.k = k;
    e.type = type;
    e.peer = peer;
    e.macRetries = -1;
    e.waitMs = -1;
    e.queueIndex = -1;
    e.dBufferMs = -1;
    e.detail = "";
    return e;
}

CsvEvent csvEvent(const char *event, const EmergencyMessage &message, uint8_t peer) {
    return csvEvent(event, message.originNode, message.id, message.priority, message.packetType, peer);
}

// Snapshot of where msg stands among everything waiting on this node (queue + DTN)
// at the moment it is put on the air. Used for Priority Queuing Accuracy.
void fillQueueState(CsvEvent &e, const EmergencyMessage &msg) {
    size_t ahead = 0, bypassed = 0, queueLen = 0;
    if (priorityQueueMutex && xSemaphoreTake(priorityQueueMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        queueLen = priorityQueue.size();
        priorityQueue.countRelativeTo(msg, ahead, bypassed);
        xSemaphoreGive(priorityQueueMutex);
    } else {
        return;   // queue busy: leave the PQA columns empty rather than guess
    }
    dtnBuffer.countRelativeTo(msg, millis(), ahead, bypassed);
    e.hasQueueState = true;
    e.queueLen = (uint16_t)queueLen;
    e.dtnLen = (uint16_t)dtnBuffer.size();
    e.ahead = (uint16_t)ahead;
    e.bypassed = (uint16_t)bypassed;
}

void printCsvHeader() {
    Serial.println("CSV,ms,node,event,origin,id,k,type,peer,rssi,snr,peer_batt_pct,peer_traffic,"
                   "peer_queue,peer_hb_age_ms,self_batt_pct,self_used_mah,mac_retries,bytes,"
                   "queue_len,dtn_len,ahead,bypassed,cart_pf,cart_class,heur_class,"
                   "wait_ms,queue_index,d_buffer_ms,detail");
}

static char csvLine[200];
static size_t csvUsed = 0;

static void append(const char *format, ...) {
    if (csvUsed >= sizeof(csvLine) - 1) return;
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(csvLine + csvUsed, sizeof(csvLine) - csvUsed, format, args);
    va_end(args);
    if (written > 0) csvUsed += (size_t)written;
    if (csvUsed > sizeof(csvLine) - 1) csvUsed = sizeof(csvLine) - 1;
}

SemaphoreHandle_t csvLogMutex = nullptr;

void logCsv(const CsvEvent &e) {
    // Rows come from the radio loop and the web-server task: serialize them.
    if (csvLogMutex && xSemaphoreTake(csvLogMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    csvUsed = 0;

    const uint32_t now = millis();
    append("CSV,%lu,%u,%s,%u,%u,%u,%u,", (unsigned long)now, NODE_ID, e.event,
           e.origin, e.id, e.k, e.type);
    if (e.peer) append("%u,", e.peer); else append(",");

    const bool peerKnown = e.peer >= 1 && e.peer <= NETWORK_NODE_COUNT &&
                           e.peer != NODE_ID && neighborTable[e.peer - 1].valid;
    const NeighborInfo *neighbor = peerKnown ? &neighborTable[e.peer - 1] : nullptr;
    if (e.hasRadio) append("%.1f,%.2f,", e.rssi, e.snr);
    else if (neighbor) append("%.1f,%.2f,", neighbor->rssi, neighbor->snr);
    else append(",,");

    if (neighbor) {
        if (neighbor->batteryPermille != BATTERY_UNKNOWN) append("%.1f,", neighbor->batteryPermille / 10.0f);
        else append(",");
        append("%u,,%lu,", neighbor->trafficLoad,
               (unsigned long)(e.hasPeerAge ? e.peerAgeMs : now - neighbor->lastHeardMs));
    } else {
        append(",,,,");
    }
    if (e.hasSelfEnergy) {
        const uint16_t level = currentBatteryPermille();
        if (level != BATTERY_UNKNOWN) append("%.1f,", level / 10.0f); else append(",");
        if (BATTERY_SOURCE == BATTERY_SOURCE_ESTIMATE) append("%.2f,", energyUsedMah()); else append(",");   // empty when off
    } else {
        append(",,");
    }
    if (e.macRetries >= 0) append("%d,", e.macRetries); else append(",");
    if (e.bytes) append("%u,", e.bytes); else append(",");
    if (e.hasQueueState) append("%u,%u,%u,%u,", e.queueLen, e.dtnLen, e.ahead, e.bypassed);
    else append(",,,,");
    if (e.hasVerdict) {
        if (e.cartPf >= 0.0f) append("%.3f,%s,", e.cartPf, e.cartRisky ? "Risky" : "Safe");
        else append(",,");
        append("%s,", e.heurRisky ? "Risky" : "Safe");
    } else {
        append(",,,");
    }
    if (e.waitMs >= 0) append("%ld,", (long)e.waitMs); else append(",");
    if (e.queueIndex >= 0) append("%d,", e.queueIndex); else append(",");
    if (e.dBufferMs >= 0) append("%ld,", (long)e.dBufferMs); else append(",");
    append("%s\n", e.detail ? e.detail : "");
    if (csvUsed > 0 && csvLine[csvUsed - 1] != '\n') csvLine[csvUsed - 1] = '\n';   // truncated row
    Serial.write((const uint8_t *)csvLine, csvUsed);   // one write per row keeps rows intact
    if (csvLogMutex) xSemaphoreGive(csvLogMutex);
}

// --- Seen-List access (used from the radio loop and the web handler task) ---
portMUX_TYPE seenListMux = portMUX_INITIALIZER_UNLOCKED;

bool seenListContains(uint8_t originNode, uint16_t messageId) {
    portENTER_CRITICAL(&seenListMux);
    const bool seen = seenList.contains(originNode, messageId);
    portEXIT_CRITICAL(&seenListMux);
    return seen;
}

void seenListAdd(uint8_t originNode, uint16_t messageId) {
    portENTER_CRITICAL(&seenListMux);
    seenList.add(originNode, messageId);
    portEXIT_CRITICAL(&seenListMux);
}

// --- Heartbeats: 16-byte authenticated 1-hop broadcast, no ACK, no forwarding ---
void scheduleNextHeartbeat(uint32_t fromMs) {
    // Random jitter keeps nodes that were reset together from heartbeating in sync.
    const uint32_t jitter = esp_random() % (HEARTBEAT_JITTER_MS + 1);
    nextHeartbeatAtMs = fromMs + HEARTBEAT_INTERVAL_MS - (HEARTBEAT_JITTER_MS / 2) + jitter;
}

void serviceHeartbeat() {
    if (!radioReady || heartbeatPending) return;
    const uint32_t now = millis();
    if (!deadlineReached(now, nextHeartbeatAtMs)) return;
    heartbeatPending = true;
    heartbeatPendingSinceMs = now;
    heartbeatCadAttempts = 0;
    nextHeartbeatTryAtMs = now;
    scheduleNextHeartbeat(now);
}

// Attempts one CAD + broadcast of the pending heartbeat. Called by the MAC
// service when the radio is idle and no k=1 message is waiting.
void transmitHeartbeat(uint32_t now) {
    if (radioIrqPending || radioRxPending) return;   // let pending RX be read first
    if (radioReceivingInProgress()) {                 // a neighbour is mid-frame: try a bit later
        nextHeartbeatTryAtMs = now + macSlotTimeMs + (esp_random() % macSlotTimeMs);
        return;   // not counted as traffic: only data backoffs are (Table 4.1)
    }

    // 0 mV = no valid LiPo reading (e.g. powered from a USB power bank only).
    const uint16_t batteryPermille = currentBatteryPermille();

    int16_t cadState = radio.scanChannel();
    radioIrqPending = false;   // consume the CAD_DONE interrupt (see serviceRadioMac)

    if (cadState == RADIOLIB_CHANNEL_FREE) {
        heartbeatPending = false;
        const uint16_t sequence = ++heartbeatSequence;
        // Link-State Update: our own measured row + one row learned from another node.
        uint8_t ownRow[NETWORK_NODE_COUNT];
        uint8_t ownWire[LS_ROW_WIRE_SIZE];
        buildOwnLinkRow(millis(), true, ownRow);
        packLinkRow(NODE_ID, ownRow, ownWire);
        uint8_t relayedOwner = 0;
        uint16_t relayedSequence = 0;
        uint8_t relayedWire[LS_ROW_WIRE_SIZE];
        if (linkStateDb.nextRowToRelay(millis(), LSDB_MAX_AGE_MS, relayedOwner)) {
            const LinkStateDb::Row &relayed = linkStateDb.row(relayedOwner);
            relayedSequence = relayed.sequence;
            packLinkRow(relayedOwner, relayed.weight, relayedWire);
        }
        buildHeartbeatBody(NODE_ID, sequence, batteryPermille, dataBackoffsSinceHeartbeat, ownWire,
                           relayedOwner, relayedSequence, relayedOwner ? relayedWire : nullptr,
                           heartbeatPacket);
        if (!AESCrypto::authenticateFrame(heartbeatPacket, HEARTBEAT_BODY_SIZE,
                                          heartbeatPacket + HEARTBEAT_BODY_SIZE, HEARTBEAT_TAG_SIZE)) {
            Serial.println("[HB TX] Could not authenticate heartbeat; skipped.");
            radio.startReceive();
            return;
        }
        radioTxDone = false;
        const int16_t txState = radio.startTransmit(heartbeatPacket, HEARTBEAT_PACKET_SIZE);
        if (txState == RADIOLIB_ERR_NONE) {
            radioTxInFlight = true;
            radioTxIsHeartbeat = true;
            markRadioTx(true);
            char battery[12];
            Serial.printf("[HB TX] V%u seq=%u batt=%s traffic=%u row=[%u,%u,%u,%u,%u] relay=V%u (%u bytes)\n",
                          NODE_ID, sequence, batteryText(batteryPermille, battery, sizeof(battery)),
                          dataBackoffsSinceHeartbeat,
                          ownWire[0], ownWire[1], ownWire[2], ownWire[3], ownWire[4], relayedOwner,
                          (unsigned int)HEARTBEAT_PACKET_SIZE);
            CsvEvent e = csvEvent("TX", NODE_ID, sequence, 0, PACKET_TYPE_HEARTBEAT, BROADCAST_NODE_ID);
            e.hasSelfEnergy = true;
            e.macRetries = dataBackoffsSinceHeartbeat;
            e.bytes = HEARTBEAT_PACKET_SIZE;
            e.detail = "heartbeat";
            logCsv(e);
            dataBackoffsSinceHeartbeat = 0;
        } else {
            Serial.printf("[HB TX] Start error %d; heartbeat skipped.\n", txState);
            radio.startReceive();
        }
        return;
    }

    radio.startReceive();
    if (++heartbeatCadAttempts >= HEARTBEAT_MAX_CAD_ATTEMPTS) {
        heartbeatPending = false;
        Serial.println("[HB TX] Channel busy; heartbeat skipped until the next interval.");
    } else {
        nextHeartbeatTryAtMs = now + 200 + (esp_random() % HEARTBEAT_CAD_RETRY_MAX_MS);
    }
}

void forgetAttempts(const EmergencyMessage &message);

void completeDataAcknowledgment() {
    forgetAttempts(activeTxMessage);
    if (radioTxFromDtn && activeTxMessage.dtnStoredAtMs) {
        // C16: the message leaves DTN custody; d_buffer = time it was held (Section 4.5.2).
        const uint32_t heldMs = millis() - activeTxMessage.dtnStoredAtMs;
        Serial.printf("[DTN] Packet V%u#%04u forwarded to V%u after %lu ms in the buffer.\n",
                      activeTxMessage.originNode, activeTxMessage.id, radioTxNextHop, (unsigned long)heldMs);
        CsvEvent f = csvEvent("FORWARD", activeTxMessage, radioTxNextHop);
        f.dBufferMs = heldMs > INT32_MAX ? INT32_MAX : (int32_t)heldMs;
        f.detail = "dtn";
        logCsv(f);
    }
    if (radioTxFromDtn) {
        dtnBuffer.remove(radioTxMessageId, radioTxOriginNode);
    } else {
        queueRemoveById(radioTxMessageId, radioTxOriginNode);
    }
    Serial.printf("[LORA ACK] Next hop V%u acknowledged packet V%u#%04u.\n",
                  radioTxNextHop, radioTxOriginNode, radioTxMessageId);
    char detail[32];
    snprintf(detail, sizeof(detail), "hop_ms=%lu", (unsigned long)(millis() - radioTxStartedAtMs));
    CsvEvent e = csvEvent("ACK", activeTxMessage, radioTxNextHop);
    e.detail = detail;
    logCsv(e);
    recordLinkAlive(radioTxNextHop, "ACK");
    radioAwaitingAck = false;
    resetMacBackoff();
}

bool startLinkAck(uint8_t destinationNode, uint8_t originNode, uint16_t messageId) {
    ackPacket[0] = encodeHeader(0, 0, PACKET_TYPE_ACK);
    ackPacket[1] = NODE_ID;
    ackPacket[2] = destinationNode;
    ackPacket[3] = destinationNode;
    ackPacket[4] = originNode;
    ackPacket[5] = (uint8_t)(messageId >> 8);
    ackPacket[6] = (uint8_t)(messageId & 0xFF);
    ackPacket[7] = 0;
    if (!AESCrypto::authenticateFrame(ackPacket, ACK_PREFIX_SIZE,
                                      ackPacket + ACK_PREFIX_SIZE, FRAME_AUTH_TAG_SIZE)) return false;

    radioTxDone = false;
    int16_t state = radio.startTransmit(ackPacket, sizeof(ackPacket));
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LORA ACK] Start failed with code %d.\n", state);
        radio.startReceive();
        return false;
    }
    radioAckTxInFlight = true;
    markRadioTx(true);
    CsvEvent e = csvEvent("TX", originNode, messageId, 0, PACKET_TYPE_ACK, destinationNode);
    e.bytes = ACK_PACKET_SIZE;
    e.detail = "ack";
    logCsv(e);
    return true;
}

// One CART label row for heartbeat number `sequence` from `peer`, which was the
// j-th heartbeat expected after the last one heard. Call BEFORE neighborTable
// is updated with a newly received heartbeat.
void logHeartbeatLabel(uint8_t peer, uint16_t sequence, uint8_t j, bool received, const char *how) {
    CsvEvent e = csvEvent(received ? "HB_OK" : "HB_MISS", peer, sequence, 0, PACKET_TYPE_HEARTBEAT, peer);
    e.hasPeerAge = true;
    e.peerAgeMs = (uint32_t)j * HEARTBEAT_INTERVAL_MS;
    char detail[24];
    snprintf(detail, sizeof(detail), "%s;j=%u", how, j);
    e.detail = detail;
    logCsv(e);
}

// A heartbeat arrived: label the ones skipped since the last (sequence gap) and this one.
void labelArrivedHeartbeat(uint8_t peer, uint16_t sequence) {
    NeighborInfo &n = neighborTable[peer - 1];
    if (!n.valid || n.labelResync) return;            // no "before" features yet
    const uint16_t gap = (uint16_t)(sequence - n.lastSequence);
    if (gap == 0 || gap > HB_LABEL_MAX_GAP) return;   // duplicate, or neighbour rebooted
    for (uint16_t j = (uint16_t)n.missesLogged + 1; j < gap; j++) {
        logHeartbeatLabel(peer, (uint16_t)(n.lastSequence + j), (uint8_t)j, false, "gap");
    }
    if (n.missesLogged >= gap) return;   // timeout already counted this slot (rare: heartbeat very late)
    logHeartbeatLabel(peer, sequence, (uint8_t)gap, true, "rx");
}

// A neighbour went quiet: label each expected heartbeat as missed once 1.5 extra
// intervals have passed, so a late or skipped heartbeat is not mislabelled.
void serviceHeartbeatLabels() {
    static uint32_t lastCheckMs = 0;
    const uint32_t now = millis();
    if ((uint32_t)(now - lastCheckMs) < 1000) return;
    lastCheckMs = now;
    if (radioSilent) {                                 // we are deaf, not the link
        for (uint8_t i = 0; i < NETWORK_NODE_COUNT; i++) neighborTable[i].labelResync = true;
        return;
    }
    for (uint8_t peer = 1; peer <= NETWORK_NODE_COUNT; peer++) {
        if (peer == NODE_ID) continue;
        NeighborInfo &n = neighborTable[peer - 1];
        if (!n.valid || n.labelResync || n.missesLogged >= HB_LABEL_MAX_TIMEOUT_MISSES) continue;
        const uint8_t j = n.missesLogged + 1;
        if ((uint32_t)(now - n.lastHeardMs) < (uint32_t)j * HEARTBEAT_INTERVAL_MS + HB_LABEL_TIMEOUT_GRACE_MS) continue;
        logHeartbeatLabel(peer, (uint16_t)(n.lastSequence + j), j, false, "timeout");
        n.missesLogged = j;
    }
}

void handleHeartbeat(const uint8_t *packet, size_t packetLen, float rssi, float snr) {
    HeartbeatInfo heartbeat{};
    if (!parseHeartbeatBody(packet, packetLen, heartbeat) ||
        !AESCrypto::verifyFrame(packet, HEARTBEAT_BODY_SIZE,
                                packet + HEARTBEAT_BODY_SIZE, HEARTBEAT_TAG_SIZE)) return;
    if (heartbeat.sourceNode < 1 || heartbeat.sourceNode > NETWORK_NODE_COUNT ||
        heartbeat.sourceNode == NODE_ID) return;

    recordLinkAlive(heartbeat.sourceNode, "heartbeat");
    noteLinkQuality(heartbeat.sourceNode, rssi, snr);

    // Link-State Update: the sender's own row, plus the row it relays.
    const uint32_t nowMs = millis();
    uint8_t row[NETWORK_NODE_COUNT];
    unpackLinkRow(heartbeat.sourceNode, heartbeat.ownRowWire, row);
    linkStateDb.update(heartbeat.sourceNode, heartbeat.sequence, row, nowMs);
    if (heartbeat.relayedOwner >= 1 && heartbeat.relayedOwner <= NETWORK_NODE_COUNT &&
        heartbeat.relayedOwner != NODE_ID) {
        unpackLinkRow(heartbeat.relayedOwner, heartbeat.relayedRowWire, row);
        linkStateDb.update(heartbeat.relayedOwner, heartbeat.relayedSequence, row, nowMs);
    }

    // CART labels use the neighbour's state from BEFORE this heartbeat.
    labelArrivedHeartbeat(heartbeat.sourceNode, heartbeat.sequence);

    NeighborInfo &neighbor = neighborTable[heartbeat.sourceNode - 1];
    neighbor.valid = true;
    neighbor.rssi = rssi;
    neighbor.snr = snr;
    neighbor.batteryPermille = heartbeat.batteryPermille;
    neighbor.trafficLoad = heartbeat.trafficLoad;
    neighbor.lastSequence = heartbeat.sequence;
    neighbor.lastHeardMs = millis();
    neighbor.missesLogged = 0;
    neighbor.labelResync = false;

    char battery[12];
    Serial.printf("[HB RX] V%u seq=%u rssi=%.1f snr=%.1f batt=%s traffic=%u\n",
                  heartbeat.sourceNode, heartbeat.sequence, rssi, snr,
                  batteryText(heartbeat.batteryPermille, battery, sizeof(battery)),
                  heartbeat.trafficLoad);
    CsvEvent e = csvEvent("RX", heartbeat.sourceNode, heartbeat.sequence, 0,
                          PACKET_TYPE_HEARTBEAT, heartbeat.sourceNode);
    e.hasRadio = true;
    e.rssi = rssi;
    e.snr = snr;
    e.bytes = (uint16_t)packetLen;
    e.detail = "heartbeat";
    logCsv(e);
    // Heartbeats are never ACKed and never forwarded.
}

void handleReceivedPacket(const uint8_t *packet, size_t packetLen, float rssi, float snr) {
    if (!packet || packetLen == 0 || packetLen > MAX_LORA_PACKET_SIZE) return;

    if (headerPacketType(packet[0]) == PACKET_TYPE_HEARTBEAT) {
        if (packetLen == HEARTBEAT_PACKET_SIZE) handleHeartbeat(packet, packetLen, rssi, snr);
        return;
    }

    if (packetLen < ACK_PACKET_SIZE ||
        !AESCrypto::verifyFrame(packet, packetLen - FRAME_AUTH_TAG_SIZE,
                                packet + packetLen - FRAME_AUTH_TAG_SIZE, FRAME_AUTH_TAG_SIZE)) return;

    const uint8_t header = packet[0];
    const uint8_t priority = header >> 6;
    const uint8_t ttl = (header >> 2) & 0x0F;
    const uint8_t packetType = headerPacketType(header);
    const uint8_t sourceNode = packet[1];
    const uint8_t nextHopNode = packet[2];
    const uint8_t destinationNode = packet[3];
    const uint8_t originNode = packet[4];
    const uint16_t messageId = ((uint16_t)packet[5] << 8) | packet[6];

    // One RX row per authenticated frame from a neighbour, including frames
    // addressed to another node ("overheard"): each is a valid RSSI/SNR sample of
    // the link sourceNode -> this node. peer = sourceNode (the previous hop).
    if (sourceNode >= 1 && sourceNode <= NETWORK_NODE_COUNT && sourceNode != NODE_ID) {
        noteLinkQuality(sourceNode, rssi, snr);
        CsvEvent e = csvEvent("RX", originNode, messageId, priority, packetType, sourceNode);
        e.hasRadio = true;
        e.rssi = rssi;
        e.snr = snr;
        e.macRetries = packet[7];
        e.bytes = (uint16_t)packetLen;
        e.detail = nextHopNode == NODE_ID ? "to_me" : "overheard";
        logCsv(e);
    }

    if (sourceNode < 1 || sourceNode > NETWORK_NODE_COUNT ||
        nextHopNode != NODE_ID || destinationNode < 1 ||
        destinationNode > NETWORK_NODE_COUNT || originNode < 1 ||
        originNode > NETWORK_NODE_COUNT || sourceNode == NODE_ID) return;
    // No static-adjacency check here: if a frame from sourceNode was received
    // and passed the HMAC check, the radio link exists, whatever the matrix says.
    if (packetType != PACKET_TYPE_ACK) recordLinkAlive(sourceNode, "data frame");

    if (packetType == PACKET_TYPE_ACK) {
        if (packetLen != ACK_PACKET_SIZE || priority != 0 || ttl != 0 ||
            !radioAwaitingAck || sourceNode != radioTxNextHop ||
            destinationNode != NODE_ID || originNode != radioTxOriginNode ||
            messageId != radioTxMessageId) return;
        completeDataAcknowledgment();
        return;
    }

    // The sender chose this node as next hop (nextHopNode == NODE_ID, checked
    // above). The receiver does not re-run Dijkstra to second-guess that choice,
    // so detours chosen by the heuristic filter or the CART classifier are accepted.
    if ((packetType != PACKET_TYPE_SOS && packetType != PACKET_TYPE_TELEMETRY) ||
        priority < 1 || priority > 3 || ttl == 0 || packetLen <= FRAME_OVERHEAD_SIZE) return;

    // Relays this packet has passed through so far, including the sender if it is a relay.
    const uint16_t relayPath = ((uint16_t)packet[8] << 8) | packet[9];

    // Duplicate suppression (Section 3.3): a copy we already accepted means our
    // earlier ACK was lost. Re-ACK so the sender stops retrying, but never queue
    // or deliver it a second time.
    if (seenListContains(originNode, messageId)) {
        Serial.printf("[SEEN] Duplicate packet V%u#%04u from V%u; re-sent ACK, not queued again.\n",
                      originNode, messageId, sourceNode);
        CsvEvent e = csvEvent("DUP", originNode, messageId, priority, packetType, sourceNode);
        e.detail = "re_ack";
        logCsv(e);
        startLinkAck(sourceNode, originNode, messageId);
        return;
    }

    uint8_t plaintext[256];
    size_t plaintextLen = 0;
    const size_t encryptedLen = packetLen - FRAME_OVERHEAD_SIZE;
    if (!AESCrypto::decrypt(packet + FRAME_PREFIX_SIZE, encryptedLen,
                            plaintext, plaintextLen, sizeof(plaintext))) {
        Serial.printf("[LORA RX] Decryption failed for packet V%u#%04u.\n", originNode, messageId);
        CsvEvent e = csvEvent("DROP", originNode, messageId, priority, packetType, sourceNode);
        e.detail = "decrypt_fail";
        logCsv(e);
        return;
    }

    uint32_t timestampTx = 0;
    uint16_t timestampMs = 0;
    char location[sizeof(EmergencyMessage::location)];
    char message[sizeof(EmergencyMessage::payload)];
    if (!parseMessageBody(plaintext, plaintextLen, timestampTx, timestampMs,
                          location, sizeof(location), message, sizeof(message))) {
        Serial.printf("[LORA RX] Invalid message body for packet V%u#%04u.\n", originNode, messageId);
        CsvEvent e = csvEvent("DROP", originNode, messageId, priority, packetType, sourceNode);
        e.detail = "bad_body";
        logCsv(e);
        return;
    }
    memset(plaintext, 0, sizeof(plaintext));   // decrypted copy no longer needed

    if (destinationNode == NODE_ID) {
        // Only the destination (Main Hub) keeps the readable message for its
        // dashboard. Relays forward the encrypted bytes and keep no text or
        // location (Section 4.3 Data Privacy: minimal PII on intermediate nodes).
        recordLiveIncident(messageId, priority, packetType, originNode, relayPath,
                           timestampTx, timestampMs, location[0] ? location : "Unspecified", message);
        char routeText[64];
        buildIncidentPath(originNode, relayPath, routeText, sizeof(routeText));
        Serial.printf("[LORA RX] Delivered packet V%u#%04u at V%u via %s: %s\n",
                      originNode, messageId, NODE_ID, routeText, message);
        char route[32];
        formatRouteForLog(originNode, relayPath, route, sizeof(route));
        char detail[80];
        const int32_t latencyMs = calculateIncidentLatencyMs(timestampTx, timestampMs);
        if (latencyMs >= 0)
            snprintf(detail, sizeof(detail), "ts=%lu.%03u;lat_ms=%ld;route=%s", (unsigned long)timestampTx,
                     timestampMs, (long)latencyMs, route);
        else
            snprintf(detail, sizeof(detail), "ts=%lu.%03u;route=%s", (unsigned long)timestampTx, timestampMs, route);
        CsvEvent e = csvEvent("DELIVER", originNode, messageId, priority, packetType, sourceNode);
        e.detail = detail;
        logCsv(e);
        seenListAdd(originNode, messageId);
        startLinkAck(sourceNode, originNode, messageId);
        return;
    }

    if (ttl <= 1) {
        // Not acknowledged: the sender must not count this packet as delivered.
        Serial.printf("[DROP] Packet V%u#%04u from V%u: TTL expired at V%u; not acknowledged.\n",
                      originNode, messageId, sourceNode, NODE_ID);
        CsvEvent e = csvEvent("DROP", originNode, messageId, priority, packetType, sourceNode);
        e.detail = "ttl";
        logCsv(e);
        return;
    }

    EmergencyMessage forwarded{};
    forwarded.id = messageId;
    forwarded.priority = priority;
    forwarded.timestamp_tx = timestampTx;
    forwarded.timestamp_ms = timestampMs;
    forwarded.originNode = originNode;
    forwarded.destinationNode = destinationNode;
    forwarded.ttl = ttl - 1;
    forwarded.packetType = packetType;
    forwarded.expiresAtMs = millis() + DTN_RETENTION_MS;
    forwarded.retryExponent = 0;   // failed DTN polls
    forwarded.relayPath = relayPathAppend(relayPath, NODE_ID);   // this node is the next relay
    // location/payload stay empty on relays: only the encrypted bytes are sent on.
    memset(location, 0, sizeof(location));
    memset(message, 0, sizeof(message));
    forwarded.encryptedLen = (uint8_t)encryptedLen;
    memcpy(forwarded.encryptedData, packet + FRAME_PREFIX_SIZE, encryptedLen);

    if (!queuePush(forwarded)) {
        Serial.printf("[LORA RX] Forward queue full; packet V%u#%04u was not acknowledged.\n", originNode, messageId);
        return;
    }
    seenListAdd(originNode, messageId);
    startLinkAck(sourceNode, originNode, messageId);
}

void serviceRadioRx() {
    if (radioSilent) { radioRxPending = false; return; }
    if (!radioReady || !radioRxPending || radioAckTxInFlight || radioTxInFlight) return;
    radioRxPending = false;

    // Read only if the SX1262 really reports a received packet. Any other DIO1
    // event (e.g. CAD done) would otherwise make readData() return the previous
    // packet still sitting in the radio buffer, causing a duplicate forward.
    if (!(radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE)) {
        radio.startReceive();
        return;
    }

    size_t packetLen = radio.getPacketLength();
    if (packetLen == 0 || packetLen > sizeof(radioPacket)) {
        radio.startReceive();
        return;
    }

    int16_t state = radio.readData(radioPacket, packetLen);
    if (state == RADIOLIB_ERR_NONE) {
        const float rssi = radio.getRSSI();
        const float snr = radio.getSNR();
        handleReceivedPacket(radioPacket, packetLen, rssi, snr);
    } else {
        Serial.printf("[LORA RX] Read failed with code %d.\n", state);
    }
    if (!radioAckTxInFlight) radio.startReceive();
}

// ---------------------------------------------------------------------------
// Link health (Section 3.3 store-and-forward + rerouting)
// A neighbour is marked DOWN after LINK_DOWN_AFTER_ACK_FAILURES consecutive ACK
// timeouts. Routing then skips that link. The link comes back UP as soon as any
// authenticated frame (heartbeat, data or ACK) is heard from that neighbour,
// which acts as the recovery beacon, or after LINK_DOWN_HOLD_MS as a fallback.
// ---------------------------------------------------------------------------
struct LinkState {
    uint8_t consecutiveAckFailures;
    bool down;
    uint32_t downUntilMs;
};
LinkState linkState[NETWORK_NODE_COUNT] = {};

bool isLinkDown(uint8_t neighborNode, uint32_t now) {
    if (neighborNode < 1 || neighborNode > NETWORK_NODE_COUNT) return false;
    LinkState &link = linkState[neighborNode - 1];
    if (link.down && deadlineReached(now, link.downUntilMs)) {
        link.down = false;
        link.consecutiveAckFailures = 0;
        Serial.printf("[LINK] V%u hold time over; link will be tried again.\n", neighborNode);
    }
    return link.down;
}

void recordAckFailure(uint8_t neighborNode, uint32_t now) {
    if (neighborNode < 1 || neighborNode > NETWORK_NODE_COUNT) return;
    LinkState &link = linkState[neighborNode - 1];
    if (link.consecutiveAckFailures < 255) link.consecutiveAckFailures++;
    if (!link.down && link.consecutiveAckFailures >= LINK_DOWN_AFTER_ACK_FAILURES) {
        link.down = true;
        link.downUntilMs = now + LINK_DOWN_HOLD_MS;
        Serial.printf("[LINK] V%u marked DOWN after %u ACK timeouts.\n",
                      neighborNode, link.consecutiveAckFailures);
        CsvEvent e = csvEvent("LINK", 0, 0, 0, 0, neighborNode);
        e.detail = "down";
        logCsv(e);
    }
}

void recordLinkAlive(uint8_t neighborNode, const char *evidence) {
    if (neighborNode < 1 || neighborNode > NETWORK_NODE_COUNT || neighborNode == NODE_ID) return;
    LinkState &link = linkState[neighborNode - 1];
    link.consecutiveAckFailures = 0;
    if (!link.down) return;
    link.down = false;
    const size_t due = dtnBuffer.makeAllDue(millis());
    Serial.printf("[LINK] V%u back UP (heard %s); %u buffered packet(s) released for retry.\n",
                  neighborNode, evidence, (unsigned int)due);
    CsvEvent e = csvEvent("LINK", 0, 0, 0, 0, neighborNode);
    e.detail = "up";
    logCsv(e);
}

void noteLinkQuality(uint8_t neighborNode, float rssi, float snr) {
    if (neighborNode < 1 || neighborNode > NETWORK_NODE_COUNT || neighborNode == NODE_ID) return;
    linkQualityAddSample(linkQuality[neighborNode - 1], rssi, snr, millis());
}

// ---------------------------------------------------------------------------
// Relay filter (Sections 3.4 and 4.2.1)
//   RELAY_FILTER_NONE       Baseline / Partial: no filter. If a trained model is
//                           present, CART still runs in shadow mode for the logs.
//   RELAY_FILTER_HEURISTIC  a neighbour failing the static rule gets no edge (255).
//   RELAY_FILTER_CART       each outgoing edge is scaled by M = 1 + alpha x Pf.
// Chosen per build (platformio.ini: -D RELAY_FILTER_CFG=...); default NONE.
// Both verdicts are logged on every data TX row (cart_pf, cart_class, heur_class),
// whichever filter is active, so CART and the heuristic can be compared on the
// same transmissions (McNemar's test, Section 4.7).
// ---------------------------------------------------------------------------
#define RELAY_FILTER_NONE 0
#define RELAY_FILTER_HEURISTIC 1
#define RELAY_FILTER_CART 2
#ifndef RELAY_FILTER_CFG
#define RELAY_FILTER_CFG RELAY_FILTER_NONE
#endif
#if __has_include("cart_model.h")
#include "cart_model.h"
#define CART_MODEL_AVAILABLE 1
#else
#define CART_MODEL_AVAILABLE 0
#endif
#if RELAY_FILTER_CFG == RELAY_FILTER_CART && !CART_MODEL_AVAILABLE
#error "RELAY_FILTER_CART needs include/cart_model.h: train it first with tools/train_cart.py"
#endif
constexpr uint8_t RELAY_FILTER = RELAY_FILTER_CFG;

// Weight multiplier M = 1 + alpha x Pf (Section 3.4). alpha = 2 lets a link that
// fails every time cost 3x its measured weight. Tune with -D CART_ALPHA_CFG=...
#ifndef CART_ALPHA_CFG
#define CART_ALPHA_CFG 2.0f
#endif
constexpr float CART_ALPHA = CART_ALPHA_CFG;

// Heuristic rule (decision D4): a neighbour is usable only if all four hold.
constexpr float HEURISTIC_MIN_RSSI_DBM = -110.0f;
constexpr float HEURISTIC_MIN_SNR_DB = -5.0f;
constexpr uint32_t HEURISTIC_MAX_HEARTBEAT_AGE_MS = 75000UL;
constexpr float HEURISTIC_MIN_BATTERY_PCT = 20.0f;

const char *relayFilterName() {
    return RELAY_FILTER == RELAY_FILTER_CART ? "CART" : RELAY_FILTER == RELAY_FILTER_HEURISTIC ? "heuristic" : "none";
}

struct NeighborVerdict {
    bool known;        // a heartbeat from this neighbour has been heard
    float pf;          // CART failure probability, < 0 = no model or no data
    bool cartRisky;    // pf >= CART_RISKY_THRESHOLD
    bool heurRisky;    // fails the D4 rule
};

// Classifies one neighbour from its latest heartbeat telemetry (neighborTable):
// the same values the CART training rows were built from.
NeighborVerdict classifyNeighbor(uint8_t node, uint32_t now) {
    NeighborVerdict v{false, -1.0f, false, true};
    if (node < 1 || node > NETWORK_NODE_COUNT || node == NODE_ID) return v;
    const NeighborInfo &n = neighborTable[node - 1];
    if (!n.valid) return v;                                  // never heard: heuristic says Risky
    v.known = true;
    const uint32_t ageMs = (uint32_t)(now - n.lastHeardMs);
    const bool batteryKnown = n.batteryPermille != BATTERY_UNKNOWN;
    const float batteryPct = batteryKnown ? n.batteryPermille / 10.0f : 100.0f;   // unknown: no battery penalty
    v.heurRisky = n.rssi <= HEURISTIC_MIN_RSSI_DBM || n.snr <= HEURISTIC_MIN_SNR_DB ||
                  ageMs > HEURISTIC_MAX_HEARTBEAT_AGE_MS ||
                  (batteryKnown && batteryPct < HEURISTIC_MIN_BATTERY_PCT);
#if CART_MODEL_AVAILABLE
    float x[CART_FEATURE_COUNT] = {};
#ifdef CART_USES_RSSI
    x[CART_F_RSSI] = n.rssi;
#endif
#ifdef CART_USES_SNR
    x[CART_F_SNR] = n.snr;
#endif
#ifdef CART_USES_BATTERY
    x[CART_F_BATTERY] = batteryPct;
#endif
#ifdef CART_USES_TRAFFIC
    x[CART_F_TRAFFIC] = n.trafficLoad;
#endif
#ifdef CART_USES_HB_AGE
    x[CART_F_HB_AGE] = (float)(ageMs > 300000UL ? 300000UL : ageMs);
#endif
    v.pf = cartFailureProbability(x);
    v.cartRisky = v.pf >= CART_RISKY_THRESHOLD;
#endif
    return v;
}

// Applies the build's relay filter to THIS node's edges (the decision is local:
// each node judges its own neighbours). Runs after the graph is built, before Dijkstra.
void applyRelayFilter(uint8_t graph[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT], uint32_t now) {
    if (RELAY_FILTER == RELAY_FILTER_NONE) return;
    const uint8_t self = NODE_ID - 1;
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        const uint8_t i = node - 1;
        if (node == NODE_ID || graph[self][i] == 0 || graph[self][i] == LINK_WEIGHT_NONE) continue;
        const NeighborVerdict v = classifyNeighbor(node, now);
        uint8_t weight = graph[self][i];
        if (RELAY_FILTER == RELAY_FILTER_HEURISTIC) {
            if (v.heurRisky) weight = LINK_WEIGHT_NONE;
        } else if (v.pf >= 0.0f) {                           // CART: w' = w x (1 + alpha x Pf)
            const float scaled = weight * (1.0f + CART_ALPHA * v.pf);
            weight = scaled >= LINK_WEIGHT_MAX ? LINK_WEIGHT_MAX : (uint8_t)(scaled + 0.5f);
        }
        graph[self][i] = weight;
        graph[i][self] = weight;
    }
}

// This node's own row of the adjacency matrix: measured weight to each neighbour
// heard recently; 255 if not heard for LINK_STALE_MS or (optionally) marked DOWN
// after ACK failures.
void buildOwnLinkRow(uint32_t now, bool markDownLinks, uint8_t row[NETWORK_NODE_COUNT]) {
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        uint8_t &weight = row[node - 1];
        if (node == NODE_ID) { weight = 0; continue; }
        const LinkQuality &quality = linkQuality[node - 1];
        if (!quality.valid || (uint32_t)(now - quality.lastFrameMs) > LINK_STALE_MS ||
            (markDownLinks && isLinkDown(node, now))) {
            weight = LINK_WEIGHT_NONE;
            continue;
        }
        weight = linkWeightFromQuality(quality.rssiAvg, quality.snrAvg);
    }
}

// Global weighted matrix (Table 3.1) from our own row and the LSU database.
void buildRoutingGraph(uint32_t now, bool markDownLinks,
                       uint8_t graph[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT]) {
    uint8_t ownRow[NETWORK_NODE_COUNT];
    buildOwnLinkRow(now, markDownLinks, ownRow);
    linkStateDb.buildGraph(ownRow, adjacencyMatrix, USE_TOPOLOGY_MASK, now, LSDB_MAX_AGE_MS, graph);
}

// Nodes a message must not be sent back to: its origin and every relay it has
// already passed (carried in relayPath). Prevents ping-pong loops while the
// link-state databases are still converging.
uint8_t routeExclusionMask(const EmergencyMessage &message) {
    uint8_t mask = 0;
    if (message.originNode != NODE_ID && message.originNode >= 1 && message.originNode <= NETWORK_NODE_COUNT) {
        mask |= (uint8_t)(1U << (message.originNode - 1));
    }
    for (uint8_t slot = 0; slot < RELAY_PATH_SLOTS; slot++) {
        const uint8_t relay = relayPathAt(message.relayPath, slot);
        if (relay == 0) break;
        if (relay != NODE_ID) mask |= (uint8_t)(1U << (relay - 1));
    }
    return mask;
}

// Dijkstra on the measured, link-state-synchronized matrix.
// markDownLinks=false gives the "normal" route, used only to detect reroutes.
bool selectNextHop(const EmergencyMessage &message, uint8_t &nextHopNode, uint32_t now,
                   bool markDownLinks = true, uint16_t *pathCost = nullptr) {
    uint8_t graph[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT];
    buildRoutingGraph(now, markDownLinks, graph);
    const uint8_t excluded = routeExclusionMask(message);
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        if (!(excluded & (1U << (node - 1))) || node == message.destinationNode) continue;
        for (uint8_t other = 0; other < NETWORK_NODE_COUNT; other++) {
            graph[node - 1][other] = LINK_WEIGHT_NONE;
            graph[other][node - 1] = LINK_WEIGHT_NONE;
        }
    }
    applyRelayFilter(graph, now);   // heuristic: drop risky neighbours; CART: scale by 1 + alpha x Pf
    return dijkstraNextHop(graph, NODE_ID, message.destinationNode, nextHopNode, pathCost) &&
           nextHopNode != NODE_ID;
}

// ---------------------------------------------------------------------------
// DTN retry timing
// message.retryExponent counts failed DTN polls for that message.
// ---------------------------------------------------------------------------
uint32_t dtnPollDelayMs(uint8_t failedPolls) {
    const uint8_t shift = failedPolls > 3 ? 3 : failedPolls;    // 10 s, 20 s, 40 s, then 60 s cap
    uint32_t delayMs = DTN_POLL_BASE_MS << shift;
    if (delayMs > DTN_POLL_MAX_MS) delayMs = DTN_POLL_MAX_MS;
    return delayMs + (esp_random() % 2000);                       // jitter so nodes do not poll in sync
}

// Short retry for a buffered packet (channel busy, or a route is still available).
void deferDtnShort(const EmergencyMessage &message, uint32_t now, uint32_t delayMs) {
    dtnBuffer.defer(message.id, message.originNode, now + delayMs, message.retryExponent);
}

// No route at all: wait for a polling interval measured in seconds.
void deferDtnPoll(const EmergencyMessage &message, uint32_t now) {
    const uint32_t delayMs = dtnPollDelayMs(message.retryExponent);
    const uint8_t failedPolls = message.retryExponent < 255 ? message.retryExponent + 1 : 255;
    dtnBuffer.defer(message.id, message.originNode, now + delayMs, failedPolls);
    Serial.printf("[DTN] Packet V%u#%04u has no usable route; next poll in %lu s.\n",
                  message.originNode, message.id, (unsigned long)(delayMs / 1000));
}

// A few random slots: used after a lost ACK while a route still exists.
uint32_t shortRetryDelayMs() {
    return macSlotTimeMs * (1 + (esp_random() % 4));
}

// Moves a queue message into DTN custody, evicting a less urgent one if full.
bool storeInDtn(const EmergencyMessage &original, uint32_t now) {
    bool evicted = false;
    EmergencyMessage evictedMessage{};
    EmergencyMessage message = original;
    message.dtnStoredAtMs = now ? now : 1;   // custody start, for the FORWARD row's d_buffer
    if (!dtnBuffer.storeWithEviction(message, now, evicted, evictedMessage)) return false;
    queueRemoveById(message.id, message.originNode);
    if (evicted) {
        Serial.printf("[DTN] Buffer full; evicted k=%u packet V%u#%04u to keep k=%u packet V%u#%04u.\n",
                      evictedMessage.priority, evictedMessage.originNode, evictedMessage.id,
                      message.priority, message.originNode, message.id);
        char detail[40];
        snprintf(detail, sizeof(detail), "dtn;admit=V%u#%u", message.originNode, message.id);
        CsvEvent e = csvEvent("EVICT", evictedMessage, 0);
        e.detail = detail;
        logCsv(e);
    }
    CsvEvent stored = csvEvent("STORE", message, 0);
    stored.detail = "dtn";
    logCsv(stored);
    Serial.printf("[DTN] Stored packet V%u#%04u (%u/%u bytes).\n",
                  message.originNode, message.id, (unsigned int)dtnBuffer.usedBytes(),
                  (unsigned int)DtnBuffer::B_MAX_BYTES);
    return true;
}

// Defers a message that cannot be sent right now, without blocking the queue.
// routeExists=false -> long poll; true -> short retry.
// Without DTN (Configs 1-3): failed hops per message, so a message is dropped
// after NO_DTN_MAX_ATTEMPTS tries instead of being held.
struct HopAttempts { uint8_t origin; uint16_t id; uint8_t count; };
HopAttempts hopAttempts[8] = {};

uint8_t countFailedAttempt(const EmergencyMessage &message) {
    HopAttempts *slot = nullptr;
    for (HopAttempts &a : hopAttempts) {
        if (a.count && a.origin == message.originNode && a.id == message.id) { slot = &a; break; }
    }
    if (!slot) {
        for (HopAttempts &a : hopAttempts) if (!a.count) { slot = &a; break; }
        if (!slot) slot = &hopAttempts[esp_random() % 8];
        *slot = HopAttempts{message.originNode, message.id, 0};
    }
    return ++slot->count;
}

void forgetAttempts(const EmergencyMessage &message) {
    for (HopAttempts &a : hopAttempts) {
        if (a.count && a.origin == message.originNode && a.id == message.id) a.count = 0;
    }
}

void dropWithoutDtn(const EmergencyMessage &message, const char *reason) {
    queueRemoveById(message.id, message.originNode);
    forgetAttempts(message);
    Serial.printf("[DROP] Packet V%u#%04u dropped (%s; DTN off in this configuration).\n",
                  message.originNode, message.id, reason);
    CsvEvent e = csvEvent("DROP", message, 0);
    e.detail = reason;
    logCsv(e);
}

void deferUnsentMessage(const EmergencyMessage &message, bool fromDtn, bool routeExists, uint32_t now) {
    if (!DTN_ENABLED) {
        // "Drop on link failure" (Table 4.1, Configs 1-3): no store-and-forward.
        if (!routeExists) {
            dropWithoutDtn(message, "no_route");
        } else if (countFailedAttempt(message) >= NO_DTN_MAX_ATTEMPTS) {
            dropWithoutDtn(message, "retries_exhausted");
        } else {
            nextMacAttemptAt = now + shortRetryDelayMs();   // stays in the queue, retried shortly
        }
        return;
    }
    if (!fromDtn && !storeInDtn(message, now)) {
        // DTN is full of equal/higher priority traffic: the message stays at the
        // head of the priority queue and the MAC pauses before trying again.
        Serial.printf("[DTN] Buffer full of equal/higher priority packets; V%u#%04u stays queued.\n",
                      message.originNode, message.id);
        nextMacAttemptAt = now + (routeExists ? shortRetryDelayMs() : DTN_POLL_BASE_MS);
        return;
    }
    if (routeExists) deferDtnShort(message, now, shortRetryDelayMs());
    else deferDtnPoll(message, now);
}

void retainUnacknowledgedMessage(uint32_t now) {
    recordAckFailure(radioTxNextHop, now);
    uint8_t alternateHop = 0;
    const bool routeExists = selectNextHop(activeTxMessage, alternateHop, now);
    Serial.printf("[LORA TX] No ACK from V%u for packet V%u#%04u; %s.\n",
                  radioTxNextHop, activeTxMessage.originNode, activeTxMessage.id,
                  routeExists ? "will retry shortly" : "holding in DTN");
    CsvEvent e = csvEvent("NOACK", activeTxMessage, radioTxNextHop);
    e.detail = routeExists ? "retry" : "hold";
    logCsv(e);
    radioAwaitingAck = false;
    resetMacBackoff();
    deferUnsentMessage(activeTxMessage, radioTxFromDtn, routeExists, now);
}

void resetMacBackoff() {
    macBackoffActive = false;
    macBusyRetries = 0;
    macBackoffExponent = MAC_MIN_BACKOFF_EXPONENT;
    nextMacAttemptAt = 0;
}

// True while the SX1262 is in the middle of receiving a frame (valid header seen,
// RX not finished). CAD only detects preambles, so this catches a neighbour that
// is already transmitting payload, which CAD alone would miss.
uint32_t headerValidSinceMs = 0;
bool radioReceivingInProgress() {
    const uint32_t flags = radio.getIrqFlags();
    if (!(flags & RADIOLIB_SX126X_IRQ_HEADER_VALID) || (flags & RADIOLIB_SX126X_IRQ_RX_DONE)) {
        headerValidSinceMs = 0;
        return false;
    }
    const uint32_t now = millis();
    if (headerValidSinceMs == 0) headerValidSinceMs = now;
    if (now - headerValidSinceMs > maxFrameAirtimeMs + 1000) {
        // Header flag never completed (aborted frame): reset RX and treat channel as free.
        headerValidSinceMs = 0;
        radio.startReceive();
        return false;
    }
    return true;
}

// Truncated Binary Exponential Backoff (Section 3.5.1):
// Wait = Random(0, 2^c - 1) x SlotTime, c capped at MAC_MAX_BACKOFF_EXPONENT.
// Total TBEB backoff/defer time the current message has waited (C16, "Wait" in 4.5.2).
uint16_t macWaitMessageId = 0;
uint8_t macWaitOriginNode = 0;
uint32_t macWaitAccumMs = 0;

void addMacWait(const EmergencyMessage &msg, uint32_t waitMs) {
    if (msg.id != macWaitMessageId || msg.originNode != macWaitOriginNode) {
        macWaitMessageId = msg.id;
        macWaitOriginNode = msg.originNode;
        macWaitAccumMs = 0;
    }
    macWaitAccumMs += waitMs;
}

// Wait logged on the TX row, then cleared for the next message.
uint32_t takeMacWait(const EmergencyMessage &msg) {
    const uint32_t wait = (msg.id == macWaitMessageId && msg.originNode == macWaitOriginNode) ? macWaitAccumMs : 0;
    macWaitAccumMs = 0;
    macWaitMessageId = 0;
    macWaitOriginNode = 0;
    return wait;
}

void handleChannelBusy(const EmergencyMessage &msg, bool fromDtn, const char *reason) {
    macBusyRetries++;
    if (dataBackoffsSinceHeartbeat < 255) dataBackoffsSinceHeartbeat++;
    const uint32_t now = millis();
    if (macBusyRetries >= MAC_MAX_CAD_RETRIES) {
        const uint32_t deferMs = maxFrameAirtimeMs + (esp_random() % macSlotTimeMs);
        addMacWait(msg, deferMs);
        Serial.printf("[LORA CAD] Channel still busy after %u checks; deferring packet V%u#%04u by %lu ms.\n",
                      MAC_MAX_CAD_RETRIES, msg.originNode, msg.id, (unsigned long)deferMs);
        resetMacBackoff();
        if (fromDtn) deferDtnShort(msg, now, deferMs);
        else nextMacAttemptAt = now + deferMs;
        return;
    }

    const uint32_t slotCount = 1U << macBackoffExponent;
    const uint32_t backoffMs = (esp_random() % slotCount) * macSlotTimeMs;
    addMacWait(msg, backoffMs);
    if (macBackoffExponent < MAC_MAX_BACKOFF_EXPONENT) macBackoffExponent++;
    nextMacAttemptAt = now + backoffMs;
    if (fromDtn) deferDtnShort(msg, now, backoffMs);
    Serial.printf("[LORA CAD] Busy (%s); TBEB retry %u/%u in %lu ms.\n",
                  reason, macBusyRetries, MAC_MAX_CAD_RETRIES, (unsigned long)backoffMs);
}

// Radio silenced (link failure / blackout test, Table 4.4 High tier): the node has
// no route, so queued messages are handled as "no route" -- DTN builds move them
// into the store-and-forward buffer (B_max and eviction apply), builds without DTN
// drop them. They leave the buffer once the radio is back on.
void handleQueueWhileSilent() {
    if ((int32_t)(millis() - nextMacAttemptAt) < 0) return;
    EmergencyMessage head{};
    if (!queuePeek(head)) return;
    if (head.destinationNode == NODE_ID) {
        queueRemoveById(head.id, head.originNode);
        return;
    }
    deferUnsentMessage(head, false, false, millis());
}

void serviceRadioMac() {
    if (!radioReady) return;
    if (radioSilent) {
        handleQueueWhileSilent();
        return;
    }

    if (radioAckTxInFlight) {
        if (!radioTxDone) return;
        radioTxDone = false;
        int16_t finishState = radio.finishTransmit();
        markRadioTx(false);
        radioAckTxInFlight = false;
        if (finishState != RADIOLIB_ERR_NONE) {
            Serial.printf("[LORA ACK] Completion error %d.\n", finishState);
        }
        radio.startReceive();
        return;
    }

    if (radioTxInFlight) {
        if (!radioTxDone) return;
        radioTxDone = false;
        int16_t finishState = radio.finishTransmit();
        markRadioTx(false);
        radioTxInFlight = false;
        if (radioTxIsHeartbeat) {
            // Broadcast heartbeat: nothing to wait for, go straight back to RX.
            radioTxIsHeartbeat = false;
            if (finishState != RADIOLIB_ERR_NONE) {
                Serial.printf("[HB TX] Completion error %d.\n", finishState);
            }
            radio.startReceive();
            return;
        }
        if (finishState == RADIOLIB_ERR_NONE) {
            radio.startReceive();
            radioAwaitingAck = true;
            const uint32_t ackAirTimeMs = (uint32_t)(radio.getTimeOnAir(ACK_PACKET_SIZE) / 1000);
            radioAckDeadlineMs = millis() + ackAirTimeMs + ACK_TIMEOUT_GUARD_MS;
            Serial.printf("[LORA TX] Packet V%u#%04u sent (%u bytes); waiting for next-hop ACK.\n",
                          radioTxOriginNode, radioTxMessageId, (unsigned int)radioTxPacketLen);
        } else {
            Serial.printf("[LORA TX] Completion error %d for packet V%u#%04u; retaining it in DTN.\n",
                          finishState, radioTxOriginNode, radioTxMessageId);
            radio.startReceive();
            radioAwaitingAck = true;
            radioAckDeadlineMs = millis();
        }
        return;
    }

    if (radioAwaitingAck) {
        if (deadlineReached(millis(), radioAckDeadlineMs)) retainUnacknowledgedMessage(millis());
        else return;
    }

    // Heartbeat slot. A due heartbeat (about 1.3 s on air) goes out when the radio
    // is idle, unless a k=1 SOS is ready to send: SOS keeps the next transmission
    // window (Section 3.2), and the heartbeat waits for the following one.
    if (heartbeatPending && deadlineReached(millis(), nextHeartbeatTryAtMs)) {
        // A k=1 SOS keeps the slot only if it can actually be sent now: the MAC is
        // not backing off and a route exists. Otherwise an SOS with no route would
        // block heartbeats forever, neighbours would drop the link, and the route
        // would never come back (deadlock). A heartbeat also never waits more than
        // one interval.
        EmergencyMessage head{};
        const uint32_t nowMs = millis();
        const bool haveSos = (queuePeek(head) && head.priority == 1) ||
                             (dtnBuffer.peekDue(nowMs, head) && head.priority == 1);
        uint8_t sosHop = 0;
        const bool sosSendable = haveSos && (int32_t)(nowMs - nextMacAttemptAt) >= 0 &&
                                 selectNextHop(head, sosHop, nowMs);
        const bool heartbeatOverdue = (uint32_t)(nowMs - heartbeatPendingSinceMs) >= HEARTBEAT_INTERVAL_MS;
        if (!sosSendable || heartbeatOverdue) {
            transmitHeartbeat(millis());
            return;
        }
    }

    if ((int32_t)(millis() - nextMacAttemptAt) < 0) return;

    // Candidates: the priority-queue root and the most urgent DTN packet whose
    // retry time has come. DTN packets waiting for their poll are not "due", so
    // a packet stuck behind a dead link never blocks the rest of the queue.
    EmergencyMessage queuedMessage{};
    EmergencyMessage dtnMessage{};
    const bool haveQueuedMessage = queuePeek(queuedMessage);
    const bool haveDtnMessage = dtnBuffer.peekDue(millis(), dtnMessage);
    // Same ordering rule as the heap: k, then local enqueue time, then arrival counter.
    const bool fromDtn = haveDtnMessage &&
        (!haveQueuedMessage || messageOutranks(dtnMessage, queuedMessage));
    const bool haveMessage = haveQueuedMessage || haveDtnMessage;
    if (!haveMessage) {
        resetMacBackoff();
        return;
    }
    EmergencyMessage msg = fromDtn ? dtnMessage : queuedMessage;
    if (msg.destinationNode == NODE_ID) {
        if (fromDtn) dtnBuffer.remove(msg.id, msg.originNode);
        else queueRemoveById(msg.id, msg.originNode);
        resetMacBackoff();
        return;
    }
    if (deadlineReached(millis(), msg.expiresAtMs)) {
        if (fromDtn) dtnBuffer.remove(msg.id, msg.originNode);
        else queueRemoveById(msg.id, msg.originNode);
        Serial.printf("[DROP] Packet V%u#%04u expired before transmission.\n", msg.originNode, msg.id);
        CsvEvent e = csvEvent("DROP", msg, 0);
        e.detail = "expired";
        logCsv(e);
        resetMacBackoff();
        return;
    }

    uint8_t nextHopNode = 0;
    uint16_t routeCost = 0;
    if (!selectNextHop(msg, nextHopNode, millis(), true, &routeCost)) {
        Serial.printf("[ROUTE] No usable path from V%u to V%u for packet V%u#%04u.\n",
                      NODE_ID, msg.destinationNode, msg.originNode, msg.id);
        resetMacBackoff();
        deferUnsentMessage(msg, fromDtn, false, millis());
        return;
    }
    if (!macBackoffActive || macBackoffMessageId != msg.id ||
        macBackoffOriginNode != msg.originNode) {
        macBackoffActive = true;
        macBackoffMessageId = msg.id;
        macBackoffOriginNode = msg.originNode;
        macBusyRetries = 0;
        macBackoffExponent = MAC_MIN_BACKOFF_EXPONENT;
        nextMacAttemptAt = 0;
    }

    // An RX interrupt that has not been serviced yet must be handled before CAD
    // reconfigures the radio, otherwise that received packet would be lost.
    if (radioIrqPending || radioRxPending) return;

    // Listen-before-talk, step 1: a frame is already being received.
    if (radioReceivingInProgress()) {
        handleChannelBusy(msg, fromDtn, "frame in progress");
        return;
    }

    // Listen-before-talk, step 2: CAD scan for a preamble.
    int16_t cadState = radio.scanChannel();
    // scanChannel() is blocking and its CAD_DONE event also fires DIO1. That
    // interrupt is fully consumed here; clearing the flag stops it from being
    // misread as TX-done for the packet started below, or as a received packet.
    radioIrqPending = false;
    if (cadState == RADIOLIB_CHANNEL_FREE) {
        size_t packetLen = 0;
        if (!serializeLoRaPacket(msg, nextHopNode, radioPacket, sizeof(radioPacket), packetLen)) {
            Serial.printf("[DROP] Packet V%u#%04u is invalid or exceeds the %u-byte MTU.\n",
                          msg.originNode, msg.id, (unsigned int)MAX_LORA_PACKET_SIZE);
            CsvEvent e = csvEvent("DROP", msg, nextHopNode);
            e.detail = "invalid";
            logCsv(e);
            if (fromDtn) dtnBuffer.remove(msg.id, msg.originNode);
            else queueRemoveById(msg.id, msg.originNode);
            resetMacBackoff();
            radio.startReceive();
            return;
        }

        uint8_t staticHop = 0;   // best hop if no link were marked DOWN
        const bool rerouted = selectNextHop(msg, staticHop, millis(), false) && staticHop != nextHopNode;
        if (rerouted) {
            Serial.printf("[ROUTE] Packet V%u#%04u rerouted via V%u (V%u is down).\n",
                          msg.originNode, msg.id, nextHopNode, staticHop);
        }

        radioTxDone = false;
        radioTxMessageId = msg.id;
        radioTxOriginNode = msg.originNode;
        radioTxNextHop = nextHopNode;
        activeTxMessage = msg;
        radioTxFromDtn = fromDtn;
        radioTxPacketLen = packetLen;
        int16_t txState = radio.startTransmit(radioPacket, packetLen);
        if (txState == RADIOLIB_ERR_NONE) {
            radioTxInFlight = true;
            radioTxStartedAtMs = millis();
            markRadioTx(true);
            Serial.printf("[LORA TX] Started packet V%u#%04u to V%u, %u bytes, header 0x%02X.\n",
                          msg.originNode, msg.id, nextHopNode, (unsigned int)packetLen, radioPacket[0]);
            // Feature row for relay viability: peer telemetry at decision time;
            // the matching ACK / NOACK row for the same origin,id,peer is the label.
            CsvEvent e = csvEvent("TX", msg, nextHopNode);
            const NeighborVerdict verdict = classifyNeighbor(nextHopNode, millis());   // C10: log both verdicts
            const uint32_t waited = takeMacWait(msg);
            e.waitMs = waited > INT32_MAX ? INT32_MAX : (int32_t)waited;
            e.queueIndex = msg.queuePosAtEntry;
            e.hasVerdict = true;
            e.cartPf = verdict.pf;
            e.cartRisky = verdict.cartRisky;
            e.heurRisky = verdict.heurRisky;
            e.macRetries = macBusyRetries;
            e.bytes = (uint16_t)packetLen;
            fillQueueState(e, msg);
            char detail[40];
            snprintf(detail, sizeof(detail), "%s%s;cost=%u", fromDtn ? "dtn" : "queue",
                     rerouted ? ";reroute" : "", routeCost);
            e.detail = detail;
            logCsv(e);
        } else {
            Serial.printf("[LORA TX] Start error %d for packet V%u#%04u; retrying later.\n", txState, msg.originNode, msg.id);
            radio.startReceive();
            resetMacBackoff();
            if (fromDtn) deferDtnShort(msg, millis(), maxFrameAirtimeMs);
            else nextMacAttemptAt = millis() + maxFrameAirtimeMs;
        }
        return;
    }

    radio.startReceive();
    if (cadState != RADIOLIB_PREAMBLE_DETECTED) {
        Serial.printf("[LORA CAD] Scan error %d; retaining packet V%u#%04u.\n", cadState, msg.originNode, msg.id);
        resetMacBackoff();
        if (fromDtn) deferDtnShort(msg, millis(), maxFrameAirtimeMs);
        else nextMacAttemptAt = millis() + maxFrameAirtimeMs;
        return;
    }
    handleChannelBusy(msg, fromDtn, "preamble detected");
}

// ---------------------------------------------------------------------------
// Creates a message originating at this node and queues it for the hub.
// Used by the captive portal (/api/submit) and by the serial "sos" command
// (automated traffic for field tests and CART data collection).
// timestampTx/timestampMs = creation time (Unix s + ms); timestampTx 0 = use this node's clock.
// ---------------------------------------------------------------------------
enum SubmitResult : uint8_t { SUBMIT_OK, SUBMIT_BAD_SIZE, SUBMIT_ENCRYPT_FAILED, SUBMIT_QUEUE_FULL };

SubmitResult submitOwnMessage(uint8_t priority, uint32_t timestampTx, uint16_t timestampMs, const char *location,
                              const char *message, const char *via, EmergencyMessage &msg) {
    if (priority < 1 || priority > 3) priority = 1;
    if (!location || !location[0]) location = "Unspecified";
    if (!message) message = "";

    // Location + message share one 200-byte budget so the encrypted
    // frame always fits the 250-byte LoRa MTU (see packet_codec.h).
    const size_t locationBytes = strlen(location);
    const size_t messageBytes = strlen(message);
    if (messageBytes == 0 || locationBytes >= sizeof(EmergencyMessage::location) ||
        messageBytes >= sizeof(EmergencyMessage::payload) ||
        locationBytes + messageBytes > MAX_TEXT_BYTES) {
        return SUBMIT_BAD_SIZE;
    }

    msg = EmergencyMessage{};
    msg.id = allocateMessageId();
    msg.priority = priority;
    // Creation time: the phone's clock if it sent one, else this node's clock
    // when it has been set (serial "time" command), else seconds since boot.
    if (timestampTx) {
        msg.timestamp_tx = timestampTx;
        msg.timestamp_ms = timestampMs > 999 ? 0 : timestampMs;
    } else {
        const uint64_t nodeMs = epochNowMs();
        const uint64_t fallbackMs = nodeMs ? nodeMs : (uint64_t)millis();   // uptime if the clock is unset
        msg.timestamp_tx = (uint32_t)(fallbackMs / 1000ULL);
        msg.timestamp_ms = (uint16_t)(fallbackMs % 1000ULL);
    }
    msg.originNode = NODE_ID;
    msg.destinationNode = HUB_NODE_ID;
    msg.ttl = INITIAL_TTL;
    msg.packetType = priority == 1 ? PACKET_TYPE_SOS : PACKET_TYPE_TELEMETRY;
    msg.expiresAtMs = millis() + DTN_RETENTION_MS;
    msg.retryExponent = 0;   // failed DTN polls
    memcpy(msg.location, location, locationBytes);
    msg.location[locationBytes] = '\0';
    memcpy(msg.payload, message, messageBytes);
    msg.payload[messageBytes] = '\0';

    // Binary body + AES-128 CBC Encryption (Sections 3.5.2, 4.3)
    uint8_t plainBody[MAX_PLAINTEXT_BODY_SIZE];
    size_t plainLen = 0;
    size_t encLen = 0;
    const bool encryptSuccess =
        buildMessageBody(msg.timestamp_tx, msg.timestamp_ms, msg.location, msg.payload,
                         plainBody, sizeof(plainBody), plainLen) &&
        AESCrypto::encrypt(plainBody, plainLen, msg.encryptedData, encLen,
                           MAX_ENCRYPTED_BODY_SIZE);
    memset(plainBody, 0, sizeof(plainBody));
    if (!encryptSuccess) {
        Serial.println("[ERROR] Message encryption failed; submission rejected.");
        return SUBMIT_ENCRYPT_FAILED;
    }
    msg.encryptedLen = (uint8_t)encLen;

    // Insert into Dual-Key Binary Min-Heap Priority Queue (Section 3.2)
    if (!queuePush(msg)) {
        Serial.println("[ERROR] Priority queue is full; submission rejected.");
        return SUBMIT_QUEUE_FULL;
    }
    // Our own message is "seen": if it ever loops back here it is dropped.
    seenListAdd(msg.originNode, msg.id);
    {
        char detail[40];
        snprintf(detail, sizeof(detail), "ts=%lu.%03u;via=%s", (unsigned long)msg.timestamp_tx,
                 msg.timestamp_ms, via);
        CsvEvent e = csvEvent("NEW", msg, 0);
        e.bytes = (uint16_t)(msg.encryptedLen + FRAME_OVERHEAD_SIZE);
        e.detail = detail;
        logCsv(e);
    }

    // Serial Telemetry Output
    Serial.println("\n=======================================================");
    Serial.printf("  [+] EMERGENCY MESSAGE ACCEPTED & QUEUED (via %s)\n", via);
    Serial.println("=======================================================");
    Serial.printf("  Ref ID        : V%u#%04u\n", msg.originNode, msg.id);
    Serial.printf("  Priority      : k=%u (%s)\n", msg.priority,
                  msg.priority == 1 ? "MEDICAL SOS (Highest)" : (msg.priority == 2 ? "GENERAL EMERGENCY" : "ROUTINE/INFO"));
    Serial.printf("  Location      : %s\n", msg.location);
    Serial.printf("  Message Text  : %s\n", msg.payload);
    Serial.printf("  Security      : %s (Encrypted body: %u bytes, LoRa frame: %u bytes)\n",
                  "AES-128-CBC + HMAC-SHA-256/128 frame tag", msg.encryptedLen,
                  (unsigned int)(msg.encryptedLen + FRAME_OVERHEAD_SIZE));
    Serial.printf("  Min-Heap State: %s (Current Queue Size: %u)\n",
                  "Inserted at O(log n)", (unsigned int)queueSize());
    Serial.printf("  Connected Stns: %u / %u max\n", WiFi.softAPgetStationNum(), MAX_AP_CLIENTS);
    Serial.println("=======================================================\n");
    return SUBMIT_OK;
}

// Joins the /api/submit body when it arrives in several pieces. The buffer
// hangs off the request and the web server frees it with the request.
void collectSubmitBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    if (index == 0) {
        if (request->_tempObject) {
            free(request->_tempObject);
            request->_tempObject = nullptr;
        }
        if (total == 0 || total > SUBMIT_BODY_MAX) return;   // answered in the request handler
        SubmitBody *body = (SubmitBody *)malloc(sizeof(SubmitBody) + total + 1);
        if (!body) return;                                   // out of memory: answered as incomplete
        body->total = total;
        body->received = 0;
        body->data[total] = '\0';
        request->_tempObject = body;
    }
    SubmitBody *body = (SubmitBody *)request->_tempObject;
    if (!body || index + len > body->total) return;
    memcpy(body->data + index, data, len);
    if (index + len > body->received) body->received = index + len;
}

bool requireDashboardAuthentication(AsyncWebServerRequest *request) {
    if (request->authenticate(ADMIN_DASHBOARD_USERNAME, ADMIN_DASHBOARD_PASSWORD)) return true;
    request->requestAuthentication("Emergency Hub Admin");
    return false;
}

void setupWebServerRoutes() {
    // 1. Captive Portal UI (Root)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", CAPTIVE_PORTAL_HTML);
    });

    // Register dashboard routes only on the stationary Main Hub V1.
    if (NODE_ID == HUB_NODE_ID) {
        server.on("/admin", HTTP_GET, [](AsyncWebServerRequest *request) {
            if (!requireDashboardAuthentication(request)) return;
            request->send_P(200, "text/html", ADMIN_DASHBOARD_HTML);
        });

        server.on("/api/live_incidents", HTTP_GET, [](AsyncWebServerRequest *request) {
            if (!requireDashboardAuthentication(request)) return;
            if (!liveIncidentMutex ||
                xSemaphoreTake(liveIncidentMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                request->send(503, "application/json", "{\"error\":\"Incident buffer busy\"}");
                return;
            }

            JsonDocument document;
            JsonArray incidents = document.to<JsonArray>();
            const size_t oldestIndex = (liveIncidentWriteIndex + LIVE_INCIDENT_CAPACITY -
                                        liveIncidentCount) % LIVE_INCIDENT_CAPACITY;
            for (size_t i = 0; i < liveIncidentCount; i++) {
                const LiveIncident &incident = liveIncidents[(oldestIndex + i) % LIVE_INCIDENT_CAPACITY];
                JsonObject row = incidents.add<JsonObject>();
                row["id"] = incident.messageId;
                row["priority"] = incident.priority;
                row["packetType"] = incident.packetType;
                row["originNode"] = incident.originNode;
                char originLabel[5];
                snprintf(originLabel, sizeof(originLabel), "V%u", incident.originNode);
                row["origin"] = originLabel;
                row["route"] = incident.path;
                if (incident.latencyMs >= 0) row["latencyMs"] = incident.latencyMs;
                else row["latencyMs"] = nullptr;
                row["location"] = incident.location;
                row["message"] = incident.message;
                row["crypto"] = "AES-128-CBC + HMAC-SHA-256/128";
                row["rxTime"] = incident.receivedAtMs;
                row["arrival"] = incident.arrivalNo;
            }
            xSemaphoreGive(liveIncidentMutex);

            AsyncResponseStream *response = request->beginResponseStream("application/json");
            serializeJson(document, *response);
            request->send(response);
        });
    }

    // Android, Windows, and Chrome captive portal probes
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/gen_204", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });

    // Apple iOS/macOS Captive Network Assistant (CNA) probe
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });

    // 3. Emergency Submission Endpoint (Intake & Security Pipeline)
    // The body handler (collectSubmitBody) runs once per received piece; this
    // handler runs once, after the whole body has arrived.
    server.on("/api/submit", HTTP_POST, [](AsyncWebServerRequest *request) {
            if (request->contentLength() > SUBMIT_BODY_MAX) {
                request->send(413, "application/json", "{\"status\":\"error\",\"message\":\"Request too large.\"}");
                return;
            }
            const SubmitBody *body = (const SubmitBody *)request->_tempObject;
            if (!body || body->received != body->total) {
                request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Empty or incomplete request. Please try again.\"}");
                return;
            }
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, body->data, body->total);

            if (err) {
                request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Malformed JSON payload\"}");
                return;
            }

            // Step A: Community Access Credential Verification (Section 4.3 Anti-Spoofing)
            const char *credential = doc["credential"] | "";
            if (!AESCrypto::verifyCommunityCredential(credential)) {
                Serial.println("[SECURITY ALERT] Rejected submission: Invalid community credential!");
                request->send(403, "application/json", "{\"status\":\"error\",\"message\":\"Invalid Community Passcode. Access Denied.\"}");
                return;
            }
            if (!radioReady) {
                request->send(503, "application/json", "{\"status\":\"error\",\"message\":\"LoRa radio is unavailable. Message was not queued.\"}");
                return;
            }

            // Step B: Extract Message Data
            int priorityVal = doc["priority"] | 1;
            uint8_t priority = (priorityVal >= 1 && priorityVal <= 3) ? (uint8_t)priorityVal : 1;
            const char *location = doc["location"] | "Unspecified";
            const char *message = doc["message"] | "";
            const uint32_t phoneTs = (uint32_t)(doc["timestamp"] | (uint32_t)0);
            const uint16_t phoneTsMs = (uint16_t)(doc["timestamp_ms"] | (uint16_t)0);

            // Steps C-E: body, encryption, min-heap insert, logging (shared with the
            // serial "sos" command used for automated field data collection).
            EmergencyMessage msg{};
            const SubmitResult result = submitOwnMessage(priority, phoneTs, phoneTsMs, location, message, "web", msg);
            if (result == SUBMIT_BAD_SIZE) {
                char errorJson[160];
                snprintf(errorJson, sizeof(errorJson),
                         "{\"status\":\"error\",\"message\":\"Location and message together must be 1-%u bytes (got %u).\"}",
                         (unsigned int)MAX_TEXT_BYTES, (unsigned int)(strlen(location) + strlen(message)));
                request->send(413, "application/json", errorJson);
                return;
            }
            if (result == SUBMIT_ENCRYPT_FAILED) {
                request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Message could not be encrypted. Please try again.\"}");
                return;
            }
            if (result == SUBMIT_QUEUE_FULL) {
                request->send(503, "application/json", "{\"status\":\"error\",\"message\":\"The message queue is full. Please try again shortly.\"}");
                return;
            }

            // Step F: Immediate JSON Response with Connect-Queue-Disconnect Header
            char responseJson[96];
            snprintf(responseJson, sizeof(responseJson), "{\"status\":\"queued\",\"ref\":\"V%u#%04u\",\"priority\":%u,\"queueSize\":%u}",
                     msg.originNode, msg.id, msg.priority, (unsigned int)queueSize());

            AsyncWebServerResponse *response = request->beginResponse(200, "application/json", responseJson);
            response->addHeader("Connection", "close"); // Instruct client to close connection to free socket
            request->send(response);
        },
        NULL, collectSubmitBody);

    // Fallback: Redirect any other URL to Captive Portal
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->redirect("/");
    });
}

// ---------------------------------------------------------------------------
// Serial commands (115200 baud, one command per line)
//   silent on   - radio stops transmitting AND receiving (simulates a dead node /
//                 lost link for DTN tests; use this instead of removing the antenna)
//   silent off  - radio returns to normal operation
//   status      - prints queue, DTN, link and neighbour state
//   routes      - prints the live link-state matrix (Table 3.1 layout) and route to the hub
//   time <unix> - sets the node clock (sent automatically by capture_telemetry.py)
//   battery     - prints the battery level (estimate on power banks, VBAT on LiPo)
//   energy ...  - power-bank energy estimate: calibrate currents, mark a full bank
//                 (type "energy" for the list of sub-commands)
// ---------------------------------------------------------------------------
char serialLine[256];   // long enough for "sos <k> <location>|<message>"
size_t serialLineLen = 0;

void printNodeStatus() {
    const uint32_t now = millis();
    Serial.printf("[STATUS] V%u radio=%s queue=%u dtn=%u slot=%lums\n", NODE_ID,
                  radioSilent ? "SILENT" : "ON", (unsigned int)queueSize(),
                  (unsigned int)dtnBuffer.size(), (unsigned long)macSlotTimeMs);
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        if (node == NODE_ID) continue;
        const NeighborInfo &neighbor = neighborTable[node - 1];
        const LinkState &link = linkState[node - 1];
        if (!neighbor.valid && !link.down && link.consecutiveAckFailures == 0) continue;
        Serial.printf("[STATUS]  V%u link=%s ackFails=%u", node, link.down ? "DOWN" : "up",
                      link.consecutiveAckFailures);
        if (neighbor.valid) {
            char battery[12];
            Serial.printf(" lastHB=%lus ago rssi=%.1f snr=%.1f batt=%s traffic=%u",
                          (unsigned long)((now - neighbor.lastHeardMs) / 1000), neighbor.rssi,
                          neighbor.snr, batteryText(neighbor.batteryPermille, battery, sizeof(battery)),
                          neighbor.trafficLoad);
        }
        Serial.println();
    }
}

// Prints the live weighted adjacency matrix in the layout of Table 3.1.
void printRoutingTable() {
    const uint32_t now = millis();
    uint8_t graph[NETWORK_NODE_COUNT][NETWORK_NODE_COUNT];
    buildRoutingGraph(now, true, graph);
    Serial.printf("[ROUTES] V%u link-state matrix (255 = no edge, %s):\n", NODE_ID,
                  USE_TOPOLOGY_MASK ? "planned links only" : "all measured links");
    Serial.print("[ROUTES]      ");
    for (uint8_t v = 1; v <= NETWORK_NODE_COUNT; v++) Serial.printf("  V%u", v);
    Serial.println();
    for (uint8_t u = 0; u < NETWORK_NODE_COUNT; u++) {
        Serial.printf("[ROUTES]  V%u ", u + 1);
        for (uint8_t v = 0; v < NETWORK_NODE_COUNT; v++) Serial.printf(" %3u", graph[u][v]);
        const bool own = u + 1 == NODE_ID;
        const bool fresh = own || linkStateDb.isFresh(u + 1, now, LSDB_MAX_AGE_MS);
        if (own) Serial.println("   (own row, measured)");
        else if (fresh) Serial.printf("   (LSU seq %u, %lus ago)\n", linkStateDb.row(u + 1).sequence,
                                      (unsigned long)((now - linkStateDb.row(u + 1).lastAdvanceMs) / 1000));
        else Serial.println("   (no LSU yet / stale)");
    }
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        if (node == NODE_ID || !linkQuality[node - 1].valid) continue;
        const LinkQuality &q = linkQuality[node - 1];
        Serial.printf("[ROUTES]  link V%u-V%u: avg RSSI %.1f dBm, SNR %.1f dB -> weight %u (heard %lus ago)\n",
                      NODE_ID, node, q.rssiAvg, q.snrAvg, linkWeightFromQuality(q.rssiAvg, q.snrAvg),
                      (unsigned long)((now - q.lastFrameMs) / 1000));
    }
    Serial.printf("[ROUTES] Relay filter: %s%s\n", relayFilterName(),
                  CART_MODEL_AVAILABLE ? (RELAY_FILTER == RELAY_FILTER_CART ? "" : " (CART model loaded, shadow mode)")
                                       : " (no CART model in this build)");
    for (uint8_t node = 1; node <= NETWORK_NODE_COUNT; node++) {
        if (node == NODE_ID || !neighborTable[node - 1].valid) continue;
        const NeighborVerdict v = classifyNeighbor(node, now);
        if (v.pf >= 0.0f)
            Serial.printf("[ROUTES]  V%u: CART Pf %.2f -> %s (x%.2f weight); heuristic %s\n", node, v.pf,
                          v.cartRisky ? "Risky" : "Safe", 1.0f + CART_ALPHA * v.pf, v.heurRisky ? "Risky" : "Safe");
        else
            Serial.printf("[ROUTES]  V%u: heuristic %s\n", node, v.heurRisky ? "Risky" : "Safe");
    }
    EmergencyMessage probe{};
    probe.originNode = NODE_ID;
    probe.destinationNode = HUB_NODE_ID;
    uint8_t nextHop = 0;
    uint16_t cost = 0;
    if (NODE_ID == HUB_NODE_ID) Serial.println("[ROUTES] This node is the hub.");
    else if (selectNextHop(probe, nextHop, now, true, &cost))
        Serial.printf("[ROUTES] Route to hub V%u: next hop V%u, total cost %u\n", HUB_NODE_ID, nextHop, cost);
    else Serial.printf("[ROUTES] No route to hub V%u yet (waiting for heartbeats).\n", HUB_NODE_ID);
}

// "sos <k> <location>|<message>" or "sos <k> <message>": queues a message from
// this node exactly as the captive portal would (tools/capture_telemetry.py
// --inject uses it to generate field traffic for CART data collection).
void handleSosCommand(const char *args) {
    char *end = nullptr;
    const long k = strtol(args, &end, 10);
    if (end == args || k < 1 || k > 3 || *end != ' ' || end[1] == '\0') {
        Serial.println("[CMD] Usage: sos <k 1-3> [location|]<message>   e.g.  sos 1 Purok 4|Test message");
        return;
    }
    if (NODE_ID == HUB_NODE_ID) {
        Serial.println("[CMD] This node is the hub; messages are only sent TO it. Use sos on V2-V6.");
        return;
    }
    if (!radioReady) {
        Serial.println("[CMD] LoRa radio is unavailable; message not queued.");
        return;
    }
    char text[256];
    strncpy(text, end + 1, sizeof(text) - 1);
    text[sizeof(text) - 1] = '\0';
    const char *location = "Serial";
    char *message = text;
    char *bar = strchr(text, '|');
    if (bar) {
        *bar = '\0';
        location = text;
        message = bar + 1;
    }
    EmergencyMessage msg{};
    const SubmitResult result = submitOwnMessage((uint8_t)k, 0, 0, location, message, "serial", msg);
    if (result == SUBMIT_BAD_SIZE)
        Serial.printf("[CMD] Location + message must be 1-%u bytes.\n", (unsigned int)MAX_TEXT_BYTES);
    else if (result != SUBMIT_OK)
        Serial.println("[CMD] Message not queued (see error above).");
}

void handleSerialCommand(const char *command) {
    if (!strcmp(command, "silent on")) {
        radioSilentRequested = true;
        Serial.println("[CMD] Radio silence requested (applies once the current transmission ends).");
    } else if (!strcmp(command, "silent off")) {
        radioSilentRequested = false;
        Serial.println("[CMD] Radio silence lifted.");
    } else if (!strcmp(command, "status")) {
        printNodeStatus();
    } else if (!strcmp(command, "routes")) {
        printRoutingTable();
    } else if (!strncmp(command, "battery cal ", 12)) {
        if (BATTERY_SOURCE == BATTERY_SOURCE_LIPO) handleBatteryCalibration(command + 12);
        else Serial.println("[BATTERY] Calibration needs BATTERY_SOURCE_LIPO.");
    } else if (!strcmp(command, "battery")) {
        if (BATTERY_SOURCE == BATTERY_SOURCE_ESTIMATE) printEnergyReport();
        else if (BATTERY_SOURCE == BATTERY_SOURCE_LIPO) printBatteryReport();
        else Serial.println("[BATTERY] Not measured (USB power bank). See BATTERY_SOURCE in main.cpp.");
    } else if (BATTERY_SOURCE == BATTERY_SOURCE_ESTIMATE &&
               (!strcmp(command, "energy") || !strncmp(command, "energy ", 7))) {
        handleEnergyCommand(command[6] ? command + 7 : "");
    } else if (!strncmp(command, "time ", 5)) {
        // Sets this node's real-time clock: "time <unix-seconds>[.milliseconds]".
        // tools/capture_telemetry.py sends it (with ms) to every node it connects to.
        char *fraction = nullptr;
        const unsigned long epoch = strtoul(command + 5, &fraction, 10);
        unsigned long ms = 0;
        if (fraction && *fraction == '.') {
            unsigned long scale = 100;
            for (const char *c = fraction + 1; *c >= '0' && *c <= '9' && scale; c++, scale /= 10) ms += (*c - '0') * scale;
        }
        if (epoch >= MIN_VALID_EPOCH) {
            struct timeval tv;
            tv.tv_sec = (time_t)epoch;
            tv.tv_usec = (suseconds_t)(ms * 1000UL);
            settimeofday(&tv, nullptr);
            Serial.printf("[CMD] Clock set to %lu.%03lu (Unix seconds).\n", epoch, ms);
            char detail[32];
            snprintf(detail, sizeof(detail), "epoch=%lu.%03lu", epoch, ms);
            CsvEvent e = csvEvent("TIME", 0, 0, 0, 0, 0);
            e.detail = detail;
            logCsv(e);
        } else {
            Serial.println("[CMD] Usage: time <unix-seconds>[.ms]");
        }
    } else if (!strncmp(command, "sos ", 4)) {
        handleSosCommand(command + 4);
    } else if (command[0] != '\0') {
        Serial.println("[CMD] Commands: silent on | silent off | status | routes | battery | battery cal <volts> | "
                       "time <unix-seconds>[.ms] | "
                       "sos <k 1-3> [location|]<message>");
    }
}

void serviceSerialCommands() {
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            serialLine[serialLineLen] = '\0';
            handleSerialCommand(serialLine);
            serialLineLen = 0;
        } else if (serialLineLen < sizeof(serialLine) - 1) {
            serialLine[serialLineLen++] = (char)c;
        }
    }
}

// Applies a requested silence change only when no frame is on the air.
void applyRadioSilence() {
    if (!radioReady || radioSilentRequested == radioSilent ||
        radioTxInFlight || radioAckTxInFlight) return;
    if (radioSilentRequested) {
        radio.standby();
        radioSilent = true;
        radioRxPending = false;
        Serial.println("[RADIO] SILENT: not transmitting or receiving. Messages keep queueing.");
    } else {
        radioIrqPending = false;
        radioRxPending = false;
        radio.startReceive();
        radioSilent = false;
        Serial.println("[RADIO] Back ON: receiving and transmitting again.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    detectBatteryControlPolarity();   // before anything drives GPIO37
    if (BATTERY_SOURCE == BATTERY_SOURCE_LIPO) batteryLoadCalibration();
    linkStateDb.begin(NODE_ID);
    if (BATTERY_SOURCE == BATTERY_SOURCE_ESTIMATE) energyBegin();

    priorityQueueMutex = xSemaphoreCreateMutex();
    if (!priorityQueueMutex) {
        Serial.println("[ERROR] Could not create priority queue mutex.");
    }
    csvLogMutex = xSemaphoreCreateMutex();
    liveIncidentMutex = xSemaphoreCreateMutex();
    if (!liveIncidentMutex) {
        Serial.println("[ERROR] Could not create live incident buffer mutex.");
    }

    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
    int16_t radioState = radio.begin(LORA_FREQUENCY_MHZ, 125.0, 12, 5,
                                     RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER_DBM, 8, 1.8, false);
    if (radioState == RADIOLIB_ERR_NONE) {
        radioState = radio.setDio2AsRfSwitch(true);
    }
    if (radioState == RADIOLIB_ERR_NONE) {
        radio.setDio1Action(onRadioDio1);
        radioState = radio.startReceive();
        if (radioState == RADIOLIB_ERR_NONE) {
            radioReady = true;
            // TBEB slot = airtime of the shortest exchange (an ACK); deferral after
            // repeated busy checks = airtime of the longest possible frame.
            macSlotTimeMs = (uint32_t)(radio.getTimeOnAir(ACK_PACKET_SIZE) / 1000);
            maxFrameAirtimeMs = (uint32_t)(radio.getTimeOnAir(MAX_LORA_PACKET_SIZE) / 1000);
            if (macSlotTimeMs == 0) macSlotTimeMs = 1500;
            if (maxFrameAirtimeMs == 0) maxFrameAirtimeMs = 9000;
            Serial.printf("[LoRa] SX1262 ready: %.1f MHz, SF12, BW125, CR4/5, %d dBm.\n",
                          LORA_FREQUENCY_MHZ, LORA_TX_POWER_DBM);
            Serial.printf("[LoRa] TBEB slot %lu ms (ACK airtime), max frame airtime %lu ms.\n",
                          (unsigned long)macSlotTimeMs, (unsigned long)maxFrameAirtimeMs);
        }
    } else {
        Serial.printf("[LoRa] SX1262 initialization failed with code %d.\n", radioState);
    }

    Serial.println("\n-------------------------------------------------------");
    Serial.println("  Barangay San Miguel Offline LoRa Mesh Architecture");
    Serial.println("  Interfaces & Security Subsystem Initializing...");
    Serial.println("-------------------------------------------------------");

    // Configure Wi-Fi SoftAP (phone cap: see MAX_AP_CLIENTS)
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, netMsk);
    const bool apStarted = WiFi.softAP(AP_SSID, NULL, 1, 0, MAX_AP_CLIENTS);

    Serial.printf("[WiFi SoftAP] SSID: '%s'%s\n", AP_SSID, apStarted ? "" : "  ** FAILED TO START **");
    Serial.printf("[WiFi SoftAP] Gateway IP: %s\n", apIP.toString().c_str());
    wifi_config_t apConfig;
    if (esp_wifi_get_config(WIFI_IF_AP, &apConfig) == ESP_OK) {
        Serial.printf("[WiFi SoftAP] Phone cap: %u at once (requested %u, Wi-Fi driver maximum %u)\n",
                      apConfig.ap.max_connection, AP_CLIENTS_REQUESTED, AP_CLIENTS_DRIVER_MAX);
    } else {
        Serial.printf("[WiFi SoftAP] Phone cap: %u at once (requested %u)\n", MAX_AP_CLIENTS, AP_CLIENTS_REQUESTED);
    }

    // Initialize Captive DNS Server
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.println("[DNS Server]  Captive DNS running on Port 53");

    // Initialize Web Server (Captive Portal + Admin Dashboard)
    setupWebServerRoutes();
    server.begin();
    Serial.println("[Web Server]  HTTP Server active on Port 80");
    Serial.println("[Web Server]  - Captive Portal: http://192.168.4.1/");
    Serial.println("[Web Server]  - Admin Dashboard: http://192.168.4.1/admin");
    Serial.println("-------------------------------------------------------");
    // Wi-Fi is running now, so esp_random() is a true hardware RNG.
    // Random message-ID start: after a reboot this node does not reuse IDs that
    // neighbours still hold in their Seen-Lists (which would be dropped as duplicates).
    globalMessageCounter = (uint16_t)(1000 + (esp_random() % 60000));
    // Random first heartbeat (0-30 s) so nodes reset together do not stay in sync.
    nextHeartbeatAtMs = millis() + (esp_random() % HEARTBEAT_INTERVAL_MS);
    Serial.printf("[STATUS] Node V%u (%s): first message ID %u, first heartbeat in %lu ms.\n",
                  NODE_ID, AP_SSID, (unsigned int)(globalMessageCounter + 1),
                  (unsigned long)(nextHeartbeatAtMs - millis()));
    if (BATTERY_SOURCE == BATTERY_SOURCE_ESTIMATE) printEnergyReport();
    else if (BATTERY_SOURCE == BATTERY_SOURCE_LIPO) printBatteryReport();
    else Serial.println("[BATTERY] Not measured (USB power bank): heartbeats report battery as unknown.");
    Serial.printf("[CONFIG] %s: queue %s, DTN %s, relay filter %s%s\n", CONFIG_NAME_CFG,
                  QUEUE_MODE_CFG == QUEUE_FIFO ? "FIFO" : "min-heap", DTN_ENABLED ? "on" : "off (drop on link failure)",
                  relayFilterName(), CART_MODEL_AVAILABLE ? " (CART model loaded)" : "");
    Serial.println("[STATUS] Ready for Emergency Message Intake & Security Processing.");
    printCsvHeader();
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "cfg=%s;queue=%s;dtn=%d;filter=%s;cart=%d", CONFIG_NAME_CFG,
                 QUEUE_MODE_CFG == QUEUE_FIFO ? "fifo" : "heap", DTN_ENABLED ? 1 : 0, relayFilterName(),
                 CART_MODEL_AVAILABLE);
        CsvEvent e = csvEvent("CONFIG", 0, 0, 0, 0, 0);
        e.detail = detail;
        logCsv(e);
    }
}

void loop() {
    // Process DNS captive portal requests
    dnsServer.processNextRequest();

    serviceSerialCommands();
    energyService();
    applyRadioSilence();
    serviceHeartbeat();
    serviceHeartbeatLabels();
    serviceBatteryLog();
    serviceRadioIrq();
    serviceRadioRx();
    serviceRadioMac();
    const size_t expiredDtn = dtnBuffer.purgeExpired(millis());
    if (expiredDtn > 0) Serial.printf("[DTN] Purged %u expired packet(s).\n", (unsigned int)expiredDtn);
}
