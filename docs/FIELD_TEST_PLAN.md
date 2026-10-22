# Field Test Plan — LoRa Mesh (Brgy. San Miguel)

Exported from the team's live Claude doc on Oct 13, 2026. The doc is the version to edit; this copy travels with the code.

Run six tests in order: range, site check, multi-hop, failover, priority, then CART data collection. Tests A–E take about 4–5 hours with the full team, and Test F about 3 hours, so F can go on a second day. Each test builds on the one before: range tells you where nodes can go, the site check confirms the planned links, and only then are delivery, failover and training-data results meaningful.

## Before you go

All six boards must run the same build, and every power bank must be full, before you leave.

1. **Re-flash all six to the new build.** It adds the `sos` serial command and the heartbeat labels (`HB_OK` / `HB_MISS`) that Test F needs, and gives the paper one firmware version. Plug in **one board at a time**; with a single board connected PlatformIO finds its port on its own. In the PlatformIO terminal, in the project folder:

   ```
   pio run -e node1 -t upload      # V1 plugged in alone
   pio run -e node2 -t upload      # unplug V1, plug in V2
   pio run -e node3 -t upload      # V3
   pio run -e node4 -t upload      # V4
   pio run -e node5 -t upload      # V5
   pio run -e node6 -t upload      # V6
   ```

   If more than one board is plugged in, add its port, e.g. `pio run -e node3 -t upload --upload-port COM5` (`pio device list` shows ports). Check each one: `pio device monitor -p COM5`, press RST, look for `[STATUS] Node V3 (EMERGENCY-HUB-03)`, then Ctrl+C. Label each board.

   **Bench check of the new `sos` command:** with V1 on its power bank and V6 on the laptop, type `sos 1 Bench|hello` in V6's monitor. You should see `EMERGENCY MESSAGE ACCEPTED & QUEUED (via serial)`, then an `ACK` within about a minute, and the message on V1's dashboard.
2. **Charge every power bank to 100%,** plus one spare.
3. **Mark the planned positions on a map** (V1 at the barangay hall, then V2–V6). Planned links: V1–V2, V1–V3, V1–V4, V2–V4, V2–V5, V3–V5, V3–V6, V5–V6.
4. **Check the laptop:** `pip install pyserial pandas` done; `tools\capture_telemetry.py`, `tools\analyze_run.py` and `tools\build_cart_dataset.py` present.

**What to bring**

- [ ] 6 nodes with antennas attached, 6 full power banks + 1 spare
- [ ] Laptops (charged) and their chargers. One is enough for every test. One USB-C data cable per laptop, plus a spare.
- [ ] A stick or pole to hold nodes about 1.5–2 m high, tape, the weatherproof enclosures
- [ ] 2–3 phones; mobile data OFF while sending through a node's Wi-Fi (calls still work)
- [ ] This log sheet (on a phone) or a notebook, a pen
- [ ] Umbrella or plastic bags in case of rain

## Team roles

Four jobs; with nine members, put two people on each and rotate. Keep a phone call or group chat open between the hub and the field team the whole time.

| Role | Stays where | Does |
| --- | --- | --- |
| Hub operator | At V1 (barangay hall) with the laptop | Runs the serial monitor or capture, keeps the dashboard open, reads out what arrives |
| Walker / sender | Moves around | Carries a node or goes to a node, sends test messages from a phone, reads out the Ref# |
| Recorder | With the walker | Fills the log sheet: time, place, GPS, line of sight, Ref#, result |
| Node crew | At the node positions | Places nodes high and vertical, unplugs and replugs power banks in Test D |

## Test A — Range survey (about 60–90 min)

Find how far two nodes can reliably talk in your barangay, so you know where the others can go. Only **V1** and **V3** are on; the other four stay **off**, so nothing else is in the air.

**Setup**

1. V1 at the barangay hall on the laptop, antenna vertical, raised about 1.5–2 m, not next to metal.
2. The hub operator starts a monitor that timestamps and saves every line:
   - `pio device monitor -p COM8 -f time -f log2file -f send_on_enter`
   - The log file appears in the project folder as `platformio-device-monitor-….log`.
3. The walker carries V3 on its power bank, held high and vertical, away from the body.

**At each stop** (about 50, 100, 200, 300, 400, 500, 700, 1000 m from the hub — Google Maps' "Measure distance" gives the distance):

1. Stand still with V3 raised for **2 minutes** (about 4 heartbeats).
2. Connect a phone to `EMERGENCY-HUB-03` and send one **Medical SOS** with the location set to the stop, e.g. `A-STOP4 300m`. Note the Ref# the phone shows.
3. The recorder writes the time, GPS or landmark, distance, and whether you can see the hall (line of sight).
4. The hub operator reads out the newest `[HB RX] V3 rssi=… snr=…` line and whether the SOS arrived; the recorder writes both down.

**When to stop:** two stops in a row where the hub hears **no V3 heartbeat** in 2 minutes. The last stop that worked is the range in that direction. Walk back: any SOS that could not get through is held on V3 and should arrive once you are back in range — note when.

If time allows, repeat in a second direction with different surroundings (open road vs. houses).

**What good looks like:** RSSI falls as you walk away. SF12 still decodes down to about −130 dBm and −20 dB SNR; once SNR drops below about −10 dB, treat the link as marginal. For deployment, place linked nodes where RSSI is about −90 to −110 dBm with SNR ≥ 0 dB (roughly ½ to ⅔ of the range found here).

## Test B — Site check at the planned positions (about 30–45 min)

Confirm every planned link works where the nodes will actually stand, all from the hub's laptop.

1. The node crew places all six nodes at their planned positions (use Test A's range as a guide), raised and vertical, then switches them on.
2. Wait **10 minutes.** Heartbeats need a few rounds to spread every node's link-state row to the hub.
3. The hub operator types `routes` in V1's monitor and takes a screenshot. The matrix shows each planned link's measured weight:

| Weight in the matrix | Meaning | Action |
| --- | --- | --- |
| 1–5 | Strong link | Keep |
| 6–15 | Usable but weaker | Keep; note it |
| 16 or more | Marginal | Raise or move one of the two nodes a little closer |
| 255 on a planned link | Not heard, or no update yet | Wait 5 more minutes; still 255 → move one of the two nodes |

4. Fix any 255 link, wait 5 minutes, and run `routes` again until all eight planned links show a number.
5. Write each final position (GPS, height, landmark) in the log sheet; these become the deployment positions for Tests C–F and for the paper.

A node hearing a node it is **not** planned to link with is fine. The topology mask keeps routing on the planned links (the matrix shows 255 there).

## Test C — Multi-hop delivery and heartbeats (about 45 min)

Send SOS messages from every node to the hub and record whether, how fast and by which route each one arrives.

**At the hub**

1. Close the monitor (Ctrl+C), then start the capture. It also sets V1's clock, so the dashboard's latency column works:
   - `python tools\capture_telemetry.py COM8 --out fieldC.csv`
2. Connect the laptop's Wi-Fi to `EMERGENCY-HUB-01` and open `http://192.168.4.1/admin`, sorted by "Arrival order".

**In the field** (start with the farthest node, V6, then V5, V4, V3, V2)

1. Stand near the node, connect a phone to its Wi-Fi (`EMERGENCY-HUB-0N`), and send **3 Medical SOS**, 1 minute apart. Put the node and the try number in the location, e.g. `C-V6-1`.
2. Write down each Ref# (e.g. `V6#16771`) and the time sent.
3. The hub operator reads back each arrival: Ref#, route (e.g. `V6 -> V5 -> V2 -> V1`) and latency.

**Expected:** every message arrives; routes follow the lowest-weight path from Test B's matrix; two hops usually take 10–30 s. One lost hop adds a retry of about 15–30 s, which is normal.

**Afterwards:** stop the capture (Ctrl+C). PDR = Ref#s delivered ÷ Ref#s sent, from the log sheet and the dashboard. `analyze_run.py` cannot count PDR here, because only V1 is on the laptop and it never sees the senders' "NEW" rows. The heartbeats V1 heard are in `fieldC.csv` as `RX` rows with `heartbeat` in the last column.

## Test D — Dead relay, reroute and store-and-forward (about 30 min)

Kill relays on purpose and check that messages take another route, or wait and get delivered once a relay comes back. Keep the capture and dashboard from Test C running. "Kill" a node by **unplugging its power bank**, never its antenna.

**D1 — Reroute**

1. From Test C, note V6's normal route, e.g. `V6 -> V3 -> V1`. The middle node (here V3) is the relay to kill.
2. Unplug that relay's power bank. Note the time.
3. Send 1 SOS from V6 (`D1-dead`). Expected: after two missed ACKs (about 15–30 s) it reroutes; the dashboard shows the other path, e.g. `V6 -> V5 -> V2 -> V1`.
4. Plug the relay back in and wait 2 minutes, then send 1 more SOS (`D1-back`). Expected: it uses the normal route again.

**D2 — Store-and-forward (no route at all)**

1. Unplug **both** of V6's neighbours (V3 and V5). Note the time.
2. Send 1 SOS from V6 (`D2-held`). Expected: it does **not** arrive; V6 holds it and retries every 10–60 s.
3. Wait 2 minutes, then plug **V5** back in. Note the time.
4. Expected: the message arrives within about 1–2 minutes of V5 starting, via `V6 -> V5 -> V2 -> V1`. Record the arrival time; the held time is the gap between sending and arriving.
5. Plug V3 back in.

## Test E — Priority order (about 15 min)

Check that a Medical SOS overtakes less urgent messages queued at the same node.

1. At V6, send three messages as fast as you can, in this order: **Routine (k=3)**, **General (k=2)**, **Medical SOS (k=1)**. Label them `E1-k3`, `E1-k2`, `E1-k1`.
2. On the dashboard (sorted by "Arrival order"), read the Arrival # of each.
3. Repeat 3 times (`E2-…`, `E3-…`).

**Expected arrival order:** k=3 first (it was already being sent, and a transmission is never interrupted), then **k=1**, then k=2. If k=1 arrives before k=2, the priority queue worked. k=1 arriving first of all is also correct; it means k=3 had not started yet.

## Test F — CART training data from heartbeats (about 3 hours, one laptop)

The training data comes from the heartbeats every node already sends every 30 s, so nobody needs to send messages. Each heartbeat carries a sequence number. The node the laptop is plugged into checks every heartbeat it expects from each neighbour and writes one row:

- **`HB_OK`**: it arrived (label 1).
- **`HB_MISS`**: it never came (label 0). Detected from a gap in the numbers (#41, then #43), or from a neighbour going silent.

Each row also holds what the node knew about that link just *before* the heartbeat was due: RSSI, SNR, the neighbour's traffic, and the heartbeat age. These are the four CART features. With 3 neighbours, the laptop node collects about **120 labels per 20 minutes**.

**Targets:** at least **300 labels** (aim for 600 or more), at least **20% HB_MISS**, at least **6 sessions**, from at least 3 different laptop positions so many links are covered. Most misses come from the weak, far and neighbour-lost sessions; normal sessions give mostly HB_OK.

**Before you start**

1. All six boards run the newest build (with heartbeat labels). Check: a node's monitor shows `HB_OK` rows about 30 s after its neighbours start.
2. Place all six nodes at their Test B positions, on power banks, switched on.
3. **No messages are needed.** Only send SOS from phones in the "busy" session.

**Each session (20 minutes)**

1. Go to the session's node with the laptop. Swap its power bank for the laptop's USB cable; the node restarts, which is fine.
2. Start recording. Name the session after the laptop node and the condition:

   ```
   python tools\capture_telemetry.py COM8 --session F1-V1-normal --out F1-V1-normal.csv
   ```

3. Set up the condition from the table below, then leave it alone for **20 minutes**. The counter shows labels and the failure share as they come in.
4. Ctrl+C to stop (the file is saved). Fill in the session log, put the power bank back, and move to the next session.
5. After every 2–3 sessions, run the check. It says what is still short:

   ```
   python tools\build_cart_dataset.py F*.csv
   ```

**Sessions**

| Session | Laptop at | Links recorded | Condition | How |
| --- | --- | --- | --- | --- |
| F1 | V1 | V2, V3, V4 → V1 | normal | As deployed |
| F2 | V5 | V2, V3, V6 → V5 | normal | As deployed |
| F3 | V3 | V1, V5, V6 → V3 | normal | As deployed |
| F4 | V1 | V2, V3, V4 → V1 | weak | Put V1 (with the laptop) on the ground, or just inside a building |
| F5 | V5 | V2, V3, V6 → V5 | weak | Same as F4, at V5 |
| F6 | V3 | V1, V5, V6 → V3 | far | Carry V3 and the laptop 100–200 m away from its neighbours, toward its Test A range limit |
| F7 | V1 | V2, V3, V4 → V1 | neighbour lost | At 5 min unplug V2's power bank; plug it back at 12 min |
| F8 | V5 | V2, V3, V6 → V5 | busy | Phones at V2 and V6 send an SOS every 30 s for the whole session (varies the traffic feature) |

The node also records any non-planned neighbour it happens to hear. Those links are usually weak, so they add useful failures.

**Rules**

- One session = one laptop position + one condition. To change either, stop and start a new session with a new name.
- Don't type `silent on` on the laptop node during a session; the node pauses labelling while it is deaf.
- In normal sessions, don't move or unplug the other nodes.
- Keep every `F*.csv` and `F*_raw.log`, even from short or failed sessions.
- The final evaluation will add a second kind of row: data packets with ACK/NOACK. They are the **test** set, used to check that CART trained on heartbeats predicts real relaying. Test F needs none of them.

**Session log**

| Session | Laptop at / port | Condition | Start | End | Labels | Failures % | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| F1 | V1 / | normal |  |  |  |  |  |
| F2 | V5 / | normal |  |  |  |  |  |
| F3 | V3 / | normal |  |  |  |  |  |
| F4 | V1 / | weak |  |  |  |  |  |
| F5 | V5 / | weak |  |  |  |  |  |
| F6 | V3 / | far |  |  |  |  |  |
| F7 | V1 / | neighbour lost |  |  |  |  |  |
| F8 | V5 / | busy |  |  |  |  |  |

## Field log sheet

One row per stop, message or action.

| Time | Test | Node | Place / stop | GPS or landmark | Distance to hub (m) | Line of sight? | Action (sent k, unplugged…) | Ref# | RSSI / SNR at hub | Arrived? (time, route) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | A | V3 | Stop 1 |  | 50 |  | SOS |  |  |  |
|  | A | V3 | Stop 2 |  | 100 |  | SOS |  |  |  |
|  | A | V3 | Stop 3 |  | 200 |  | SOS |  |  |  |
|  | A | V3 | Stop 4 |  | 300 |  | SOS |  |  |  |
|  | A | V3 | Stop 5 |  | 400 |  | SOS |  |  |  |
|  | A | V3 | Stop 6 |  | 500 |  | SOS |  |  |  |
|  | A | V3 | Stop 7 |  | 700 |  | SOS |  |  |  |
|  | A | V3 | Stop 8 |  | 1000 |  | SOS |  |  |  |
|  | C | V6 | C-V6-1 |  |  |  | SOS |  |  |  |
|  | C | V6 | C-V6-2 |  |  |  | SOS |  |  |  |
|  | C | V6 | C-V6-3 |  |  |  | SOS |  |  |  |
|  | D | V3 | D1 relay unplugged |  |  |  | unplug |  |  |  |
|  | D | V6 | D1-dead |  |  |  | SOS |  |  |  |
|  | D | V6 | D2-held |  |  |  | SOS |  |  |  |
|  | E | V6 | E1-k3 / k2 / k1 |  |  |  | 3 msgs |  |  |  |

## Safety and troubleshooting

**Never power a node without its antenna, and never remove an antenna while a node is on.** Transmitting without an antenna can damage the radio. To take a node down, unplug its power bank.

| Problem | First thing to try |
| --- | --- |
| A node's Wi-Fi network is missing | Check the power bank is on (some switch off at low current; press its button); unplug and replug the node |
| Hub hears no heartbeats from a node | Antenna screwed on firmly and vertical? Node raised? Move it 20–50 m closer and wait 1 minute |
| Phone drops the node's Wi-Fi | Tap "Stay connected" / "Use without internet"; turn mobile data off while sending |
| Portal page won't open | Type `http://192.168.4.1` in the browser yourself |
| Monitor or capture says the port is busy | Only one program can use a COM port: close the other monitor or capture first |
| Laptop monitor is blank | Press the node's RST button; if still blank, that board needs re-flashing with the current build |
| Rain | Keep nodes in their enclosures; keep the laptop and its USB connection dry |

**After the test:** save `fieldC.csv`, `fieldC_raw.log`, every `F*.csv` and `F*_raw.log`, every `platformio-device-monitor-….log`, the `routes` screenshots, this log sheet and the Test F session log in one folder, and send them over for analysis and CART training.

## After the field test — what comes next

Five steps, in order. Each one needs the output of the one before it.

| Step | What | Output | Who |
| --- | --- | --- | --- |
| 1 | Review the field data | Final node positions, range per direction, Test C–E results, enough CART labels? | Team sends files → analysis |
| 2 | Train CART | Trained tree, its F1 score, `cart_model.h` for the firmware | Script written for the team; team reviews |
| 3 | Build the comparison firmware | One build per configuration (baseline, heuristic, CART, FIFO, no DTN) | Firmware work |
| 4 | Evaluation runs | PDR, latency, PQA and filter F1 for every configuration | Team in the field, then analysis |
| 5 | Update the paper | Chapter 3 corrections, Chapter 4 results tables and figures | Team |

### Step 1 — Review the field data (1–2 days after the test)

- Send the folder from "After the test". From it come: RSSI/SNR vs distance for each direction (Test A), the measured link weights (Test B), delivery, route and latency per node (Test C), reroute time and hold time (Test D), and the arrival order (Test E).
- **Decide:** keep the positions from Test B, or move a node whose link was marginal. If many links score 16+, the weight constants in `link_state.h` get retuned to your barangay's real numbers.
- **CART label check:** run `python tools\build_cart_dataset.py F*.csv`. It needs at least 300 heartbeat labels, at least 20% HB_MISS and at least 6 sessions. If anything is short, run more Test F sessions before Step 2 (weak, far or neighbour-lost sessions give the misses).

### Step 2 — Train CART (about 1 day)

- `tools/build_cart_dataset.py` (already written) turns the Test F `HB_OK` / `HB_MISS` rows into the training table. Features: RSSI, SNR, traffic, heartbeat age; label: arrived or missed. Missing heartbeat age is capped at 300 000 ms.
- Split by session, 80% training / 20% testing, then train a CART tree (Gini, depth 3–5 so it fits on the ESP32).
- Report accuracy, precision, recall, F1 and the confusion matrix, plus a picture of the tree and which features mattered most. These go in Chapter 4.
- Export the tree as `include/cart_model.h`, so each leaf gives a failure probability Pf.

### Step 3 — Build the comparison firmware (2–3 days)

- **Configuration switches** as PlatformIO environments, so one command builds each variant: queue = min-heap or FIFO; DTN = on or off; relay filter = none, heuristic, or CART.
- **Heuristic filter:** skip a neighbour when RSSI ≤ −110 dBm or no heartbeat for over 75 s.
- **CART filter:** the node runs the tree on each neighbour and adds a penalty to that link's weight, w′ = w + M · Pf, so Dijkstra steers around links likely to fail.
- **Extra logging** the paper needs: how long each message was held in DTN (carry time) and the filter's decision per attempt, so its F1 can be measured live.
- **Leftover fixes:** don't pre-fill the passcode on the portal (item 27), the 25-device Wi-Fi cap (29), long form bodies on `/api/submit` (30).
- **Key:** the repo is public, so change the AES key in `aes_crypto.h` and `test_crypto.py` in this build. All six boards get re-flashed anyway.
- Each change is tested on the bench with all six boards before it goes to the field.

### Step 4 — Evaluation runs (1–2 field days)

- Same positions, same scenario for every configuration: the laptop injects traffic (`--inject`), and one relay is unplugged at a fixed time. That way the only difference between runs is the firmware.
- **At least 3 runs per configuration**, about 20–30 minutes each, alternating configurations so weather and time of day hit them all equally.
- **Clocks:** with several laptops, latency compares their PC clocks. Before going out, sync every laptop's clock (Windows Settings → Time → Sync now).
- `analyze_run.py` gets extended to compare configurations side by side: PDR, mean and max latency, PQA, filter F1, and overhead (heartbeat and retry airtime).
- **Energy:** the paper's energy metric needs a USB power meter. Without one, report the firmware's airtime-based estimate and say so in the limitations.

**CART on real relaying:** the data-packet rows from these runs (each transmission with its ACK or NOACK) are CART's **test set**. `build_cart_dataset.py` marks them `source = data`. The model trained on heartbeats in Step 2 predicts each one, and its accuracy and F1 on those rows go in Chapter 4.

### Step 5 — Update the paper

- **Chapter 3 corrections** so the paper matches what was built: 915 MHz at 19 dBm; radio silence by command instead of removing antennas; heartbeat age instead of battery as a CART feature; CART trained on heartbeat reception and tested on data-packet ACKs; the link-weight formula; how link-state rows spread in heartbeats; frame sizes and airtimes; the PQA definition; 72-hour DTN retention; the 200-byte location + message limit.
- **Chapter 4:** results tables and figures from Steps 1, 2 and 4.
- **Limitations:** battery not measured (power banks), energy estimated, one barangay, six nodes.
