#!/usr/bin/env python3
"""
Build the CART training/test table from capture files and check it is enough.

Each output row = one attempt to get a frame across one link:
  features : rssi, snr, battery, peer_traffic, peer_hb_age_ms
             (what the node knew about the neighbour BEFORE the attempt;
              battery = the neighbour's LiPo level from its last heartbeat, %)
  label    : 1 = Danger (the frame was lost), 0 = Safe (it got through)
             -- the paper's convention (4.4.2, Table 4.5); Danger is the positive class

Two sources (column "source"):
  heartbeat  HB_OK / HB_MISS rows. The RECEIVING node labels every heartbeat it
             expected from a neighbour (sequence numbers). Link = peer -> node.
             Used to TRAIN CART (Test F: passive, no messages needed).
  data       data TX row + its ACK / NOACK. The SENDING node logs them.
             Link = node -> peer. Used to TEST CART on real relaying (final evaluation).

Usage (Windows):
  python tools\\build_cart_dataset.py F*.csv
  python tools\\build_cart_dataset.py F*.csv --out cart_dataset.csv

Requires: pip install pandas
"""

import argparse
import glob
import sys
from pathlib import Path

try:
    import pandas as pd
except ImportError:
    sys.exit("pandas is missing: run  pip install pandas")

HB_AGE_CAP_MS = 300_000          # missing / very old heartbeat age is capped here
ACK_TIMEOUT_MS = 60_000          # a data TX with no ACK within 60 s counts as failed
DATA_TYPES = {0, 2}              # SOS, TELEMETRY (not heartbeat 1, not ACK 3)
FEATURES = ["rssi", "snr", "battery", "peer_traffic", "peer_hb_age_ms"]
DANGER, SAFE = 1, 0

# Targets for training data (Test F)
MIN_LABELS = 300
MIN_FAIL_SHARE = 0.20
MIN_SESSIONS = 6


def load(paths):
    frames = []
    for path in paths:
        df = pd.read_csv(path)
        if "session" not in df.columns:
            df["session"] = Path(path).stem
        df["file"] = Path(path).name
        frames.append(df)
    if not frames:
        sys.exit("No capture files given.")
    return pd.concat(frames, ignore_index=True)


def heartbeat_rows(df):
    hb = df[df.event.isin(["HB_OK", "HB_MISS"])].copy()
    hb["source"] = "heartbeat"
    hb["label"] = (hb.event == "HB_MISS").astype(int)          # 1 = Danger
    hb["link"] = "V" + hb.peer.astype(int).astype(str) + "->V" + hb.node.astype(int).astype(str)
    return hb


def data_rows(df):
    tx = df[(df.event == "TX") & (df.type.isin(DATA_TYPES))].copy()
    outcomes = df[df.event.isin(["ACK", "NOACK"])]
    labels = []
    for _, row in tx.iterrows():
        same = outcomes[(outcomes.file == row.file) & (outcomes.node == row.node) &
                        (outcomes.origin == row.origin) & (outcomes.id == row.id) &
                        (outcomes.peer == row.peer) & (outcomes.ms >= row.ms) &
                        (outcomes.ms <= row.ms + ACK_TIMEOUT_MS)]
        if len(same):
            labels.append(SAFE if same.iloc[0].event == "ACK" else DANGER)
        else:
            last_ms = df[(df.file == row.file) & (df.node == row.node)].ms.max()
            labels.append(DANGER if last_ms >= row.ms + ACK_TIMEOUT_MS else None)   # capture ended: unknown
    tx["label"] = labels
    tx = tx[tx.label.notna()].copy()
    tx["label"] = tx.label.astype(int)
    tx["source"] = "data"
    tx["link"] = "V" + tx.node.astype(int).astype(str) + "->V" + tx.peer.astype(int).astype(str)
    return tx


def report(ds):
    train = ds[ds.source == "heartbeat"]
    test = ds[ds.source == "data"]
    print(f"\nRows: {len(ds)}  (heartbeat/training: {len(train)}, data/testing: {len(test)})")
    if len(train):
        print("\nTraining rows per session:")
        per = train.groupby("session").label.agg(rows="count", danger="sum")
        per["fail_%"] = (100 * per.danger / per.rows).round(0)
        print(per.to_string())
        print("\nTraining rows per link:")
        links = train.groupby("link").label.agg(rows="count", danger="sum")
        links["fail_%"] = (100 * links.danger / links.rows).round(0)
        print(links.to_string())

    fails = int((train.label == DANGER).sum())
    share = fails / len(train) if len(train) else 0.0
    sessions = train.session.nunique()
    battery_known = float(train.battery.notna().mean()) if len(train) else 0.0
    print("\nTraining data check:")
    checks = [
        (len(train) >= MIN_LABELS, f"labels      {len(train)} (need >= {MIN_LABELS})"),
        (share >= MIN_FAIL_SHARE, f"Danger rows {fails} = {100 * share:.0f}% (need >= {100 * MIN_FAIL_SHARE:.0f}%)"),
        (battery_known >= 0.8, f"battery     known on {100 * battery_known:.0f}% of rows (need >= 80%: LiPo build)"),
        (sessions >= MIN_SESSIONS, f"sessions    {sessions} (need >= {MIN_SESSIONS} for the 80/20 split by session)"),
    ]
    for ok, text in checks:
        print(("  OK    " if ok else "  SHORT ") + text)
    if share < MIN_FAIL_SHARE:
        print("  -> add 'weak' or 'far' sessions (node lowered, behind a house, near its range limit).")
    if battery_known < 0.8:
        print("  -> battery is missing: re-flash with the LiPo build (BATTERY_SOURCE_LIPO) before Test F,")
        print("     or train without it: python tools\\train_cart.py cart_dataset.csv --features rssi,snr,traffic")
    if all(ok for ok, _ in checks):
        print("  Enough to train CART.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", help="capture CSV files (wildcards allowed, e.g. F*.csv)")
    parser.add_argument("--out", default="cart_dataset.csv")
    args = parser.parse_args()

    paths = sorted({p for pattern in args.files for p in (glob.glob(pattern) or [pattern])})
    paths = [p for p in paths if not p.endswith(Path(args.out).name)]
    df = load(paths)

    ds = pd.concat([heartbeat_rows(df), data_rows(df)], ignore_index=True)
    before = len(ds)
    ds = ds[ds.rssi.notna() & ds.snr.notna()].copy()        # no measurement of that link yet
    ds["battery"] = ds.peer_batt_pct                         # neighbour's LiPo %, empty = unknown
    dropped = before - len(ds)
    ds["peer_traffic"] = ds.peer_traffic.fillna(0)
    ds["peer_hb_age_ms"] = ds.peer_hb_age_ms.fillna(HB_AGE_CAP_MS).clip(upper=HB_AGE_CAP_MS)

    out = ds[["session", "file", "source", "link", "node", "peer", "event", "ms"] + FEATURES + ["label"]]
    out.to_csv(args.out, index=False)
    print(f"Read {len(paths)} file(s). Wrote {len(out)} rows to {args.out}"
          + (f" ({dropped} without RSSI/SNR dropped)" if dropped else ""))
    report(out)


if __name__ == "__main__":
    main()
