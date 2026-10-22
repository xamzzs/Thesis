#!/usr/bin/env python3
"""
Metrics from captures made with tools/capture_telemetry.py.

  python tools/analyze_run.py run01.csv                 one run in detail
  python tools/analyze_run.py runs/*.csv --compare      configurations side by side (Section 4.7):
        per configuration and tier: mean and 95% confidence interval (t, across runs) of
        PDR, latency, PQA and battery drop rate; Kruskal-Wallis across configurations;
        McNemar's test of CART vs the heuristic on the same transmissions.
        Writes the per-run table to compare_runs.csv (--table to rename).

  Energy   R_drop = (V_initial - V_final) / hours, per node, from its BATT rows (LiPo voltage
           logged every minute). Only nodes running on their LiPo (not plugged into the
           laptop, which charges it) give a meaningful value.

Reports, per priority class k:
  PDR      unique messages DELIVERed at the hub / unique messages created (NEW)
  Latency  (a) PC clock: pc_epoch_ms(DELIVER) - pc_epoch_ms(NEW), same (origin, id);
               valid when the origin node and the hub were captured on this PC.
           (b) hub clock: lat_ms in the DELIVER row = hub time at delivery minus the
               creation time carried in the packet (seconds + milliseconds); valid when
               the origin's clock (phone, or node set by "time") agrees with the hub's.
  PQA      Priority Queuing Accuracy (Section 4.7 metric 3), counted per k=1 message
           on every node that queued it: the message is handled correctly at that node
           if no LESS urgent data message was transmitted by that node between the
           moment the k=1 message entered its queue and the moment it was sent.
           This works for the FIFO Baseline too, where the k=1 row itself always
           shows ahead = 0 and the inversion only shows on the other rows.
           Also shown: share of data TX rows with ahead == 0 (nothing more urgent waiting).
  Wait     TBEB backoff wait before each data transmission (wait_ms)
  Buffer   DTN custody time of messages that left the buffer (FORWARD rows, d_buffer_ms)
Messages are keyed on (origin, id); duplicates are counted once.

Requires: pip install pandas
"""

import argparse
import glob
import sys

import pandas as pd

DATA_TYPES = {0, 2}     # SOS, TELEMETRY (heartbeat = 1, ACK = 3)


def hub_latency(detail):
    for part in str(detail).split(";"):
        if part.startswith("lat_ms="):
            return float(part[7:])
    return float("nan")


def pqa_per_message(df):
    """One row per (node, origin, id) of a k=1 message that node queued and sent."""
    data_tx = df[(df.event == "TX") & df.type.isin(DATA_TYPES)].sort_values(["node", "ms"])
    entries = df[(df.event == "NEW") |
                 ((df.event == "RX") & df.type.isin(DATA_TYPES) & (df.peer != df.node))]
    rows = []
    for (node, origin, mid), sent in data_tx[data_tx.k == 1].groupby(["node", "origin", "id"]):
        first_tx = sent.ms.min()
        entered = entries[(entries.node == node) & (entries.origin == origin) & (entries.id == mid) &
                          (entries.ms <= first_tx)]
        if entered.empty:
            continue                        # entry not captured (e.g. capture started late)
        start = entered.ms.max()
        between = data_tx[(data_tx.node == node) & (data_tx.ms > start) & (data_tx.ms < first_tx) &
                          (data_tx.k > 1)]
        rows.append({"node": node, "origin": origin, "id": mid, "waited_ms": first_tx - start,
                     "overtaken_by": len(between)})
    return pd.DataFrame(rows)


def battery_drop(df, min_span_min=5.0):
    """R_drop per node in V/h from BATT rows (detail mv=...)."""
    batt = df[(df.event == "BATT") & df.detail.astype(str).str.startswith("mv=")].copy()
    batt["mv"] = pd.to_numeric(batt.detail.astype(str).str.extract(r"mv=(\d+)")[0], errors="coerce")
    batt = batt.dropna(subset=["mv"])
    out = {}
    for node, rows in batt.sort_values("ms").groupby("node"):
        span_h = (rows.ms.iloc[-1] - rows.ms.iloc[0]) / 3_600_000
        if span_h * 60 >= min_span_min:
            out[int(node)] = (rows.mv.iloc[0] - rows.mv.iloc[-1]) / 1000 / span_h
    return out


def run_metrics(path):
    """One row of summary numbers for one capture file (one run)."""
    df = pd.read_csv(path)
    for col in ("wait_ms", "queue_index", "d_buffer_ms", "cart_class", "heur_class"):
        if col not in df.columns:
            df[col] = float("nan")
    cfg = df[df.event == "CONFIG"].detail.astype(str)
    config = cfg.str.extract(r"cfg=([^;]+)")[0].dropna()
    config = config.mode().iloc[0] if len(config) else "unknown"
    session = str(df.session.iloc[0]) if "session" in df.columns and len(df) else ""
    tier = next((t for t in ("low", "medium", "high") if t in session.lower()), "-")
    new = df[df.event == "NEW"].drop_duplicates(["origin", "id"])
    delivered = df[df.event == "DELIVER"].drop_duplicates(["origin", "id"]).copy()
    delivered["hub_latency_ms"] = delivered.detail.map(hub_latency)
    merged = new.merge(delivered[["origin", "id", "pc_epoch_ms", "hub_latency_ms"]], on=["origin", "id"],
                       how="left", suffixes=("_new", "_deliver"))
    merged["latency_ms"] = (merged.pc_epoch_ms_deliver - merged.pc_epoch_ms_new).fillna(merged.hub_latency_ms)
    got = merged.pc_epoch_ms_deliver.notna()
    k1 = merged.k == 1
    pqa = pqa_per_message(df)
    drops = battery_drop(df)
    return {
        "file": path, "config": config, "tier": tier, "session": session,
        "sent": len(merged), "delivered": int(got.sum()),
        "pdr": got.mean() if len(merged) else float("nan"),
        "pdr_k1": got[k1].mean() if k1.any() else float("nan"),
        "latency_ms": merged.latency_ms[got].mean() if got.any() else float("nan"),
        "latency_k1_ms": merged.latency_ms[got & k1].mean() if (got & k1).any() else float("nan"),
        "pqa": (pqa.overtaken_by == 0).mean() if len(pqa) else float("nan"),
        "r_drop_v_per_h": sum(drops.values()) / len(drops) if drops else float("nan"),
        "drops": int((df.event == "DROP").sum()),
    }, df


def labelled_tx(df):
    """Data TX rows with the outcome: 1 = Danger (no ACK within 60 s), 0 = Safe."""
    tx = df[(df.event == "TX") & df.type.isin(DATA_TYPES)].copy()
    acks = df[df.event.isin(["ACK", "NOACK"])]
    labels = []
    for _, row in tx.iterrows():
        same = acks[(acks.node == row.node) & (acks.origin == row.origin) & (acks.id == row.id) &
                    (acks.peer == row.peer) & (acks.ms >= row.ms) & (acks.ms <= row.ms + 60_000)]
        labels.append(0 if len(same) and same.iloc[0].event == "ACK" else (1 if len(same) else None))
    tx["danger"] = labels
    return tx.dropna(subset=["danger"])


def mean_ci(values):
    from math import sqrt
    v = pd.Series(values).dropna()
    if len(v) == 0:
        return float("nan"), float("nan"), 0
    if len(v) == 1:
        return v.iloc[0], float("nan"), 1
    try:
        from scipy import stats
        t = stats.t.ppf(0.975, len(v) - 1)
    except ImportError:
        t = 1.96
    return v.mean(), t * v.std(ddof=1) / sqrt(len(v)), len(v)


def compare(paths, table):
    runs, frames = [], []
    for path in paths:
        row, df = run_metrics(path)
        runs.append(row)
        df["_config"] = row["config"]
        frames.append(df)
    runs = pd.DataFrame(runs)
    runs.to_csv(table, index=False)
    print(f"{len(runs)} runs. Per-run table written to {table}\n")

    def fmt(values, scale=1.0, unit="", digits=1):
        m, h, n = mean_ci(values)
        if n == 0:
            return "-"
        return f"{m * scale:.{digits}f}{unit}" + (f" +/- {h * scale:.{digits}f}" if n > 1 else "") + f" (n={n})"

    for tier, group in runs.groupby("tier"):
        print(f"=== Tier: {tier}   (mean +/- 95% CI across runs)")
        rows = []
        for config, g in group.groupby("config"):
            rows.append({"config": config, "runs": len(g),
                         "PDR %": fmt(g.pdr, 100), "PDR k=1 %": fmt(g.pdr_k1, 100),
                         "latency s": fmt(g.latency_ms, 1 / 1000), "latency k=1 s": fmt(g.latency_k1_ms, 1 / 1000),
                         "PQA %": fmt(g.pqa, 100), "R_drop V/h": fmt(g.r_drop_v_per_h, 1, "", 3)})
        print(pd.DataFrame(rows).to_string(index=False))
        try:
            from scipy import stats
            for metric in ("pdr", "latency_ms", "pqa"):
                samples = [g[metric].dropna().values for _, g in group.groupby("config")]
                samples = [x for x in samples if len(x) > 0]
                if len(samples) >= 2 and sum(len(x) for x in samples) > len(samples):
                    if len({float(v) for x in samples for v in x}) == 1:
                        print(f"  Kruskal-Wallis on {metric}: every run has the same value; test not applicable")
                        continue
                    try:
                        h, p = stats.kruskal(*samples)
                        print(f"  Kruskal-Wallis on {metric}: H = {h:.2f}, p = {p:.4f} "
                              f"({'differs' if p < 0.05 else 'no significant difference'} across configurations)")
                    except ValueError as error:
                        print(f"  Kruskal-Wallis on {metric}: not computable ({error})")
        except ImportError:
            print("  (pip install scipy for Kruskal-Wallis)")
        print()

    # McNemar: CART vs heuristic, judged on the same data transmissions.
    tx = pd.concat([labelled_tx(df) for df in frames], ignore_index=True)
    tx = tx[tx.cart_class.isin(["Risky", "Safe"]) & tx.heur_class.isin(["Risky", "Safe"])]
    print("=== McNemar: CART vs heuristic on the same transmissions (Danger = no ACK)")
    if tx.empty:
        print("  no data TX rows with both verdicts and an outcome (needs a build with cart_model.h)")
        return
    cart_ok = (tx.cart_class == "Risky") == (tx.danger == 1)
    heur_ok = (tx.heur_class == "Risky") == (tx.danger == 1)
    b, c = int((cart_ok & ~heur_ok).sum()), int((~cart_ok & heur_ok).sum())
    print(f"  {len(tx)} transmissions: CART correct {100 * cart_ok.mean():.1f}%, heuristic correct "
          f"{100 * heur_ok.mean():.1f}%")
    print(f"  only CART right: {b}   only heuristic right: {c}")
    if b + c == 0:
        print("  the two classifiers never disagree: McNemar not applicable")
        return
    try:
        from scipy import stats
        if b + c < 25:
            p = stats.binomtest(b, b + c, 0.5).pvalue
            print(f"  exact McNemar (binomial): p = {p:.4f}")
        else:
            chi2 = (abs(b - c) - 1) ** 2 / (b + c)
            p = stats.chi2.sf(chi2, 1)
            print(f"  McNemar chi2 (continuity-corrected) = {chi2:.2f}, p = {p:.4f}")
        print("  " + ("significant difference" if p < 0.05 else "no significant difference") + " at alpha = 0.05")
    except ImportError:
        print("  (pip install scipy for the p-value)")


def main(path: str) -> None:
    df = pd.read_csv(path)
    for col in ("wait_ms", "queue_index", "d_buffer_ms"):
        if col not in df.columns:
            df[col] = float("nan")          # capture from older firmware
    config = df[df.event == "CONFIG"].detail.unique().tolist()
    new = df[df.event == "NEW"].drop_duplicates(["origin", "id"])
    delivered = df[df.event == "DELIVER"].drop_duplicates(["origin", "id"]).copy()
    delivered["hub_latency_ms"] = delivered.detail.map(hub_latency)

    print(f"File: {path}   rows: {len(df)}   nodes: {sorted(df.node.unique().tolist())}")
    if config:
        print(f"Configuration: {', '.join(map(str, config))}")
    print("\nPDR and end-to-end latency (by k)")
    merged = new.merge(delivered[["origin", "id", "pc_epoch_ms", "hub_latency_ms"]], on=["origin", "id"],
                       how="left", suffixes=("_new", "_deliver"))
    merged["latency_ms"] = merged.pc_epoch_ms_deliver - merged.pc_epoch_ms_new
    merged["delivered"] = merged.pc_epoch_ms_deliver.notna()
    for k, group in merged.groupby("k"):
        sent, got = len(group), int(group.delivered.sum())
        lat = group.latency_ms.dropna()
        hub = group.hub_latency_ms.dropna()
        pc_text = (f"PC clock mean {lat.mean():.0f} / median {lat.median():.0f} / max {lat.max():.0f} ms"
                   if len(lat) else "PC clock: none")
        hub_text = f"; hub clock mean {hub.mean():.0f} ms" if len(hub) else ""
        print(f"  k={k}: PDR {got}/{sent} = {100 * got / sent:.2f}%   {pc_text}{hub_text}")
    if new.empty:
        print("  no NEW rows: was the origin node captured?")
        hub = delivered.hub_latency_ms.dropna()
        if len(hub):
            print(f"  hub-clock latency of {len(hub)} delivered messages: mean {hub.mean():.0f} ms, "
                  f"median {hub.median():.0f} ms, max {hub.max():.0f} ms")

    drops = df[df.event == "DROP"]
    if len(drops):
        print("\nDrops: " + ", ".join(f"{reason} {n}" for reason, n in drops.detail.value_counts().items()))

    print("\nPriority Queuing Accuracy")
    per_msg = pqa_per_message(df)
    if per_msg.empty:
        print("  no k=1 message whose queue entry and transmission were both captured")
    else:
        ok = int((per_msg.overtaken_by == 0).sum())
        print(f"  {ok}/{len(per_msg)} = {100 * ok / len(per_msg):.2f}% of k=1 messages left a node without "
              f"any less urgent message being sent first")
        bad = per_msg[per_msg.overtaken_by > 0]
        if len(bad):
            print("  k=1 messages that waited behind less urgent traffic:")
            print(bad.to_string(index=False))
    tx = df[(df.event == "TX") & df.type.isin(DATA_TYPES) & df.ahead.notna()]
    if len(tx):
        clean = int((tx.ahead == 0).sum())
        print(f"  data transmissions with nothing more urgent waiting: {clean}/{len(tx)} "
              f"= {100 * clean / len(tx):.2f}%")

    waits = tx.wait_ms.dropna() if len(tx) else pd.Series(dtype=float)
    if len(waits):
        print(f"\nTBEB wait before data transmissions: mean {waits.mean():.0f} ms, max {waits.max():.0f} ms, "
              f"{int((waits > 0).sum())}/{len(waits)} had to back off")
    held = df[df.event == "FORWARD"].d_buffer_ms.dropna()
    if len(held):
        print(f"DTN custody (d_buffer) of {len(held)} forwarded messages: mean {held.mean() / 1000:.1f} s, "
              f"max {held.max() / 1000:.1f} s")
    drops = battery_drop(df)
    if drops:
        print("Battery drop rate R_drop: " + ", ".join(f"V{n} {v:.3f} V/h" for n, v in sorted(drops.items())))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", help="capture CSV file(s); wildcards allowed")
    parser.add_argument("--compare", action="store_true", help="compare configurations across runs")
    parser.add_argument("--table", default="compare_runs.csv", help="per-run table for --compare")
    args = parser.parse_args()
    paths = sorted({p for pattern in args.files for p in (glob.glob(pattern) or [pattern])})
    if args.compare or len(paths) > 1:
        compare(paths, args.table)
    else:
        main(paths[0])
