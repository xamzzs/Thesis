#!/usr/bin/env python3
"""
Train the CART relay classifier and export it to the firmware (paper 4.4.2, 4.6 Steps 1.3 and 2.1).

Input : cart_dataset.csv from tools/build_cart_dataset.py
        label 1 = Danger (frame lost), 0 = Safe (got through); Danger is the positive class.
Output: <out-dir>/report.txt          metrics, chosen Pf threshold, tree rules
        <out-dir>/report.json         the same numbers, machine-readable
        <out-dir>/threshold_sweep.csv / .png   Precision, Recall, F1 for each Pf threshold
        <out-dir>/confusion_matrix.png         test split, chosen threshold
        <out-dir>/feature_importance.png
        <out-dir>/tree.png
        include/cart_model.h          the tree in C++ for the ESP32 (cartFailureProbability(), cartIsRisky(),
                                      plus micromlgen's class predictor)

CART settings (Table 4.3): Gini, max depth 5, min samples split 10, class weight balanced.

Pf of a leaf = the share of training rows in that leaf that were Danger (real, unweighted ratio).
The same Pf is used for the threshold sweep, the metrics and the firmware, so they always agree.

Split (decision D3):
  --split session      (default) whole sessions go to train or test, so rows recorded seconds apart
                       never sit on both sides
  --split stratified   80/20 stratified random split, as 4.4.2 currently describes

Usage (Windows):
  python tools\\train_cart.py cart_dataset.csv
  python tools\\train_cart.py cart_dataset.csv --split stratified
  python tools\\train_cart.py cart_dataset.csv --features rssi,snr,traffic      (no battery data yet)

Requires: pip install pandas scikit-learn matplotlib micromlgen
"""

import argparse
import json
import sys
from pathlib import Path

try:
    import numpy as np
    import pandas as pd
    from sklearn.metrics import (accuracy_score, confusion_matrix, f1_score, precision_score,
                                 recall_score)
    from sklearn.model_selection import train_test_split
    from sklearn.tree import DecisionTreeClassifier, export_text, plot_tree
except ImportError as error:
    sys.exit(f"{error.name} is missing: run  pip install pandas scikit-learn matplotlib micromlgen")

# Feature name used on the command line -> dataset column, unit, firmware enum name
FEATURE_COLUMNS = {
    "rssi":    ("rssi",           "dBm", "CART_F_RSSI"),
    "snr":     ("snr",            "dB",  "CART_F_SNR"),
    "battery": ("battery",        "%",   "CART_F_BATTERY"),
    "traffic": ("peer_traffic",   "data backoffs (TBEB) per 30-s heartbeat window", "CART_F_TRAFFIC"),
    "hb_age":  ("peer_hb_age_ms", "ms",  "CART_F_HB_AGE"),
}
DEFAULT_FEATURES = "rssi,snr,battery,traffic"      # the paper's x = [x_rssi, x_snr, x_bat, x_traf]
DANGER, SAFE = 1, 0
SWEEP = np.round(np.arange(0.05, 0.951, 0.05), 2)
PAPER_THRESHOLD = 0.70                              # decision D5 starting point


# ----------------------------------------------------------------------------- data
def load(path, features, train_source):
    df = pd.read_csv(path)
    missing = [FEATURE_COLUMNS[f][0] for f in features if FEATURE_COLUMNS[f][0] not in df.columns]
    if missing:
        sys.exit(f"{path} has no column(s) {missing}. Rebuild it with tools/build_cart_dataset.py.")
    columns = [FEATURE_COLUMNS[f][0] for f in features]
    pool = df if train_source == "all" else df[df.source == train_source]
    external = df[df.source == "data"] if train_source == "heartbeat" else df.iloc[0:0]
    before = len(pool)
    pool = pool.dropna(subset=columns)
    if before - len(pool):
        print(f"Dropped {before - len(pool)} of {before} training rows with an empty feature "
              f"({', '.join(c for c in columns if df[c].isna().any())}).")
    if "battery" in features and len(pool) < 0.5 * before:
        print("WARNING: most rows have no battery value. Re-flash with the LiPo build, "
              "or train with --features rssi,snr,traffic.")
    external = external.dropna(subset=columns)
    return pool.reset_index(drop=True), external.reset_index(drop=True), columns


def split(data, how, test_size, seed):
    """Returns (train_index, test_index)."""
    y = data.label.values
    if how == "session":
        sessions = data.session.unique()
        if len(sessions) >= 3:
            rng = np.random.default_rng(seed)
            # Add whole sessions to the test side until it holds ~test_size of the rows.
            # Try several orders and keep the one closest to the target that has both classes.
            best = None
            for _ in range(200):
                order = rng.permutation(sessions)
                test, rows = [], 0
                for s in order:
                    if rows >= test_size * len(data):
                        break
                    test.append(s)
                    rows += int((data.session == s).sum())
                mask = data.session.isin(test).values
                if len(set(y[mask])) < 2 or len(set(y[~mask])) < 2 or mask.all():
                    continue
                gap = abs(mask.mean() - test_size)
                if best is None or gap < best[0]:
                    best = (gap, mask)
            if best is not None:
                mask = best[1]
                return np.where(~mask)[0], np.where(mask)[0]
        print(f"NOTE: only {len(sessions)} session(s) with both classes; using a stratified split instead.")
    train_idx, test_idx = train_test_split(np.arange(len(data)), test_size=test_size,
                                           stratify=y, random_state=seed)
    return train_idx, test_idx


# ----------------------------------------------------------------------------- model
def make_tree(args):
    return DecisionTreeClassifier(criterion="gini", max_depth=args.max_depth,
                                  min_samples_split=args.min_samples_split,
                                  class_weight="balanced", random_state=args.seed)


def leaf_failure_ratio(tree, X, y):
    """Pf per leaf = share of the given (training) rows in that leaf that were Danger."""
    leaves = tree.apply(X)
    ratio = {}
    for leaf in np.unique(leaves):
        in_leaf = y[leaves == leaf]
        ratio[int(leaf)] = float((in_leaf == DANGER).mean())
    return ratio


def predict_pf(tree, ratio, X):
    # Every leaf of a fitted tree holds at least one training row, so every leaf has a ratio.
    return np.array([ratio[int(leaf)] for leaf in tree.apply(X)])


def scores(y, pf, threshold):
    pred = (pf >= threshold).astype(int)
    cm = confusion_matrix(y, pred, labels=[SAFE, DANGER])
    return {
        "threshold": float(threshold),
        "accuracy": float(accuracy_score(y, pred)),
        "precision": float(precision_score(y, pred, pos_label=DANGER, zero_division=0)),
        "recall": float(recall_score(y, pred, pos_label=DANGER, zero_division=0)),
        "f1": float(f1_score(y, pred, pos_label=DANGER, zero_division=0)),
        "confusion": {"TN_safe_ok": int(cm[0, 0]), "FP_safe_flagged": int(cm[0, 1]),
                      "FN_danger_missed": int(cm[1, 0]), "TP_danger_caught": int(cm[1, 1])},
        "rows": int(len(y)),
    }


def sweep(y, pf):
    return pd.DataFrame([scores(y, pf, t) for t in SWEEP]).drop(columns=["confusion"])


# ----------------------------------------------------------------------------- export
def c_tree_code(tree, columns, features, ratio):
    t = tree.tree_
    lines = []

    def walk(node, depth):
        pad = "    " * depth
        if t.children_left[node] == t.children_right[node]:          # leaf
            lines.append(f"{pad}return {ratio.get(int(node), 0.0):.4f}f;   // leaf {node}: "
                         f"{int(t.n_node_samples[node])} training rows")
            return
        f = features[t.feature[node]]
        lines.append(f"{pad}if (x[{FEATURE_COLUMNS[f][2]}] <= {t.threshold[node]:.6f}f) {{")
        walk(t.children_left[node], depth + 1)
        lines.append(f"{pad}}} else {{")
        walk(t.children_right[node], depth + 1)
        lines.append(f"{pad}}}")

    walk(0, 1)
    return "\n".join(lines)


def micromlgen_code(tree, features):
    try:
        from micromlgen import port
    except ImportError:
        return None
    try:
        return port(tree, classname="CartClassPredictor", classmap={SAFE: "Safe", DANGER: "Danger"})
    except Exception as error:          # keep the export usable even if micromlgen changes
        print(f"micromlgen could not port the tree ({error}); header has the Pf function only.")
        return None


def write_header(path, tree, columns, features, ratio, threshold, summary):
    enum = "\n".join(f"    {FEATURE_COLUMNS[f][2]} = {i},   // {f}: {FEATURE_COLUMNS[f][1]}"
                     for i, f in enumerate(features))
    uses = "\n".join(f"#define CART_USES_{f.upper()} 1" for f in features)
    mml = micromlgen_code(tree, features)
    mml_block = ""
    if mml:
        mml_block = ("\n// ---- micromlgen port (class only: 0 = Safe, 1 = Danger). The routing uses\n"
                     "// cartFailureProbability() above, which also gives the leaf's Pf.\n"
                     "#ifdef CART_INCLUDE_MICROMLGEN\n" + mml + "\n#endif\n")
    text = f"""#ifndef CART_MODEL_H
#define CART_MODEL_H

// Generated by tools/train_cart.py -- do not edit by hand; re-run the script instead.
// {summary}
//
// CART relay classifier (Gini, max depth {tree.get_params()['max_depth']}, min samples split \
{tree.get_params()['min_samples_split']}, class weight balanced).
// Input x[]: the neighbour's features, in this order and these units.
// Output Pf: share of training transmissions in the same leaf that were lost (0..1).
// Risky when Pf >= CART_RISKY_THRESHOLD (tuned on the validation split).

#include <stdint.h>

enum CartFeature : uint8_t {{
{enum}
}};
{uses}
constexpr uint8_t CART_FEATURE_COUNT = {len(features)};
constexpr float CART_RISKY_THRESHOLD = {threshold:.2f}f;

inline float cartFailureProbability(const float x[CART_FEATURE_COUNT]) {{
{c_tree_code(tree, columns, features, ratio)}
}}

inline bool cartIsRisky(const float x[CART_FEATURE_COUNT]) {{
    return cartFailureProbability(x) >= CART_RISKY_THRESHOLD;
}}
{mml_block}
#endif
"""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(text, encoding="utf-8")


# ----------------------------------------------------------------------------- plots
def plots(out, tree, features, sweep_df, chosen, test_scores):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib missing: skipped the pictures.")
        return

    fig, ax = plt.subplots(figsize=(7, 4))
    for col, style in (("precision", "--"), ("recall", ":"), ("f1", "-")):
        ax.plot(sweep_df.threshold, sweep_df[col], style, label=col.capitalize() if col != "f1" else "F1")
    ax.axvline(chosen, color="grey", lw=1)
    ax.text(chosen, 0.02, f"  chosen {chosen:.2f}", color="grey")
    ax.set_xlabel("Pf threshold (Risky when Pf >= threshold)")
    ax.set_ylabel("Score on validation split (Danger = positive)")
    ax.set_ylim(0, 1.02)
    ax.legend(frameon=False)
    ax.set_title("Pf threshold sweep")
    fig.tight_layout()
    fig.savefig(out / "threshold_sweep.png", dpi=150)
    plt.close(fig)

    c = test_scores["confusion"]
    cm = np.array([[c["TN_safe_ok"], c["FP_safe_flagged"]], [c["FN_danger_missed"], c["TP_danger_caught"]]])
    fig, ax = plt.subplots(figsize=(4.2, 3.8))
    ax.imshow(cm, cmap="Blues")
    for i in range(2):
        for j in range(2):
            ax.text(j, i, str(cm[i, j]), ha="center", va="center",
                    color="white" if cm[i, j] > cm.max() / 2 else "black", fontsize=13)
    ax.set_xticks([0, 1], ["Safe", "Danger"])
    ax.set_yticks([0, 1], ["Safe", "Danger"])
    ax.set_xlabel("Predicted")
    ax.set_ylabel("Actual")
    ax.set_title(f"Test split, Pf >= {test_scores['threshold']:.2f}")
    fig.tight_layout()
    fig.savefig(out / "confusion_matrix.png", dpi=150)
    plt.close(fig)

    order = np.argsort(tree.feature_importances_)
    fig, ax = plt.subplots(figsize=(6, 0.5 + 0.5 * len(features)))
    ax.barh([features[i] for i in order], tree.feature_importances_[order], color="#3b6ea5")
    ax.set_xlabel("Gini importance")
    ax.set_title("Feature importance")
    fig.tight_layout()
    fig.savefig(out / "feature_importance.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(max(10, 3 * tree.get_n_leaves() ** 0.8), 7))
    plot_tree(tree, feature_names=features, class_names=["Safe", "Danger"], filled=True,
              impurity=True, proportion=False, rounded=True, fontsize=7, ax=ax)
    fig.tight_layout()
    fig.savefig(out / "tree.png", dpi=150)
    plt.close(fig)


# ----------------------------------------------------------------------------- main
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", help="cart_dataset.csv from build_cart_dataset.py")
    parser.add_argument("--features", default=DEFAULT_FEATURES,
                        help=f"comma list from {','.join(FEATURE_COLUMNS)} (default {DEFAULT_FEATURES})")
    parser.add_argument("--train-source", choices=["heartbeat", "data", "all"], default="heartbeat",
                        help="rows to train on (default heartbeat; data rows are then the external test)")
    parser.add_argument("--split", choices=["session", "stratified"], default="session")
    parser.add_argument("--test-size", type=float, default=0.2)
    parser.add_argument("--max-depth", type=int, default=5)
    parser.add_argument("--min-samples-split", type=int, default=10)
    parser.add_argument("--threshold", type=float, default=None,
                        help="fix the Pf threshold instead of tuning it (e.g. 0.70)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out-dir", default="cart_output")
    parser.add_argument("--header", default="include/cart_model.h")
    args = parser.parse_args()

    features = [f.strip() for f in args.features.split(",") if f.strip()]
    unknown = [f for f in features if f not in FEATURE_COLUMNS]
    if unknown or not features:
        sys.exit(f"Unknown feature(s) {unknown}; choose from {', '.join(FEATURE_COLUMNS)}")

    data, external, columns = load(args.dataset, features, args.train_source)
    if len(data) < 30 or data.label.nunique() < 2:
        sys.exit(f"Not enough training rows ({len(data)}) or only one class. Collect more Test F data.")
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    X, y = data[columns].values.astype(float), data.label.values.astype(int)
    train_idx, test_idx = split(data, args.split, args.test_size, args.seed)

    # 1. Tune the Pf threshold on a validation part of the TRAINING rows only.
    inner = data.iloc[train_idx].reset_index(drop=True)
    fit_idx, val_idx = split(inner, args.split, args.test_size, args.seed + 1)
    Xi, yi = inner[columns].values.astype(float), inner.label.values.astype(int)
    tuning_tree = make_tree(args).fit(Xi[fit_idx], yi[fit_idx])
    tuning_ratio = leaf_failure_ratio(tuning_tree, Xi[fit_idx], yi[fit_idx])
    sweep_df = sweep(yi[val_idx], predict_pf(tuning_tree, tuning_ratio, Xi[val_idx]))
    sweep_df.to_csv(out / "threshold_sweep.csv", index=False)
    if args.threshold is not None:
        chosen = args.threshold
    else:
        best = sweep_df[sweep_df.f1 == sweep_df.f1.max()]
        chosen = float(best.iloc[(best.threshold - 0.5).abs().argsort().iloc[0]].threshold)

    # 2. Final tree on all training rows; Pf per leaf from those rows.
    tree = make_tree(args).fit(X[train_idx], y[train_idx])
    ratio = leaf_failure_ratio(tree, X[train_idx], y[train_idx])

    # 3. Test split (never seen in training or tuning).
    pf_test = predict_pf(tree, ratio, X[test_idx])
    test_scores = scores(y[test_idx], pf_test, chosen)
    test_at_paper = scores(y[test_idx], pf_test, PAPER_THRESHOLD)
    result = {
        "dataset": args.dataset, "features": features, "train_source": args.train_source,
        "split": args.split, "rows_train": int(len(train_idx)), "rows_test": int(len(test_idx)),
        "danger_share_train": float(y[train_idx].mean()), "danger_share_test": float(y[test_idx].mean()),
        "test_sessions": sorted(map(str, data.iloc[test_idx].session.unique())),
        "chosen_threshold": chosen, "test": test_scores, "test_at_0.70": test_at_paper,
        "feature_importance": dict(zip(features, map(float, tree.feature_importances_))),
        "depth": int(tree.get_depth()), "leaves": int(tree.get_n_leaves()),
    }

    # 4. External test: real data packets (ACK/NOACK) from the evaluation runs.
    if len(external):
        pf_ext = predict_pf(tree, ratio, external[columns].values.astype(float))
        result["data_packets"] = scores(external.label.values.astype(int), pf_ext, chosen)

    summary = (f"Trained on {len(train_idx)} {args.train_source} rows, split by {args.split}; "
               f"test F1 = {test_scores['f1']:.3f}, Recall = {test_scores['recall']:.3f}")
    write_header(args.header, tree, columns, features, ratio, chosen, summary)
    plots(out, tree, features, sweep_df, chosen, test_scores)
    (out / "report.json").write_text(json.dumps(result, indent=2), encoding="utf-8")

    def block(name, s):
        c = s["confusion"]
        return (f"{name} ({s['rows']} rows, Risky when Pf >= {s['threshold']:.2f})\n"
                f"  Accuracy {s['accuracy']:.3f}   Precision {s['precision']:.3f}   "
                f"Recall {s['recall']:.3f}   F1 {s['f1']:.3f}\n"
                f"                 predicted Safe   predicted Danger\n"
                f"  actual Safe    {c['TN_safe_ok']:>14}   {c['FP_safe_flagged']:>16}\n"
                f"  actual Danger  {c['FN_danger_missed']:>14}   {c['TP_danger_caught']:>16}\n")

    text = [
        f"CART relay classifier -- {summary}",
        f"Features: {', '.join(features)}   Tree depth {result['depth']}, {result['leaves']} leaves",
        f"Train rows {len(train_idx)} (Danger {100 * result['danger_share_train']:.0f}%), "
        f"test rows {len(test_idx)} (Danger {100 * result['danger_share_test']:.0f}%), "
        f"test sessions: {', '.join(result['test_sessions'])}",
        f"Pf threshold: {chosen:.2f} ({'fixed' if args.threshold is not None else 'best F1 on validation'})",
        "",
        block("TEST SPLIT", test_scores),
        block("TEST SPLIT at the paper's 0.70", test_at_paper),
    ]
    if "data_packets" in result:
        text.append(block("REAL DATA PACKETS (ACK/NOACK)", result["data_packets"]))
    text.append("Feature importance: " + ", ".join(f"{f} {v:.2f}" for f, v in
                                                     sorted(result["feature_importance"].items(),
                                                            key=lambda kv: -kv[1])))
    text.append("")
    text.append("Tree rules:")
    text.append(export_text(tree, feature_names=features))
    report = "\n".join(text)
    (out / "report.txt").write_text(report, encoding="utf-8")
    print(report)
    print(f"Wrote {args.header} and the pictures in {out}/")


if __name__ == "__main__":
    main()
