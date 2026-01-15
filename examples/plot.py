"""Plot gyro-only and filtered attitude through an outage; requires matplotlib."""
import argparse
import csv
import io
import json
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--binary", default="build/attitude-demo")
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--output", default="docs/tracker-outage.png")
args = parser.parse_args()
run = subprocess.run([args.binary, str(args.seed)], check=True, capture_output=True, text=True)
rows = list(csv.DictReader(io.StringIO(run.stdout)))
times = [float(row["time_s"]) for row in rows]
fig, axes = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True, layout="constrained")
axes[0].semilogy(times, [float(r["gyro_only_error_deg"]) for r in rows], label="Gyro only", color="#b35806")
axes[0].semilogy(times, [float(r["angle_error_deg"]) for r in rows], label="Attitude filter", color="#2166ac")
axes[0].set(ylabel="Attitude error (degrees)", title=f"Attitude estimation through a tracker outage · seed {args.seed}")
axes[0].axvline(150, color="#b2182b", linestyle=":", label="Rejected tracker outlier")
axes[0].legend(loc="lower right")
axes[1].plot(times, [float(r["bias_error_rad_s"])*1e6 for r in rows], color="#2166ac")
axes[1].set(ylabel="Gyro-bias error (µrad/s)", xlabel="Elapsed time (seconds)")
for axis in axes:
    axis.axvspan(80, 120, color="gray", alpha=0.15)
    axis.grid(alpha=0.2)
    axis.spines[["top", "right"]].set_visible(False)
axes[0].text(100, 0.72, "Tracker\nunavailable", transform=axes[0].get_xaxis_transform(),
             ha="center", va="center", fontsize=9)
output = Path(args.output)
output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(output, dpi=160)
summary = json.loads(run.stderr)
output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary))
