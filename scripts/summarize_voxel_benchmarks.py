"""Summarize raw GPU samples, without discarding outliers or warmup metadata."""
import argparse
import csv
import json
import math
import statistics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+")
    args = parser.parse_args()
    for path in args.csv:
        with open(path, newline="", encoding="utf-8") as source:
            frames = [row for row in csv.DictReader(source) if row["kind"] == "frame"]
        if not frames:
            raise SystemExit(f"No raw frames: {path}")
        samples = sorted(float(row["total_ms"]) for row in frames)
        mean = statistics.fmean(samples)
        print(json.dumps(dict(path=path, frames=len(samples),
                              mean_ms=mean, median_ms=statistics.median(samples),
                              p95_ms=samples[math.ceil(len(samples) * 0.95) - 1],
                              mean_gpu_fps=1000.0 / mean,
                              over_2ms=sum(ms > 2.0 for ms in samples))))


if __name__ == "__main__":
    main()
