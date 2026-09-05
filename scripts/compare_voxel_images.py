"""Compare pre-UI benchmark captures without resizing or color conversion tricks."""
import argparse
import json

from PIL import Image, ImageChops


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference")
    parser.add_argument("candidate", nargs="?")
    parser.add_argument("--preview", help="Lossless PNG of candidate (or reference)")
    parser.add_argument("--diff", help="Amplified difference PNG")
    args = parser.parse_args()
    reference = Image.open(args.reference).convert("RGB")
    candidate = Image.open(args.candidate).convert("RGB") if args.candidate else reference
    if reference.size != candidate.size:
        raise SystemExit("Image dimensions differ")
    if args.preview:
        candidate.save(args.preview)
    diff = ImageChops.difference(reference, candidate)
    red, green, blue = diff.split()
    count = reference.width * reference.height
    changed = count - ImageChops.lighter(red, ImageChops.lighter(green, blue)).histogram()[0]
    maximum = max(hi for _, hi in diff.getextrema())
    mae = sum((i % 256) * n for i, n in enumerate(diff.histogram())) / (count * 3)
    print(json.dumps(dict(width=reference.width, height=reference.height,
                          changed_pixels=changed, total_pixels=count,
                          max_channel_error=maximum, mean_absolute_error=mae)))
    if args.diff:
        diff.point(lambda value: min(255, value * 16)).save(args.diff)


if __name__ == "__main__":
    main()
