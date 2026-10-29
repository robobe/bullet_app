# Bullet Perception Library

## Goal

Move image-to-bounding-box detection into a reusable `bullet_perception`
workspace package and expose a generic ndarray detector API.

## Implementation steps

1. Create the `bullet_perception` package with the existing detector behavior
   and migrate its unit tests.
2. Replace `bullet_app.detector.detect_red_box` imports and calls with
   `bullet_perception.detect_bbox`.
3. Update workspace metadata, lockfile, and documentation.

## Interface changes

Expose `detect_bbox(image: numpy.ndarray) -> tuple[int, int, int, int] | None`.
The accepted image format, thresholds, validation, and input-preservation
behavior remain unchanged.

## Verification

Run focused perception and bullet-app tests, then the complete locked workspace
test suite and check that no old API references remain.

## Status

Pending

## Implementation commit

Pending
