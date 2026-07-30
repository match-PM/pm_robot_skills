# Calibration Fit Sensitivity To Ball Diameter

The smarpod/hexapod calibration is meaningfully sensitive to the assumed ball diameter, mainly as a Z/offset bias. It is usually less sensitive in XY and rotation unless the diameter error is large or the measurement geometry is poor.

## Where Diameter Enters

- The runtime constant is currently:

  ```python
  BALL_DIAMETER = 2 * 3.125  # 6.25 mm
  ```

  There is also an older/commented value of `6.35 mm`.

- The measurement procedure uses `BALL_DIAMETER / 2` as the Z offset when moving the sensor to the sphere top.

- The analysis uses the same diameter to create a fixed-radius sphere fit. The fitted sphere center is recovered from the measured surface points by minimizing radial residuals:

  ```text
  ||point_i - center|| - radius
  ```

- The fitted centers are then passed into the pivot solve for `B__T__P` and `J__t__P`.

## Practical Sensitivity

A diameter error `delta_D` produces roughly a radius error of:

```text
delta_R = delta_D / 2
```

That mostly appears as a bias in the recovered sphere-center height.

For example:

```text
6.35 mm - 6.25 mm = 0.10 mm diameter difference
0.10 mm / 2 = 0.05 mm = 50 um radius/center shift
```

So using `6.25 mm` instead of `6.35 mm` can shift the recovered sphere center by about `50 um`, primarily in Z.

The sphere curvature correction over the current 1 mm measurement grid is much smaller. For a `0.10 mm` diameter difference, the off-axis surface-height difference is only on the order of a few micrometres, roughly `3-6 um`.

## Effect On Final Calibration

The final pivot calibration can absorb some of this bias between:

- `J__t__P`, the pivot-point-to-sphere-center offset
- `B__T__P`, the calibrated base/platform transform

But the diameter still matters because all fitted sphere centers are derived from the assumed radius. If the diameter is wrong, the calibration can look internally consistent while carrying a systematic offset.

## Best Way To Quantify It

The code already includes a diameter sweep helper in `CalibrationAnalysis`:

```python
analysis.print_diameter_sweep([6.20, 6.25, 6.30, 6.35, 6.40])
```

The values to compare are:

- `J_t_P_mm`
- `B_T_P_translation_mm`
- `rms_error_um`
- `max_abs_error_um`

That sweep should be run on the actual smarpod measurement JSON. There was no saved smarpod measurement file in the checked repository, so the real numeric sensitivity could not be computed from local data.

## Short Conclusion

The calibration is sensitive enough that a `0.10 mm` diameter mismatch is not negligible. Expect about `50 um` of direct radius/Z-center bias, plus smaller curvature-related effects. If the ball is truly closer to `6.25 mm`, the current constant is appropriate; if it is a nominal `6.35 mm` ball, using `6.25 mm` will bias the fit.
