# Public demonstration guide

Use the real simulator and its saved evaluation artifacts. Do not replace a failed placement with a staged success shot. The generated video is an actual simulation recording and explicitly labels the assisted weld grasp.

## Rehearsal

```bash
bash launch/run_simulation.sh
bash launch/run_vision.sh
bash launch/run_teleop.sh
bash launch/run_autonomy.sh --seed 0
```

Close each viewer before starting the next command. Show the robot, all four colored 10 cm pegs, matching socket openings, and the two RGB-D views. The third camera is an overview used for recording. In teleoperation, rehearse W/S, A/D, R/F, O/C, P/L, Space, and Q. Space is a simulation motion stop. Manual human keyboard interaction is a rehearsal step; automated command handlers have already been exercised.

For reproducible unattended recording:

```bash
bash launch/run_autonomy.sh --headless --seed 0 \
  --output results/demo_repeat --record results/demo_repeat.mp4
```

Playback with an installed video player, or `ffplay results/demo_repeat.mp4`. The MP4 contains an overview, overhead shape/color labels and estimated positions, angled view, depth previews, simulation timestamp, weld disclosure, and the measured final outcome. The default recording rate is six frames per simulated second. No robot movement or object placement is generated from a static image. The final two seconds repeat the actual last frame for legibility.

The current local recording is `results/demo_fast.mp4`, with its episode data and final frames in `results/performance/demo/`. It demonstrates the continuous-motion controller. The earlier `results/demo_final.mp4` and `results/demo_final/` artifacts preserve the original slower controller as a historical reference. Video files are deliberately excluded from Git to keep the source repository small. A public upload has not been performed.

## Suggested narration

1. State the assignment and that this is a simulation-only WidowX-250S, sourced from Menagerie/Interbotix with retained licensing.
2. Identify the four colors and four shapes, the physical socket cutouts, and the two RGB-D sensors.
3. Explain HSV segmentation, contour classification, depth-based top-face localization, and shape-based socket matching. Show coordinates in meters and the camera depth panels.
4. Follow the arm through pregrasp, close/assisted attachment, lift, transfer, yaw alignment, insertion, physical release, and retreat. Explicitly disclose the proximity-gated weld aid.
5. Show all four seated pegs and the saved camera/physics verification, including insertion depth and stability.
6. Report the current measured five-trial results: 20/20 first-attempt placements, 5/5 complete episodes, mean 90.206 simulated seconds, mean initial pose error 0.282 mm. Compare the preserved original mean of 228.862 s: 2.54× faster, with unchanged success thresholds. Identify the narrow randomization region and small sample size.
7. Show `pytest -q` (61 passing tests) and `results/performance/trials.csv`. Explain that wall time is separate from simulated execution time and depends strongly on recording/GUI rendering.

## Recording the desktop

For a live narrated demonstration, use Ubuntu's existing screen recorder or OBS if already installed. Capture only the simulator/camera windows and terminal metrics. Close unrelated applications and hide account details. Start autonomous mode from the terminal so the command and result are visible. Optional FFmpeg transcoding can reduce the already generated MP4 for sharing; retain the original episode JSON beside the published artifact.

## Publication gate

Obtain the owner's confirmation before creating a public GitHub repository, pushing files, or uploading the video. After approval, create `anjilshah/tri-wizard-peg-trial`, push the local `main` branch, and upload the generated MP4 to the chosen public video host or a GitHub release. Add the actual public links to README after they exist. No URL should claim a demo has been published before upload is verified.
