# gradio-rerun Viewer (Simple Tutorial)

This tutorial explains how Gradio and Rerun are connected when using `gradio_rerun` from pip.

## Install (pip-only)

```bash
/home/$USER/anaconda3/envs/robotic_notes/bin/python -m pip install "gradio_rerun==0.29.2" "gradio==6.5.1" "opencv-python"
```

## How Gradio and Rerun are connected

The connection has 4 small steps:

1. **Create a Rerun recording stream in Python**
   - `rec = rr.RecordingStream(...)`
   - `stream = rec.binary_stream()`

2. **Log data to Rerun**
   - Example: `rec.log("scene/points", rr.Points3D(points))`

3. **Return stream bytes from a Gradio callback**
   - Example: `return stream.read()` or `yield stream.read()`

4. **Bind that callback to a `Rerun(streaming=True)` output**
   - `run_btn.click(my_callback, outputs=[viewer])`

So Gradio handles the UI and button events, while Rerun handles the 2D/3D visualization data.  
`gradio_rerun` is the bridge between them.

## Minimal flow diagram

- Click button in Gradio
- Python callback runs
- Callback logs data to Rerun stream
- Callback returns stream bytes
- `Rerun` component displays the new points

## Minimal example in this repo

File:
- `src/scripts/gradio_rerun_minimal.py`

Run:

```bash
/home/$USER/anaconda3/envs/robotic_notes/bin/python /home/$USER/anaconda3/envs/robotic_notes/src/scripts/gradio_rerun_minimal.py
```

What it does:
- Shows one button: `Generate Points`
- On click, generates one small 3D point cloud
- Sends bytes to the Rerun viewer and renders immediately

## Version notes

- Recommended:
  - `python==3.12`
  - `gradio_rerun==0.29.2`
  - `gradio==6.5.1`
- Avoid upgrading `gradio` alone unless you re-test compatibility.

