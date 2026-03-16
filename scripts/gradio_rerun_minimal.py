import math
import uuid
from typing import Generator

import gradio as gr
import rerun as rr
import rerun.blueprint as rrb
from gradio_rerun import Rerun


def generate_points() -> Generator[bytes, None, None]:
    rec = rr.RecordingStream(
        application_id="gradio_rerun_minimal",
        recording_id=str(uuid.uuid4()),
    )
    stream = rec.binary_stream()  # type: ignore

    rec.send_blueprint(rrb.Spatial3DView(origin="scene"))

    points: list[tuple[float, float, float]] = [
        (math.cos(i * 0.2) * 4.0, math.sin(i * 0.2) * 4.0, i * 0.05) for i in range(80)
    ]
    colors: list[tuple[int, int, int]] = [(255, 120, 40) for _ in points]
    rec.log("scene/points", rr.Points3D(points, colors=colors, radii=0.06))

    # Send one binary chunk back to the Gradio Rerun component.
    chunk: bytes | None = stream.read()
    if not chunk:
        raise gr.Error("No Rerun data was produced.")
    yield chunk


with gr.Blocks() as demo:
    run_btn: gr.Button = gr.Button("Generate Points")
    viewer: Rerun = Rerun(
        streaming=True,
        height=700,
        panel_states={"blueprint": "hidden", "selection": "hidden", "time": "collapsed"},
    )
    run_btn.click(generate_points, inputs=None, outputs=[viewer])


demo.launch(ssr_mode=False)