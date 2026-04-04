import os
import gradio as gr
import rerun as rr
from gradio_rerun import Rerun

os.environ.setdefault("GRADIO_TEMP_DIR", os.path.expanduser("/tmp/tmp_gradio"))


def generate():
    rr.init("demo", spawn=False)

    rec = rr.binary_stream()

    rr.log("points", rr.Points3D([[0, 0, 0], [1, 1, 1]]))

    return rec.read()


with gr.Blocks() as demo:
    btn = gr.Button("Generate")
    viewer = Rerun()

    btn.click(generate, outputs=viewer)

demo.launch()
