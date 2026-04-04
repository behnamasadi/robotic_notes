import gradio as gr
import types


def greet(name: str, count: int) -> str:
    return name*count


with gr.Blocks() as demo:
    name = gr.Text(label="name:")
    count = gr.Slider(label="count")
    out = gr.Text(label="out")
    btn = gr.Button()
    btn.click(fn=greet, api_name="present", inputs=[name, count], outputs=out)


demo.launch()
