import gradio as gr


def greet(name, intensity):
    return "Hello, " + name + "!" * int(intensity)


demo = gr.Interface(fn=greet, inputs=["text", "slider"], outputs=[
                    "text"], api_name="predict", flagging_mode="never")

demo.launch()

# https://www.gradio.app/docs/gradio/introduction
# gr.Textbox(), gr.Image(), and gr.HTML()


# https://www.gradio.app/main/guides/the-interface-class
# https://www.gradio.app/docs/gradio/introduction
# gr.Textbox(), gr.Image(), and gr.HTML()
# gr.Blocks()




# def update(name):
#     return f"Hello {name}"

# with gr.Blocks() as demo:
#     gr.Markdown("Start typing below and then click **Run** to see the output.")
#     with gr.Row():
#         inp = gr.Textbox(placeholder="What is your name?")
#         out = gr.Textbox()
#     btn = gr.Button("Run")
#     btn.click(fn=update, inputs=inp, outputs=out)

# demo.launch()