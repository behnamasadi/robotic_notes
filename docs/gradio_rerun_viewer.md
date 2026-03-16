# gradio-rerun Viewer

## Install

```bash
pip install "rerun-sdk==0.29.2"
pip install "gradio_rerun==0.29.2"
pip install "gradio==6.5.1"
```

## Simple Application  with gradio Interface
`gr.Interface` — Simple, automatic layout

```python
import gradio as gr

def greet(name, age):
    return f"Hello {name}, you are {age}"

demo = gr.Interface(
    fn=greet,
    inputs=[gr.Textbox(label="Name"), gr.Slider(0, 100, label="Age")],
    outputs=gr.Textbox(label="Result"),
)
demo.launch()
```

Gradio **automatically** builds the layout for you: inputs on the left, outputs on the right, a Submit button in the middle. One function, one trigger. That's it.

**Limitation**: you can't control layout, can't have multiple buttons that do different things, can't store per-session state, can't wire up custom events like `viewer.selection_change(...)`.



## Gradio `gr.Blocks`  Full control

`Blocks` is the **low-level UI framework of Gradio**.
While `gr.Interface` lets you build simple demos quickly, `Blocks` gives **full control over layout, components, and interactions**.


A `Blocks` app contains:

1. **Layout containers**
2. **Components** (textbox, image, button, etc.)
3. **Events** connecting components to functions

Basic structure:

```python
import gradio as gr

def greet(name):
    return f"Hello {name}"

with gr.Blocks() as demo:

    name = gr.Textbox(label="Your name")
    output = gr.Textbox()

    btn = gr.Button("Run")

    btn.click(
        fn=greet,
        inputs=name,
        outputs=output
    )

demo.launch()
```

Flow:

```
Textbox → Button → Python function → Output textbox
```

---

#### Layout System

`Blocks` supports **flexbox-style layout**.

Main containers:

| Container   | Purpose            |
| ----------- | ------------------ |
| `Row`       | horizontal layout  |
| `Column`    | vertical layout    |
| `Tab`       | tabs               |
| `Accordion` | collapsible panels |
| `Group`     | logical grouping   |

---

#### Example Layout

```python
with gr.Blocks() as demo:

    with gr.Row():
        with gr.Column():
            input1 = gr.Textbox()
            input2 = gr.Textbox()

        with gr.Column():
            output = gr.Textbox()

    btn = gr.Button("Process")
```

Layout:

```
+-----------------------------+
| input1   |                  |
| input2   |      output      |
+-----------------------------+
|           button            |
+-----------------------------+
```

---

####  Components

Gradio provides many UI components.

Common ones:

| Component  | Purpose          |
| ---------- | ---------------- |
| `Textbox`  | text input       |
| `Image`    | image upload     |
| `Video`    | video upload     |
| `Audio`    | microphone input |
| `Slider`   | numeric range    |
| `Dropdown` | select option    |
| `Checkbox` | boolean          |
| `Button`   | trigger event    |
| `Markdown` | formatted text   |
| `Chatbot`  | chat interface   |

Example:

```python
image = gr.Image()
text = gr.Textbox()
slider = gr.Slider(0,100)
```

---

####  Events System

Components trigger **events**.

Examples:

| Event       | Meaning        |
| ----------- | -------------- |
| `.click()`  | button pressed |
| `.change()` | value changed  |
| `.submit()` | enter pressed  |
| `.upload()` | file uploaded  |
| `.select()` | item selected  |

Example:

```python
btn.click(
    fn=predict,
    inputs=[image, slider],
    outputs=text
)
```

This means:

```
Button click
      ↓
predict(image, slider)
      ↓
result → textbox
```

---

####  Multiple Inputs and Outputs

Functions can handle many inputs.

Example:

```python
def add(a,b):
    return a+b

with gr.Blocks() as demo:

    a = gr.Number()
    b = gr.Number()

    result = gr.Number()

    btn = gr.Button("Add")

    btn.click(add, [a,b], result)
```

---

####  State (Important)

You can keep **persistent state** across interactions.

Example:

```python
state = gr.State(0)

def increment(x):
    x += 1
    return x, x

btn.click(
    increment,
    inputs=state,
    outputs=[state, textbox]
)
```

State allows building:

* chatbots
* workflows
* multi-step pipelines

---

####  Dynamic UI Updates

Gradio can dynamically update components.

Example:

```python
def toggle(show):
    if show:
        return gr.update(visible=True)
    else:
        return gr.update(visible=False)
```

---

####  Chatbot Example

```python
def chat(message, history):
    history.append((message, "AI response"))
    return history

with gr.Blocks() as demo:

    chatbot = gr.Chatbot()
    msg = gr.Textbox()

    msg.submit(chat, [msg, chatbot], chatbot)

demo.launch()
```

---

####  Streaming Outputs

Very useful for:

* LLM responses
* progress updates
* video generation

Example:

```python
import time

def generate():
    for i in range(5):
        time.sleep(1)
        yield f"Step {i}"

btn.click(generate, None, textbox)
```

---

####  Tabs Example

```python
with gr.Blocks() as demo:

    with gr.Tab("Image Model"):
        image = gr.Image()
        out = gr.Image()

    with gr.Tab("Text Model"):
        text = gr.Textbox()
        answer = gr.Textbox()
```

---

####  Using Gradio for AI Systems

For someone building **AI tools locally** (like your Jarvis stack or MASt3R experiments), `Blocks` is ideal for creating:

* model dashboards
* prompt playgrounds
* dataset labeling tools
* training monitors

Example layout:

```
+-----------------------------------+
|        Model Settings             |
| model dropdown | temperature      |
+-----------------------------------+
|            Prompt Box             |
+-----------------------------------+
|              Output               |
+-----------------------------------+
| Logs | GPU usage | History        |
+-----------------------------------+
```

---

####  Example: Local AI Dashboard

Example useful for your **local AI server**:

```python
import gradio as gr

def run_model(prompt, model):
    return f"Model {model} says: {prompt}"

with gr.Blocks() as app:

    gr.Markdown("# Local AI Control Panel")

    with gr.Row():
        model = gr.Dropdown(["Local LLM","Cloud GPT"])
        prompt = gr.Textbox()

    output = gr.Textbox()

    run = gr.Button("Run")

    run.click(run_model, [prompt, model], output)

app.launch()
```

This creates a **simple agent controller UI**.

---

####  Deploy Options

You can run:

Local:

```
python app.py
```

LAN access:

```
demo.launch(server_name="0.0.0.0")
```

Public:

```
demo.launch(share=True)
```

Or inside Docker.

---






```python
import gradio as gr

def update(name):
    return f"Hello {name}"

with gr.Blocks() as demo:
    gr.Markdown("Start typing below and then click **Run** to see the output.")
    with gr.Row():
        inp = gr.Textbox(placeholder="What is your name?")
        out = gr.Textbox()
    btn = gr.Button("Run")
    btn.click(fn=update, inputs=inp, outputs=out)

demo.launch()
```



You manually place every component, create every button, and wire every event yourself. More verbose — but you get:

- **Custom layouts** (`gr.Row`, `gr.Column`, `gr.Tab`)
- **Multiple buttons** each calling different functions
- **Session state** via `gr.State`
- **Custom events** — not just click, but also `change`, `select`, `upload`, or Rerun-specific ones like `viewer.selection_change(...)`
- **Lifecycle hooks** — `demo.load()` and `demo.close()` for setup/teardown per user




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
python /home/$USER/anaconda3/envs/robotic_notes/src/scripts/gradio_rerun_minimal.py
```



## Why `gr.Blocks` is required for Rerun

The Rerun integration specifically needs Blocks because:

1. **`gr.State`** — you need to store a `recording_id` per user session. `gr.Interface` has no concept of per-session state.
2. **Custom events** — `viewer.selection_change(...)`, `viewer.time_update(...)` are event bindings you wire manually. There's no place to do that in `gr.Interface`.
3. **Tabs** — real apps have multiple panels (Streaming tab, RRD tab, etc.), which require `gr.Tab`.
4. **Lifecycle** — `demo.load(initialize_user)` and `demo.close(cleanup_user)` are needed to isolate multi-user state. Only available in Blocks.

---


## Check if Gradio is running and which port is open:

```bash
# Check default Gradio URL
curl -i "http://127.0.0.1:7860/"

# Find process listening on port 7860
lsof -i :7860 -n -P

# Get only PID
lsof -ti :7860

# Inspect the command for that PID
ps -fp <PID>
```

If you need to stop a previously running Gradio server:

```bash
# Graceful stop
kill <PID>

# Force stop only if needed
kill -9 <PID>
```

What it does:
- Shows one button: `Generate Points`
- On click, generates one small 3D point cloud
- Sends bytes to the Rerun viewer and renders immediately




