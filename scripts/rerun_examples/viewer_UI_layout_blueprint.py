import rerun as rr
import rerun.blueprint as rrb


import cv2
import rerun as rr
import numpy as np


rr.init(application_id="rerun_video_frame_by_frame", spawn=True)
video_path = "video/video.mp4"

cap = cv2.VideoCapture(video_path)

frame_idx = 0
fps = cap.get(cv2.CAP_PROP_FPS)
print(fps)
while True:
    ok, frame_bgr = cap.read()
    if not ok:
        break

    # ---- define BOTH timelines ----
    rr.set_time("frame_nr", sequence=frame_idx)
    rr.set_time("video_time", duration=frame_idx / fps)

    # ----  image ----
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    cv2.putText(frame_rgb, f"Frame {frame_idx}", (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    rr.log("video", rr.Image(frame_rgb))

    # ---- fake moving point (like pose) ----
    x = frame_idx * 0.05
    rr.log("world/point", rr.Points3D([[x, 0, 0]], radii=0.2))

    frame_idx += 1
cap.release()

rr.send_blueprint(
    rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world"),
            rrb.Spatial2DView(origin="/video"),
        ),
        rrb.BlueprintPanel(state="hidden"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed")
    )
)

# # --- Layout: Tabs — switch between 3D world view and 2D video using tabs - --
# rr.send_blueprint(
#     rrb.Blueprint(
#         rrb.Tabs(
#             rrb.Spatial3DView(origin="/world"),
#             rrb.Spatial2DView(origin="/video")
#         )
#     )
# )
