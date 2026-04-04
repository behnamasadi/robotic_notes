# video_stream_minimal.py
import av
import cv2
import rerun as rr

VIDEO_PATH = "video/video.mp4"

rr.init("video_stream_minimal", spawn=True)

cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    raise RuntimeError(f"Could not open {VIDEO_PATH}")

fps = cap.get(cv2.CAP_PROP_FPS)
if fps <= 0:
    fps = 30.0

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

codec = rr.VideoCodec.H264

# Log codec once
rr.log("video", rr.VideoStream(codec=codec), static=True)

# PyAV encoder setup
container = av.open("/dev/null", mode="w", format="h264")
stream = container.add_stream("libx264", rate=fps)
assert isinstance(stream, av.video.stream.VideoStream)
stream.width = width
stream.height = height
stream.pix_fmt = "yuv420p"
stream.max_b_frames = 0  # important for current Rerun video stream support

frame_idx = 0
while True:
    ok, frame_bgr = cap.read()
    if not ok:
        break

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    av_frame = av.VideoFrame.from_ndarray(frame_rgb, format="rgb24")

    packets = stream.encode(av_frame)
    for packet in packets:
        rr.set_time("frame_nr", sequence=frame_idx)
        rr.set_time("video_time", duration=frame_idx / fps)
        rr.log("video", rr.VideoStream(sample=bytes(packet)))

    frame_idx += 1

# Flush encoder
for packet in stream.encode():
    rr.set_time("frame_nr", sequence=frame_idx)
    rr.set_time("video_time", duration=frame_idx / fps)
    rr.log("video", rr.VideoStream(sample=bytes(packet)))
    frame_idx += 1

cap.release()
container.close()
