# asset_video_manual.py
import rerun as rr
import rerun.blueprint as rrb

VIDEO_PATH = "video/video.mp4"

rr.init("asset_video_manual", spawn=True)

rr.log("video_asset", rr.AssetVideo(path=VIDEO_PATH), static=True)

rr.log(
    "frame_1s",
    rr.VideoFrameReference(seconds=1.0, video_reference="video_asset"),
)
rr.log(
    "frame_2s",
    rr.VideoFrameReference(seconds=2.0, video_reference="video_asset"),
)

rr.send_blueprint(
    rrb.Horizontal(
        rrb.Spatial2DView(origin="frame_1s"),
        rrb.Spatial2DView(origin="frame_2s"),
    )
)
