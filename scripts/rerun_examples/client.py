import rerun as rr
import numpy as np


application_id = "my_web_app"
grpc_port = 10000

rr.init(application_id=application_id)

server_info = f"rerun+http://127.0.0.1:{grpc_port}/proxy"

print("connecting: ", server_info)

rr.connect_grpc(server_info)


points = np.array([[0, 0, 0], [1, 0, 1], [0, 1, 2]])

rr.log(entity=rr.Points3D(points), entity_path="/world/points")
