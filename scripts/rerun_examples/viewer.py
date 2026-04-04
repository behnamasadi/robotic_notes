import rerun as rr
import time
rr.start_web_viewer_server()


application_id = "my_web_app"
grpc_port = 10000


server_info = f"rerun+http://127.0.0.1:{grpc_port}/proxy"

print(server_info)

rr.init(application_id=application_id)
rr.serve_web_viewer(connect_to=server_info)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("viewer stop")
