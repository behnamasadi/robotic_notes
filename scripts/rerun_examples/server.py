import rerun as rr
import time

application_id = "my_web_app"
grpc_port = 10000

rr.init(application_id=application_id)
server_info = rr.serve_grpc(grpc_port=grpc_port, server_memory_limit="1GiB")


print(f"Server running at: {server_info}")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Server stopped.")
