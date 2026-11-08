# Broadcast from runtime_logger_node so every producer joins the same
# session and connects to the collector's actual AF_UNIX socket.
bool enabled
string run_id
string socket_path
