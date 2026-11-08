# 1 Hz aggregate snapshot for the rqt panel. Latest-wins cache of
# UART telemetry plus the latest latency status.
builtin_interfaces/Time stamp
my_project_interfaces/UartTelemetryUi uart
my_project_interfaces/ExtruderLatencyStatus latency
bool latency_valid
float64 latency_age_s
