# Since the argos and driver nodes can not be killed with Ctrl+C
# but only with Ctrl+Z, they end up in the background, leading
# to interference with later executions of the nodes.

# The purpose of this script is to grab nodes running
# in the background and kill them.

pkill -9 -f "argos3"
pkill -9 -f "rvr_async_driver"