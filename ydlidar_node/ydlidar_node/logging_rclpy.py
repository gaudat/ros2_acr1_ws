import rclpy

logger = rclpy.logging.get_logger("ydlidar")

log_info = logger.info
log_warning = logger.warning
log_debug = logger.debug
log_debug2 = log_debug3 = lambda *a: None
