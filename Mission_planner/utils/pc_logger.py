import os
import logging
from logging.handlers import TimedRotatingFileHandler
from datetime import datetime

LOG_FILE=""

def setup_logger(logger_name='MavlinkController', log_subdir="../logs"):
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), log_subdir))
    os.makedirs(log_dir, exist_ok=True)
    
    current_date = datetime.now().strftime("%Y-%m-%d-%H%M%S")
    log_file = os.path.join(log_dir, f"pc_{current_date}.log")
    
    global LOG_FILE
    LOG_FILE=log_file

    logger = logging.getLogger(logger_name)
    
    if logger.handlers:
        return logger
    
    logger.setLevel(logging.INFO)
    
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', 
                                datefmt='%Y-%m-%d %H:%M:%S')
    
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    file_handler = TimedRotatingFileHandler(
        log_file,
        when='midnight',
        interval=1,
        backupCount=30
    )
    file_handler.setFormatter(formatter)
    file_handler.suffix = "%Y-%m-%d"
    logger.addHandler(file_handler)
    
    logger.info("=" * 50)
    logger.info(f"Log file: {log_file}")
    logger.info("=" * 50)
    
    return logger

LOG = setup_logger()