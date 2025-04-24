import os
import logging
from logging.handlers import TimedRotatingFileHandler
from datetime import datetime


def setup_logger(name="System", log_subdir="../logs"):
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), log_subdir))
    os.makedirs(log_dir, exist_ok=True)
    
    current_date = datetime.now().strftime("%Y-%m-%d-%H")
    log_file = os.path.join(log_dir, f"raspi_{current_date}.log")
    
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    
    if logger.handlers:
        return
    
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    file_handler = TimedRotatingFileHandler(
        log_file, 
        when='midnight',
        interval=1,
        backupCount=30
    )
    file_handler.setLevel(logging.INFO)
    file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(file_formatter)
    file_handler.suffix = "%Y-%m-%d-%H"
    logger.addHandler(file_handler)
    
    logger.propagate = False
    
    logger.info("=" * 50)
    logger.info(f"{name} Logger initialized")
    logger.info(f"Log file: {log_file}")
    logger.info("=" * 50)
    
    return logger

LOG = setup_logger()