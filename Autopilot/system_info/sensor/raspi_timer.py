import time

last_check_time = time.time()

def marked():
    global last_check_time
    last_check_time = time.time()

def get_time_difference():
    return time.time() - last_check_time