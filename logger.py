import sys
import logging


def get_configured_logger_by_name(name):
    logger = logging.getLogger(name)
    formatter = logging.Formatter('%(asctime)s : %(levelname)s : %(name)s : %(message)s')
    logger.setLevel('INFO')
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger