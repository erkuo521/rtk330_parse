import os
import logging

class Logger(object):
    def __init__(self, dirname=None, filename=None):
        self.log = logging.getLogger(__file__)
        self.log.setLevel(logging.DEBUG)
        self.log.propagate = False
        
        if not self.log.handlers:
            logfile = os.path.join(dirname, filename)
            fh = logging.FileHandler(logfile, mode='w')
            fh.setLevel(logging.DEBUG)

            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)

            # formatter = logging.Formatter('%(asctime)s - %(levelname)s: %(message)s')
            # fh.setFormatter(formatter)
            # ch.setFormatter(formatter)

            self.log.addHandler(fh)
            self.log.addHandler(ch)
    
    def debug(self, msg, *args, **kwargs):
        self.log.debug(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self.log.info(msg, *args, **kwargs)
    
    def warning(self, msg, *args, **kwargs):
        self.log.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.log.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self.log.critical(msg, *args, **kwargs)
