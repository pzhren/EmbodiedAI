import logging

class Logger(object):
    def __init__(self, log_level:int=logging.INFO, log_file:str=None):
        self.log = logging.getLogger(log_file)
        self.log.setLevel(log_level)
        self._log_file = log_file
        self._log_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self._console_handler = logging.StreamHandler()
        self._console_handler.setFormatter(self._log_formatter)
        self.log.addHandler(self._console_handler)
        if log_file is not None:
            self._file_handler = logging.FileHandler(log_file)
            self._file_handler.setFormatter(self._log_formatter)
            self.log.addHandler(self._file_handler)
            

log = Logger(
    log_level=logging.ERROR,
    log_file=None
).log