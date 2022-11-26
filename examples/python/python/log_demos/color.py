import logging

FMT = "%(asctime)s - %(name)s - %(levelname)s -%(message)s"

class CustomFormatter(logging.Formatter):
    GRAY = '\u001b[38;21m'
    BLUE = '\u001b[38;5;39m'
    YELLOW = '\u001b[38;5;226m'
    RED = '\u001b[38;5;196m'
    BOLD_RED = '\u001b[31;1m'
    __reset = '\u001b[0m'
    GREEN = "\u001b[32m"
    MAGENTA= "\u001b[35m"
    CYAN= "\u001b[36m"
    BACKGROUND_RED = "\u001b[41m"
    BACKGROUND_GREEN = "\u001b[42m"

    def __init__(self, fmt):
        super().__init__()
        self.fmt = fmt
        self.__color_formats = {
            logging.DEBUG: self.GRAY + self.fmt + self.__reset,
            logging.INFO: self.BLUE + self.fmt + self.__reset,
            logging.WARNING: self.YELLOW + self.fmt + self.__reset,
            logging.ERROR: self.RED + self.fmt + self.__reset,
            logging.CRITICAL: self.BOLD_RED + self.fmt + self.__reset,
        }

    def set_level_color(self, level, color):
        """
        Color code: https://www.lihaoyi.com/post/BuildyourownCommandLinewithANSIescapecodes.html
        """
        self.__color_formats[level] = color + self.fmt + self.__reset

    def format(self, record):
        log_fmt = self.__color_formats.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

LOG_LEVEL_CLIENT = 21
logging.CLIENT = LOG_LEVEL_CLIENT 
logging.addLevelName(logging.CLIENT, "CLIENT")

logger = logging.getLogger("color_logger")
logger.setLevel(logging.DEBUG)

#create console handler and set level to debug
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
color_formatter = CustomFormatter(FMT)
color_formatter.set_level_color(logging.CLIENT, CustomFormatter.BACKGROUND_GREEN)
handler.setFormatter(color_formatter)
logger.addHandler(handler)

# usage
logger.debug('This is a debug-level message')
logger.info('This is an info-level message')
logger.warning('This is a warning-level message')
logger.error('This is an error-level message')
logger.critical('This is a critical-level message')
logger.log(logging.CLIENT, "this is client")