import logging

#create new log level
LOG_LEVEL_CLIENT = 21
logging.CLIENT = LOG_LEVEL_CLIENT 
logging.addLevelName(logging.CLIENT, "CLIENT")

#create logger with "mylogger"
logger = logging.getLogger("mylogger")
logger.setLevel(logging.DEBUG)

#create console handler and set level to debug
handler = logging.StreamHandler()
handler.setLevel(logging.INFO)
logger.addHandler(handler)

#create formatter
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s -%(message)s")
handler.setFormatter(formatter)


logger.debug("this is debug")
logger.info("this is info")
logger.log(logging.CLIENT, "this is client")
logger.warning("this is warning")
logger.error("this is error")
logger.critical("this is critical")