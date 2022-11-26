import logging

FMT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
logging.basicConfig(format=FMT, level=logging.INFO)

log = logging.getLogger("demo")
log.info("formatter example")