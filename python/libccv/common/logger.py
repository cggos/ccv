import logging


def init_logger(log_file_path=None, log_level=logging.WARNING):
    if log_file_path is not None:
        handlers = [
            logging.FileHandler(log_file_path),
            logging.StreamHandler(),
        ]
    else:
        handlers = [
            logging.StreamHandler(),
        ]

    logging.basicConfig(
        level=log_level,
        format="[%(asctime)s %(levelname)s] %(message)s",
        handlers=handlers,
    )


if __name__ == "__main__":
    init_logger(logging.INFO, "/tmp/test.log")
    logging.info("Hello, World!")
