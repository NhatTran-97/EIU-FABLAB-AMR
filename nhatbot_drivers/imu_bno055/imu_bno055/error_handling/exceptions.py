class TransmissionException(Exception):
    """
    Exception thrown in case of failing transmissions between host and BNO055 sensor device
    """

    pass

class BusOverRunException(TransmissionException):
    """
    Exception thrown when BNo055 sensor device data fusion was not ready
    """
    pass