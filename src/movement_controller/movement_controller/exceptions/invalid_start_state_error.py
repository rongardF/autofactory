

class InvalidStartStateError(Exception):
    """Exception raised if robot start state is invalid for planning (violation of joint limits, etc.)."""
    pass