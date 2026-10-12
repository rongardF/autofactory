

def hex_to_string(hex_str: str) -> str:
    """Convert a hex string to a human-readable string."""
    # Convert the hex string to bytes
    byte_array = bytes.fromhex(hex_str)
    # Decode the bytes to a string, ignoring any non-UTF-8 characters
    return byte_array.decode('utf-8', errors='ignore')