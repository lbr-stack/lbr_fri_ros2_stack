def string_to_bool(string: str = "true"):
    if string in ["True", "true"]:
        return True
    elif string in ["False", "false"]:
        return False
    else:
        raise ValueError(
            f"String must be one of ['True', 'true', 'False', 'false'], got {string}."
        )
