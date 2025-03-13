def point_color(r):
    """
    0-200: green
    252-500: yellow
    500-750: orange
    750-: red
    """

    if r <= 250:
        return (0, 255, 0)
    elif r <= 500:
        return (0, 255, 255)
    elif r <= 750:
        return (0, 165, 255)
    else:
        return (0, 0, 255)


