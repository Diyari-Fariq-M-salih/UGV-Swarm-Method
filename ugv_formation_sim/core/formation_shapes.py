def triangle(d=1.5):
    return [
        (0, 0),
        (-d, -d),
        (d, -d)
    ]

def diamond(d=1.5):
    return [
        (0, 0),
        (-d, -d),
        (d, -d),
        (0, -2*d)
    ][:3]

def arrow(d=1.5):
    return [
        (0, 0),
        (-d*1.5, -d),
        (d*1.5, -d)
    ]
