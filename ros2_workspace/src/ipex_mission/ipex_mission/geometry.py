import math

Point = tuple[float, float]         # (x, y)
Pose = tuple[float, float, float]   # (x, y, yaw)
Circle = tuple[float, float, float] # (x, y, radius)


def wrap(a):
    """Wrap an angle to -pi, pi."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(x, y, z, w):
    """Extract yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def dist(a, b):
    """Distance between points a and b using pythagora."""
    return math.hypot(a[0] - b[0], a[1] - b[1])

# https://paulbourke.net/geometry/pointlineplane/
def point_segment_distance(p, a, b):
    """Distance between point p and segment ab."""
    ax, ay = a
    bx, by = b
    px, py = p

    dx = bx - ax
    dy = by - ay
    
    dist_squared = dx * dx + dy * dy
    if dist_squared < 1e-12:
        return math.hypot(px - ax, py - ay)
    
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / dist_squared))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
