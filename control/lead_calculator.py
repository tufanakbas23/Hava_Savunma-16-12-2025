# control/lead_calculator.py
def compute_lead_point(pos, vel, distance_m, projectile_speed_mps):
    """
    pos: (x, y) piksel
    vel: (vx, vy) piksel/saniye
    distance_m: hedef uzaklığı (m)
    projectile_speed_mps: mermi/hava aracı hızı (m/s)

    return: (lead_x, lead_y) piksel
    """
    x, y = pos
    vx, vy = vel

    if projectile_speed_mps <= 0:  # hesapla sisteme gore
        return x, y

    time_to_target = distance_m / projectile_speed_mps
    lead_x = x + vx * time_to_target
    lead_y = y + vy * time_to_target
    return lead_x, lead_y
