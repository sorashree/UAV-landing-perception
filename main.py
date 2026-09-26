def hud(frame, tracking, confidence, alignment, distance, lateral, vertical, angle, source):
    h, w = frame.shape[:2]

    # Dark translucent panel.
    panel = frame.copy()
    cv2.rectangle(panel, (18, 18), (330, 215), (8, 15, 22), -1)
    cv2.addWeighted(panel, 0.78, frame, 0.22, 0, frame)

    status = "TRACKING" if tracking else "SEARCHING"
    status_color = (70, 235, 150) if tracking else (60, 170, 245)

    lines = [
        ("PERCEPTION / LANDING", (235, 240, 245), 0.60),
        (f"STATUS       {status}", status_color, 0.52),
        (f"CONFIDENCE   {confidence*100:5.1f} %", (220, 230, 235), 0.48),
        (f"ALIGNMENT    {alignment:>7.1f} %", (220, 230, 235), 0.48),
        (f"DISTANCE     {distance:>7.1f} m", (220, 230, 235), 0.48),
        (f"LATERAL      {lateral:+7.2f} m", (220, 230, 235), 0.48),
        (f"VERTICAL     {vertical:+7.2f} m", (220, 230, 235), 0.48),
        (f"APPROACH     {angle:>7.1f} deg", (220, 230, 235), 0.48),
    ]
