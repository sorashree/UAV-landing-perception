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
    y = 42
    for i, (txt, color, scale) in enumerate(lines):
        cv2.putText(
            frame, txt, (31, y), cv2.FONT_HERSHEY_SIMPLEX,
            scale, color, 1, cv2.LINE_AA
        )
        y += 23 if i else 27
    cv2.putText(
        frame, source, (20, h-18), cv2.FONT_HERSHEY_SIMPLEX,
        0.43, (165, 180, 190), 1, cv2.LINE_AA
    
    bx, by, bw, bh = w - 250, 24, 210, 12
    cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), (50, 65, 75), 1)
    fill = int(np.clip(alignment, 0, 100) / 100 * (bw - 2))
    cv2.rectangle(frame, (bx+1, by+1), (bx+1+fill, by+bh-1), status_color, -1)
    cv2.putText(
        frame, "ALIGNMENT", (bx, by+32), cv2.FONT_HERSHEY_SIMPLEX,
        0.42, (180, 195, 205), 1, cv2.LINE_AA
    )
