import re

BLOCK_ORIGINS = [
    (10, 5), (57, 5), (105, 5),
    (10, 52), (57, 52), (105, 52)
]

def add_vias_and_routes(pcb_path):
    with open(pcb_path, 'r') as f:
        content = f.read()

    elements = []
    # 1. High-current Via Stitching for each block
    for i, (ox, oy) in enumerate(BLOCK_ORIGINS):
        # Stitch HS Drains to VBATT Plane (at y+20 for FETs, pads are rotated 270)
        # Footprint is at x_fet, y_fet. HS FETs are at ox+7+j*15, oy+20
        for j in range(3):
            fx = ox + 7 + j*15
            fy = oy + 20
            # Array of vias on the drain pad area
            for vx in [-2, 0, 2]:
                for vy in [-2, 0, 2]:
                    elements.append(f'\t(via (at {fx + vx} {fy + vy}) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") (net 156))\n')

        # Stitch LS Sources and Shunts to GND Plane
        for j in range(3):
            fx = ox + 7 + j*15
            fy = oy + 32
            for vx in [-2, 0, 2]:
                for vy in [-2, 0, 2]:
                    elements.append(f'\t(via (at {fx + vx} {fy + vy}) (size 0.6) (drill 0.3) (layers "F.Cu" "In2.Cu") (net 8))\n')

        # Stitch Shunts to GND (at oy+42)
        for j in range(9): # Max 9 shunts
            sx = ox + 5 + j*3
            sy = oy + 42
            elements.append(f'\t(via (at {sx} {sy}) (size 0.6) (drill 0.3) (layers "F.Cu" "In2.Cu") (net 8))\n')

    # 2. Gate Signal Routing (Simplified representation)
    # HS Gates are on pad 4. LS Gates on pad 4.
    # In my layout FETs are rotated 270. Pad 4 is at relative (-2.7, 1.9)
    # Correcting for rotation 270: (x,y) -> (y, -x).
    # Relative (-2.7, 1.9) -> (1.9, 2.7)
    for i, (ox, oy) in enumerate(BLOCK_ORIGINS):
        for j in range(3): # HS
            fx = ox + 7 + j*15
            fy = oy + 20
            elements.append(f'\t(segment (start {fx + 1.9} {fy + 2.7}) (end {ox + 22} {oy + 5}) (width 0.2) (layer "F.Cu") (net 100{i}{j}))\n')
        for j in range(3): # LS
            fx = ox + 7 + j*15
            fy = oy + 32
            elements.append(f'\t(segment (start {fx + 1.9} {fy + 2.7}) (end {ox + 22} {oy + 5}) (width 0.2) (layer "F.Cu") (net 101{i}{j}))\n')

    insert_pos = content.rfind(')')
    new_content = content[:insert_pos] + "".join(elements) + content[insert_pos:]

    with open(pcb_path, 'w') as f:
        f.write(new_content)

add_vias_and_routes('hardware/esc/esc/esc.kicad_pcb')
