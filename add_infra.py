import re

def add_infrastructure(pcb_path):
    with open(pcb_path, 'r') as f:
        content = f.read()

    # VBATT Plane (In1.Cu)
    # GND Plane (In2.Cu)
    # Shared power backbone

    zones = """
	(zone
		(net 156) (layer "In1.Cu") (tstamp "vbat-plane")
		(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
		(polygon (pts (xy 0 0) (xy 160 0) (xy 160 100) (xy 0 100)))
	)
	(zone
		(net 8) (layer "In2.Cu") (tstamp "gnd-plane")
		(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
		(polygon (pts (xy 0 0) (xy 160 0) (xy 160 100) (xy 0 100)))
	)
"""
    # Inserting zones before final )
    insert_pos = content.rfind(')')
    new_content = content[:insert_pos] + zones + content[insert_pos:]

    with open(pcb_path, 'w') as f:
        f.write(new_content)

add_infrastructure('hardware/esc/esc/esc.kicad_pcb')
