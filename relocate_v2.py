import re
import json

# Grid settings
BOARD_W = 160
BOARD_H = 100
BLOCK_W = 45
BLOCK_H = 45

# Block origins (2 rows, 3 columns)
BLOCK_ORIGINS = [
    (10, 5), (57, 5), (105, 5),
    (10, 52), (57, 52), (105, 52)
]

# Controller area in the center (approx 50x50mm centered)
CTRL_ORIGIN = (80, 50)

def relocate_components(pcb_path):
    with open(pcb_path, 'r') as f:
        content = f.read()

    # Motor mapping (simplified for script)
    # Using the results from previous analysis
    motor_groups = {
        1: {'FETs':['Q1','Q2','Q3','Q4','Q5','Q6'], 'Driver':'IC1', 'Shunts':['R10','R11','R12','R7','R8','R9'], 'Amps':['U2','U3','U8','U10'], 'GateRes':['R1','R2','R3','R4','R5','R6'], 'Caps':['C1','C2','C3','C4','C5','C6','C7']},
        2: {'FETs':['Q7','Q8','Q9','Q10','Q11','Q12'], 'Driver':'IC2', 'Shunts':['R13','R14','R15','R16','R17','R18'], 'Amps':['U4'], 'GateRes':['R22','R23','R24','R25','R26','R27'], 'Caps':['C8','C9','C10','C11','C12','C13','C14']},
        3: {'FETs':['Q13','Q14','Q15','Q16','Q17','Q18'], 'Driver':'IC3', 'Shunts':['R19','R20','R21','R28','R29','R30'], 'Amps':['U5'], 'GateRes':['R34','R35','R36','R37','R38','R39'], 'Caps':['C15','C16','C17','C18','C19','C20','C21']},
        4: {'FETs':['Q19','Q20','Q21','Q22','Q23','Q24'], 'Driver':'IC4', 'Shunts':['R31','R32','R33','R40','R41','R42'], 'Amps':['U6'], 'GateRes':['R46','R47','R48','R49','R50','R51'], 'Caps':['C22','C23','C24','C25','C26','C27','C28']},
        5: {'FETs':['Q25','Q26','Q27','Q28','Q29','Q30'], 'Driver':'IC5', 'Shunts':['R43','R44','R45','R52','R53','R54','R55','R56','R57','R64','R65','R66'], 'Amps':['U7','U11','U12'], 'GateRes':['R58','R59','R60','R61','R62','R63'], 'Caps':['C29','C30','C31','C32','C33','C34','C35']},
        6: {'FETs':['Q31','Q32','Q33','Q34','Q35','Q36'], 'Driver':'IC6', 'Shunts':['R67','R68','R69','R70','R71','R72','R73','R74','R75','R88','R89','R90'], 'Amps':['U9'], 'GateRes':['R79','R80','R81','R82','R83','R84'], 'Caps':['C36','C37','C38','C39','C40','C41','C42']}
    }

    pos_map = {}

    # Layout each motor block
    for i, (m_id, g) in enumerate(motor_groups.items()):
        ox, oy = BLOCK_ORIGINS[i]

        # Driver centered at top of block
        pos_map[g['Driver']] = (ox + 22, oy + 5, 0)

        # MOSFETs in two rows (HS, LS)
        for j, fet in enumerate(g['FETs'][:3]): # HS
            pos_map[fet] = (ox + 7 + j*15, oy + 20, 270)
        for j, fet in enumerate(g['FETs'][3:]): # LS
            pos_map[fet] = (ox + 7 + j*15, oy + 32, 270)

        # Gate Resistors near Driver
        for j, res in enumerate(g['GateRes']):
            pos_map[res] = (ox + 5 + j*6, oy + 12, 90)

        # Shunts at bottom
        for j, shunt in enumerate(g['Shunts']):
            pos_map[shunt] = (ox + 5 + j*3, oy + 42, 90)

        # Amps near shunts
        for j, amp in enumerate(g['Amps']):
            pos_map[amp] = (ox + 35, oy + 35 + j*5, 0)

        # Caps distributed
        for j, cap in enumerate(g['Caps']):
            pos_map[cap] = (ox + 40, oy + 5 + j*4, 0)

    # Controller and Power Management
    pos_map['U1'] = (CTRL_ORIGIN[0], CTRL_ORIGIN[1], 0)
    pos_map['IC7'] = (CTRL_ORIGIN[0] + 15, CTRL_ORIGIN[1] + 15, 0)
    pos_map['IC8'] = (CTRL_ORIGIN[0] - 15, CTRL_ORIGIN[1] - 15, 0)

    # Apply to file
    footprints = content.split('(footprint')
    new_footprints = [footprints[0]]

    for fp in footprints[1:]:
        m_ref = re.search(r'\(property "Reference" "([^"]+)"', fp)
        if m_ref:
            ref = m_ref.group(1)
            if ref in pos_map:
                x, y, rot = pos_map[ref]
                fp = re.sub(r'\(at [^)]+\)', f'(at {x} {y} {rot})', fp, 1)
            else:
                # Move unused to side
                fp = re.sub(r'\(at [^)]+\)', f'(at 155 50)', fp, 1)
        new_footprints.append(fp)

    new_content = '(footprint'.join(new_footprints)

    # Clean segments/vias/zones as we are doing a fresh layout
    new_content = re.sub(r'\(segment [^)]+\)\s+', '', new_content)
    new_content = re.sub(r'\(via [^)]+\)\s+', '', new_content)
    new_content = re.sub(r'\(zone [^)]+\(polygon[^)]+\)[^)]+\)\s+', '', new_content, flags=re.DOTALL)

    with open(pcb_path, 'w') as f:
        f.write(new_content)

relocate_components('hardware/esc/esc/esc.kicad_pcb')
