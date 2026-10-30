import copy
import json
from pathlib import Path
import pytest
from engineering_acceptance.interface_contract import build_contract, validate_contract

ROOT=Path(__file__).parents[2]

def test_checked_in_system_interface_matches_single_source_config():
    contract=build_contract()
    validate_contract(contract)
    static=json.loads((ROOT/'aircraft_interfaces/system_interface.json').read_text())
    assert static == contract
    assert {a['id'] for a in contract['source_artifacts']} == set(contract['source_artifacts'][i]['id'] for i in range(len(contract['source_artifacts'])))
    assert contract['sources']['drone-arm']['revision'] == '744994a096ca73e872b396a9b0636c58e7c1afe8'
    assert contract['sources']['drone-mech']['revision'] == '5246c90657071772e220b118967fa5f3e7c9ff7c'
    assert all(a['sha256'] for a in contract['source_artifacts'] if a['status']=='KNOWN')

def test_interface_contract_preserves_known_architecture_and_open_gates():
    contract=build_contract()
    assert contract['propulsion']['motor_count'] == 6
    assert contract['propulsion']['independent_power_stages'] == 6
    assert contract['power_and_control']['esc_control']['path'] == 'CAN-FD'
    assert contract['actuators']['modeled_effector_count'] == 7
    assert contract['actuators']['available_pwm_output_count'] == 6
    assert contract['interfaces'][2]['status'] == 'BLOCKED'
    for key in ('order','command_limits','feedback_interfaces','fault_behavior','firmware_capabilities'):
        assert contract['actuators'][key]['value'] is None
        assert contract['actuators'][key]['status'] == 'BLOCKED'
        assert contract['actuators'][key]['reason']

def test_seven_to_six_mapping_cannot_be_promoted_to_solved():
    contract=copy.deepcopy(build_contract())
    contract['interfaces'][2]['status']='KNOWN'
    with pytest.raises(ValueError): validate_contract(contract)
