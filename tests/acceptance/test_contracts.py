from copy import deepcopy
from pathlib import Path
import math
import yaml
import pytest
from engineering_acceptance.contracts import ContractError, validate

CONFIG = yaml.safe_load((Path(__file__).parents[2] / 'engineering_acceptance/aircraft_system.yaml').read_text())

def test_current_contract_loads():
    validate(CONFIG)

@pytest.mark.parametrize('mutate', [
    lambda c: c.update(extra={'made_up': True}),
    lambda c: c['propulsion'].update(ducts='two'),
    lambda c: c['propulsion'].update(nominal_stages_per_duct=-3),
    lambda c: c.update(actuators=None),
    lambda c: c['source_of_truth']['drone_arm'].update(revision='unknown'),
    lambda c: c['source_of_truth']['drone_arm'].update(revision='a'*40),
    lambda c: c['cost_policy'].update(remote_compute_budget_usd=float('inf')),
    lambda c: c['battery'].update(nominal_voltage_V=float('nan')),
])
def test_rejects_invalid_or_ambiguous_config(mutate):
    bad=deepcopy(CONFIG); mutate(bad)
    with pytest.raises(ContractError): validate(bad)

def test_null_requires_explicit_unknown_provenance():
    bad=deepcopy(CONFIG); bad['battery']['capacity_Ah']={'value':None,'status':'KNOWN','reason':None}
    with pytest.raises(ContractError): validate(bad)

def test_unknown_override_revision_rejected():
    bad=deepcopy(CONFIG); bad['overrides']=[{'path':'battery.capacity_Ah','value':2,'unit':'Ah','source':{'reference':'fixture','revision':'a'*40}}]
    with pytest.raises(ContractError): validate(bad)

def test_actual_result_row_contract_and_missing_raw_receipt():
    from engineering_acceptance.contracts import validate_result
    row={
      'id':'TEST-FIXTURE','status':'FAIL','run_id':'a'*64,'timestamp_utc':'2026-01-01T00:00:00Z',
      'drone_arm_sha':'b'*40,'drone_mech_sha':'c'*40,'test_definition_sha256':'d'*64,
      'exact_command':'python -m engineering_acceptance run --profile smoke','command':'fixture command',
      'runner':'fixture','check_exit_code':1,'source':{'repo':'drone-arm','issue':21,'url':'https://example.invalid/issue/21','revision':'e'*40},
      'source_revision':'f'*40,'artifact_paths':[],'artifact_sha256':None,'artifacts':[],
      'evidence_required':False,'profile':'smoke','system':'twin-electric-edf','variant':'all','severity':'P0',
      'applicability':'all','blocking':True,'unit':None,'required':{'condition':'fixture'},'measured':None,'errors':['deliberate negative fixture'],
      'config_path':'engineering_acceptance/aircraft_system.yaml','config_sha256':'1'*64,'tool_version':'fixture','python_version':'3.12','tool_versions':{}}
    validate_result(row)
    bad=deepcopy(row); bad['unit']='furlongs'
    with pytest.raises(ContractError): validate_result(bad)
    bad=deepcopy(row); bad['status']='PASS'; bad['evidence_required']=True; bad['artifacts']=[]; bad['artifact_sha256']=None
    with pytest.raises(ContractError): validate_result(bad)

def test_actual_run_manifest_contract():
    from engineering_acceptance.contracts import validate_manifest
    manifest={'run_id':'a'*64,'timestamp_utc':'2026-01-01T00:00:00Z','profile':'smoke',
      'drone_arm_sha':'b'*40,'drone_mech_sha':'c'*40,'test_definition_sha256':'d'*64,
      'config_path':'engineering_acceptance/aircraft_system.yaml','config_sha256':'1'*64,
      'tool_version':'fixture','python_version':'3.12','results':[{'id':'TEST-FIXTURE','status':'FAIL','artifact_sha256':None,'check_exit_code':1}]}
    validate_manifest(manifest)

def test_current_budget_cannot_be_negative_or_assumed_shared_trunk():
    bad=deepcopy(CONFIG)
    bad['limits']['per_motor_battery_current_A']={'value':-1,'status':'KNOWN','reason':None}
    with pytest.raises(ContractError): validate(bad)
    bad=deepcopy(CONFIG)
    bad['limits']['per_motor_battery_current_A']={'value':60,'status':'KNOWN','reason':None}
    bad['limits']['bus_current_A']={'value':360,'status':'KNOWN','reason':None}
    with pytest.raises(ContractError): validate(bad)

def test_current_source_units_must_match_typed_field():
    bad=deepcopy(CONFIG)
    bad['limits']['per_motor_battery_current_A']={'value':60,'unit':'V','status':'KNOWN','reason':None}
    with pytest.raises(ContractError): validate(bad)

def test_component_unit_enum_rejects_unrecognized_units():
    import json
    from jsonschema import Draft202012Validator
    schema=json.loads((Path(__file__).parents[2] / 'engineering_acceptance/schema/component.schema.json').read_text())
    component={'id':'motor','kind':'motor','source':{'repo':'drone-arm','revision':'a'*40,'path':'hardware/motor'},'properties':{'current':{'value':1,'unit':'bananas','provenance':'measured','evidence_id':'EV-1'}}}
    assert list(Draft202012Validator(schema).iter_errors(component))

def test_unknown_revision_in_nested_source_artifact_rejected():
    bad=deepcopy(CONFIG)
    bad['source_artifacts']['geometry_generator']['revision']='a'*40
    with pytest.raises(ContractError): validate(bad)
