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
