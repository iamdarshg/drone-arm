from copy import deepcopy
from pathlib import Path
import yaml

from engineering_acceptance.runners.local import execute
from engineering_acceptance.units import convert
from engineering_acceptance.contracts import ContractError, validate
import engineering_acceptance.core as core
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[2]
CONFIG=yaml.safe_load((ROOT/'engineering_acceptance/aircraft_system.yaml').read_text())

def _check(id, runner):
    return {"id":id,"runner":runner,"evidence_required":True,"input_paths":[],"expected":None,"unit":None}

def test_six_outputs_for_seven_effectors_remains_a_real_failure():
    result=execute(_check("BASE-013","actuator_allocation"),config=deepcopy(CONFIG))
    assert result["status"]=="FAIL"
    assert result["measured"]["unallocated"]==1

def test_missing_firmware_sources_fail_source_and_neutral_gates():
    source=execute(_check("BASE-016","firmware_source_presence"))
    startup=execute(_check("BASE-014","actuator_safe_start"))
    assert source["status"]=="FAIL" and len(source["measured"]["missing"])==2
    assert startup["status"]=="FAIL" and not startup["measured"]["neutral_on_reset_proven"]

def test_payload_contract_with_active_radar_is_not_passed():
    assert execute(_check("BASE-022","payload_claim_audit"),config=deepcopy(CONFIG))["status"]=="BLOCKED"
    config=deepcopy(CONFIG); config["payload"]["radar_active"]=True
    assert execute(_check("BASE-022","payload_claim_audit"),config=config)["status"]=="BLOCKED"
    with __import__('pytest').raises(ContractError): validate(config)

def test_current_unit_conversion_rejects_nonfinite_and_dimension_mismatch():
    import pytest
    with pytest.raises(ValueError): convert(float('nan'),'A','m')
    with pytest.raises(ValueError): convert(1,'Wh','W')
    with pytest.raises(ValueError): convert(True,'m','mm')

def test_configuration_unknown_numbers_cannot_be_used_as_known_limits():
    config=deepcopy(CONFIG); config['limits']['bus_current_A']={'value':float('inf'),'status':'KNOWN','reason':None}
    with __import__('pytest').raises(ContractError): validate(config)

def test_provenance_check_compares_pinned_baseline_to_actual_heads():
    result=execute(_check('BASE-001','provenance'),config=deepcopy(CONFIG))
    assert result['status']=='PASS'
    assert result['measured']['current_head']['drone_arm']
    assert result['measured']['baseline_is_ancestor']=={'drone_arm':True,'drone_mech':True}

def test_schema_runner_uses_the_explicit_candidate_config_path(tmp_path, monkeypatch):
    selected=tmp_path/'aircraft_system_candidate.yaml'
    selected.write_text('candidate config')
    seen=[]
    monkeypatch.setattr(core,'validate_inputs',lambda path=None: seen.append(path) or [])
    result=execute(_check('BASE-002','schema_validation'),config=deepcopy(CONFIG),config_path=selected)
    assert result['status']=='PASS'
    assert seen==[selected]

@pytest.mark.parametrize(('artifact_key','relative_path'), [('geometry','data/input.txt'),('main_board','hardware/main/main_rev_b.kicad_pcb')])
def test_dirty_pinned_engineering_input_fails_source_integrity(tmp_path, monkeypatch, artifact_key, relative_path):
    arm=tmp_path/'drone-arm'; mech=tmp_path/'drone-mech'
    for repo in (arm,mech):
        repo.mkdir(); subprocess.run(['git','init','-q',str(repo)],check=True)
        subprocess.run(['git','-C',str(repo),'config','user.email','acceptance@example.invalid'],check=True)
        subprocess.run(['git','-C',str(repo),'config','user.name','Acceptance Fixture'],check=True)
        (repo/'README').write_text('source fixture\n')
        subprocess.run(['git','-C',str(repo),'add','README'],check=True)
        subprocess.run(['git','-C',str(repo),'commit','-qm','initialize fixture'],check=True)
    source=arm/relative_path; source.parent.mkdir(parents=True,exist_ok=True); source.write_text('pinned engineering input\n')
    subprocess.run(['git','-C',str(arm),'add',relative_path],check=True)
    subprocess.run(['git','-C',str(arm),'commit','-qm','fixture source'],check=True)
    revision=subprocess.check_output(['git','-C',str(arm),'rev-parse','HEAD'],text=True).strip()
    mech_revision=subprocess.check_output(['git','-C',str(mech),'rev-parse','HEAD'],text=True).strip()
    monkeypatch.setattr(core,'ROOT',arm)
    config={'source_of_truth':{'drone_arm':{'revision':revision},'drone_mech':{'revision':mech_revision}},
            'source_artifacts':{artifact_key:{'repo':'drone-arm','revision':revision,'path':relative_path}}}
    assert core.verify_source_artifacts(config)['status']=='PASS'
    source.write_text('mutated geometry\n')
    result=core.verify_source_artifacts(config)
    assert result['status']=='FAIL'
    assert any('dirty/altered source' in message for message in result['errors'])

def test_repository_head_drift_invalidates_the_run():
    start={'heads':{'drone_arm':'a'*40,'drone_mech':'b'*40},'config_sha256':'1'*64,'test_definition_sha256':'2'*64,'source_artifacts_sha256':'3'*64}
    end={'heads':{'drone_arm':'c'*40,'drone_mech':'b'*40},'config_sha256':'1'*64,'test_definition_sha256':'2'*64,'source_artifacts_sha256':'3'*64}
    errors=core.run_input_drift(start,end)
    assert len(errors)==1 and 'drone_arm HEAD changed' in errors[0]

