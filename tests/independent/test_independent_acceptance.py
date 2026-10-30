"""Independent adversarial suite. It exercises public APIs and native adapters.

The suite lives outside engineering_acceptance so implementation agents cannot
change the oracle and implementation in the same patch. Unknown capabilities may
be BLOCKED; a missing-evidence mutation must never produce PASS.
"""
from __future__ import annotations
import copy, json, math, sys
from pathlib import Path
import pytest, yaml

ARM=Path(__file__).resolve().parents[2]; MECH=ARM.parent/'drone-mech'
sys.path.insert(0,str(ARM)); sys.path.insert(0,str(MECH/'highspeed_edf'))
from engineering_acceptance import contracts, core
from engineering_acceptance.adapters import evidence
from engineering_acceptance.runners import local
from verification import native_adapters as native


def test_catalog_has_all_25_fixed_mandatory_ids_and_profiles_cover_them():
    checks=core.discover()['checks']; ids={x['id'] for x in checks}
    assert len(checks)==25 and ids=={f'BASE-{i:03d}' for i in range(1,26)}
    profiles=core.discover()['profiles']['profiles']
    assert set(profiles['all']['checks'])==ids
    assert ids <= set().union(*(set(p.get('checks',[])) for p in profiles.values()))
    assert len(core.load_yaml(core.CATALOG/'mandatory_checks.yaml')['negative_fixtures']['required_by_baseline']) >= 9


def test_malformed_system_contract_is_rejected():
    cfg=contracts.load_config(); cfg['propulsion']['ducts']='two'; cfg['motors']=[]; cfg['actuators']=None
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_dimensionally_wrong_battery_capacity_unit_is_rejected():
    cfg=contracts.load_config(); cfg['battery']['capacity_Ah']['unit']='W'
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_nonfinite_and_dimensionally_invalid_unit_conversions_rejected():
    from engineering_acceptance.units import convert
    for value in (math.nan,math.inf,-math.inf):
        with pytest.raises(ValueError): convert(value,'m','mm')
    with pytest.raises(ValueError): convert(1,'m','A')


def _box_motor(path: Path, center_z: float, length: float=100):
    import cadquery as cq
    cq.Workplane('XY').box(42,42,length).translate((0,0,center_z)).val().exportStep(str(path))
    return native.import_step(path)


@pytest.mark.parametrize('pitch',[0,36])
def test_motor_pitch_mutations_fail_native_interference(tmp_path,pitch):
    a=_box_motor(tmp_path/'motor-a.step',0,100)
    b=_box_motor(tmp_path/'motor-b.step',pitch,100)
    result=native.motor_bbox_collision([a,b],min_required_clearance_mm=0)
    assert result['status']=='FAIL' and result['collisions']


def test_stale_pcb_source_hash_mutation_fails():
    result=native.validate_pcb_sha('f'*64,'a'*64)
    assert result['status']=='FAIL'


def test_extra_independent_servo_with_six_outputs_rejected():
    cfg=contracts.load_config(); cfg['actuators']['modeled_independent_effectors']=8
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_payload_mass_omission_cannot_pass():
    cfg=contracts.load_config(); cfg['payload']['modeled_payload_mass_kg']=0
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_forged_solver_completion_without_raw_logs_not_pass(tmp_path):
    out=tmp_path/'attempts/a/worker_output';out.mkdir(parents=True)
    (out/'phase15_fsi_worker_receipt.json').write_text(json.dumps({'status':'coupled_solvers_completed'}))
    assert evidence.fsi(tmp_path)['status'] in ('FAIL','BLOCKED')


def test_empty_solver_logs_do_not_prove_completion(tmp_path):
    out=tmp_path/'attempts/a/worker_output';case=out/'wing_t30mm';case.mkdir(parents=True)
    (out/'wing_t30mm_receipt.json').write_text(json.dumps({'status':'coupled_solvers_completed','coupling':{'fluidLog':'wing_t30mm/rhoPimpleFoam.log','solidLog':'wing_t30mm/ccx_preCICE.log'}}))
    (case/'rhoPimpleFoam.log').touch();(case/'ccx_preCICE.log').touch();(case/'wing.dat').touch()
    assert evidence.fsi(tmp_path)['status'] in ('FAIL','BLOCKED')


def test_negative_unknown_material_property_rejected():
    cfg=contracts.load_config(); cfg['materials']['core']['value']=-1
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_out_of_contract_servo_command_rejected():
    cfg=contracts.load_config(); cfg['actuators']['servo_command_deg']=181
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_missing_firmware_fails_safe_start_gate():
    result=local.execute({'id':'BASE-014','runner':'actuator_safe_start','evidence_required':True})
    assert result['status'] in ('FAIL','BLOCKED')


def test_positive_claim_without_physical_evidence_fails(tmp_path):
    (tmp_path/'release_status.md').write_text('This aircraft is flight-ready.\n')
    assert evidence.claims(tmp_path)['status']=='FAIL'


def test_current_paid_launch_under_zero_policy_fails(tmp_path):
    arm=tmp_path/'arm';mech=tmp_path/'mech';ea=arm/'engineering_acceptance';ea.mkdir(parents=True)
    (ea/'aircraft_system.yaml').write_text('cost_policy: {remote_compute_budget_usd: 0, allow_remote_compute: false}\n')
    attempt=mech/'highspeed_edf/phase15/fsi/results/attempts/attempt';attempt.mkdir(parents=True)
    (attempt/'gcp_attempt.log').write_text('2026-10-30T09:14:30+00:00 VM_CREATE_REQUEST gcloud compute instances create hsedf-test\n')
    assert evidence.cost(arm,mech,today='2026-10-30')['status']=='FAIL'


def test_verified_raw_artifact_tampering_is_detected(tmp_path,monkeypatch):
    root=tmp_path/'arm';root.mkdir(); parent=root.parent
    reports=root/'engineering_acceptance/reports';reports.mkdir(parents=True)
    raw=parent/'native.log';raw.write_text('residual=0.001\n')
    manifest_rel='engineering_acceptance/reports/runs/run.json';manifest=root/manifest_rel;manifest.parent.mkdir(parents=True)
    manifest.write_text('{"run_id":"r"}\n')
    sha=lambda p: core.sha256(p)
    art_sha=sha(raw)
    row={'id':'X','status':'PASS','run_id':'r','drone_arm_sha':'a'*40,'drone_mech_sha':'b'*40,
         'test_definition_sha256':'c'*64,'exact_command':'python -m test','timestamp_utc':'2026-10-30T00:00:00Z',
         'runner':'fixture','check_exit_code':0,'evidence_required':True,'run_manifest_path':manifest_rel,
         'run_manifest_sha256':sha(manifest),'artifacts':[{'path':'native.log','sha256':art_sha}],
         'artifact_sha256':__import__('hashlib').sha256(art_sha.encode()).hexdigest(),'previous_record_sha256':None}
    row['record_sha256']=__import__('hashlib').sha256(json.dumps(row,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    ledger=reports/'evidence.jsonl';ledger.write_text(json.dumps(row,sort_keys=True)+'\n')
    monkeypatch.setattr(core,'ROOT',root);monkeypatch.setattr(core,'LEDGER',ledger)
    assert core.verify_ledger()==[]
    raw.write_text('tampered\n')
    assert any('artifact SHA mismatch' in e for e in core.verify_ledger())


def test_cli_returns_nonzero_for_required_blocked_gate(monkeypatch):
    from engineering_acceptance import cli
    monkeypatch.setattr(cli,'run',lambda profile,config=None:{'run_id':'x','profile':profile,'counts':{'FAIL':0,'BLOCKED':1,'NOT_RUN':0,'PASS':0,'NOT_APPLICABLE':0},'results':[{'id':'BASE-021','status':'BLOCKED','errors':['no raw evidence']}]})
    assert cli.main(['run','--profile','fsi'])==1


def test_current_limit_with_wrong_dimension_unit_is_rejected():
    cfg=contracts.load_config(); cfg['limits']['bus_current_A']['unit']='Wh'
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_invalid_sensor_timestamp_and_power_state_mutation_is_rejected():
    cfg=contracts.load_config(); cfg['sensor_telemetry']={'timestamp_s':-1,'power_on_state':'armed'}
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_fsi_timeout_receipt_cannot_be_promoted_by_summary(tmp_path):
    out=tmp_path/'attempts/a/worker_output';out.mkdir(parents=True)
    (out/'attempt_receipt.json').write_text(json.dumps({'status':'timeout','completed':True,'summary':'converged'}))
    result=evidence.fsi(tmp_path)
    assert result['status']=='FAIL'


def test_a_positive_servo_command_outside_declared_contract_is_rejected():
    cfg=contracts.load_config(); cfg['actuators']['servo_command_deg']=181
    with pytest.raises(contracts.ContractError): contracts.validate(cfg)


def test_pinned_git_tree_byte_mutation_fails(tmp_path,monkeypatch):
    import subprocess
    def git(repo,*args):
        return subprocess.check_output(['git','-C',str(repo),*args],text=True).strip()
    def make_repo(name,rel,contents):
        source=tmp_path/(name+'-origin');source.mkdir()
        subprocess.run(['git','init','-q',str(source)],check=True)
        git(source,'config','user.email','independent@example.invalid');git(source,'config','user.name','Independent verifier')
        path=source/rel;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(contents)
        git(source,'add',rel);git(source,'commit','-q','-m','pinned input')
        worktree=tmp_path/name
        subprocess.run(['git','-C',str(source),'worktree','add','-q','--detach',str(worktree),'HEAD'],check=True)
        return worktree,git(worktree,'rev-parse','HEAD')
    arm,asha=make_repo('drone-arm','hardware/board.kicad_pcb','pinned board\n')
    mech,msha=make_repo('drone-mech','cad/generator.py','LENGTH_MM = 72\n')
    monkeypatch.setattr(core,'ROOT',arm)
    config={'source_of_truth':{'drone_arm':{'revision':asha},'drone_mech':{'revision':msha}},
            'source_artifacts':{'board':{'repo':'drone-arm','revision':asha,'path':'hardware/board.kicad_pcb'},
                                'generator':{'repo':'drone-mech','revision':msha,'path':'cad/generator.py'}}}
    assert core.verify_source_artifacts(config)['status']=='PASS'
    (arm/'hardware/board.kicad_pcb').write_text('mutated board\n')
    result=core.verify_source_artifacts(config)
    assert result['status']=='FAIL' and any('dirty/altered source' in x for x in result['errors'])
