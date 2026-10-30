import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from engineering_acceptance.adapters import evidence


def test_forged_solver_completed_receipt_is_not_pass(tmp_path):
    out=tmp_path/'attempts/a/worker_output'; out.mkdir(parents=True)
    (out/'phase15_fsi_worker_receipt.json').write_text(json.dumps({'status':'coupled_solvers_completed'}))
    r=evidence.fsi(tmp_path)
    assert r['status'] in ('FAIL','BLOCKED')
    assert 'unsubstantiated' in ' '.join(r['errors'])


def test_empty_right_name_log_does_not_prove_completion(tmp_path):
    out=tmp_path/'attempts/a/worker_output'; case=out/'wing_t30mm'; case.mkdir(parents=True)
    (out/'wing_t30mm_receipt.json').write_text(json.dumps({'candidate':'wing_t30mm','status':'coupled_solvers_completed','coupling':{'fluidLog':'rhoPimpleFoam.log','solidLog':'ccx_preCICE.log'}}))
    (case/'rhoPimpleFoam.log').touch(); (case/'ccx_preCICE.log').touch()
    (case/'wing.dat').write_text('')
    r=evidence.fsi(tmp_path)
    assert r['status'] in ('FAIL','BLOCKED')
    assert r['status'] != 'PASS'


def test_stale_receipt_raw_sha_fails(tmp_path):
    out=tmp_path/'attempts/a/worker_output'; out.mkdir(parents=True)
    raw=out/'solver.log'; raw.write_text('actual solver output')
    (out/'attempt_receipt.json').write_text(json.dumps({'status':'timeout','rawFiles':[{'path':'solver.log','sha256':'0'*64}]}))
    r=evidence.fsi(tmp_path)
    assert r['status']=='FAIL'
    assert 'SHA mismatch' in ' '.join(r['errors'])


def test_claim_parser_finds_positive_claim_without_physical_evidence(tmp_path):
    (tmp_path/'status.md').write_text('This aircraft is flight-ready.')
    r=evidence.claims(tmp_path)
    assert r['status']=='FAIL'
    assert r['measured']['unsupported_positive_claims']


def test_missing_motor_temperature_evidence_is_blocked(tmp_path):
    ea=tmp_path/'engineering_acceptance'; ea.mkdir()
    (ea/'aircraft_system.yaml').write_text('motors:\n  temperature_measurements: {value: null, status: BLOCKED}\n')
    r=evidence.thermal(tmp_path)
    assert r['status']=='BLOCKED'
    assert not r['measured']['measurement_complete']


def test_current_remote_launch_spend_cannot_be_reported_as_zero(tmp_path):
    arm=tmp_path/'arm'; mech=tmp_path/'mech'; ea=arm/'engineering_acceptance'; ea.mkdir(parents=True)
    (ea/'aircraft_system.yaml').write_text('cost_policy: {remote_compute_budget_usd: 0, allow_remote_compute: false}\n')
    attempt=mech/'highspeed_edf/phase15/fsi/results/attempts/one'; attempt.mkdir(parents=True)
    (attempt/'gcp_attempt.log').write_text('2026-10-30T09:14:30+00:00 VM_CREATE_REQUEST gcloud compute instances create hsedf-fsi-test')
    (mech/'highspeed_edf/phase15/fsi/results/phase15_fsi_cost_history.json').write_text(json.dumps([{'jobId':'phase15-wing-fsi-20261030T091430Z','plannedMaxCostUsd':0.25,'allInConservativeEstimateUsd':0.25,'resourceReleased':False}]))
    r=evidence.cost(arm,mech,today='2026-10-30')
    assert r['status']=='FAIL'
    assert r['measured']['current_paid_launches']


def test_historical_governed_attempts_do_not_violate_new_zero_launch_policy(tmp_path):
    arm=tmp_path/'arm'; mech=tmp_path/'mech'; ea=arm/'engineering_acceptance'; ea.mkdir(parents=True)
    (ea/'aircraft_system.yaml').write_text('cost_policy: {remote_compute_budget_usd: 0, allow_remote_compute: false}\n')
    results=mech/'highspeed_edf/phase15/fsi/results'; attempt=results/'attempts/20260930T091336Z'; attempt.mkdir(parents=True)
    (attempt/'gcp_attempt.log').write_text('2026-09-30T09:14:00+00:00 PREFLIGHT_PASS workbenchCostCeilingUsd=0.05\n2026-09-30T09:14:30.964311+00:00 VM_CREATE_REQUEST gcloud compute instances create hsedf-fsi-test\n2026-09-30T09:20:00+00:00 VM_DELETE rc=0\n')
    (results/'phase15_fsi_cost_history.json').write_text(json.dumps([{'jobId':'phase15-wing-fsi-20260930T091337Z','plannedMaxCostUsd':0.05,'allInConservativeEstimateUsd':0.036577,'resourceReleased':True}]))
    r=evidence.cost(arm,mech,today='2026-10-30')
    assert r['status']=='PASS'
    assert r['measured']['historical_governed_launches']==1
    assert not r['measured']['current_paid_launches']


def test_native_failed_run_is_fail_and_absent_run_is_blocked(tmp_path):
    root=tmp_path/'fsi'; receipt=root/'attempts/a/worker_output/wing_t30mm_receipt.json'; receipt.parent.mkdir(parents=True)
    receipt.write_text(json.dumps({'status':'coupled_solver_failed_or_not_converged'}))
    assert evidence.fsi(root)['status']=='FAIL'
    empty=tmp_path/'empty_fsi'; empty.mkdir()
    assert evidence.fsi(empty)['status']=='BLOCKED'


def test_claim_audit_returns_source_paths_and_hash(tmp_path):
    source=tmp_path/'status.yaml'; source.write_text('status: flight-ready\n')
    ignored=tmp_path/'.venv/test/status.md'; ignored.parent.mkdir(parents=True); ignored.write_text('flight-ready: true\n')
    r=evidence.claims(tmp_path)
    assert str(source) in r['artifact_paths']
    assert r['artifact_sha256']
    assert str(ignored) not in r['artifact_paths']


def test_completed_receipt_needs_matching_candidate_logs_and_input_hash(tmp_path):
    out=tmp_path/'attempts/a/worker_output'; case=out/'wing_t30mm'; case.mkdir(parents=True)
    (case/'rhoPimpleFoam.log').write_text('End\n')
    (case/'ccx_preCICE.log').write_text('solver completed\n')
    receipt={'candidate':'wing_t30mm','status':'coupled_solvers_completed','tipDisplacement':0.001,'forceHistoryLastLine':'0 1 2',
             'inputFiles':[{'path':'input.dat','sha256':'0'*64}],
             'coupling':{'fluidLog':'rhoPimpleFoam.log','solidLog':'ccx_preCICE.log','fluidReturnCode':0,'solidReturnCode':0,'timedOut':False,'fluidReachedEnd':True,'preCICECompletionLogMarkers':1}}
    (out/'wing_t30mm_receipt.json').write_text(json.dumps(receipt))
    r=evidence.fsi(tmp_path)
    assert r['status']=='FAIL'
    assert 'SHA mismatch' in ' '.join(r['errors']) or 'lacks both nonempty' in ' '.join(r['errors'])


def test_completed_label_cannot_borrow_logs_from_another_candidate(tmp_path):
    out=tmp_path/'attempts/a/worker_output'; wrong=out/'wing_t36mm'; right=out/'wing_t30mm'
    wrong.mkdir(parents=True); right.mkdir()
    (wrong/'rhoPimpleFoam.log').write_text('End\n'); (wrong/'ccx_preCICE.log').write_text('done')
    (out/'wing_t30mm_receipt.json').write_text(json.dumps({'candidate':'wing_t30mm','status':'completed','coupling':{'fluidLog':'rhoPimpleFoam.log','solidLog':'ccx_preCICE.log'}}))
    assert evidence.fsi(tmp_path)['status']!='PASS'


def test_negative_release_label_lists_and_pipeline_status_are_not_claims():
    root=Path(evidence.MECH)/'highspeed_edf'
    r=evidence.claims(root)
    false_sources={'phase14/results/phase14_closure_summary.json',
                   'phase14/configs/phase14_coupled_design.json',
                   'phase15/reports/phase15_gcp_rans_status.md'}
    assert not any(item['source'] in false_sources for item in r['measured']['unsupported_positive_claims'])
    assert r['status']=='BLOCKED'


def test_structured_do_not_claim_and_limitations_are_negative_context(tmp_path):
    source=tmp_path/'candidate.json'
    source.write_text(json.dumps({'limitations':['flight-ready'],'evidencePolicy':{'doNotClaim':['flight-ready']},'status':'screening-only'}))
    r=evidence.claims(tmp_path)
    assert r['status']=='BLOCKED'
    assert not r['measured']['unsupported_positive_claims']


def test_negative_aero_status_strings_are_not_positive_claims(tmp_path):
    values=[
        'pending pressure recovery; packaging start only',
        'worker-finished-no-valid-pressure-recovery',
        'no-converged-pressure-recovery',
        'solver-batch-blocked-no-valid-pressure-recovery',
        'execution-complete-no-valid-pressure-recovery',
    ]
    for index,value in enumerate(values):
        (tmp_path/f'status-{index}.json').write_text(json.dumps({'status':value}))
    r=evidence.claims(tmp_path,kind='aero')
    assert r['status']=='BLOCKED'
    assert r['measured']['unsupported_positive_claims']==[]


def test_actual_aero_failure_statuses_are_not_claimed_as_recovery(tmp_path):
    values=[
        'highspeed_edf/deliverables/phase13/configs/phase13_fixed_powertrain_design.json',
        'highspeed_edf/phase11/results/phase11_execution_receipt.json',
        'highspeed_edf/phase11/results/phase11_solver_receipt.json',
        'highspeed_edf/phase10/results/phase10_execution_receipt.json',
        'highspeed_edf/phase10/configs/phase10_openfoam_study.json',
    ]
    r=evidence.claims(Path(evidence.MECH)/'highspeed_edf',kind='aero')
    actual=[item for item in r['measured']['unsupported_positive_claims'] if item['source'] in values]
    assert not actual
    assert r['status']=='BLOCKED'


def test_explicit_positive_pressure_recovery_claim_is_still_rejected(tmp_path):
    (tmp_path/'result.json').write_text(json.dumps({'claim':'measured pressure recovery = 0.95'}))
    r=evidence.claims(tmp_path,kind='aero')
    assert r['status']=='FAIL'
    assert len(r['measured']['unsupported_positive_claims'])==1
