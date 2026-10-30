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
    assert 'lacks both nonempty participant logs' in ' '.join(r['errors'])


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


def test_real_remote_launch_spend_cannot_be_reported_as_zero(tmp_path):
    arm=tmp_path/'arm'; mech=tmp_path/'mech'; ea=arm/'engineering_acceptance'; ea.mkdir(parents=True)
    (ea/'aircraft_system.yaml').write_text('cost_policy: {remote_compute_budget_usd: 0, allow_remote_compute: false}\n')
    attempt=mech/'highspeed_edf/phase15/fsi/results/attempts/one'; attempt.mkdir(parents=True)
    (attempt/'gcp_attempt.log').write_text('Created instance hsedf-fsi-test')
    r=evidence.cost(arm,mech)
    assert r['status']=='FAIL'
    assert r['measured']['observed_launch_log_count']==1
