import json
from engineering_acceptance import cli

def test_report_scorecard_counts_release_critical_and_excludes_unrun(tmp_path, monkeypatch):
    checks=[{"id":"EA-1","severity":"P0","blocking":True},{"id":"EA-2","severity":"P1","blocking":False}]
    rows=[{"id":"EA-1","run_id":"a"*64,"status":"PASS","drone_arm_sha":"b"*40,"drone_mech_sha":"c"*40,"profile":"smoke","severity":"P0","blocking":True},
          {"id":"EA-2","run_id":"a"*64,"status":"NOT_RUN","drone_arm_sha":"b"*40,"drone_mech_sha":"c"*40,"profile":"smoke","severity":"P1","blocking":False}]
    monkeypatch.setattr(cli,"latest_run",lambda: rows)
    monkeypatch.setattr(cli,"discover",lambda:{"requirements":{"requirements":[{"id":"R1"}]},"checks":checks})
    output=tmp_path/"report.json"
    assert cli.main(["report","--json",str(output)])==0
    report=json.loads(output.read_text())
    assert report["checks_discovered"]==2
    assert report["checks_attempted"]==1
    assert report["release_critical_pass"]==1 and report["release_critical_total"]==1
    assert report["counts"]["NOT_RUN"]==1
