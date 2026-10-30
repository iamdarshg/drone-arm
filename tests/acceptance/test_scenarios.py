import itertools
import pytest
from engineering_acceptance.scenarios import generate_pairwise

def test_pairwise_matrix_has_stable_unique_ids_and_never_claims_execution():
    axes={name:[{"value":value,"unit":"1","source":"fixture:1"} for value in values]
          for name,values in {"propulsion":["both","one","none"],"actuator":["normal","frozen","delayed"],"wind":["low","high"]}.items()}
    a=generate_pairwise(axes,seed=37); b=generate_pairwise(axes,seed=37)
    assert a==b
    assert len({r['id'] for r in a['cases']})==len(a['cases'])
    assert all(r['status']=='NOT_RUN' for r in a['cases'])
    for left,right in itertools.combinations(axes,2):
        covered={(row['values'][left]['value'],row['values'][right]['value']) for row in a['cases']}
        wanted={(x['value'],y['value']) for x in axes[left] for y in axes[right]}
        assert covered==wanted

def test_pairwise_matrix_requires_source_unit_and_unique_levels():
    with pytest.raises(ValueError): generate_pairwise({'a':[{'value':1,'unit':'m'}],'b':[{'value':1,'unit':'m','source':'x'}]})
