"""App recording settings are pure path resolution, not recorder execution."""
import json
from pathlib import Path

import pytest
from bullet_app.recording_config import DEFAULT_APP_CONFIG, load_recording_output


def write_config(tmp_path,value):
    path=tmp_path/'settings/app.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value))
    return path


def test_default_uses_workspace_recordings():
    assert load_recording_output()==DEFAULT_APP_CONFIG.parents[2]/'recordings'


def test_relative_config_and_cli_paths_use_their_own_bases(tmp_path,monkeypatch):
    path=write_config(tmp_path,{'recording':{'output_dir':'../flights'}})
    working=tmp_path/'other-working-directory'
    working.mkdir()
    monkeypatch.chdir(working)
    assert load_recording_output(path)==tmp_path/'flights'
    assert load_recording_output(path,output_dir='one-run')==working/'one-run'
    assert not (tmp_path/'flights').exists()
    assert not (working/'one-run').exists()


def test_absolute_and_home_paths_and_cli_precedence(tmp_path):
    absolute=tmp_path/'absolute output'
    path=write_config(tmp_path,{'recording':{'output_dir':str(absolute)}})
    assert load_recording_output(path)==absolute
    path.write_text(json.dumps({'recording':{'output_dir':'~/flight-test-output'}}))
    assert load_recording_output(path)==Path.home()/'flight-test-output'
    assert load_recording_output(path,output_dir=absolute)==absolute
    assert load_recording_output(path,output_dir='~/one-run')==Path.home()/'one-run'


@pytest.mark.parametrize('value',[None,[],{}, {'other':1}, {'recording':None}, {'recording':{}},
                                  {'recording':{'output_dir':None}}, {'recording':{'output_dir':True}},
                                  {'recording':{'output_dir':3}}, {'recording':{'output_dir':''}},
                                  {'recording':{'output_dir':'   '}}, {'recording':{'output_dir':'ok','typo':1}},
                                  {'recording':{'output_dir':'ok'},'typo':1}])
def test_rejects_missing_unknown_and_invalid_values(tmp_path,value):
    path=write_config(tmp_path,value)
    with pytest.raises(ValueError,match=str(path)):
        load_recording_output(path)


@pytest.mark.parametrize('override',['','  ',False,7])
def test_rejects_invalid_cli_override(tmp_path,override):
    path=write_config(tmp_path,{'recording':{'output_dir':'ok'}})
    with pytest.raises(ValueError,match='override'):
        load_recording_output(path,output_dir=override)


def test_rejects_missing_and_malformed_file_even_with_override(tmp_path):
    path=tmp_path/'missing.json'
    with pytest.raises(ValueError,match=str(path)):
        load_recording_output(path)
    path.write_text('{bad json')
    with pytest.raises(ValueError,match=str(path)):
        load_recording_output(path,output_dir=tmp_path/'override')
