"""App-owned recording location; the launcher owns recorder execution."""
import json
from pathlib import Path

DEFAULT_APP_CONFIG = Path(__file__).resolve().parents[2] / 'configs/app.json'


def load_recording_output(path=DEFAULT_APP_CONFIG, *, output_dir=None):
    """File paths are config-relative; a CLI override is working-directory-relative."""
    path = Path(path).expanduser().resolve()
    try:
        config = json.loads(path.read_text())
        if not isinstance(config, dict) or set(config) != {'recording'}:
            raise ValueError('expected only a recording object')
        recording = config['recording']
        if not isinstance(recording, dict) or set(recording) != {'output_dir'}:
            raise ValueError('recording must contain only output_dir')
        value = recording['output_dir']
        if not isinstance(value, str) or not value.strip():
            raise ValueError('recording.output_dir must be a nonempty string')
        if output_dir is not None:
            if not isinstance(output_dir, (str, Path)) or not str(output_dir).strip():
                raise ValueError('MCAP output override must be a nonempty path')
            return Path(output_dir).expanduser().resolve()
        return (path.parent / Path(value).expanduser()).resolve()
    except (OSError, ValueError, RuntimeError) as error:
        raise ValueError(f'Invalid app recording configuration {path}: {error}') from error
