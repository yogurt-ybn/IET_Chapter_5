import json
from pathlib import Path
from iet_ch5.backends.airsim_backend import AirSimBackend
from iet_ch5.runner import run
from iet_ch5.scene_capture import SceneCapture
from iet_ch5.logging import config_hash,dump,revision

root=Path('results/runs/figure-eight-cruise-20261020')
local=json.loads(Path('.local/environment.json').read_text())
manifest=json.loads((root/'manifest.json').read_text())
controllers=json.loads(Path('configs/controllers.json').read_text())
assert config_hash(controllers)==manifest['controllers_sha256']
assert revision()==manifest['software']
task=manifest['task_config']
backend=AirSimBackend(local['host'],local['port'],local['vehicle'],exclusive=True)
try: backend.stop()
finally: backend.close()
partial=root/'lqr'
if partial.exists() and not (partial/'result.json').exists():
 partial.rename(root/'lqr-interrupted-client')
 (root/'client_interruption.json').write_text(dump({
  'observed_exit_code':143,'interrupted_run':'lqr-interrupted-client',
  'observation':'Capture client stopped before final result; simulator remained running',
  'cause':'SIGTERM indicated by shell exit code; sender/cause unverified',
  'handling':'Partial record retained; complete LQR restarted from common reset with unchanged configuration'})+'\n')
summary=[json.loads((root/'pid'/'result.json').read_text())]
for method in ['lqr','geometric','dob','mpc']:
 print('Capturing',method,flush=True)
 result=run(local,controllers,task,method,'figure_eight',root/method,
            scene_capture=SceneCapture(position=task['demo_camera_position_vehicle_origin_ned_m']))
 summary.append(result)
 (root/'results.json').write_text(dump(summary)+'\n')
 print(dump(result),flush=True)
 if result.failure and result.failure!='Tracking acceptance tolerance not met':
  raise RuntimeError(result.failure)
(root/'capture_complete.json').write_text(dump({'completed':True,'methods':5,'source':revision()})+'\n')
