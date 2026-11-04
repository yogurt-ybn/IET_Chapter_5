"""Residual semantics, real frame rotations, held-out fitting and immutable pair settings."""
from copy import deepcopy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from iet_ch5.allocation import Allocator
from iet_ch5.controllers.configuration import load_controller_configs
from iet_ch5.controllers.nmpc.model import dynamics, quaternion_from_rotation
from iet_ch5.frames import C
from iet_ch5.learning.dataset import aligned_flight
from iet_ch5.learning.gp import ForceGP, fit_force_gp, sha256
from iet_ch5.learning.workflow import matched_pair_configuration
from iet_ch5.types import VehicleParameters


@pytest.fixture
def parameters():
    return VehicleParameters(1.1,np.diag([.01,.012,.02]),9.81,
        np.array([[.16,-.16,0],[-.16,.16,0],[.16,.16,0],[-.16,-.16,0]]),
        np.tile([0.,0.,1.],(4,1)),np.array([-1,-1,1,1]),4.2,.056,1.,.005,.003)


@pytest.fixture
def gp():
    return ForceGP({'schema':'iet-ch5/force-gp-v1','input_center':[0,0,0,10.],
        'input_scale':[1.,1.,1.,1.],'support_normalized':[[0.,0.,0.,0.]],
        'alpha':[[.2,-.3,.5]],'length_scales':[[1.,1.,1.,1.]]*3,
        'amplitude':[1.,1.,1.],'output_mean_N':[0.,0.,0.],'output_scale_N':[1.,1.,1.],
        'force_bound_N':3.,'context':{'nominal_mass_kg':1.1}})


def test_body_force_enters_only_translation_with_correct_mass_and_rotation(parameters,gp):
    R=Rotation.from_euler('xyz',[.7,-.4,1.1]).as_matrix()
    x=np.r_[[1,2,3],[.2,-.1,.3],quaternion_from_rotation(R),[.3,.4,-.2],[10.,.01,-.02,.03]]
    u=x[13:]
    delta=dynamics(x,u,parameters,gp)-dynamics(x,u,parameters)
    features=np.r_[R.T@x[3:6],0.]
    force=np.array([.2,-.3,.5])*np.exp(-.5*features@features)
    assert np.allclose(delta[3:6],R@force/parameters.mass)
    assert np.allclose(np.r_[delta[:3],delta[6:]],0)


def test_symbolic_force_and_dynamics_jacobian_match_independent_numeric(parameters,gp):
    ca=pytest.importorskip('casadi')
    pytest.importorskip('acados_template')
    from iet_ch5.controllers.nmpc.model import symbolic_model
    model=symbolic_model(parameters,'learning_test',gp)
    function=ca.Function('f',[model.x,model.u],[model.f_expl_expr])
    jac=ca.Function('j',[model.x,model.u],[ca.jacobian(model.f_expl_expr,model.x)])
    x=np.r_[[1.,2.,3.],[.4,-.2,.3],quaternion_from_rotation(Rotation.from_euler('xyz',[.5,.2,.9]).as_matrix()),[.1,-.2,.3],[10.,.01,.02,-.01]]
    u=np.array([11.,.02,-.03,.01])
    assert np.allclose(np.asarray(function(x,u)).ravel(),dynamics(x,u,parameters,gp),atol=1e-10)
    h=1e-6
    finite=np.column_stack([(dynamics(x+h*e,u,parameters,gp)-dynamics(x-h*e,u,parameters,gp))/(2*h) for e in np.eye(17)])
    assert np.allclose(np.asarray(jac(x,u)),finite,atol=1e-7)


def test_gp_fit_generalizes_to_unseen_inputs_and_hash_rejects_tampering(tmp_path):
    random=np.random.default_rng(45)
    X=random.uniform(-1,1,(60,4))
    X[:,3]+=10
    def force(x):
        return np.column_stack([-.08*x[:,0],-.06*x[:,1],.1*(x[:,3]-10)+.8])
    model=fit_force_gp(X,force(X),support_count=32)
    held=random.uniform(-.8,.8,(25,4))
    held[:,3]+=10
    assert np.sqrt(np.mean((model.predict(held)-force(held))**2))<.02
    path=tmp_path/'gp.json'
    path.write_text(json.dumps(model.document))
    digest=sha256(path)
    assert np.allclose(ForceGP.load(path,digest).predict(held),model.predict(held))
    path.write_text(path.read_text()+' ')
    with pytest.raises(ValueError,match='checksum'):
        ForceGP.load(path,digest)


def test_only_residual_enable_may_change_in_a_pair():
    off=load_controller_configs()
    off['nmpc']['residual'].update(model_path='model.json',model_sha256='a'*64)
    on=deepcopy(off)
    on['nmpc']['residual']['enabled']=True
    assert matched_pair_configuration(off,on)
    on['nmpc']['horizon']+=1
    with pytest.raises(ValueError,match='beyond'):
        matched_pair_configuration(off,on)
    with pytest.raises(ValueError,match='SHA256'):
        load_controller_configs(overrides={'nmpc':{'residual':{'enabled':True}}})


def test_label_uses_applied_telemetry_not_requested_thrust_at_nonzero_attitude(tmp_path,parameters):
    from dataclasses import asdict
    from iet_ch5.logging import dump
    p=parameters
    R=Rotation.from_euler('xyz',[.4,-.3,.7]).as_matrix()
    native=Rotation.from_matrix(C@R@C).as_quat()
    body_force=np.array([.2,-.3,.4])
    collective=p.mass*p.gravity
    acceleration=collective/p.mass*R[:,2]-[0,0,p.gravity]+R@body_force/p.mass
    pwm=Allocator(p).inverse@np.r_[collective,[0.,0.,0.]]
    rotors=[{'control_signal_filtered':float(v)} for v in pwm]
    def vec(v):
        return dict(zip(('x_val','y_val','z_val'),v))
    stamp=1000000000
    start={'p':[0,0,3],'v':[0,0,0],'R':R.tolist(),'omega':[0,0,0],'timestamp_ns':stamp}
    meta={'verified_initial_state':start,'parameters':asdict(p),'controller_parameters':asdict(p),
        'software':{},'config_sha256':'test','config':{'task':'transfer','task_config':{'condition':{'id':'nominal'}}}}
    (tmp_path/'metadata.json').write_text(dump(meta))
    (tmp_path/'result.json').write_text('{}')
    physical=[]
    for i in range(1,101):
        t=i*.003
        physical.append({'timestamp':stamp+i*3000000,'sequence':i,'has_collided':False,'rotors':rotors,
            'kinematics':{'position':vec(C@(np.array([0,0,3])+.5*acceleration*t*t)),
                'linear_velocity':vec(C@(acceleration*t)), 'angular_velocity':vec([0,0,0]),
                'orientation':dict(zip(('x_val','y_val','z_val','w_val'),native))}})
    (tmp_path/'physics.jsonl').write_text(''.join(dump(r)+'\n' for r in physical))
    control=[{'simulation_timestamp_ns':stamp+i*12000000,'requested_wrench':[collective+2,0,0,0],
              'telemetry_estimated_wrench':[collective,0,0,0]} for i in range(25)]
    (tmp_path/'log.jsonl').write_text(''.join(dump(r)+'\n' for r in control))
    data,_=aligned_flight(tmp_path,warmup=0)
    assert np.allclose(data['target_force_N'],body_force,atol=1e-10)
    assert np.allclose(data['features'][:,3],collective)
    physical[5]['sequence']+=1
    (tmp_path/'physics.jsonl').write_text(''.join(dump(r)+'\n' for r in physical))
    with pytest.raises(ValueError,match='skipped'):
        aligned_flight(tmp_path)
