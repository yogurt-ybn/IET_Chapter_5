"""Physical invariants and generated model checks; no simulator performance is inferred."""
from dataclasses import replace
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from iet_ch5.controllers.configuration import load_controller_configs
from iet_ch5.controllers.nmpc.model import dynamics, quaternion_from_rotation, rotation_from_quaternion
from iet_ch5.controllers.nmpc.reference import flat_reference, ReferenceTable
from iet_ch5.frames import hat
from iet_ch5.trajectories import load_trajectory, Trajectory
from iet_ch5.types import VehicleParameters


@pytest.fixture
def parameters():
    return VehicleParameters(1.1,np.array([[.009,.001,0],[.001,.012,.0005],[0,.0005,.017]]),9.81,
        np.array([[.16,-.16,0],[-.16,.16,0],[.16,.16,0],[-.16,-.16,0]]),
        np.tile([0.,0.,1.],(4,1)),np.array([-1,-1,1,1]),4.2,.056,.99,.005,.003)


def test_large_attitude_force_and_gyroscopic_dynamics(parameters):
    p=parameters
    R=Rotation.from_euler('xyz',[.8,-.5,.7]).as_matrix()
    q=quaternion_from_rotation(R)
    omega=np.array([1.3,-.9,.7])
    applied=np.array([13.,.02,-.03,.01])
    x=np.r_[[1,2,3],[2,-1,.5],q,omega,applied]
    command=applied+np.array([.1,.01,0,0])
    dx=dynamics(x,command,p)
    assert np.allclose(dx[3:6],13/p.mass*R[:,2]-[0,0,p.gravity])
    assert abs(q@dx[6:10])<1e-12
    # Rdot=R[omega]x at large attitudes, including wxyz convention.
    h=1e-6
    assert np.allclose((rotation_from_quaternion(q+h*dx[6:10])-rotation_from_quaternion(q-h*dx[6:10]))/(2*h),R@hat(omega),atol=1e-8)
    assert np.allclose(p.J@dx[10:13]+np.cross(omega,p.J@omega),applied[1:])
    assert np.allclose(dx[13:17],(command-applied)/p.motor_time_constant)
    unforced=dynamics(x,np.r_[applied],p)
    torque_free=x.copy()
    torque_free[14:17]=0
    dw=dynamics(torque_free,applied,p)[10:13]
    assert abs(omega@p.J@dw)<1e-12  # Torque-free rotational kinetic energy conserved.
    assert np.allclose(unforced[13:17],0)


def test_symbolic_rigid_body_matches_independent_rhs(parameters):
    ca=pytest.importorskip('casadi',reason='Optional NMPC symbolic dependency not installed')
    pytest.importorskip('acados_template',reason='Optional pinned acados interface not installed')
    from iet_ch5.controllers.nmpc.model import symbolic_model
    model=symbolic_model(parameters,'numeric_test')
    function=ca.Function('rhs',[model.x,model.u],[model.f_expl_expr])
    random=np.random.default_rng(42)
    for _ in range(12):
        x=random.normal(size=17)
        x[6:10]/=np.linalg.norm(x[6:10])
        x[13]=10
        u=random.normal(size=4)
        u[0]=11
        assert np.allclose(np.array(function(x,u)).ravel(),dynamics(x,u,parameters),atol=1e-10)


def test_flatness_reference_and_interpolation_on_spatial_manoeuvre(parameters):
    task=Trajectory(load_trajectory('agile_course'))
    table=ReferenceTable(task.at,task.duration+1,parameters,.006)
    for t in [1.31,2.75,4.71,7.63,9.0]:
        flat=flat_reference(task.at(t),parameters)
        sampled=table.at([t])[0]
        assert abs(np.linalg.norm(sampled[6:10])-1)<1e-12
        assert np.max(np.abs(sampled[:6]-flat[:6]))<1e-4
        assert np.max(np.abs(sampled[13:]-flat[13:]))<.002
        h=1e-5
        qplus=flat_reference(task.at(t+h),parameters)[6:10]
        qminus=flat_reference(task.at(t-h),parameters)[6:10]
        R=rotation_from_quaternion(flat[6:10])
        Rdot=(rotation_from_quaternion(qplus)-rotation_from_quaternion(qminus))/(2*h)
        assert np.allclose(Rdot,R@hat(flat[10:13]),atol=2e-7)
        assert np.allclose(dynamics(flat,flat[13:],parameters)[3:6],task.at(t).a,atol=1e-10)
    # Lag/geometry transform consistency is tested with a different plant, not hardcoded nominal mass.
    heavier=replace(parameters,mass=parameters.mass*1.2)
    assert flat_reference(task.at(4),heavier)[13]>flat_reference(task.at(4),parameters)[13]


@pytest.mark.integration
def test_acados_solver_hover_reset_bounds_and_failure(parameters,monkeypatch):
    if os.environ.get('IET_CH5_TEST_ACADOS')!='1':
        pytest.skip('Explicitly opt in with IET_CH5_TEST_ACADOS=1 after pinned installation')
    pytest.importorskip('acados_template')
    from iet_ch5.controllers.nmpc.controller import NMPC
    from iet_ch5.types import State,Reference
    c=load_controller_configs()['nmpc']
    # Keep build in the normal ignored cache; shared cache keys include parameters/source.
    controller=NMPC(c)
    state=State(np.array([0.,0.,3.]),np.zeros(3),np.eye(3),np.zeros(3),0)
    ref=Reference(state.p.copy())
    controller.reset(state,ref,parameters)
    controller.prepare_reference(lambda t:ref,1.)
    u,d=controller.update(state,ref,.012)
    assert not d['fallback']
    assert np.allclose(u.vector(),[parameters.mass*parameters.gravity,0,0,0],atol=2e-4)
    assert np.max(np.abs(controller.allocator.inverse@u.vector()))<1
    controller.reset(state,ref,parameters)
    controller.prepare_reference(lambda t:ref,1.)
    assert np.allclose(controller.update(state,ref,.012)[0].vector(),u.vector(),atol=2e-4)
    # A genuine failed solver status must activate and identify geometric fallback.
    monkeypatch.setattr(controller.solver,'solve',lambda:1)
    controller.elapsed=1
    _,d=controller.update(state,ref,.012)
    assert d['fallback'] and d['fallback_controller']=='geometric'
    assert d['solver_status']=='acados_1'
