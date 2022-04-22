#include "gcomp_example.h"

bool GCompExample::on_initialize()
{
    setJournalLevel(Journal::Level::Low);

    /* Create model */
    _model = ModelInterface::getModel(_robot->getConfigOptions());

    int dofs = _model->getJointNum();

    /* Initialize variables */
    _robot->getMotorPosition(_q);
    _robot->getPositionReference(_qref);
    _tau.setZero(_robot->getJointNum());
    _kp_start = _kd_start = _kp = _kd = _kp_zero = _kd_zero = _tau;

    // we must explicitly set the control mode for our robot
    _robot->setControlMode(ControlMode::PosImpedance() + ControlMode::Effort());

    // same for the homing time
    _trj_time = getParamOr("~transition_time", 5.0);  // default to 5.0s

    // check valid time
    if(_trj_time <= 0)
    {
        jerror("got invalid transition time {} s \n",
               _trj_time);

        return false;
    }

    return true;
}

void GCompExample::starting()
{
    // initialize qref
    _robot->sense(false);
    _model->syncFrom(*_robot, Sync::Sensors, Sync::Position, Sync::Velocity, Sync::Effort, Sync::MotorSide);

    _robot->getPositionReference(_qref);
    _robot->getDamping(_kd_start);
    _robot->getStiffness(_kp_start);

    // initialize our fake time variable
    // this will increment by the nominal control
    // period at each iteration
    _fake_time = 0.0;


    start_completed();
}

void GCompExample::run()
{
    // call sense & read motor position
    _robot->sense(false);
    _model->syncFrom(*_robot, Sync::Sensors, Sync::Position, Sync::Velocity, Sync::Effort, Sync::MotorSide);


    // define a simplistic linear trajectory
    double tau = _fake_time/_trj_time;

    // quintic poly 6t^5 - 15t^4 + 10t^3
    double alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;

    // interpolate
    if(tau <= 1.0){
        _kp = _kp_start + alpha * (_kp_zero - _kp_start);
        _kd = _kd_start + alpha * (_kd_zero - _kd_start);
    }

    // set reference
    _robot->setStiffness(_kp);
    _robot->setDamping(_kd);

    // reset position reference to actual value
    _robot->getMotorPosition(_q);
    _robot->setPositionReference(_q);

    // compute non-linear torque term and set it as reference
    _model->computeNonlinearTerm(_tau);
    _robot->setEffortReference(_tau);

    _robot->move();

    // increment fake time
    // note: getPeriodSec() returns the nominal period
    _fake_time += getPeriodSec();
}

void GCompExample::stopping()
{
    _tau.setZero(_tau.size());
    _robot->setEffortReference(_tau);
    _robot->setPositionReference(_q);
    _robot->setDamping(_kd_start);
    _robot->setStiffness(_kp_start);
    _robot->move();
    stop_completed();
}

XBOT2_REGISTER_PLUGIN(GCompExample, gcomp_example)
