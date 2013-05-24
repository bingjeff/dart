#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/BodyNode.h"
#include <iostream>

using namespace Eigen;

Controller::Controller(dynamics::SkeletonDynamics *_skel) 
{
    mSkel = _skel;
    int nDof = 2;
        
    mTorques.resize(mSkel->getNumDofs());
    mTorques.fill(0);
    mDesiredDofs.resize(nDof);
    mKp.resize(nDof);
    mKd.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mDesiredDofs[i] = 0.0;
        mKp[i] = 1.0;
        mKd[i] = 0.5;
    }
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) 
{ 
    // Solve for the appropriate joint torques
    mTorques[4] = -mKp[0]*(_dof[4]-mDesiredDofs[0]) - mKd[0]*_dofVel[4];
    mTorques[5] = -mKp[1]*(_dof[5]-mDesiredDofs[1]) - mKd[1]*_dofVel[5];
//     std::cout << _dof.transpose() << mDesiredDofs.transpose() << std::endl;
    std::cout << mTorques.transpose() << std::endl;
}
