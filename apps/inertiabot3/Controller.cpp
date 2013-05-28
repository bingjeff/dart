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
    mTorques[6] = -mKp[0]*(_dof[6]-mDesiredDofs[0]) - mKd[0]*_dofVel[6];
    mTorques[7] = -mKp[1]*(_dof[7]-mDesiredDofs[1]) - mKd[1]*_dofVel[7];
//     std::cout << _dof.transpose() << mDesiredDofs.transpose() << std::endl;
    std::cout << mTorques.transpose() << std::endl;
}

/*
Vector3d Controller::evalLinMomentum(const VectorXd& _dofVel) {
    MatrixXd J(MatrixXd::Zero(3, mSkel->getNumDofs()));
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        MatrixXd localJ = node->getJacobianLinear() * node->getMass();
        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int index = node->getDependentDof(j);
            J.col(index) += localJ.col(j);
        }
    }
    Vector3d cDot = J * _dofVel;
    return cDot / mSkel->getMass();
}

Vector3d Controller::evalAngMomentum(const VectorXd& _dofVel) {
    Vector3d c = mSkel->getWorldCOM();
    Vector3d sum = Vector3d::Zero();
    Vector3d temp = Vector3d::Zero();
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        node->evalVelocity(_dofVel);
        node->evalOmega(_dofVel);
        sum += node->getWorldInertia() * node->mOmega;
        sum += node->getMass() * (node->getWorldCOM() - c).cross(node->mVel);
    }
    return sum;
}*/
