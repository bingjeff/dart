#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/BodyNode.h"
#include <iostream>

using namespace Eigen;

Controller::Controller(dynamics::SkeletonDynamics *_skel) 
{
    mSkel = _skel;
    int nDof = mSkel->getNumDofs();
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    mKp.resize(nDof);
    mKd.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();    
        mKp[i] = 200.0;
        mKd[i] = 10;
    }
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel, const Eigen::VectorXd& _boxPt) 
{ 
    int nNodes = mSkel->getNumNodes();
    // Find the last node of the rigid body chain
    kinematics::BodyNode* lstNode = mSkel->getNode(nNodes - 1);
    Vector3d nodePt(0.0,-0.1,0.0);
    MatrixXd J = mSkel->getJacobian(lstNode, nodePt);
    VectorXd X = lstNode->evalWorldPos(nodePt);
    VectorXd V = J*_dofVel;
    // Create an endpoint force equal to a virtual spring and damper
    // connected to the box.
    VectorXd F = -300*(X - _boxPt) - 50*V;
    // Solve for the appropriate joint torques T = J'*F
    mTorques = J.transpose()*F;
    //std::cout << "..." << std::endl << _boxPt << std::endl;
}
