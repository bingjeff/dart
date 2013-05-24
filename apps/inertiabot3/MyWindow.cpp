#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/BodyNodeDynamics.h"
#include "yui/GLFuncs.h"

using namespace Eigen;
using namespace dynamics;
using namespace math;
using namespace yui;


void MyWindow::timeStepping()
{
    // add damping
    VectorXd damping = computeDamping();
    // add control force
    mController->computeTorques(mWorld->getSkeleton(0)->getPose(), mWorld->getSkeleton(0)->getPoseVelocity());
    mWorld->getSkeleton(0)->setInternalForces( mController->getTorques() );
    // simulate one step
    mWorld->step();
}

VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(0)->getNumDofs();
    VectorXd damping = VectorXd::Zero(nDof);
    // add damping to each joint
    damping = -0.1 * mWorld->getSkeleton(0)->getPoseVelocity();
    return damping;
}

void MyWindow::drawSkels()
{
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    const double pi = std::acos(-1.0);
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSimulating = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSimulating) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSimulating) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case '1': // linkage extended
        mController->setDesiredDof(0, 0.0);
        mController->setDesiredDof(1, 0.0);
        break;
    case '2': // linkage collapsed
        mController->setDesiredDof(0, 0.9*pi);
        mController->setDesiredDof(1, -0.9*pi);
        break;
    case '3': // left link extended
        mController->setDesiredDof(0, 0.0);
        mController->setDesiredDof(1, -0.9*pi);
        break;
    case '4': // right link extended
        mController->setDesiredDof(0, 0.9*pi);
        mController->setDesiredDof(1, 0.0);
        break;
    case '5': // linkage collapsed (inverted)
        mController->setDesiredDof(0, -0.9*pi);
        mController->setDesiredDof(1, 0.9*pi);
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
