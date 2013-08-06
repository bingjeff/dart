#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/BodyNodeDynamics.h"
#include "yui/GLFuncs.h"
#include <unsupported/Eigen/Splines>

using namespace Eigen;
using namespace dynamics;
using namespace dart_math;
using namespace yui;


void MyWindow::timeStepping()
{
//     typedef Spline<double,1> Spline2d;
// 
//     const double pi = std::acos(-1.0);
//     const VectorXd xvals = (VectorXd(5) << 0.0,0.1,0.2,0.3,0.4).finished();
//     const VectorXd yvals = (VectorXd(5) << -1.0*pi,0.5*pi,0.0,0.5*pi,1.0*pi).finished();
//     const Spline2d spline = SplineFitting<Spline2d>::Interpolate(yvals.transpose(), 3, xvals.transpose());
// 
// 
//     double t = mWorld->getTime();
//     if (t < 0.4)
//     {
//         std::cout << t << " " << spline(t) << std::endl; 
//     }
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
