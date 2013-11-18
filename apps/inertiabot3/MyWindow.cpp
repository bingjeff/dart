#include "MyWindow.h"
#include "simulation/World.h"
#include "kinematics/Shape.h"
#include "utils/Paths.h"
#include "dynamics/BodyNodeDynamics.h"
#include "yui/GLFuncs.h"
#include <Eigen/Core>

using namespace Eigen;
using namespace dynamics;
using namespace renderer;
using namespace dart_math;
using namespace yui;


void MyWindow::timeStepping()
{
    // Update the desired control based on a stored policy
//    mController->updateControlPolicy( mWorld->getTime() );
    if( mWorld->getTime() > mCurTime + mLearner->getStepTime() )
    {
        VectorXd action = mLearner->policy(mWorld->getState());
        cout << "T: " << mWorld->getTime();
        for(int c=0; c<mNumServos; ++c)
        {
            cout <<" [" << c << "]: " << action[c];
            mController->setDesiredDof(c, action[c]);
        }
        cout << endl;
        this->latchTime();
    }

//*
    // add control force
    mController->computeTorques();
    mWorld->getSkeleton(0)->setInternalForces( mController->getTorques() );
    // simulate one step
    mWorld->step();
//*/
}

// Code not currently utilized
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
    // Used for building inertia ellipsoids
    SelfAdjointEigenSolver<Matrix3d> es;
    Vector3d inertiaAxis;
    Affine3d inertiaLocation;
    double s11, s22, s33, m;
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); ++i)
    {
        // Hide/show render mesh and show/hide inertia ellipsoid
        if ( mVisibleInertiaEllipsoid )
        {
            for (unsigned int j = 0; j < mWorld->getSkeleton(i)->getNumNodes(); ++j)
            {
                mWorld->getSkeleton(i)->getNode(j)->updateTransform();
                es.compute( mWorld->getSkeleton(i)->getNode(j)->getWorldInertia() );
                s11 = es.eigenvalues()[0];
                s22 = es.eigenvalues()[1];
                s33 = es.eigenvalues()[2];
                m = mWorld->getSkeleton(i)->getNode(j)->getMass();
                inertiaAxis[0] = sqrt( 2.5 / m * (-s11 + s22 + s33 ) );
                inertiaAxis[1] = sqrt( 2.5 / m * ( s11 - s22 + s33 ) );
                inertiaAxis[2] = sqrt( 2.5 / m * ( s11 + s22 - s33 ) );
                
                mRI->pushMatrix();
                mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
                inertiaLocation.setIdentity();
                inertiaLocation.translate( mWorld->getSkeleton(i)->getNode(j)->getWorldCOM() );
                inertiaLocation.rotate( es.eigenvectors() );
                mRI->transform( inertiaLocation );
                mRI->drawEllipsoid( inertiaAxis );
                mRI->popMatrix();
            }
        }
        else
        {
            glLineWidth( 2.0f );
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            mWorld->getSkeleton(i)->draw(mRI);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            mWorld->getSkeleton(i)->draw(mRI);
        }
        // Hide/show collision volume
        if( mVisibleCollisionShape )
        {
            kinematics::Shape *collisionShape;
            for (unsigned int j = 0; j < mWorld->getSkeleton(i)->getNumNodes(); ++j)
            {
                for (unsigned int k = 0; k < mWorld->getSkeleton(i)->getNode(j)->getNumCollisionShapes(); ++k)
                {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    collisionShape = mWorld->getSkeleton(i)->getNode(j)->getCollisionShape(k);
                    collisionShape->draw(mRI, Vector4d(0.2, 0.8, 0.2, 1.0), false);
                }
            }
        }
    }
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    const double pi = std::acos(-1.0);
    VectorXd st;
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'c': // use c key to visualize the collision shapes
        mVisibleCollisionShape = !mVisibleCollisionShape;
        break;
    case 'm': // use c key to visualize the collision shapes
        mVisibleInertiaEllipsoid = !mVisibleInertiaEllipsoid;
        break;
    case 'l': // use l key to load a script
        mController->setReferenceTime( mWorld->getTime() );
        mController->scriptLoadXML(DART_DATA_PATH"urdf/inertiabot.xml");
        break;
    case 'r': // use r key to reset simulation
        mWorld->reset();
        mWorld->step();
        this->latchTime();
        break;
    case '?': // use ? key to run an episode of learning
        mLearner->learnepisode();
        break;
    case 'i': // use i-key to print out debug information
        for(int c = 0; c < mWorld->getNumSkeletons(); ++c)
        {
            cout << "SKELETON: " << c << endl
                 << " NUM NODES: " << mWorld->getSkeleton(c)->getNumNodes() << endl
                 << " NUM DOF: " << mWorld->getSkeleton(c)->getNumDofs() << endl
                 << " NUM JOINTS: " << mWorld->getSkeleton(c)->getNumJoints() << endl;
            for(int j = 0; j < mWorld->getSkeleton(c)->getNumDofs(); ++j)
            {
                cout << "  DOF [" << j << "]: " 
                     << mWorld->getSkeleton(c)->getDof(j)->getName() << endl;
            }
        }
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
    case '0': // rotate center link
        st = mWorld->getState();
        st[3] += 0.05;
        st[11] += 0.05;
        mWorld->setState(st);
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

