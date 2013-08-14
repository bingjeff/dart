#include "MyWindow.h"
#include "simulation/World.h"
#include "kinematics/Shape.h"
#include "utils/Paths.h"
#include "dynamics/BodyNodeDynamics.h"
#include "renderer/OpenGLRenderInterface.h"
#include "yui/GLFuncs.h"
#include <unsupported/Eigen/Splines>
#include <Eigen/Core>

using namespace Eigen;
using namespace dynamics;
using namespace renderer;
using namespace dart_math;
using namespace yui;
using namespace tinyxml2;


void MyWindow::timeStepping()
{
    typedef Spline<double,1> Spline2d;

    const double pi = std::acos(-1.0);
    const VectorXd xvals = (VectorXd(6) <<  0.0, 0.2, 0.4, 0.6, 0.8, 0.8).finished();
    //const VectorXd yvals = (VectorXd(6) << -0.5*pi,0.25*pi,0.0,0.25*pi,0.5*pi,0.5*pi).finished();
    const VectorXd yvals = (VectorXd(6) << -1.0, 0.5, 0.0, 0.5, 1.0, 1.0).finished();
    //const Spline2d splA = SplineFitting<Spline2d>::Interpolate(mCmdServoA.col(1).transpose(), 3, mCmdServoA.col(0).transpose());
    //const Spline2d splB = SplineFitting<Spline2d>::Interpolate(mCmdServoB.col(1).transpose(), 3, mCmdServoB.col(0).transpose());
    const Spline2d splA = SplineFitting<Spline2d>::Interpolate(yvals.transpose(), 3, xvals.transpose());

    double t = mWorld->getTime();
    mWorld->step();
    mController->setDesiredDof(0, splA(t)[0]);
    if (t < 0.8)
    {
      if ( abs(t  - floor(t / 0.01) * 0.01) < 1.e-3 )
      {
        std::cout << t << " "<< splA(t) << endl;
                  //<< " " << splB(t) << std::endl; 
      }
    }
    else
    {
        mSimulating = false;
    }
//*
    // add damping
    VectorXd damping = computeDamping();
    // add control force
    mController->computeTorques(mWorld->getSkeleton(0)->getPose(), mWorld->getSkeleton(0)->getPoseVelocity());
    mWorld->getSkeleton(0)->setInternalForces( mController->getTorques() );
    // simulate one step
    mWorld->step();
//*/
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
        loadXmlScript(DART_DATA_PATH"urdf/inertiabot.xml");
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

void MyWindow::loadXmlScript(const char* _xmlFileName)
{
    XMLDocument xmldoc;
    XMLElement* rootElem = NULL;
    XMLElement* scriptElem = NULL;
    XMLElement* curElem = NULL;
    int numRepeats = 0;

    // Clear out the previous commands and seed with current time
    for(int c=0; c<2; ++c)
    {
        mCommands[2*c].clear();
        mCommands[2*c].push_back( mWorld->getTime() );
        mCommands[2*c+1].clear();
        mCommands[2*c+1].push_back( 0.0 );
    }
    xmldoc.LoadFile(_xmlFileName);
    rootElem = xmldoc.FirstChildElement("RobotProgram");
    if( rootElem )
    {
        scriptElem = rootElem->FirstChildElement("script");
        if( scriptElem )
        {
            curElem = scriptElem->FirstChildElement("run");
            while( curElem )
            {
                // Find out if the command is repeated
                const char* strAttribute = curElem->Attribute("repeat");
                numRepeats = strAttribute ? atoi(strAttribute) : 0;
                // Search for command to add
                const char* strCmdName = curElem->GetText();
                if( strCmdName && numRepeats > 0 )
                {
                    for(int c=0; c < numRepeats; ++c)
                    {
                      loadXmlCommand(rootElem, strCmdName, mCommands);
                    }
                    while(mCommands[0].size() > 0)
                    {
                        for(int c=0; c < 4; ++c)
                        {
                            if( mCommands[c].size() > 0 )
                            {
                              cout<<mCommands[c].front()<<" ";
                              mCommands[c].pop_front();
                            }
                            else
                            {
                              cout<<"- ";
                            }
                        }
                        cout<<endl;
                    }
                    //TODO: write script parsing
                }
                curElem = curElem->NextSiblingElement("run");
            }
        }
    }
}

void MyWindow::loadXmlCommand(tinyxml2::XMLElement* _rootElem, const char* _strCmdName, list<double> (& _cmd)[4])
{
    XMLElement* cmdElem = NULL;
    XMLElement* servoElem = NULL;
    XMLElement* curElem = NULL;
    list<double> lstTime, lstPosition;
    int numServo = -1;
    double tlast = 0;
    double degtorad = asin(1.0) / 90.0;

    cmdElem = _rootElem->FirstChildElement("command");
    while( cmdElem )
    {
        if( cmdElem->Attribute("name", _strCmdName) )
        {
            servoElem = cmdElem->FirstChildElement("servo");
            while( servoElem )
            {
                numServo = -1;
                if( servoElem->Attribute("name", "A") )
                    numServo = 0;
                else if( servoElem->Attribute("name", "B") )
                    numServo = 1;
                if( numServo >= 0 )
                {
                    curElem = servoElem->FirstChildElement("time");
                    lstTime = parseTextVector( curElem->GetText() );
                    curElem = servoElem->FirstChildElement("position");
                    lstPosition = parseTextVector( curElem->GetText() );
                    if( lstTime.size() == lstPosition.size() )
                    {
                        // If there is already a time vector started we will add previous time
                        tlast = _cmd[2*numServo].size()>0 ? _cmd[2*numServo].back() : 0;
                        for(list<double>::iterator i=lstTime.begin(); i!=lstTime.end(); i++)
                        {
                            _cmd[2*numServo].push_back( *i + tlast );
                        }
                        // Add the points to the command
                        for(list<double>::iterator i=lstPosition.begin(); i!=lstPosition.end(); i++)
                        {
                            _cmd[2*numServo+1].push_back( (*i) * degtorad );
                        }
                    }
                    else
                    {
                        cout << "ERROR: Command[" << _strCmdName << "] Servo["
                             << numServo << "]time and position vectors are different length." << endl;
                    }
                }
                servoElem = servoElem->NextSiblingElement("servo");
            }
        }
        cmdElem = curElem->NextSiblingElement("command");
    }
}


list<double> MyWindow::parseTextVector(const char* _txtVector)
{
    list<double> lstVector;
    char* strVal;
    strVal = new char[ strlen( _txtVector ) ];
    strcpy( strVal, _txtVector );
    if( strlen(strVal) > 0 )
    {
        while( strlen(strVal) > 0 )
        {
            lstVector.push_back( strtod( strVal, &strVal ) );
        }
    }

    return lstVector;
}
