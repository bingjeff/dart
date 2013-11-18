#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/BodyNode.h"
#include <iostream>
#include <iomanip>

using namespace Eigen;
using namespace std;
using namespace tinyxml2;

Controller::Controller(dynamics::SkeletonDynamics *_skel) 
{
    mSkel = _skel;
    mReferenceTime = 0.0;
    mTorques.resize(mSkel->getNumDofs());
    mTorques.fill(0.0);
    mDesiredDofs.resize(mNumServos);
    mKp.resize(mNumServos);
    mKd.resize(mNumServos);
    idxDOFs[0] = 6; // Servo A
    idxDOFs[1] = 7; // Servo B
    for (int i = 0; i < mNumServos; i++){
        mDesiredDofs[i] = 0.0;
        mKp[i] = 0.1;
        mKd[i] = 0.025;
    }
}

void Controller::setCommands(std::list<double> _commands[])
{
    for(int c=0; c<2*mNumServos; ++c)
    {
        mCommands[c].clear();
        mCommands[c].insert(mCommands[c].begin(), _commands[c].begin(), _commands[c].end());
    }
}

void Controller::computeTorques()
{
    int idof;
    VectorXd dof = mSkel->getPose();
    VectorXd dofVel = mSkel->getPoseVelocity();
    // Solve for the appropriate joint torques
    for(int c=0; c<mNumServos; ++c)
    {
        idof = idxDOFs[c];
        mTorques[idof] = -mKp[c]*(dof[idof]-mDesiredDofs[c]) - mKd[c]*dofVel[idof];
    }
    //std::cout << _dof.transpose() << mDesiredDofs.transpose() << std::endl;
    //std::cout << mTorques.transpose() << std::endl;
}

// This sets the desired control value based on the buffered commands
void Controller::updateControlPolicy(double _time)
{
    for(int s=0; s<mNumServos; ++s)
    {
        while( mCommands[2*s].size() > 0 && mCommands[2*s].front() < _time )
        {
            mCommands[2*s].pop_front();
            mCommands[2*s+1].pop_front();
        }
        if( mCommands[2*s].size() > 0 )
        {
            setDesiredDof( s, mCommands[2*s+1].front() );
            if( _time - floor( 100*_time ) * 0.01 < 0.0005 )
                cout<<"SERVO "<< s << setw(8) << _time
                    << setw(6) << mCommands[2*s].front()
                    << setw(8) << mCommands[2*s+1].front()
                    << setw(12) << mSkel->getPose()[idxDOFs[s]]
                    << setw(1) << mTorques.transpose()
                    << endl;
        }
    }
}

bool Controller::isCommandBufferEmpty()
{
    for(int s=0; s<mNumServos; ++s)
    {
        if( mCommands[2*s].size() > 0)
        {
            return false;
        }
    }
    return true;
}

// This resets the reference time base (assumed starting time) for the buffered commands
void Controller::updateReferenceTime(double _time)
{
    for(int s=0; s<mNumServos; ++s)
    {
        double dt = _time - mReferenceTime;
        if( mCommands[2*s].size() > 0 && mCommands[2*s].front() != _time )
        {
          for(list<double>::iterator i=mCommands[2*s].begin(); i!=mCommands[2*s].end(); i++)
          {
            *i += dt;
          }
        }
    }
    mReferenceTime = _time;
}

void Controller::scriptLoadXML(const char* _xmlFileName)
{
    XMLDocument xmldoc;
    XMLElement* rootElem = NULL;
    XMLElement* scriptElem = NULL;
    XMLElement* curElem = NULL;
    int numRepeats = 0;

    // Clear out the previous commands and seed with current time
    for(int c=0; c<mNumServos; ++c)
    {
        mCommands[2*c].clear();
        mCommands[2*c].push_back( mReferenceTime );
        mCommands[2*c+1].clear();
        mCommands[2*c+1].push_back( 0.0 );
    }
    // Load the commands into the buffer
    xmldoc.LoadFile(_xmlFileName);
    rootElem = xmldoc.FirstChildElement("RobotProgram");
    if( rootElem )
    {
        scriptElem = rootElem->FirstChildElement("script");
        if( scriptElem )
        {
            curElem = scriptElem->FirstChildElement();
            while( curElem )
            {
                if( strcmp(curElem->Name(), "run") == 0 )
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
                          scriptParseCommand(rootElem, strCmdName, mCommands);
                        }
                    }
                }
                else if( strcmp(curElem->Name(), "pause") == 0 )
                {
                  const char* strAttribute = curElem->GetText();
                  double cmdPause = strAttribute ? atof(strAttribute) : 0.0;
                  for(int c=0; c<mNumServos; ++c)
                  {
                      mCommands[2*c].push_back( mCommands[2*c].back() + cmdPause );
                      mCommands[2*c+1].push_back( mCommands[2*c+1].back() );
                  }
                }
                curElem = curElem->NextSiblingElement();
            }
        }
    }
}

void Controller::scriptParseCommand(XMLElement* _rootElem, const char* _strCmdName, list<double> (& _cmd)[2*mNumServos])
{
    XMLElement* cmdElem = NULL;
    XMLElement* servoElem = NULL;
    XMLElement* curElem = NULL;
    list<double> lstTime, lstPosition;
    int servoNum = -1;
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
                servoNum = -1;
                if( servoElem->Attribute("name", "A") )
                    servoNum = 0;
                else if( servoElem->Attribute("name", "B") )
                    servoNum = 1;
                if( servoNum >= 0 )
                {
                    curElem = servoElem->FirstChildElement("time");
                    lstTime = scriptParseTextVector( curElem->GetText() );
                    curElem = servoElem->FirstChildElement("position");
                    lstPosition = scriptParseTextVector( curElem->GetText() );
                    if( lstTime.size() == lstPosition.size() )
                    {
                        // Time is specified as deltas in the script, but as absolute
                        // in the buffered commands. We attempt to pull previous time
                        // if it exists.
                        tlast = _cmd[2*servoNum].size()>0 ? _cmd[2*servoNum].back() : 0;
                        for(list<double>::iterator i=lstTime.begin(); i!=lstTime.end(); i++)
                        {
                            tlast += *i;
                            _cmd[2*servoNum].push_back( tlast );
                        }
                        // Add the points to the command
                        for(list<double>::iterator i=lstPosition.begin(); i!=lstPosition.end(); i++)
                        {
                            _cmd[2*servoNum+1].push_back( (*i) * degtorad );
                        }
                    }
                    else
                    {
                        cout << "ERROR: Command[" << _strCmdName << "] Servo["
                             << servoNum << "]time and position vectors are different length." << endl;
                    }
                }
                servoElem = servoElem->NextSiblingElement("servo");
            }
        }
        cmdElem = curElem->NextSiblingElement("command");
    }
}

list<double> Controller::scriptParseTextVector(const char* _txtVector)
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
