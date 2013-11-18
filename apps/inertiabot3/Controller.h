#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <list>
#include <tinyxml2.h>

namespace dynamics{
    class SkeletonDynamics;
}

class Controller {
 public:
    Controller(dynamics::SkeletonDynamics *_skel);
    virtual ~Controller() {};

    static const int mNumServos = 2;

    void setReferenceTime(double _time) { mReferenceTime = _time; };
    void setCommands(std::list<double> _commands[]);
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    void updateReferenceTime(double _time);
    void updateControlPolicy(double _time);
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    Eigen::VectorXd getKp() {return mKp; };
    Eigen::VectorXd getKd() {return mKd; };
    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };

    bool isCommandBufferEmpty();

    void computeTorques();
    Eigen::Vector3d evalLinMomentum(const Eigen::VectorXd& _dofVel);
    Eigen::Vector3d evalAngMomentum(const Eigen::VectorXd& _dofVel);
    void scriptLoadXML(const char* _xmlFileName);

 protected:
    dynamics::SkeletonDynamics *mSkel;
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    std::list<double> mCommands[2*mNumServos];
    int idxDOFs[mNumServos];
    double mReferenceTime;

    void scriptParseCommand(tinyxml2::XMLElement* _rootElem, const char* _strCmdName, std::list<double> (& _cmd)[2*mNumServos]);
    std::list<double> scriptParseTextVector(const char* _txtVector);
};

#endif // #CONTROLLER_H
