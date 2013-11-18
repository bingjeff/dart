#ifndef QLEARNING_H
#define QLEARNING_H

#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "simulation/World.h"
#include <Eigen/Core>
#include <vector>

using namespace Eigen;
using namespace dynamics;
using namespace simulation;

static const double deg2rad = acos(0) / 90.0;

class Qlearning
{
public:
    Qlearning(World* _world, SkeletonDynamics* _skeleton, Controller* _controller);

    VectorXd qfunction(VectorXd _state, VectorXd _action);
    VectorXd policy(VectorXd _state);
    double reward(VectorXd _state);

    double learnepisode();
    int learnstep(int _ia_cur);

    void clearQTable();

    inline double getStepTime() { return mStepTime; }
    inline double getEpisodeTime() { return mEpisodeTime; }

protected:
    static const int mNumStates = 2;

    static const int mNsta = 40;
    static const int mNact = 9;


    VectorXd mStateHi;
    VectorXd mStateLo;
    MatrixX2d mActions;
    VectorXd mStateRef;
    VectorXi mStateMap;

    double mAlpha;
    double mGamma;
    double mEpisodeTime;
    double mStepTime;

    World* mWorld;
    Controller* mController;
    SkeletonDynamics* mSkeleton;

    std::vector< std::vector< std::vector<double> > > mQtable;

    VectorXd calcNextState(VectorXd _state, VectorXd _action);
    VectorXi calcTableState(VectorXd _state);
};

#endif // QLEARNING_H
