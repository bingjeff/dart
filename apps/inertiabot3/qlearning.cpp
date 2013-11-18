#include "qlearning.h"

#include "math/UtilsMath.h"

using namespace dart_math;
using namespace dynamics;
using namespace simulation;
using namespace std;

Qlearning::Qlearning(World* _world, SkeletonDynamics* _skeleton, Controller* _controller)
{
    mWorld = _world;
    mSkeleton = _skeleton;
    mController = _controller;
    mStateHi = (VectorXd(2) << 1.6, 5.0).finished();
    mStateLo = (VectorXd(2) <<-1.6,-5.0).finished();
    mStateMap = (VectorXi(2) << 3, 11).finished();
    mActions = (MatrixX2d(9,2) <<
                0, 0,
                0, 90,
                90, 90,
                90, 0,
                90, -90,
                0, -90,
                -90, -90,
                -90, 0,
                -90, 90
                ).finished() * deg2rad;
    mStateRef = mWorld->getState();
    mStateRef[mStateMap[0]] = 90.0 * deg2rad;
    mStateRef[mStateMap[1]] = 0.0 * deg2rad;
    mAlpha = 0.9;
    mGamma = 1.0;
    mEpisodeTime = 2.0;
    mStepTime = 0.1;

    clearQTable();
}

void Qlearning::clearQTable()
{
    mQtable.resize(mNsta);
    for(int i=0; i<mNsta; ++i)
    {
        mQtable[i].resize(mNsta);
        for(int j=0; j<mNsta; ++j)
        {
            mQtable[i][j].resize(mNact);
            for(int k=0; k<mNact; ++k)
            {
                mQtable[i][j][k] = 0.0;
            }
        }
    }
}

VectorXd Qlearning::calcNextState(VectorXd _state, VectorXd _action)
{
    VectorXd nxtState;
    double curTime = mWorld->getTime();

    mWorld->setState( _state );
    for(int c=0; c<mNumStates; ++c)
        mController->setDesiredDof(c, _action[c]);
    while( mWorld->getTime() < curTime + mStepTime )
    {
        mController->computeTorques();
        mSkeleton->setInternalForces( mController->getTorques() );
        mWorld->step();
    }
    nxtState = mWorld->getState();
    mWorld->setState( _state );
    mWorld->setTime( curTime );
    return nxtState;
}

VectorXi Qlearning::calcTableState(VectorXd _state)
{
    VectorXi idxState( mNumStates );

    for(int c=0; c<mNumStates; ++c)
    {
        if( _state[mStateMap[c]] > mStateHi[c] )
        {
            idxState[c] = mNsta - 1;
        }
        else if( _state[mStateMap[c]] < mStateLo[c] )
        {
            idxState[c] = 0;
        }
        else
        {
            idxState[c] = dart_math::round( (mNsta-1) * (_state[mStateMap[c]] - mStateLo[c]) / (mStateHi[c] - mStateLo[c]) );
        }
//        cout <<"[" << idxState[c] << "] _state | mStateHi | mStateLo: " << _state[mStateMap[c]] <<" | "<< mStateHi[c] <<" | "<< mStateLo[c] << endl; //DEBUG
    }

    return idxState;
}

double Qlearning::reward(VectorXd _state)
{

    if( calcTableState(_state) == calcTableState(mStateRef) )
    {
        return 0.0;
    }
    else
    {
        return -1.0;
    }
}

double Qlearning::learnepisode()
{
    double curTime;
    int ia_cur = 0;
    mWorld->reset();
    while( mWorld->getTime() < mEpisodeTime )
    {
        curTime = mWorld->getTime();
        cout << curTime << " ";
        ia_cur = learnstep(ia_cur);
    }

    return 0.0;
}

int Qlearning::learnstep(int _ia_cur)
{
    // action index
    int ia_star = 0;
    // current state
    VectorXd xcur = mWorld->getState();
    // new state due to action
    VectorXd xstep = calcNextState(xcur, mActions.row(ia_star));
    VectorXd xstep_star = xstep;
    VectorXi ix = calcTableState(xstep);
    VectorXi ix_star = ix;
    // value of Qtable at new state and action
    double Qcur = mQtable[ix[0]][ix[1]][ia_star];
    double Qmax = Qcur;

//    cout << "Qcur: "<< Qcur << " " << "idx: "<< ix.transpose() << " " << ia_star << endl; //DEBUG
//    cout << "mActions.row(ia): "<< mActions.row(ia_star) << endl; //DEBUG
//    cout << "xstep: "<< xstep.transpose() << endl; //DEBUG
//    cout << endl; //DEBUG

    // find best Qtable value for new state
    for(int ia=1; ia < mActions.rows(); ia++)
    {
        xstep = calcNextState(xcur, mActions.row(ia));
        ix = calcTableState(xstep);
        Qcur = mQtable[ix[0]][ix[1]][ia];
//        cout << "Qcur: "<< Qcur << " " << "idx: "<< ix.transpose() << " " << ia << endl; //DEBUG
//        cout << "mActions.row(ia): "<< mActions.row(ia) << endl; //DEBUG
//        cout << "xstep: "<< xstep.transpose() << endl; //DEBUG
//        cout << endl; //DEBUG
        if( Qcur > Qmax)
        {
            Qmax = Qcur;
            ix_star = ix;
            ia_star = ia;
            xstep_star = xstep;
        }
    }
    // update Qtable with new action
    ix = calcTableState(xcur);
    Qcur = mQtable[ix[0]][ix[1]][_ia_cur];
    Qmax = Qcur + mAlpha * ( reward(xcur) + mGamma * Qmax - Qcur);
    mQtable[ix[0]][ix[1]][_ia_cur] = Qmax;
    cout << "Q[" << ix[0] << "][" << ix[1] <<"]["<< _ia_cur << ", " << ia_star << "*]: " << Qmax << endl;
    // move
    mWorld->setState( xstep_star );
    mWorld->setTime( mWorld->getTime() + mStepTime );
    return ia_star;
}


VectorXd Qlearning::policy(VectorXd _state)
{
    VectorXi ix = calcTableState(_state);
    int ia_star = 0;
    double Qcur = mQtable[ix[0]][ix[1]][ia_star];
    double Qmax = Qcur;
    cout<<"ix: " << ix[0] <<", " << ix[1]<<endl;
    for(int ia=1; ia<mActions.rows(); ia++)
    {
        Qcur = mQtable[ix[0]][ix[1]][ia];
        if(Qcur > Qmax)
        {
            ia_star = ia;
            Qmax = Qcur;
        }
    }
    return mActions.row( ia_star );
}
