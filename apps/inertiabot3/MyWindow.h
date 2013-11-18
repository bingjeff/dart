#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "simulation/SimWindow.h"
#include "Controller.h"
#include "qlearning.h"
#include <Eigen/Core>

class MyWindow : public simulation::SimWindow
{
 public:
    MyWindow(): SimWindow() {
        mVisibleCollisionShape = false;
        mVisibleInertiaEllipsoid = false;
    }
    virtual ~MyWindow() {}

    virtual void timeStepping();
    virtual void drawSkels();
    //  virtual void displayTimer(int _val);
    //  virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);

    inline void setController(Controller* _controller) { mController = _controller; }
    inline void setLearner(Qlearning* _learner) { mLearner = _learner; }
    inline void latchTime() { mCurTime = mWorld->getTime(); }

 private:
    double mCurTime;
    Eigen::VectorXd computeDamping();
    static const int mNumServos = 2;
    Controller *mController;
    Qlearning *mLearner;
    bool mVisibleCollisionShape;
    bool mVisibleInertiaEllipsoid;
};

#endif
