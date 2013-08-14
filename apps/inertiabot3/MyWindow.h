#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "simulation/SimWindow.h"
#include "Controller.h"
#include <tinyxml2.h>
#include <Eigen/Core>
#include <list>

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

 private:
    void loadXmlScript(const char* _xmlFileName);
    void loadXmlCommand(tinyxml2::XMLElement* _rootElem, const char* _strCmdName, std::list<double> (& _vecCmd)[4]);
    std::list<double> parseTextVector(const char* _txtVector);
    Eigen::VectorXd computeDamping();
    std::list<double> mCommands[4];
    Controller *mController;
    bool mVisibleCollisionShape;
    bool mVisibleInertiaEllipsoid;
};

#endif
