#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"
#include "math/UtilsMath.h"
#include "simulation/World.h"

#include "MyWindow.h"
#include "Controller.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace math;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model;
    //model.loadFile(DART_DATA_PATH"/skel/nunchuck.skel", SKEL);
    model.loadFile(DART_DATA_PATH"/skel/inertiabot3.skel", SKEL);
    
    // create and initialize the world
    World *myWorld = new World();
    //Vector3d gravity(0.0, -9.81, 0.0);
    Vector3d gravity(0.0, 0.0, 0.0);
    myWorld->setGravity(gravity);

    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());

    myWorld->setTimeStep(1.0/2000);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);

    // create controller
    Controller *myController = new Controller(myWorld->getSkeleton(0));
    window.setController(myController);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Forward Simulation");
    glutMainLoop();

    return 0;
}
