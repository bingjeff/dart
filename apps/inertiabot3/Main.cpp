#include "dynamics/SkeletonDynamics.h"
#include "utils/Paths.h"
#include "math/UtilsMath.h"
#include "simulation/World.h"
#include "robotics/parser/dart_parser/DartLoader.h"
#include <iostream>

#include "MyWindow.h"
#include "Controller.h"
#include "qlearning.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace dart_math;

int main(int argc, char* argv[])
{
    // load a URDF
    DartLoader dl;
    SkeletonDynamics* skelRobot;
    std::string urdfFileName(DART_DATA_PATH"urdf/inertiabot3.urdf");
    skelRobot = dl.parseSkeleton(urdfFileName);
    
    // create and initialize the world
    World *worldRobot = new World();
    //Vector3d gravity(0.0, -9.81, 0.0);
    Vector3d gravity(0.0, 0.0, 0.0);
    worldRobot->setGravity(gravity);
    worldRobot->addSkeleton(skelRobot);
    worldRobot->setTimeStep(1.0/2000);

    // create controller
    Controller *ctrlRobot = new Controller(worldRobot->getSkeleton(0));

    // create QLearning object
    Qlearning *learnRobot = new Qlearning(worldRobot, skelRobot, ctrlRobot);


//    // headless version of simulation
//    ctrlRobot->scriptLoadXML(DART_DATA_PATH"urdf/inertiabot.xml");
//    while( !ctrlRobot->isCommandBufferEmpty() )
//    {
//        // Update the desired control based on a stored policy
//        ctrlRobot->updateControlPolicy( worldRobot->getTime() );
//        // add control force
//        ctrlRobot->computeTorques();
//        skelRobot->setInternalForces( ctrlRobot->getTorques() );
//        // simulate one step
//        worldRobot->step();
//    }

    // rendered version of simulation
    // create a window and link it to the world
    MyWindow window;
    window.setWorld(worldRobot);
    window.setController(ctrlRobot);
    window.setLearner(learnRobot);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Forward Simulation");
    glutMainLoop();

    return 0;
}
