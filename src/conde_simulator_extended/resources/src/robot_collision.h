#ifndef _ROBOT_COLLISION_H_
#define _ROBOT_COLLISION_H_

#include <string>

using namespace std;

class RobotCollision
{
  public:
    string component;
    bool isColliding;

    RobotCollision(string component, bool colliding);

    void setComponent(string new_component);
    void setCollisionState(bool state);
};

#endif