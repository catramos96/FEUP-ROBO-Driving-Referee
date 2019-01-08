#include "robot_collision.h"

RobotCollision::RobotCollision(string component, bool colliding)
{
    setComponent(component);
    setCollisionState(colliding);
};

void RobotCollision::setComponent(string new_component)
{
    component = new_component;
};

void RobotCollision::setCollisionState(bool state)
{
    isColliding = state;
};