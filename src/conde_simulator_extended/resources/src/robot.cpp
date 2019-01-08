#include "robot.h"

Robot::Robot(string name)
{
    driving_score = 0;
    parking_score = 0;
    driving_penalties = 0;
    parking_penalties = 0;
    setName(name);
    next_route.push_back(START_WAYPOINT);

    /* TO DELETE , IT'S FOR TEST 
        setRaceState(PARALLEL_PARKING);
        parking_started = false;
        route.push_back(6);
        next_route.push_back(11);*/

    parking_started = false;
    setBoundaryCollision(false);
    initializeCollisionVectors();
};

void Robot::initializeCollisionVectors()
{
    RobotCollision right_wheel = RobotCollision("right_wheel", false);
    RobotCollision left_wheel = RobotCollision("left_wheel", false);
    RobotCollision front_wheel = RobotCollision("chassis", false);

    inside_collisions.push_back(right_wheel);
    inside_collisions.push_back(left_wheel);
    inside_collisions.push_back(front_wheel);

    outside_collisions.push_back(right_wheel);
    outside_collisions.push_back(left_wheel);
    outside_collisions.push_back(front_wheel);

    boundaries_collisions.push_back(right_wheel);
    boundaries_collisions.push_back(left_wheel);
    boundaries_collisions.push_back(front_wheel);
};

void Robot::setName(string new_name)
{
    name = new_name;
};

void Robot::startRace()
{
    start_time = ros::Time::now().toSec();
    race_state = ONGOING;
};

void Robot::endDrivingChallenge()
{
    end_time = ros::Time::now().toSec();
};

void Robot::addDrivingPenalty(int new_penalty)
{
    driving_penalties += new_penalty;
    last_penalty_time = ros::Time::now().toSec();
};

void Robot::addParkingPenalty(int new_penalty)
{
    parking_penalties += new_penalty;
    last_penalty_time = ros::Time::now().toSec();
};

void Robot::setBoundaryCollision(bool state)
{
    had_boundary_collision = state;
};

void Robot::setInsideParking(bool state)
{
    is_inside_parking = state;
};

void Robot::startParking()
{
    parking_started = true;
    parking_time = ros::Time::now().toSec();
};

void Robot::endParking()
{
    parking_started = false;
};

void Robot::setRaceState(RaceState state)
{
    race_state = state;
};

void Robot::setLastPenalty(Sensor sensor)
{
    last_penalty = sensor;
};

void Robot::setParkingScore(double score)
{
    parking_score = score;
}

RaceState Robot::getRaceState()
{
    return race_state;
};

Sensor Robot::getLastPenalty()
{
    return last_penalty;
};

double Robot::getLastPenaltyTime()
{
    return last_penalty_time;
};

double Robot::getParkingTime()
{
    return parking_time;
};

bool Robot::getHadBoundaryCollision()
{
    return had_boundary_collision;
};

bool Robot::isParking()
{
    return parking_started;
};

int Robot::getLastWaypoint()
{
    if (route.size() != 0)
        return route[route.size() - 1];
    else
        return 8;
};

double Robot::getRaceTime()
{
    if (!hasFinishRace(route))
        return -1;
    else
        return (ros::Duration(end_time) - ros::Duration(start_time)).toSec();
};

void Robot::calculateDrivingScore()
{ // Driving with signs
    int half_laps = LAPS * 2;
    driving_score = half_laps * DRIVING_SCORE_REFERENCE + (half_laps * TIME_REFERENCE - end_time) - driving_penalties;
};

void Robot::calculateParkingScore()
{ // Parallel or Bay parking without obstacles
    parking_score = PARKING_SCORE_REFERENCE - parking_penalties;
};

void Robot::consumeRouteWaypoint(int waypoint, SemaphoreState state)
{
    if (race_state == DISQUALIFIED || race_state == FINISHED)
        return;

    // Keeps refreshing flag in case robot had left the parking
    // No more waypoints, referee is just waiting for the robot to park
    if (isParkingWaypoint(waypoint) && isParking())
    {
        setInsideParking(true);

        double current = ros::Time::now().toSec();

        // Robot has 5 seconds to finish the parking
        if (current - parking_time > 5)
        {
            endParking();
            calculateParkingScore();
            setRaceState(FINISHED);
            print();
        }
        return;
    }

    int last_waypoint = getLastWaypoint();

    // No need to double check unless is a parking waypoint - Several messages are received during the time the robot passes through the sensor
    if (waypoint == last_waypoint)
        return;

    int next_waypoint = START_WAYPOINT;

    if (next_route.size() != 0)
        next_waypoint = next_route[0];
    else
    {
        next_route.clear();
        next_route.push_back(START_WAYPOINT);
        next_waypoint = START_WAYPOINT;
    }

    //correct path
    if (waypoint == next_waypoint)
    {
        //near semaphore
        if (waypoint == START_WAYPOINT)
        {
            //check if the robot already started the race
            if (route.size() == 0)
                startRace();

            if (state == STOP)
            {
                // Check time of last semaphore penalty
                if (last_penalty == SEMAPHORE)
                {
                    double current = ros::Time::now().toSec();

                    // After 2 seconds it's okay to penalty again
                    if (current - last_penalty_time >= SEMAPHORE_PENALTY_TIME_INTERVAL)
                    {
                        addDrivingPenalty(60);
                        last_penalty = SEMAPHORE;
                        cout << "\n\nSemaphore Penalty for robot: " << name << endl;
                    }
                }
                else
                {
                    addDrivingPenalty(60);
                    last_penalty = SEMAPHORE;
                    cout << "\n\nSemaphore Penalty for robot: " << name << endl;
                }
            }

            if (state == PARK)
            {
                endDrivingChallenge();
                calculateDrivingScore();
                setRaceState(PARALLEL_PARKING);
            }
            
            if(!hasFinishRace(route))
                next_route = getNextRoute(state, goesToRight(last_waypoint));
        }

        //add waypoint to route
        route.push_back(next_waypoint);

        //remove waypoint from next route if not removed
        //when route was updated (near semaphore)
        if (next_route[0] == next_waypoint)
            next_route.erase(next_route.begin());

        if (isParkingWaypoint(waypoint) && !isParking())
        {
            setInsideParking(true);
            startParking();
        }

        //finish route
        if (hasFinishRace(route))
        {
            // Bay parking challenge starts after driving challenge is done
            if (race_state != PARALLEL_PARKING)
            {
                endDrivingChallenge();
                calculateDrivingScore();
                setRaceState(BAY_PARKING);
                next_route.clear();
                next_route.push_back(9);
            }
        }

        print();
    }
    else if (last_waypoint == START_WAYPOINT && !isParkingWaypoint(waypoint)) // Wrong direction after semaphore (ignores entering parallel parking area)
    {
        // Check time of last semaphore penalty
        if (last_penalty == SEMAPHORE)
        {
            double current = ros::Time::now().toSec();

            // After 2 seconds it's okay to penalty again
            if (current - last_penalty_time >= SEMAPHORE_PENALTY_TIME_INTERVAL)
            {
                addDrivingPenalty(25);
                last_penalty = SEMAPHORE;
                cout << "\n\nSemaphore Penalty for robot: " << name << endl;
            }
        }
        else
        {
            addDrivingPenalty(25);
            last_penalty = SEMAPHORE;
            cout << "\n\nSemaphore Penalty for robot: " << name << endl;
        }
    }
};

bool Robot::setCollisionStateBySensor(string component, Sensor sensor, bool state)
{
    switch (sensor)
    {
    case TRACK_OUTSIDE:

        for (int i = 0; i < outside_collisions.size(); i++)
        {
            if (component.compare(outside_collisions[i].component) == 0)
            {
                outside_collisions[i].setCollisionState(state);
                return true;
            }
        }
        break;
    case TRACK_INSIDE:
        for (int i = 0; i < inside_collisions.size(); i++)
        {
            if (component.compare(inside_collisions[i].component) == 0)
            {
                inside_collisions[i].setCollisionState(state);
                return true;
            }
        }
        break;
    case TRACK_BOUNDS:
        for (int i = 0; i < boundaries_collisions.size(); i++)
        {
            if (component.compare(boundaries_collisions[i].component) == 0)
            {
                boundaries_collisions[i].setCollisionState(state);
                return true;
            }
        }
        break;
    }

    return false;
};

bool Robot::isInsideTrack()
{
    for (int i = 0; i < inside_collisions.size(); i++)
    {
        if (inside_collisions[i].isColliding == false)
        {
            return false;
        }
    }

    return true;
};

bool Robot::isOutsideTrack()
{
    for (int i = 0; i < outside_collisions.size(); i++)
    {
        if (outside_collisions[i].isColliding == false)
        {
            return false;
        }
    }

    return true;
};

void Robot::print(void)
{
    string p = "\n\nNAME: " + name + "\n";
    p += "LAPS: " + lexical_cast<string>(getCurrentLap(route)) + lexical_cast<string>("/") + lexical_cast<string>(LAPS) + "\n";
    if (race_state == BAY_PARKING || race_state == PARALLEL_PARKING) // It means the driving challenge is over
    {
        p += "STATUS: " + getRaceStateName(race_state) + "\n";
        double secs = (ros::Duration(end_time) - ros::Duration(start_time)).toSec();
        double mils = (int)((secs - (int)secs) * 1000);
        secs = secs - mils / 1000;
        double mins = (int)(secs / 60);
        secs = (int)(secs - mins * 60);

        p += "ELAPSED TIME: " + lexical_cast<string>(mins) + ":" + lexical_cast<string>(secs) + "." +
             lexical_cast<string>(mils) + "\n";
    }
    else
    {
        p += "STATUS: " + getRaceStateName(race_state) + "\n";

        double current = ros::Time::now().toSec();
        double secs = (ros::Duration(current) - ros::Duration(start_time)).toSec();
        double mils = (int)((secs - (int)secs) * 1000);
        secs = secs - mils / 1000;
        double mins = (int)(secs / 60);
        secs = (int)(secs - mins * 60);

        p += "ELAPSED TIME: " + lexical_cast<string>(mins) + ":" + lexical_cast<string>(secs) + "." +
             lexical_cast<string>(mils) + "\n";
    }

    p += "ROUTE: ";

    for (int i = 0; i < route.size(); i++)
    {
        if (i == 0)
            p += lexical_cast<string>(route[i]);
        else
            p += " - " + lexical_cast<string>(route[i]);
    }

    p += "\nNEXT ROUTE: ";

    for (int i = 0; i < next_route.size(); i++)
    {
        if (i == 0)
            p += lexical_cast<string>(next_route[i]);
        else
            p += " - " + lexical_cast<string>(next_route[i]);
    }

    p += "\nDRIVING PENALTIES: " + lexical_cast<string>(driving_penalties);

    if (race_state == FINISHED)
    {
        p += "\nPARKING PENALTIES: " + lexical_cast<string>(parking_penalties);
        p += "\n\nDRIVING SCORE: " + lexical_cast<string>(driving_score);
        p += "\nPARKING SCORE: " + lexical_cast<string>(parking_score);
    }

    cout << p << endl;
};