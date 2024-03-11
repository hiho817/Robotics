#include "ros/ros.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <string>
#include <iostream>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawnturtles");
    ros::NodeHandle n;

    // Check if the service is on
    ros::service::waitForService("/turtlesim/spawn");

    // initialize kill turtle service
    ros::ServiceClient client_kill=n.serviceClient<turtlesim::Kill>("/turtlesim/kill");
    turtlesim::Kill kill_name;
    // define parameters in service
    kill_name.request.name = "turtle1";
    // call the service
    client_kill.call(kill_name); 


    // initialize spawn turtle service
    ros::ServiceClient client_spawn = n.serviceClient<turtlesim::Spawn>("/turtlesim/spawn");
    turtlesim::Spawn turtle;

    // Finish your code, you will need to spawn turtles
    for(int i=1; i<5; i++)
    {
        turtle.request.x = i;
        turtle.request.y = i;
        turtle.request.name = "turtle_new" + std::to_string(i);

        client_spawn.call(turtle);
    }


    return 0;

}