#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"

void ProjectOne::setup()
{
	// Make one adult with one child
	auto* agent = agents->create_behavior_agent("Adult", BehaviorTreeTypes::Adult);
    {
        auto* chick = agents->create_behavior_agent("Kid", BehaviorTreeTypes::ChickBehavior);
        chick->set_color(Vec3(255 / 255.0f, 140.0f / 255, 0));
        chick->set_scaling(1.0f);
    }


    // Make workers    for (int i = 0; i < 5; ++i)
    {
        agent = agents->create_behavior_agent("Worker", BehaviorTreeTypes::Worker);
        agent->set_position(Vec3(50, 0, 50));
        agent->set_color(Vec3(1, 0, 0));
        agent->set_scaling(1.5f);
        agent->get_blackboard().set_value("HasJob", false);
    }

    // Make one crazy person
    agent = agents->create_behavior_agent("Crazy", BehaviorTreeTypes::CrazyMan);
    agent->set_position(Vec3(40, 0, 40));
    agent->set_color(Vec3(0.5, 0.5, 1));
    agent->set_scaling(2.0f);
    agent->get_blackboard().set_value("IsCrazy", false);

    // you can technically load any map you want, even create your own map file,
    // but behavior agents won't actually avoid walls or anything special, unless you code that yourself
    // that's the realm of project 2 though
    terrain->goto_map(0);

    // you can also enable the pathing layer and set grid square colors as you see fit
    // works best with map 0, the completely blank map
    terrain->pathLayer.set_enabled(true);

    // Worker areas
    terrain->pathLayer.set_value(0, 0, Colors::Red);
    terrain->pathLayer.set_value(19, 19, Colors::Red);
    terrain->pathLayer.set_value(0, 19, Colors::Red);
    terrain->pathLayer.set_value(19, 0, Colors::Red);
    terrain->pathLayer.set_value(10, 10, Colors::Red);

    // Adult Office
    terrain->pathLayer.set_value(10, 0, Colors::Blue);

    // camera position can be modified from this default as well
    auto camera = agents->get_camera_agent();
    camera->set_position(Vec3(-62.0f, 70.0f, terrain->mapSizeInWorld * 0.5f));
    camera->set_pitch(0.610865); // 35 degrees
}