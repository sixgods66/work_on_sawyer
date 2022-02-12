#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_collasion");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle;

    static const std::string PLANNING_GROUP = "right_arm";

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);

    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    /* method 1 */
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "base";
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "base";
    /* The id of the object */
    attached_object.object.id = "table";

    /* 设置table的信息 */
    /* A default pose */
    geometry_msgs::Pose pose;
    pose.position.x = 0.4;
    pose.position.y = 0.0;
    pose.position.z = -0.03;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.20;
    primitive.dimensions[1] = 2;
    primitive.dimensions[2] = 0.05;

    geometry_msgs::Pose boardpose;
    pose.position.x = 0.12;
    pose.position.y = -0.62;
    pose.position.z = 0.25;
    pose.orientation.x = 0.10602;
    pose.orientation.y = 0.0106104;
    pose.orientation.z = 0.706746;
    pose.orientation.w = 0.707309;

    shape_msgs::SolidPrimitive board;
    board.type = board.BOX;
    board.dimensions.resize(3);
    board.dimensions[0] = 0.02;
    board.dimensions[1] = 1.0;
    board.dimensions[2] = 0.5;

    
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    ros::ServiceClient planning_scene_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);

    moveit_msgs::CollisionObject remove_object;
    remove_object.id = "table";
    remove_object.header.frame_id = "base";
    remove_object.operation = remove_object.REMOVE;

    ROS_INFO("Attaching the object to the hand and removing it from the world.");
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    planning_scene_interface.applyPlanningScene(planning_scene);


    // moveit_msgs::CollisionObject collision_object;
    // collision_object.header.frame_id = move_group.getPlanningFrame();
    // collision_object.id = "table";

    // geometry_msgs::Pose pose;
    // pose.position.x = 0.4;
    // pose.position.y = 0.0;
    // pose.position.z = -0.03;
    // pose.orientation.w = 1.0;

    // /* Define a box to be attached */
    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 1.20;
    // primitive.dimensions[1] = 2;
    // primitive.dimensions[2] = 0.05;

    // collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(pose);
    // collision_object.operation = collision_object.ADD;

    // planning_scene_interface.addCollisionObjects(collision_object);
    // move_group.setStartState(*move_group.getCurrentState());
}