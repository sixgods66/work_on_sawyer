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


void pubCollasion(){
  
}


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

   
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "base";
    attached_object.object.header.frame_id = "base";
    attached_object.object.id = "collasion";

    float x_baseline = 0.28;
     /* table */
    geometry_msgs::Pose pose;
    pose.position.x = x_baseline;
    pose.position.y = 0.0;
    pose.position.z = -0.03;
    pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.20;
    primitive.dimensions[1] = 2;
    primitive.dimensions[2] = 0.05;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    /* board */
    geometry_msgs::Pose boardpose;
    boardpose.position.x = x_baseline;
    boardpose.position.y = -0.8;
    boardpose.position.z = 0.25;
    boardpose.orientation.x = 0;
    boardpose.orientation.y = 0;
    boardpose.orientation.z = 0.707;
    boardpose.orientation.w = 0.707;
    shape_msgs::SolidPrimitive board;
    board.type = board.BOX;
    board.dimensions.resize(3);
    board.dimensions[0] = 0.4;
    board.dimensions[1] = 1.0;
    board.dimensions[2] = 0.5;

    attached_object.object.primitives.push_back(board);
    attached_object.object.primitive_poses.push_back(boardpose);
    attached_object.object.operation = attached_object.object.ADD;

    /* leftpole */
    geometry_msgs::Pose leftpose;
    leftpose.position.x = x_baseline + 0.3;
    leftpose.position.y = 0.52;
    leftpose.position.z = 0.60;
    leftpose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive leftpole;
    leftpole.type = leftpole.BOX;
    leftpole.dimensions.resize(3);
    leftpole.dimensions[0] = 0.07;
    leftpole.dimensions[1] = 0.07;
    leftpole.dimensions[2] = 1.20;

    attached_object.object.primitives.push_back(leftpole);
    attached_object.object.primitive_poses.push_back(leftpose);
    attached_object.object.operation = attached_object.object.ADD;

    /* rightpole */
    geometry_msgs::Pose rightpose;
    rightpose.position.x = x_baseline + 0.3;
    rightpose.position.y = -0.52;
    rightpose.position.z = 0.60;
    rightpose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive rightpole;
    rightpole.type = rightpole.BOX;
    rightpole.dimensions.resize(3);
    rightpole.dimensions[0] = 0.10;
    rightpole.dimensions[1] = 0.10;
    rightpole.dimensions[2] = 1.20;

    attached_object.object.primitives.push_back(rightpole);
    attached_object.object.primitive_poses.push_back(rightpose);
    attached_object.object.operation = attached_object.object.ADD;

    /* uppole */
    geometry_msgs::Pose uppose;
    uppose.position.x = x_baseline + 0.3;
    uppose.position.y = 0;
    uppose.position.z = 1.03;
    uppose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive uppole;
    uppole.type = uppole.BOX;
    uppole.dimensions.resize(3);
    uppole.dimensions[0] = 0.07;
    uppole.dimensions[1] = 0.98;
    uppole.dimensions[2] = 0.07;

    attached_object.object.primitives.push_back(uppole);
    attached_object.object.primitive_poses.push_back(uppose);
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
    remove_object.id = "collasion";
    remove_object.header.frame_id = "base";
    remove_object.operation = remove_object.REMOVE;

    ROS_INFO("Attaching the object to the hand and removing it from the world.");
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    planning_scene_interface.applyPlanningScene(planning_scene);

}