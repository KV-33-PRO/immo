#include "ros/ros.h"
#include <math.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
//#include "odometry/SetPose.h"

nav_msgs::Odometry odom;
sensor_msgs::LaserScan scan;
ros::ServiceClient set_pose_client;

#define D_MAX 10
double d_front = D_MAX;
double d_back = D_MAX;
double d_left = D_MAX;
double d_right = D_MAX;

#define GRAPH_MAX 255

typedef void (*t_step_func)(geometry_msgs::Twist::Ptr);
bool graph_init = false;

int current_position = -1;
int finish_position = -1;
double max_covariance = 0;
double max_rudder = 0.4;
double min_speed = 0.2;
double max_speed = 0.4;
double min_front = 0.6;

ros::Time start_time;
bool started = false;

std::string odom_topic = "odom";
std::string scan_topic = "scan";
std::string graph_topic = "graph";
std::string target_topic = "targetpose";
std::string pose_topic = "correctpose";
std::string pose_frame_id = "odom";
std::string cmd_topic = "cmd_vel";

enum GraphNodeKind {NONE, NODE, MARKER};

struct GraphNode {
    GraphNodeKind kind;
    double x;
    double y;
    int m1;
    int m2;
    int m3;
};

struct GraphLink
{
    uint8_t from;
    uint8_t to;
    double  len;
    int  orient;
    double orient_x;
    double orient_y;
    double orient_a;
    double orient_d;
    int orient_k;
    int marker_a;
};

struct Graph {
    GraphNode nodes[GRAPH_MAX];
    GraphLink links[GRAPH_MAX];
    GraphNode markers[GRAPH_MAX];
    uint8_t map[GRAPH_MAX][GRAPH_MAX];
};

Graph graph;

struct RouteBuilder;
struct Route {
    int id;
    double len;
    uint8_t count;
    bool complete;
    uint8_t steps[GRAPH_MAX];

    bool nodeExists(uint8_t node);
    bool process(uint8_t to, RouteBuilder *builder);
    void add(uint8_t step);
};

struct RouteBuilder {
    int count;
    int opt;
    bool complete;
    Route routes[4096];
    void build(uint8_t fromNode, uint8_t toNode);
};

bool Route::nodeExists(uint8_t node) {
    for(int i = 0; i < count; i++) {
        if(steps[i] == node) {
            return true;
        }
    }
    return false;
}

bool Route::process(uint8_t to, RouteBuilder *builder) {
    if(steps[count - 1] == to) {
        complete = true;
        return false;
    }
    bool add_self = true;
    bool res = false;
    int save_len = len;
    int cs = steps[count - 1];
    for(int i = 0; i < GRAPH_MAX; i++) {
        if(graph.nodes[i].kind != NONE && graph.map[cs][i] > 0) {
            if(!nodeExists(i)) {
                if(add_self) {
                    add(i);
                    res = true;
                    add_self = false;
                } else {
                    builder->routes[builder->count] = builder->routes[id];
                    builder->routes[builder->count].id = builder->count;
                    builder->routes[builder->count].count--;
                    builder->routes[builder->count].len = save_len;
                    builder->routes[builder->count].add(i);
                    builder->count++;
                    res = true;
                }
            }
        }
    }
    return res;
}

void Route::add(uint8_t step) {
    if(count > 0) {
        uint8_t link_id = graph.map[steps[count - 1]][step];
        if(link_id > 0)
            len += graph.links[link_id].len;
    }
    steps[count] = step;
    //ROS_INFO("Graph %d add step %d", id, step);
    count++;
}

void RouteBuilder::build(uint8_t fromNode, uint8_t toNode) {
    opt = -1;
    complete = false;
    if(fromNode == toNode)
        return;

    ROS_INFO("Building route %d -> %d", fromNode, toNode);
    for(int i = 0; i <  count; i++) {
        routes[i].id = 0;
        routes[i].len = 0;
        routes[i].count = 0;
        routes[i].complete = false;
        for(int j = 0; j < GRAPH_MAX; j++) {
            routes[i].steps[j] = 0;
        }
    }
    count = 1;
    routes[0].id = 0;
    routes[0].add(fromNode);
    bool done = false;
    while(!done) {
        done = true;
        for(int i = 0; i < count; i++) {
            if(routes[i].process(toNode, this)) {
                done = false;
            }
        }
    }
    complete = false;
    int id = -1;
    double min = 1000;
    for(int i = 0; i < count; i++) {
        if(routes[i].complete && routes[i].len < min) {
            id = i;
            min = routes[i].len;
        }
    }

    if(id >= 0) {
        opt = id;
        complete = true;
        ROS_INFO("Found optimal route %d. Length: %0.2f. Items:", opt, min);
        for(int i = 0; i < routes[opt].count; i++)
            ROS_INFO("%d", routes[opt].steps[i]);
    }
}

RouteBuilder builder;

void initGraph(ros::NodeHandle &nh) {
    int max_node = 0;
    for(int i = 0; i < GRAPH_MAX; i++){
        graph.nodes[i].kind = NONE;
        graph.nodes[i].m1 = -1;
        graph.nodes[i].m2 = -1;
        graph.nodes[i].m3 = -1;
        std::stringstream ss;
        ss << "node" << i << "/";
        if(nh.getParam(ss.str() + "x", graph.nodes[i].x) &&
           nh.getParam(ss.str() + "y", graph.nodes[i].y)) {
            graph.nodes[i].kind = NODE;
            max_node = i;
            nh.getParam(ss.str() + "m1", graph.nodes[i].m1);
            nh.getParam(ss.str() + "m2", graph.nodes[i].m2);
            nh.getParam(ss.str() + "m3", graph.nodes[i].m3);
        }
    }
    for(int i = 0; i < GRAPH_MAX; i++){
        graph.markers[i].kind = NONE;
        std::stringstream ss;
        ss << "marker" << i << "/";
        if(nh.getParam(ss.str() + "x", graph.markers[i].x) &&
           nh.getParam(ss.str() + "y", graph.markers[i].y)) {
            ROS_INFO("%s", ss.str().c_str());
            graph.markers[i].kind = MARKER;
        }
    }
    for(int i = 0; i < GRAPH_MAX; i++){
        graph.links[i].from = 0;
        graph.links[i].to = 0;
    }
    for(int i = 0; i <= GRAPH_MAX; i++){
        for(int j = 0; j <= GRAPH_MAX; j++){
            graph.map[i][j] = 0;
        }
    }

    uint8_t link = 1;
    for(int i = 0; (i <= max_node) && (link < GRAPH_MAX); i++){
        for(int j = 0; (j <= max_node) && (link < GRAPH_MAX); j++){
            std::stringstream ss;
            ss << "link" << i << "_" << j << "/";
            int z = 0;
            if(nh.getParam(ss.str() + "enabled", z) && z > 0) {
                graph.links[link].from = i;
                graph.links[link].to = j;
                graph.links[link].orient = 0;
                graph.links[link].orient_x = 0;
                graph.links[link].orient_y = 0;
                graph.links[link].orient_k = 0;
                graph.links[link].orient_a = 0;
                graph.links[link].orient_d = 0;
                graph.links[link].marker_a = 0;
                nh.getParam(ss.str() + "orient", graph.links[link].orient);
                nh.getParam(ss.str() + "orient_x", graph.links[link].orient_x);
                nh.getParam(ss.str() + "orient_y", graph.links[link].orient_y);
                nh.getParam(ss.str() + "orient_k", graph.links[link].orient_k);
                nh.getParam(ss.str() + "orient_a", graph.links[link].orient_a);
                nh.getParam(ss.str() + "orient_d", graph.links[link].orient_d);
                nh.getParam(ss.str() + "marker_a", graph.links[link].marker_a);
                double dx = graph.nodes[graph.links[link].from].x - graph.nodes[graph.links[link].to].x;
                double dy = graph.nodes[graph.links[link].from].y - graph.nodes[graph.links[link].to].y;
                graph.links[link].len = sqrt(dx*dx + dy*dy);
                graph.map[i][j] = link;
                link++;
            }
        }
    }
    ROS_INFO("Route link count: %d", link);
    int z;
    if(nh.getParam("start", z)) {
        if(z >= -1 && z < GRAPH_MAX && graph.nodes[z].kind != NONE)  {
            current_position = z;
        }
    }
    if(nh.getParam("finish", z)) {
        if(z >= -1 && z < GRAPH_MAX && graph.nodes[z].kind != NONE)  {
            finish_position = z;
        }
    }
    nh.getParam("max_covariance", max_covariance);
}

void drawGraph(ros::Publisher &pub) {
    static ros::Time prev_time;

    ros::Duration dt = ros::Time::now() - prev_time;
    if(dt.toSec() < 1)
        return;

    prev_time = ros::Time::now();

    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker marker;
    marker.id = 0;
    for(int i = 0; i < GRAPH_MAX; i++) {
        if(graph.nodes[i].kind != NONE) {
            marker.id++;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "node";
            marker.type = marker.SPHERE;
            marker.pose.position.x = graph.nodes[i].x;
            marker.pose.position.y = graph.nodes[i].y;
            marker.pose.position.z = 0;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 0.3;
            marker.color.r = (i == current_position) ? 1 : 0;
            marker.color.g = (i == finish_position) ? 1 : 0;
            marker.color.b = (i != current_position && i != finish_position) ? 1 : 0;
            marker.text = "";
            arr.markers.push_back(marker);

            marker.id++;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "node";
            marker.type = marker.TEXT_VIEW_FACING;
            marker.pose.position.x = graph.nodes[i].x;
            marker.pose.position.y = graph.nodes[i].y;
            marker.pose.position.z = 0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 0;
            std::stringstream ss;
            ss << i;
            marker.text = ss.str();
            arr.markers.push_back(marker);
        }
    }
    for(int i = 0; i < GRAPH_MAX; i++) {
        if(graph.markers[i].kind != NONE) {
            marker.id++;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "marker";
            marker.type = marker.SPHERE;
            marker.pose.position.x = graph.markers[i].x;
            marker.pose.position.y = graph.markers[i].y;
            marker.pose.position.z = 0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 0.6;
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.text = "";
            arr.markers.push_back(marker);

            marker.id++;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "marker";
            marker.type = marker.TEXT_VIEW_FACING;
            marker.pose.position.x = graph.markers[i].x;
            marker.pose.position.y = graph.markers[i].y;
            marker.pose.position.z = 0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 0;
            std::stringstream ss;
            ss << i;
            marker.text = ss.str();
            arr.markers.push_back(marker);
        }
    }
    marker.points.resize(2);
    for(int i = 0; i < GRAPH_MAX; i++) {
        if(graph.links[i].from != graph.links[i].to) {
            marker.id++;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "link";
            marker.type = marker.ARROW;
            marker.scale.x = 0.08;
            marker.scale.y = 0.2;
            marker.scale.z = 0.3;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 0.3;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.text = "";
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0;
            marker.points[0].x = graph.nodes[graph.links[i].from].x;
            marker.points[0].y = graph.nodes[graph.links[i].from].y;
            marker.points[0].z = 0;
            marker.points[1].x = graph.nodes[graph.links[i].to].x;
            marker.points[1].y = graph.nodes[graph.links[i].to].y;
            marker.points[1].z = 0;
            arr.markers.push_back(marker);
        }
    }
    if(builder.opt >= 0) {
        Route r = builder.routes[builder.opt];
        for(int i = 0; i < r.count - 1; i++) {
            marker.id++;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "route";
            marker.type = marker.ARROW;
            marker.scale.x = 0.1;
            marker.scale.y = 0.2;
            marker.scale.z = 0.3;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 0.5;
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
            marker.text = "";
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0;
            marker.points[0].x = graph.nodes[r.steps[i]].x;
            marker.points[0].y = graph.nodes[r.steps[i]].y;
            marker.points[0].z = 0;
            marker.points[1].x = graph.nodes[r.steps[i + 1]].x;
            marker.points[1].y = graph.nodes[r.steps[i + 1]].y;
            marker.points[1].z = 0;
            arr.markers.push_back(marker);
        }
    }
    pub.publish(arr);
}

void odometryCallback(const nav_msgs::Odometry &msg)
{
    odom = msg;
}

void laserCallback(const sensor_msgs::LaserScan &msg)
{
    scan = msg;
}

void buttonCallback(const std_msgs::Bool &msg)
{
    if(msg.data && !started) {
        ROS_INFO("START signal received!");
        started = true;
        start_time = ros::Time::now();
    }
    if(!msg.data && started) {
        ROS_INFO("STOP signal received!");
        started = false;
    }
}

bool process(geometry_msgs::PoseStamped::Ptr target, geometry_msgs::Twist::Ptr cmd, geometry_msgs::Pose2D::Ptr corr_pose);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_route_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ph.param("odom_topic", odom_topic, odom_topic);
    ph.param("scan_topic", scan_topic, scan_topic);
    ph.param("graph_topic", graph_topic, graph_topic);
    ph.param("target_topic", target_topic, target_topic);
    ph.param("pose_topic", pose_topic, pose_topic);
    ph.param("pose_frame_id", pose_frame_id, pose_frame_id);
    ph.param("cmd_topic", cmd_topic, cmd_topic);
    ph.param("max_rudder", max_rudder, max_rudder);
    ph.param("min_speed", min_speed, min_speed);
    ph.param("max_speed", max_speed, max_speed);
    ph.param("min_front", min_front, min_front);

    ros::Subscriber sub1 = nh.subscribe(odom_topic, 10, odometryCallback);
    ros::Subscriber sub2 = nh.subscribe(scan_topic, 10, laserCallback);
    ros::Subscriber sub3 = nh.subscribe("robot_button", 10, buttonCallback);
    ros::Publisher trg_pub = nh.advertise<geometry_msgs::PoseStamped>(target_topic, 10);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Pose2D>(pose_topic, 10);
    ros::Publisher arr_pub = nh.advertise<visualization_msgs::MarkerArray>(graph_topic, 1);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_topic, 10);
    initGraph(ph);
    ros::Rate r(20);
    geometry_msgs::PoseStamped::Ptr target(new geometry_msgs::PoseStamped);
    geometry_msgs::Pose2D::Ptr corr_pose(new geometry_msgs::Pose2D);
    geometry_msgs::Twist::Ptr cmd_vel(new geometry_msgs::Twist);
    target->header.frame_id = pose_frame_id;
    ros::Time correctPoseTime = ros::Time::now();
    while(ros::ok()){
        ros::spinOnce();
        drawGraph(arr_pub);
        cmd_vel->angular.z = 0;
        cmd_vel->linear.x = 0;
        corr_pose->x = 0;
        corr_pose->y = 0;
        corr_pose->theta = 0;
        if(process(target, cmd_vel, corr_pose)) {
            trg_pub.publish(target);
            cmd_pub.publish(cmd_vel);
            if(corr_pose->x != 0 || corr_pose->y != 0 || corr_pose->theta != 0){
                ros::Duration dt = ros::Time::now() - correctPoseTime;
                if(dt.toSec() > 0.5) {
                    pos_pub.publish(corr_pose);
                    correctPoseTime = ros::Time::now();
                }
            }
        }
        r.sleep();
    }
    return 0;
}

double getRange(uint8_t a, uint8_t b) {
    return scan.ranges[a];
    double sum = 0.0;
    for(int i = a - b; i < a + b; i++) {
        sum += scan.ranges[(i < 0) ? (360 - i) : ((i >= 360) ? i - 360 : i)];
    }
    return sum/(2*b + 1);
}

void getDist(double &d, int f, int t) {
    for(int i = f; i < t; i++)
        if(scan.ranges[i] > 0 && d > scan.ranges[i])
            d = scan.ranges[i];
}

bool checkFrontObstacle() {
    double fr = 10;
    getDist(fr, 0, 20);
    getDist(fr, 340, 360);
    if(fr < min_front){
        ROS_WARN("Front obstacle detected! D: %0.2f", fr);
        return false;
    } else {
        return true;
    }
}

bool checkStarted() {
    ros::Duration scan_dt = ros::Time::now() - scan.header.stamp;
    ros::Duration odom_dt = ros::Time::now() - odom.header.stamp;
    ros::Duration start_dt = ros::Time::now() - start_time;
    if(!started)
        return false;
    if(start_dt.toSec() < 5) {
        ROS_INFO("Wait %0.2f sec.", start_dt.toSec());
        return false;
    }
    if(start_dt.toSec() > 240) {
        ROS_INFO("Out of time!");
        return false;
    }
    if(scan_dt.toSec() > 0.2) {
        ROS_WARN("Scan info is too old (%0.2f sec)", scan_dt.toSec());
        return false;
    }
    if(odom_dt.toSec() > 0.2) {
        ROS_WARN("Odom info is too old (%0.2f sec)", odom_dt.toSec());
        return false;
    }
    return  true;
}



bool checkMarker(int dir) {
    double delta = 0.5;
    int ray_delta =  5;
    int ray_from = dir - ray_delta;
    int ray_to = dir + ray_delta;
    double l1 = scan.ranges[ray_from];
    double l2 = scan.ranges[dir];
    double l3 = scan.ranges[ray_to];
    double c1 = l1 * cos(ray_from * M_PI/180);
    double c2 = l2 * cos(dir * M_PI/180);
    double c3 = l3 * cos(ray_to * M_PI/180);
    double s1 = l1 * sin(ray_from * M_PI/180);
    double s2 = l2 * sin(dir * M_PI/180);
    double s3 = l3 * sin(ray_to * M_PI/180);
    double cd = c2 - (c3 + (c1-c3)/2.0);
    double sd = s2 - (s3 + (s1-s3)/2.0);
    if(std::abs(cd) > delta || std::abs(sd) > delta) {
        return true;
    }
    return false;
}

bool checkWall(double &wall_a, double &wall_d, int dir) {
    double delta = 0.15;
    int ray_delta =  10;
    int ray_from = dir - ray_delta;
    int ray_to = dir + ray_delta;
    double l1 = scan.ranges[ray_from];
    double l2 = scan.ranges[dir];
    double l3 = scan.ranges[ray_to];
    double c1 = l1 * cos(ray_from * M_PI/180);
    double c2 = l2 * cos(dir * M_PI/180);
    double c3 = l3 * cos(ray_to * M_PI/180);
    double s1 = l1 * sin(ray_from * M_PI/180);
    double s2 = l2 * sin(dir * M_PI/180);
    double s3 = l3 * sin(ray_to * M_PI/180);
    double cd = c2 - (c3 + (c1-c3)/2.0);
    double sd = s2 - (s3 + (s1-s3)/2.0);
    if(std::abs(cd) < delta && std::abs(sd) < delta) {
        wall_a = -atan((s3 - s1)/(c3 - c1));
        ray_from = dir - wall_a * 180/M_PI;
        if(ray_from >= 360) ray_from = ray_from - 360;
        if(ray_from < 0) ray_from = ray_from + 360;
        wall_d = scan.ranges[ray_from];
        return true;
    }
    return false;
}

void createCmd(geometry_msgs::Twist::Ptr cmd) {
    if(checkStarted() && checkFrontObstacle()) {
        GraphNode node0 = graph.nodes[builder.routes[builder.opt].steps[0]];
        GraphNode node1 = graph.nodes[builder.routes[builder.opt].steps[1]];
        GraphLink link = graph.links[graph.map[builder.routes[builder.opt].steps[0]][builder.routes[builder.opt].steps[1]]];
        double yaw = 0;
        if(link.orient > 0 && link.orient_d > 0) {
            double wall_a, wall_d;
            if(checkWall(wall_a, wall_d, link.orient)) {
                ROS_INFO("Wall at %d detected. A: %0.2f, D: %0.2f", link.orient, wall_a, wall_d);
                if(std::abs(wall_d - link.orient_d) > 0.1 * link.orient_d) {
                    ROS_INFO("Control by wall distance %0.2f", wall_d);
                    yaw = 0.7 * ((180 - link.orient) > 0 ? 1 : -1) * (wall_d - link.orient_d)/link.orient_d;
                } else {
                    ROS_INFO("Control by wall angle %0.2f", wall_a);
                    yaw = -wall_a * 1.5;
                }
            }  else {
                ROS_WARN("Cannot detect wall at %d deg.", link.orient);
                return;
            }
        } else {
            double target_yaw = atan2(node1.y - odom.pose.pose.position.y, node1.x - odom.pose.pose.position.x);
            double robot_yaw = tf::getYaw(odom.pose.pose.orientation);
            yaw = target_yaw - robot_yaw;
            if(yaw > M_PI)  yaw = yaw - 2*M_PI;
            if(yaw < -M_PI) yaw = yaw + 2*M_PI;
            ROS_INFO("Control by target angle %0.2f", yaw);
            if(std::abs(yaw) > 1.7) {
                ROS_WARN("Angle between robot and target is too large (%0.2f)", yaw);
            }
        }
        if(yaw > max_rudder) yaw = max_rudder;
        if(yaw < -max_rudder) yaw = -max_rudder;
        cmd->angular.z = yaw;
        cmd->linear.x = max_speed * (max_rudder - abs(yaw))/max_rudder;
        cmd->linear.x = (cmd->linear.x < min_speed) ? min_speed : cmd->linear.x;
        ROS_INFO("Yaw: %0.2f; Speed: %0.2f", yaw, cmd->linear.x);
    }
}

bool process(geometry_msgs::PoseStamped::Ptr target, geometry_msgs::Twist::Ptr cmd, geometry_msgs::Pose2D::Ptr corr_pose) {
    if(current_position < 0) {
        ROS_INFO("Find robot current position in graph...");
        double min_d = 1000;
        int min_i = -1;
        for(int i = 0; i < GRAPH_MAX; i++) {
            if(graph.nodes[i].kind != NONE) {
                double dx = odom.pose.pose.position.x - graph.nodes[i].x;
                double dy = odom.pose.pose.position.y - graph.nodes[i].y;
                double d = sqrt(dx*dx + dy*dy);
                ROS_INFO("Node %d,  Distance: %0.2f", i, d);
                if(d < min_d) {
                    min_d = d;
                    min_i = i;
                }
            }
        }
        if(min_i >= 0) {
            ROS_INFO("Current position: Node%d=[%0.2f, %0.2f], Distance: %0.2f", min_i, graph.nodes[min_i].x, graph.nodes[min_i].y, min_d);
            current_position = min_i;
        }
    }

    if(current_position < 0 || finish_position < 0) {
        return false;
    }

    if(builder.complete && builder.routes[builder.opt].steps[0] == current_position && builder.routes[builder.opt].count > 1) {
        GraphLink link = graph.links[graph.map[builder.routes[builder.opt].steps[0]][builder.routes[builder.opt].steps[1]]];
        bool step_finished = false;
        if(link.marker_a != 0) {
            checkMarker(link.marker_a);
            step_finished = true;
            ROS_INFO("Found marker at %d. Step finished.", link.marker_a);
        } else {
            double dx = odom.pose.pose.position.x - graph.nodes[builder.routes[builder.opt].steps[1]].x;
            double dy = odom.pose.pose.position.y - graph.nodes[builder.routes[builder.opt].steps[1]].y;
            double d = sqrt(dx*dx + dy*dy);
            if(d <= max_covariance) {
                step_finished = true;
                ROS_INFO("Step finished by odometry data.");
            }
        }
        if(step_finished) {
            if(link.orient > 0 && link.orient_d > 0 && (link.orient_x > 0 || link.orient_y > 0)) {
                double wall_a, wall_d;
                ROS_INFO("Correcting position by wall at %d", link.orient);
                if(checkWall(wall_a, wall_d, link.orient)) {
                    ROS_INFO("Wall detected. A: %0.2f, D: %0.2f", wall_a, wall_d);
                    if(link.orient_x > 0) {
                        corr_pose->x = link.orient_x + ((double)link.orient_k * wall_d);
                        corr_pose->y = odom.pose.pose.position.y;
                        if(link.orient_a != 0) {
                            corr_pose->theta = link.orient_a + wall_a;
                        } else {
                            corr_pose->theta = tf::getYaw(odom.pose.pose.orientation);
                        }
                    } else if(link.orient_y > 0) {
                        corr_pose->y = link.orient_y + ((double)link.orient_k * wall_d);
                        corr_pose->x = odom.pose.pose.position.x;
                        if(link.orient_a != 0) {
                            corr_pose->theta = link.orient_a + wall_a;
                        } else {
                            corr_pose->theta = tf::getYaw(odom.pose.pose.orientation);
                        }
                    }
                }
            }
            current_position = builder.routes[builder.opt].steps[1];
        }
    }

    if(!builder.complete || builder.routes[builder.opt].steps[0] != current_position) {
        if(current_position == finish_position) {
            ROS_INFO("Finished successfully!!!");
            finish_position = -1;
            return true;
        } else {
            builder.build(current_position, finish_position);
        }
    }

    if(builder.complete && builder.routes[builder.opt].count > 1) {
        target->header.stamp = ros::Time::now();
        target->header.seq++;
        GraphNode node0 = graph.nodes[builder.routes[builder.opt].steps[0]];
        GraphNode node1 = graph.nodes[builder.routes[builder.opt].steps[1]];
        target->pose.position.x = node1.x;
        target->pose.position.y = node1.y;
        double yaw = atan2(node1.y - node0.y, node1.x - node0.x);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), target->pose.orientation);
        createCmd(cmd);
        return true;
    }

    return false;
}
