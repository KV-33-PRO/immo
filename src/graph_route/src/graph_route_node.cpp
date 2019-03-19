#include "ros/ros.h"
#include <math.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

nav_msgs::Odometry odom;
sensor_msgs::LaserScan scan;
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

std::string odom_topic = "odom";
std::string scan_topic = "scan";
std::string graph_topic = "graph";
std::string pose_topic = "target_pose";
std::string pose_frame_id = "odom";

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
    uint8_t id;
    double len;
    uint8_t count;
    bool complete;
    uint8_t steps[GRAPH_MAX];

    bool nodeExists(uint8_t node);
    bool process(uint8_t to, RouteBuilder *builder);
    void add(uint8_t step);
};

struct RouteBuilder {
    uint8_t count;
    uint8_t opt;
    bool complete;
    Route routes[GRAPH_MAX];
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
    count++;
}

void RouteBuilder::build(uint8_t fromNode, uint8_t toNode) {
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
                double dx = graph.nodes[graph.links[link].from].x - graph.nodes[graph.links[link].to].x;
                double dy = graph.nodes[graph.links[link].from].y - graph.nodes[graph.links[link].to].y;
                graph.links[link].len = sqrt(dx*dx + dy*dy);
                graph.map[i][j] = link;
                link++;
            }
        }
    }

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
    std::string odom_topic = "odom";
    std::string pose_topic = "target_pose";
    std::string pose_frame_id = "odom";

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

bool process(geometry_msgs::PoseStamped::Ptr cmd);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_route_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ph.param("odom_topic", odom_topic, odom_topic);
    ph.param("scan_topic", scan_topic, scan_topic);
    ph.param("graph_topic", graph_topic, graph_topic);
    ph.param("pose_topic", pose_topic, pose_topic);
    ph.param("pose_frame_id", pose_frame_id, pose_frame_id);

    ros::Subscriber sub1 = nh.subscribe(odom_topic, 10, odometryCallback);
    ros::Subscriber sub2 = nh.subscribe(scan_topic, 10, laserCallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
    ros::Publisher arr_pub = nh.advertise<visualization_msgs::MarkerArray>(graph_topic, 1);

    initGraph(ph);
    ros::Rate r(20);
    geometry_msgs::PoseStamped::Ptr target(new geometry_msgs::PoseStamped);
    target->header.frame_id = pose_frame_id;
    while(ros::ok()){
        ros::spinOnce();
        drawGraph(arr_pub);
        if(process(target))
            cmd_pub.publish(target);
        r.sleep();
    }
    return 0;
}

double getRange(uint8_t a, uint8_t b) {
    double sum = 0;
    for(int i = a - b; i < a + b; i++) {
        sum += scan.ranges[(i < 0) ? (360 - i) : ((i >= 360) ? i - 360 : i)];
    }
    return sum/(2*b + 1);
}

bool findMarkers(uint8_t id) {
/*
    for(int i = 10; i < 350; i += 10) {
        double l1 = getRange(i-10, 4);
        double l2 = getRange(i, 4);
        double l3 = getRange(i+10, 4);
        double c1 = l1 * cos((i-10) * M_PI/180);
        double c2 = l2 * cos((i) * M_PI/180);
        double c3 = l3 * cos((i+10) * M_PI/180);
        double s1 = l1 * sin((i-10) * M_PI/180);
        double s2 = l2 * sin((i) * M_PI/180);
        double s3 = l3 * sin((i+10) * M_PI/180);
        ROS_INFO("A = %d; L = %0.2f; COS: %0.2f; SIN: %0.2f", i-10, l1, c1, s1);
        ROS_INFO("A = %d; L = %0.2f; COS: %0.2f; SIN: %0.2f", i, l2, c2, s2);
        ROS_INFO("A = %d; L = %0.2f; COS: %0.2f; SIN: %0.2f", i+10, l3, c3, s3);
        ROS_INFO("%0.2f ~ %0.2f; %0.2f ~ %0.2f;", c2, c3 + (c1-c3)/2.0, s2, s3 + (s1-s3)/2.0);
        ROS_INFO("---------------");
    }
*/
    /*
    double prev_x = 0;
    double prev_y = 0;
    for(int i = 0; i < 360; i += 10) {
        double r = getRange(i, 4);
        double dy = r * sin(i*180/M_PI);
        double dx = r * cos(i*180/M_PI);
        ROS_INFO("%d - %0.2f, dx: %0.2f, dy: %0.2f", i, r, dx - prev_x, dy - prev_y);
        prev_x = dx;
        prev_y = dy;
    }
    */
/*
    double mx = graph.markers[id].x;
    double my = graph.markers[id].y;
    double robot_yaw = tf::getYaw(odom.pose.pose.orientation);
    double robot_x = odom.pose.pose.position.y;
    double robot_y = odom.pose.pose.position.x;

    robot_yaw += 0.2;

    double yaw = atan2(my - robot_y, mx - robot_x) - robot_yaw;
    if(yaw < 0)
        yaw = 2 * M_PI + yaw;
    double yaw_deg = yaw*180/M_PI;
    ROS_INFO("Find marker %0.2f, %0.2f, %0.2f", mx, my, yaw_deg);
    int delta = 20;
    int range_from = round(yaw_deg) - delta;
    int range_to = round(yaw_deg) + delta;
    ROS_INFO("Scan %d - %d", range_from, range_to);
    double ppx, ppy;

    for(int i = range_from; i < range_to; i++){
        double px = robot_x + getRange(i, 3)*cos(robot_yaw + i*M_PI/180);
        double py = robot_y + getRange(i, 3)*sin(robot_yaw + i*M_PI/180);
        ROS_INFO("%d - %0.2f, x: %0.2f, y: %0.2f, dx: %0.2f, dy: %0.2f", i, scan.ranges[i],
                 px,
                 py,
                 px - ppx,
                 py - ppy);
        ppx = px;
        ppy = py;
    }
    */

}

void correctPosition() {
    return;
    ros::Duration dt = ros::Time::now() - scan.header.stamp;
    if(dt.toSec() > 0.5) return;

    double delta = 0.1;
    int ray_delta =  20;
    int ray_width =  10;
    double marker_l[GRAPH_MAX];
    double marker_a[GRAPH_MAX];
    int marker_n = 0;
    bool in_marker = false;
    double max_a = 0;
    double max_d = 0;
    for(int i = 5; i < 355; i += 3) {
        double l1 = getRange(i - ray_delta, ray_width);
        double l2 = getRange(i, ray_width);
        double l3 = getRange(i + ray_delta, ray_width);
        double c1 = l1 * cos((i - ray_delta) * M_PI/180);
        double c2 = l2 * cos((i) * M_PI/180);
        double c3 = l3 * cos((i + ray_delta) * M_PI/180);
        double s1 = l1 * sin((i - ray_delta) * M_PI/180);
        double s2 = l2 * sin((i) * M_PI/180);
        double s3 = l3 * sin((i + ray_delta) * M_PI/180);
        double cd = c2 - (c3 + (c1-c3)/2.0);
        double sd = s2 - (s3 + (s1-s3)/2.0);
/*
        ROS_INFO("A = %d; L = %0.2f; COS: %0.2f; SIN: %0.2f", i-10, l1, c1, s1);
        ROS_INFO("A = %d; L = %0.2f; COS: %0.2f; SIN: %0.2f", i, l2, c2, s2);
        ROS_INFO("A = %d; L = %0.2f; COS: %0.2f; SIN: %0.2f", i+10, l3, c3, s3);
        ROS_INFO("%0.2f ~ %0.2f; %0.2f ~ %0.2f;", c2, c3 + (c1-c3)/2.0, s2, s3 + (s1-s3)/2.0);
        ROS_INFO("%0.2f; %0.2f;", std::abs(cd), std::abs(sd));
*/
        if(std::abs(cd) > delta || std::abs(sd) > delta) {
            in_marker = true;
            if(max_d < std::abs(cd) + std::abs(sd)) {
                max_d = std::abs(cd) + std::abs(sd);
                max_a = i;
            }
        } else {
            if(in_marker) {
                marker_l[marker_n] = getRange(max_a, 4);
                marker_a[marker_n] = max_a;
                ROS_INFO("Found marker L: %0.2f, A: %0.2f", marker_l[marker_n], marker_a[marker_n]);
                marker_n++;
                in_marker = false;
                max_d = 0;
                max_a = 0;
            }
        }
        //ROS_INFO("---------------");
    }


    //findMarker(graph.nodes[current_position].m1);
/*
    uint8_t markers[GRAPH_MAX];
    uint8_t markers_cnt = 0;

    for(int i = 0; i < GRAPH_MAX; i++) {
        if(graph.markers[i].kind != NONE) {
            double dx = odom.pose.pose.position.x - graph.markers[i].x;
            double dy = odom.pose.pose.position.y - graph.markers[i].y;
            double d = sqrt(dx*dx + dy*dy);

        }
    }y
    */
}

bool process(geometry_msgs::PoseStamped::Ptr target) {
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
        double dx = odom.pose.pose.position.x - graph.nodes[builder.routes[builder.opt].steps[1]].x;
        double dy = odom.pose.pose.position.y - graph.nodes[builder.routes[builder.opt].steps[1]].y;
        double d = sqrt(dx*dx + dy*dy);
        if(d <= max_covariance) {
            current_position = builder.routes[builder.opt].steps[1];
        }
    }

    if(!builder.complete || builder.routes[builder.opt].steps[0] != current_position) {
        builder.build(current_position, finish_position);
        correctPosition();
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
        return true;
    }

    return false;
}
