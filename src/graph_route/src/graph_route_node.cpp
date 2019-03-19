#include "ros/ros.h"
#include <math.h>
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
std::string graph_topic = "graph";
std::string pose_topic = "target_pose";
std::string pose_frame_id = "odom";

enum GraphNodeKind {NONE, NODE, START, FINISH};

struct GraphNode {
    GraphNodeKind kind;
    double x;
    double y;
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
        std::stringstream ss;
        ss << "node" << i << "/";
        if(nh.getParam(ss.str() + "x", graph.nodes[i].x) &&
           nh.getParam(ss.str() + "y", graph.nodes[i].y)) {
            graph.nodes[i].kind = NODE;
            max_node = i;
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
            graph.nodes[z].kind = START;
            current_position = z;
        }
    }
    if(nh.getParam("finish", z)) {
        if(z >= -1 && z < GRAPH_MAX && graph.nodes[z].kind != NONE)  {
            graph.nodes[z].kind = FINISH;
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
    for(int i = 0; i < GRAPH_MAX; i++) {
        if(graph.nodes[i].kind != NONE) {
            marker.id = i;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "node";
            marker.type = marker.SPHERE;
            marker.pose.position.x = graph.nodes[i].x;
            marker.pose.position.y = graph.nodes[i].y;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 0.3;
            marker.color.r = (graph.nodes[i].kind == START) ? 1 : 0;
            marker.color.g = (graph.nodes[i].kind == FINISH) ? 1 : 0;
            marker.color.b = (graph.nodes[i].kind == NODE) ? 1 : 0;
            marker.text = "";
            arr.markers.push_back(marker);

            marker.id = GRAPH_MAX+i;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "text";
            marker.type = marker.TEXT_VIEW_FACING;
            marker.pose.position.x = graph.nodes[i].x;
            marker.pose.position.y = graph.nodes[i].y;
            marker.pose.position.z = 1;
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
    marker.points.resize(2);
    for(int i = 0; i < GRAPH_MAX; i++) {
        if(graph.links[i].from != graph.links[i].to) {
            marker.id = 1000 +i;
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
            marker.id = 2000 +i;
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

bool process(geometry_msgs::PoseStamped::Ptr cmd);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_route_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ph.param("odom_topic", odom_topic, odom_topic);
    ph.param("graph_topic", graph_topic, graph_topic);
    ph.param("pose_topic", pose_topic, pose_topic);
    ph.param("pose_frame_id", pose_frame_id, pose_frame_id);

    ros::Subscriber sub1 = nh.subscribe(odom_topic, 10, odometryCallback);
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

    if(!builder.complete || builder.routes[builder.opt].steps[0] != current_position)
        builder.build(current_position, finish_position);

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
