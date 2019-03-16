#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

nav_msgs::Odometry odom;
sensor_msgs::LaserScan scan;
#define D_MAX 10
double d_front = D_MAX;
double d_back = D_MAX;
double d_left = D_MAX;
double d_right = D_MAX;

#define GRAPH_NODES 12

typedef void (*t_step_func)(geometry_msgs::Twist::Ptr);
bool graph_init = false;
int graph_weight[GRAPH_NODES][GRAPH_NODES];
t_step_func graph_func[GRAPH_NODES][GRAPH_NODES];

int start_position = 0;
int finish_position = GRAPH_NODES - 1;

struct RouteBuilder;

struct Route {
    int id;
    int len;
    int count;
    bool complete;
    int steps[255];

    bool nodeExists(int node);
    bool process(int to, RouteBuilder *builder);
    void add(int step);
};

struct RouteBuilder {
    int count;
    int opt;
    Route routes[255];
    void build(int fromNode, int toNode);
};

bool Route::nodeExists(int node) {
    for(int i = 0; i < count; i++) {
        if(steps[i] == node) {
            return true;
        }
    }
    return false;
}

bool Route::process(int to, RouteBuilder *builder) {
    if(steps[count - 1] == to) {
        complete = true;
        return false;
    }
    bool add_self = true;
    bool res = false;
    int save_len = len;
    int cs = steps[count - 1];
    for(int i = 0; i < GRAPH_NODES; i++) {
        if(graph_weight[i][cs] > 0) {
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

void Route::add(int step) {
    if(count > 0) {
        len += graph_weight[steps[count - 1]][step];
    }
    steps[count] = step;
    count++;
}

void RouteBuilder::build(int fromNode, int toNode) {
    ROS_INFO("Building route %d -> %d", fromNode, toNode);
    for(int i = 0; i <  count; i++) {
        routes[i].id = 0;
        routes[i].len = 0;
        routes[i].count = 0;
        routes[i].complete = false;
        for(int j = 0; j < 255; j++) {
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
    opt = -1;
    int min = 1000;
    for(int i = 0; i < count; i++) {
        if(routes[i].complete && routes[i].len < min) {
            opt = i;
            min = routes[i].len;
        }
    }
    if(opt >= 0) {
        ROS_INFO("Found optimal route %d. Length: %d. Items:", opt, min);
        for(int i = 0; i < routes[opt].count; i++)
            ROS_INFO("%d", routes[opt].steps[i]);
    }
}

RouteBuilder builder;

void move_fwd(geometry_msgs::Twist::Ptr cmd) {
    double lr = d_left - d_right;
    if(lr > 0.3) lr = 0.1;
    if(lr < -0.3) lr = -0.1;
    cmd->angular.z = lr;
    cmd->linear.x = 0.3;
}

void step0_1(geometry_msgs::Twist::Ptr cmd) {
    if(d_back < D_MAX) {
        ROS_INFO("Back: %f", d_back);
        if(d_back < 1.25) {
            move_fwd(cmd);
        } else {
            start_position = 1;
        }
    }
}

void step1_3(geometry_msgs::Twist::Ptr cmd) {

}

void initGraph() {
    graph_weight[0][1] = 2;
    graph_func[0][1] = &step0_1;
    graph_weight[1][2] = 3;
    graph_weight[1][3] = 4;
    graph_func[1][3] = &step1_3;

    graph_weight[2][4] = 4;
    graph_weight[3][4] = 3;
    graph_weight[3][5] = 4;
    graph_weight[5][6] = 5;
    graph_weight[6][7] = 4;
    graph_weight[6][9] = 3;
    graph_weight[7][8] = 3;
    graph_weight[8][10] = 2;
    graph_weight[9][10] = 2;
    graph_weight[10][11] = 3;

    for(int i = 0; i < GRAPH_NODES; i++) {
        for(int j = 0; j < GRAPH_NODES; j++) {
            if(graph_weight[i][j] > 0 && graph_weight[j][i] == 0)
                graph_weight[j][i] = graph_weight[i][j];
            if(graph_weight[j][i] > 0 && graph_weight[i][j] == 0)
                graph_weight[i][j] = graph_weight[j][i];
        }
    }
}

void odometryCallback(const nav_msgs::Odometry &msg)
{
    odom = msg;
}

void getDist(double &d, int f, int t) {
    for(int i = f; i < t; i++)
        if(scan.ranges[i] > 0 && d > scan.ranges[i])
            d = scan.ranges[i];
}

void laserCallback(const sensor_msgs::LaserScan &msg)
{
    scan = msg;
    d_front = D_MAX;
    d_back = D_MAX;
    d_left = D_MAX;
    d_right = D_MAX;
    getDist(d_front, 0, 10);
    getDist(d_front, 350, 359);
    getDist(d_left, 80, 100);
    getDist(d_back, 170, 190);
    getDist(d_right, 260, 280);
}

void process(geometry_msgs::Twist::Ptr cmd);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kostyl_node");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("odom", 10, odometryCallback);
    ros::Subscriber sub2 = nh.subscribe("scan", 10, laserCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate r(50);
    initGraph();

    nh.param("start_node",  start_position,  start_position);
    nh.param("finish_node", finish_position, finish_position);
    builder.build(start_position, finish_position);

    geometry_msgs::Twist::Ptr twist(new geometry_msgs::Twist);
    while(ros::ok()){
        ros::spinOnce();
        twist->angular.z = 0;
        twist->linear.x = 0;
        process(twist);
        pub.publish(twist);
        ROS_INFO("ODOM X: %0.2f; Y: %0.2f; Z: %0.2f; W: %0.2f; F: %0.2f; L: %0.2f; B: %0.2f; R: %0.2f;",
                 odom.pose.pose.position.x,
                 odom.pose.pose.position.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w,
                 d_front,
                 d_left,
                 d_back,
                 d_right);

        r.sleep();
    }
    return 0;
}

void process(geometry_msgs::Twist::Ptr cmd) {
    if(builder.opt >= 0) {
        Route r = builder.routes[builder.opt];
        if(r.count > 1) {
            t_step_func f = graph_func[r.steps[0]][r.steps[1]];
            int pp = start_position;
            if(f) {
                f(cmd);
                if(start_position != pp) {
                    builder.build(start_position, finish_position);
                }
            }
        }
    }
}
