### move_base输入和输出的数据

1. 机器人需要发布sensor_msgs/LaserScan或者sensor_msgs/PointCloud格式的消息，对应二维激光或三维点云

2. 机器人需要发布nav_msgs/Odometry格式的里程计信息，同时发布相应的TF变换

3. 导航功能输出geometry/Twist格式的控制命令

### move_base功能包的各种接口

##### 1. 话题和服务

---

move_base/goal

move_base_msgs::MoveBaseActionGoal

move_base的运动规划目标

```
// move_base.cpp
ros::NodeHandle action_nh("move_base");
action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
```
需要注意的是，上面的消息是move_base内部发送goal使用的。

==整个move_base订阅下面的话题来接受goal==

```
// move_base.cpp，MoveBase构造函数
ros::NodeHandle simple_nh("move_base_simple");
goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));
```
这里绑定了`goalCB`回调函数，将`geometry_msgs::PoseStamped`格式的goal封装成`move_base_msgs::MoveBaseActionGoal`，然后用`action_goal_pub_`发布给move_base内部

---

"cmd_vel"

geometry_msgs::Twist

==输出到机器人底盘的速度命令==

```
// move_base.cpp，MoveBase构造函数
vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
```

```
// move_base.cpp
bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan) {
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;
    
    ...
    
    if(tc_->computeVelocityCommands(cmd_vel)){
        ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
        last_valid_control_ = ros::Time::now();
        //make sure that we send the velocity command to the base
        vel_pub_.publish(cmd_vel);
        if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
    }
    
}
```

```
// move_base.cpp
void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
}
```

---

==amcl节点发送的消息==

"/tf"

tf2_msgs::TFMessage

```
// amcl_node.cpp
void AmclNode::runFromBag(const std::string &in_bag_fn) {
    std::vector<std::string> topics;
    topics.push_back(std::string("tf"));
    std::string scan_topic_name = "base_scan"; // TODO determine what topic this actually is from ROS，amcl内部发送的laser消息的话题？
    topics.push_back(scan_topic_name);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    ros::Publisher laser_pub = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);

    ros::Publisher tf_pub = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);
    
    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != NULL)
    {
        tf_pub.publish(msg);
        for (size_t ii=0; ii<tf_msg->transforms.size(); ++ii)
        {
            tf_->getBuffer().setTransform(tf_msg->transforms[ii], "rosbag_authority");
        }
        continue;
    }
}

```

==local_planner订阅里程计数据==

```
// base_local_planner/src/odometry_helper_ros.cpp
void OdometryHelperRos::setOdomTopic(std::string odom_topic) {
    if( odom_topic != odom_topic_ ) {
        odom_topic_ = odom_topic;
        
        if( odom_topic_ != "" ) {
            ros::NodeHandle gn;
            odom_sub_ = gn.subscribe<nav_msgs::Odometry>( odom_topic_, 1, boost::bind( &OdometryHelperRos::odomCallback, this, _1 ));
        }
        else {
            odom_sub_.shutdown();
        }
    }
}

void OdometryHelperRos::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("odom received!");
    
    // we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    base_odom_.child_frame_id = msg->child_frame_id;
    // ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
    // base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}
```

构造`TrajectoryPlannerROS`的时候使用"odom"构造`OdometryHelperRos`实例`odom_helper_`

```
// base_local_planner/src/trajectory_planner_ros.cpp
TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
    world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {
    
    //initialize the planner
    initialize(name, tf, costmap_ros);
}
```

---

==laser相关的话题==

move_base需要指定相关sensor的话题和消息

```
// amcl_node.cpp
static const std::string scan_topic_ = "scan";

// AmclNode构造函数
laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                                    *tf_, 
                                                                    odom_frame_id_, 
                                                                    100);
laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived, this, _1));
initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
```

### 参数配置

### 代价地图配置

两种代价地图存储环境中的障碍信息：一种用于全局路径规划（global_planner），一种用于本地路径规划和实时避障（local_planner）。
它们使用一些==共用的或独立的==配置文件：==通用配置文件==、==全局规划配置文件==和==本地规划配置文件==

##### 1. 通用配置文件

针对两种代价地图的通用配置文件：`mrobot_navigation/config/fake/costmap_common_params.yaml`

```
obstacle_range: 2.5 # 机器人检测障碍物的最大范围，2.5m范围内的障碍物信息会在地图更新
raytrace_range: 3.0 # 机器人检测自由空间的最大范围
#footprint: [[0.175, 0.175], [0.175, -0.175], [-0.175, -0.175], [-0.175, 0.175]] # 设置机器人在二维地图上的占用面积，以机器人的中心为原点
#footprint_inflation: 0.01
robot_radius: 0.175 # 圆形机器人的半径
inflation_radius: 0.1 # 机器人与障碍物的最小安全距离，0.1m
max_obsracle_height: 0.6
min_obsracle_height: 0.0 # 障碍物的最大、最小高度
observation_sources: scan # 代价地图的所有传感器信息
# sensor_frame传感器参考系名称，data_type激光数据或点云数据使用的消息类型sensor_msga/LaserScan
# topic_name传感器发布的话题名称，marking和clearing表示是否需要使用传感器的实时信息来添加或清除代价地图中的障碍物信息
scan: {data_type: LaserScan, topic: /scan, marking: true, clearing: true, expected_update_rate: 0}
```

##### 2. 全局规划配置文件

用于存储配置全局代价地图参数：`global_costmap_params.ymal`

```
global_costmap:
   global_frame: map # 表示全局代价地图需要在哪个参考系下运行，这里选择map参考系
   robot_base_frame: base_footprint # 表示代价地图可以参考的机器人本体的坐标系
   update_frequency: 1.0 # 全局地图信息更新频率Hz
   publish_frequency: 1.0 # 代价地图发布可视化信息的频率Hz
   static_map: true # 决定代价地图是否需要根据map_server提供的地图信息进行初始化，如果不使用已有的地图或者map_server，最好设置为false
   rolling_window: false # 设置机器人移动过程中是否需要滚动窗口，以保持机器人处于中心位置
   resolution: 0.01 # 代价地图的分辨率，0.01m/格
   transform_tolerance: 1.0
   map_type: costmap
```

##### 3. 本地规划配置文件

本地代价地图参数： `lcoal_costmap_params.ymal`

```
local_costmap:
   global_frame: map
   robot_base_frame: base_footprint
   update_frequency: 3.0
   publish_frequency: 1.0 # 代价地图发布可视化信息的频率Hz
   static_map: true
   rolling_window: false # 设置机器人移动过程中是否需要滚动窗口，以保持机器人处于中心位置
   width: 6.0   # 代价地图长（米）
   height: 6.0 # 高
   resolution: 0.01 # 分辨率
   transform_tolerance: 1.0
```

