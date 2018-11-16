#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <dynamic_reconfigure/server.h>
#include <turtlebot_mapper/turtlebot_mapper_nodeConfig.h>

#define LIVE

#define INITIAL_ODDS 0.5
#define EMPTY_ODDS 0.2
#define OCCUPIED_ODDS 0.85
#define OCCUPANCY_ODDS_SCALE 100

geometry_msgs::PoseStamped posMsg;
short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

class Mapper
{
  struct RangeReading
  {
    std::vector<float> ranges;
    float minAngle;
    float maxAngle;
    float maxRange;
    float minRange;
    float angleResolution;
  };

  public:
    Mapper(float xMin, float xMax, float yMin, float yMax, int xNum, int yNum)
    : m_xMin(xMin)
    , m_xMax(xMax)
    , m_yMin(yMin)
    , m_yMax(yMax)
    , m_xNum(xNum)
    , m_yNum(yNum)
    , m_isPoseSet(false)
    , m_isScanSet(false)
    , m_emptyOdds(0.4)
    , m_occupiedOdds(0.8)
    , m_maxRange(2)
    {
      m_mapSize = m_xNum * m_yNum;
      m_xRes = (m_xMax - m_xMin) / (float) m_xNum;
      m_yRes = (m_yMax - m_yMin) / (float) m_yNum;

      // Is this valid
      m_xPos = 0;
      m_yPos = 0;
      m_theta = 0;

      m_map.resize(m_mapSize, (int) (INITIAL_ODDS * OCCUPANCY_ODDS_SCALE));

      m_initialLogOdds = log(INITIAL_ODDS / (1 - INITIAL_ODDS));
      m_logOddsMap.resize(m_mapSize, m_initialLogOdds);
    }

    ~Mapper(){}

    //Callback function for the Position topic (SIMULATION)
    void pose_callback(const gazebo_msgs::ModelStates &msg) 
    {
      int i;
      for (i = 0; i < msg.name.size(); i++)
          if (msg.name[i] == "mobile_base")
              break;

      m_xPos = msg.pose[i].position.x;
      m_yPos = msg.pose[i].position.y;
      m_theta = tf::getYaw(msg.pose[i].orientation);

      if(m_theta < 0)
      {
          m_theta += M_PI * 2;
      }

      if (!m_isPoseSet)
        m_isPoseSet = true;
    }

    //Callback function for the Position topic (SIMULATION)
    void live_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, tf::getYaw(msg->pose.pose.orientation));

      if (abs(q.getW()) > 1 || abs(q.getX()) > 1 || abs(q.getY()) > 1 || abs(q.getZ()) > 1) return;

      m_xPos = msg->pose.pose.position.x;
      m_yPos = msg->pose.pose.position.y;
      m_theta = tf::getYaw(msg->pose.pose.orientation);
      // ROS_INFO("x=%f y=%f theta=%f", m_xPos, m_yPos, m_theta);

      if(m_theta < 0)
      {
          m_theta += M_PI * 2;
      }

      if (!m_isPoseSet)
        m_isPoseSet = true;
      
      q.setRPY(0, 0, m_theta);

      posMsg.pose.position.x = m_xPos;
      posMsg.pose.position.y = m_yPos;
      posMsg.pose.position.z = 0;
      posMsg.pose.orientation.w = q.getW();
      posMsg.pose.orientation.x = q.getX();
      posMsg.pose.orientation.y = q.getY();
      posMsg.pose.orientation.z = q.getZ();
    }

    //Callback function for the map
    void scan_callback(const sensor_msgs::LaserScan &msg) 
    {
      m_currentReading.minAngle = msg.angle_min;
      m_currentReading.maxAngle = msg.angle_max;
      m_currentReading.minRange = msg.range_min;
      m_currentReading.maxRange = msg.range_max;
      m_currentReading.angleResolution = msg.angle_increment;
      m_currentReading.ranges = msg.ranges;

      if (!m_isScanSet)
        m_isScanSet = true;
    }

    void config_callback(turtlebot_mapper::turtlebot_mapper_nodeConfig &config, uint32_t level)
    {
      m_emptyOdds = config.empty_odds;
      m_occupiedOdds = config.occupied_odds;
      m_maxRange = config.max_range;

      // ROS_INFO("%.3f %.3f %.3f", m_emptyOdds, m_occupiedOdds, m_maxRange);
    }

    void getCurrentMapCoordinates(int &x_0, int &y_0)
    {
      x_0 = (int) ((m_xPos - m_xMin) / m_xRes);
      x_0 = std::max(0, std::min(x_0, m_xNum - 1));

      y_0 = (int) ((m_yPos - m_yMin) / m_yRes);
      y_0 = std::max(0, std::min(y_0, m_yNum - 1));
    }

    void inverseBresenhamScanner(int x_0, int y_0, int measurementIndex)
    {
      if (std::isnan(m_currentReading.ranges[measurementIndex])) return;
      if (m_currentReading.ranges[measurementIndex] > m_maxRange) return;
      if (m_currentReading.ranges[measurementIndex] < m_currentReading.minRange) return;

      std::vector<int> x_line;
      std::vector<int> y_line;

      float bearing_angle = m_currentReading.minAngle + m_currentReading.angleResolution * measurementIndex + m_theta;      

      int x_1 = x_0 + (int) ((m_currentReading.ranges[measurementIndex] * cos(bearing_angle)) / m_xRes);
      x_1 = std::max(0, std::min(x_1, m_xNum - 1));

      int y_1 = y_0 + (int) ((m_currentReading.ranges[measurementIndex] * sin(bearing_angle)) / m_yRes);
      y_1 = std::max(0, std::min(y_1, m_yNum - 1));
      
      bresenham(x_0, y_0, x_1, y_1, x_line, y_line);

      int line_size = x_line.size();
      std::vector<float> log_odds_update;
      for (int line_it = 0; line_it < line_size; line_it++)
      {
        log_odds_update.push_back(log(m_emptyOdds / (1 - m_emptyOdds)));
      }

      //if (abs(m_currentReading.ranges[measurementIndex] - m_currentReading.maxRange) )
      //{
      log_odds_update[line_size - 1] = log(m_occupiedOdds / (1 - m_occupiedOdds));
      //}

      for (int line_it = 0; line_it < line_size; line_it++)
      {
        // std::cout << x_0 << " " << y_0 << std::endl;
        //int index = (x_line[line_it] * m_xNum + y_line[line_it]);
        int index = (y_line[line_it] * m_yNum + x_line[line_it]);

        m_logOddsMap[index] = m_logOddsMap[index] + log_odds_update[line_it] - m_initialLogOdds;
        m_map[index] = (int) (exp(m_logOddsMap[index]) / (1 + exp(m_logOddsMap[index])) * OCCUPANCY_ODDS_SCALE);

      }
    }

    void updateProbabilities()
    {
      for (int it = 0; it < m_mapSize; it++)
      {
        m_map[it] = (int) (exp(m_logOddsMap[it]) / (1 + exp(m_logOddsMap[it])) * OCCUPANCY_ODDS_SCALE);
      }
    }

    bool isReady()
    {
      return (m_isPoseSet && m_isScanSet);
    }

    std::vector<int8_t> getMap()
    {
      return m_map;
    }

    float getResolution()
    {
      return m_xRes;
    }

    float getHeight()
    {
      return m_yNum;
    }

    float getWidth()
    {
      return m_xNum;
    }

    int getNumRangeReadings()
    {
      return m_currentReading.ranges.size();
    }

  private:
    // Map size/resolution parameters
    int m_xNum;
    int m_yNum;
    int m_mapSize;
    float m_xRes;
    float m_yRes;
    float m_xMax;
    float m_xMin;
    float m_yMax;
    float m_yMin;

    // Current position of robot
    float m_xPos;
    float m_yPos;
    float m_theta;

    volatile bool m_isPoseSet;
    volatile bool m_isScanSet;

    // Occupancy grid
    std::vector<int8_t> m_map;
    std::vector<float> m_logOddsMap;
    float m_initialLogOdds; // ASSUMES ALL INITIAL LOG ODDS ARE THE SAME ACROSS THE MAP
    float m_emptyOdds;
    float m_occupiedOdds;
    float m_maxRange;

    // Current Scan measurement
    RangeReading m_currentReading;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");

  ros::NodeHandle n;

  // Initialize map class
  float xMin = -3;
  float yMin = -5;
  Mapper mapper(xMin, 7, yMin, 5, 800, 800); 
  float resolution = mapper.getResolution();
  int width = mapper.getWidth();
  int height = mapper.getHeight();

  // Set up ROS publish and subscribe
  ros::Subscriber pose_sub;
  #ifdef LIVE
  pose_sub = n.subscribe("/indoor_pos", 1, &Mapper::live_pose_callback, &mapper);
  #else
  pose_sub = n.subscribe("/gazebo/model_states", 1, &Mapper::pose_callback, &mapper);
  #endif

  ros::Subscriber scan_sub = n.subscribe("/scan", 1, &Mapper::scan_callback, &mapper);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>( "/robotPose", 0 );

  dynamic_reconfigure::Server<turtlebot_mapper::turtlebot_mapper_nodeConfig> server;
  dynamic_reconfigure::Server<turtlebot_mapper::turtlebot_mapper_nodeConfig>::CallbackType f;

  f = boost::bind(&Mapper::config_callback, &mapper, _1, _2);
  server.setCallback(f);

  // pos msg params
  posMsg.header.frame_id = "/map";
  posMsg.header.stamp = ros::Time::now();

  // Set OccupancyGrid msg parameters
  nav_msgs::OccupancyGrid map_msg;

  map_msg.info.resolution = resolution;
  map_msg.info.width = width;
  map_msg.info.height = height;

  geometry_msgs::Point position;
  position.x = (float)(xMin); 
  position.y = (float)(yMin);
  geometry_msgs::Quaternion orientation;
  orientation = tf::createQuaternionMsgFromYaw(0);

  geometry_msgs::Pose origin;
  origin.position = position;
  origin.orientation = orientation;
  
  map_msg.info.origin = origin;

  ros::Rate loop_rate(30);

  ROS_INFO("Waiting for mapper ready\n");
  while(mapper.isReady());
  ROS_INFO("Mapper is now ready\n");

  while (ros::ok())
  {
    ros::spinOnce();
  
    int x_0, y_0;
    mapper.getCurrentMapCoordinates(x_0, y_0);
  
  	// iterate over all laser scans
  	for (int scan_it = 0; scan_it < mapper.getNumRangeReadings(); scan_it++)
  	{
      mapper.inverseBresenhamScanner(x_0, y_0, scan_it);
  	}

    // extract real odds
    //mapper.updateProbabilities();

  	// // publish map
    map_msg.data = mapper.getMap();
    map_pub.publish(map_msg);
    pose_pub.publish(posMsg);
    
    loop_rate.sleep();
  }


  return 0;
}
