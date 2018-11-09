#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#define INITIAL_ODDS 0.5
#define EMPTY_ODDS 0.4
#define OCCUPIED_ODDS 0.6
#define OCCUPANCY_ODDS_SCALE 100

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

    void getCurrentMapCoordinates(int &x_0, int &y_0)
    {
      x_0 = (int) ((m_xPos - m_xMin) / m_xRes);
      x_0 = std::max(1, std::min(x_0, m_xNum));

      y_0 = (int) ((m_yPos - m_yMin) / m_yRes);
      y_0 = std::max(1, std::min(y_0, m_yNum));
    }

    void inverseBresenhamScanner(int x_0, int y_0, int measurementIndex)
    {
      if (std::isnan(m_currentReading.ranges[measurementIndex])) return;
      std::vector<int> x_line;
      std::vector<int> y_line;

      float bearing_angle = m_currentReading.minAngle + m_currentReading.angleResolution * measurementIndex + m_theta;
      /*
      if (measurementIndex == 1)
        ROS_INFO("range= %f bearing_angle= %f\n", m_currentReading.ranges[measurementIndex], bearing_angle);
      */

      int x_1 = (int) ((m_xPos + m_currentReading.ranges[measurementIndex] * cos(bearing_angle) - m_xMin) / m_xRes);
      /*
      if (measurementIndex == 1)
        ROS_INFO("X pos = %f X_1= %d\n", m_xPos, x_1);
      */

      x_1 = std::max(1, std::min(x_1, m_xNum));

      int y_1 = (int) ((m_yPos + m_currentReading.ranges[measurementIndex] * sin(bearing_angle) - m_yMin) / m_yRes);
      y_1 = std::max(1, std::min(y_1, m_yNum));
      
      /*
      if (measurementIndex == 1)
        ROS_INFO("Y pos = %f Y_1= %d\n", m_yPos, y_1);
      */

      bresenham(x_0, y_0, x_1, y_1, x_line, y_line);

      int line_size = x_line.size();
      std::vector<float> log_odds_update;
      for (int line_it = 0; line_it < line_size; line_it++)
      {
        log_odds_update.push_back(log(EMPTY_ODDS / (1 - EMPTY_ODDS)));
      }

      if (m_currentReading.ranges[measurementIndex] < m_currentReading.maxRange)
      {
        log_odds_update[line_size - 1] = log(OCCUPIED_ODDS / (1 - OCCUPIED_ODDS));
      }

      for (int line_it = 0; line_it < line_size; line_it++)
      {
        int index = (x_line[line_it] * m_xNum + y_line[line_it]);
        m_logOddsMap[index] = m_logOddsMap[index] + log_odds_update[line_it] - m_initialLogOdds;
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

    // Current Scan measurement
    RangeReading m_currentReading;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");

  ros::NodeHandle n;

  // Initialize map class
  Mapper mapper(-15, 15, -15, 15, 300, 300); 
  float resolution = mapper.getResolution();
  int width = mapper.getWidth();
  int height = mapper.getHeight();

  // Set up ROS publish and subscribe
  ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, &Mapper::pose_callback, &mapper);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, &Mapper::scan_callback, &mapper);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

  // Set OccupancyGrid msg parameters
  nav_msgs::OccupancyGrid map_msg;

  map_msg.info.resolution = resolution;
  map_msg.info.width = width;
  map_msg.info.height = height;

  geometry_msgs::Point position;
  position.x = (float)(-1 * resolution * width / 2); 
  position.y = (float)(-1 * resolution * height / 2);

  geometry_msgs::Quaternion orientation;
  orientation.w = 0.0;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 0.0;

  geometry_msgs::Pose origin;
  origin.position = position;
  origin.orientation = orientation;
  
  map_msg.info.origin = origin;

  ros::Rate loop_rate(10);

  ROS_INFO("Waiting for mapper ready\n");
  while(mapper.isReady());
  ROS_INFO("Mapper is now ready\n");

  while (ros::ok())
  {
    ROS_INFO("Mapper running\n");
    int x_0, y_0;
    mapper.getCurrentMapCoordinates(x_0, y_0);
    ROS_INFO("X0: %d Y0: %d\n", x_0, y_0);

  	// iterate over all laser scans
  	for (int scan_it = 0; scan_it < mapper.getNumRangeReadings(); scan_it++)
  	{
      mapper.inverseBresenhamScanner(x_0, y_0, scan_it);
  	}

    // extract real odds
    mapper.updateProbabilities();

  	// publish map
    map_msg.data = mapper.getMap();
    map_pub.publish(map_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}