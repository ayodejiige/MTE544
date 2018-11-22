//  ///////////////////////////////////////////////////////////
//
// turtlebot_planner_node.cpp
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
// #include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>


struct Point
{
    float x;
    float y;
};

class PRM
{
    struct Node;
    struct Edge
    {
        Node *dest;
        float cost;
    };

    struct Node
    {
        float x;
        float y;
        std::vector<Edge> edges;
    };

    public: 
        PRM(std::string& frameId)
        : m_nodesPublisher(NULL)
        {
            m_nodesMarker.header.frame_id = frameId;
            m_nodesMarker.header.stamp = ros::Time();
            m_nodesMarker.ns = "nodes";
            m_nodesMarker.id = 1;
            m_nodesMarker.type = visualization_msgs::Marker::POINTS;
            m_nodesMarker.action = visualization_msgs::Marker::ADD;
            m_nodesMarker.scale.x = 0.2;
            m_nodesMarker.scale.y = 0.2;
            m_nodesMarker.color.a = 1.0;
            m_nodesMarker.color.r = 1.0;
            m_nodesMarker.color.g = 0.6;
            m_nodesMarker.color.b = 0.1;
        }
        ~PRM(){}

        void AddMap(std::vector<int8_t> map, uint32_t width, uint32_t height, double resolution, Point origin)
        {
            uint32_t mapSize = height*width;

            // Populate map
            m_map.resize(mapSize);
            for (int i = 0; i < mapSize; i++)
            {
                m_map[i] = map[i];
            }

            // Store metadata
            m_mapHeight = height;
            m_mapWidth = width;
            m_mapResolution = resolution;
            m_xMin = origin.x;
            m_xMax = origin.x + width * resolution;
            m_yMin = origin.y;
            m_yMax =  origin.y + height * resolution;
        }
        
        void GetPath(Point start, Point goal, uint32_t nNodes, double maxDist)
        {  
            Node newNode;
            newNode.x = start.x; 
            newNode.y = start.y;
            m_nodes.push_back(newNode);
            newNode.x = goal.x; 
            newNode.y = goal.y;
            m_nodes.push_back(newNode);

            // Generate Node
            CreateNodes(nNodes);
            
            
            // Create Edges
            CreateEdges(maxDist);
        }

        void SetPublishers(ros::Publisher *nodesPublisher)
        {
            m_nodesPublisher = nodesPublisher;
        }
    private:
        
        bool Collides(Node node)
        {
            int mapX = (node.x - m_xMin) / m_mapResolution;
            int mapY  = (node.y - m_yMin) / m_mapResolution;
            mapX = std::max(0, std::min(mapX, m_mapWidth-1));
            mapY = std::max(0, std::min(mapY, m_mapHeight-1));
            bool xCollision = m_map[mapY*m_mapHeight + mapX];
        }   

        void CreateNodes(uint32_t nNodes)
        {
            std::random_device rd; // Obtain a random number from hardware
            std::mt19937 kENG(rd()); // seed the generator
            std::uniform_real_distribution<> xDistr(m_xMin, m_xMax); 
            std::uniform_real_distribution<> yDistr(m_yMin, m_yMax);
            uint32_t nodesCount = 0;
            ROS_INFO("XMIN : %.3f; XMAX : %.3f", m_xMin, m_xMax);
            ROS_INFO("YMIN : %.3f; YMAX : %.3f", m_yMin, m_yMax);
            m_nodesMarker.points.resize(nNodes);
            m_nodesMarker.header.stamp = ros::Time();

            while(nodesCount < nNodes)
            {
                Node newNode;
                newNode.x = xDistr(kENG); 
                newNode.y = yDistr(kENG);
                m_nodes.push_back(newNode);
                if(Collides(newNode))
                {
                    continue;
                }

                // Points for visualize
                geometry_msgs::Point p;
                p.x = newNode.x;
                p.y = newNode.y;
                p.z = 0;
                m_nodesMarker.points[nodesCount] = p;
                nodesCount++;
            }

            if(m_nodesPublisher)
            {   ROS_INFO("Publish");
                m_nodesPublisher->publish(m_nodesMarker);
            }
        }

        void CreateEdges(double maxDist)
        {
            // find edges for a sampled set of nodes
        }
        
        void Search(Node start, Node goal, double cost)
        {

        }

        std::vector<int8_t> m_map;
        std::vector<Node> m_nodes;
        int m_mapWidth;
        int m_mapHeight;
        double m_xMin;
        double m_xMax;
        double m_yMin;
        double m_yMax;
        double m_mapResolution;

        // Visualization
        ros::Publisher *m_nodesPublisher;
        visualization_msgs::Marker m_nodesMarker;
};

class MapHandler
{
    public:
        MapHandler(PRM *planner)
        : m_hasMap(false)
        , m_planner(planner)
        {}
        ~MapHandler()
        {}

        void MapCallback(const nav_msgs::OccupancyGrid& msg)
        {
            m_hasMap = true;
            Point origin = {msg.info.origin.position.x, msg.info.origin.position.y};
            m_planner->AddMap(msg.data, (uint32_t)msg.info.width, (uint32_t)msg.info.height, (double)msg.info.resolution, origin);
        }

        bool HasMap()
        {
            return m_hasMap;
        }

    private:
        bool m_hasMap;
        PRM *m_planner;
};

int main(int argc, char **argv)
{
    // Static variables
    std::string frame = "map";
    uint32_t nParticles = 150;

    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    PRM planner(frame);
    MapHandler mapHandler(&planner);
   
    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber mapSub = n.subscribe("/map", 1, &MapHandler::MapCallback, &mapHandler);
    ros::Publisher nodesPublisher = n.advertise<visualization_msgs::Marker>( "/nodes", 0);


    planner.SetPublishers(&nodesPublisher);

    //Set the loop rate
    ros::Rate loop_rate(10);    //20Hz update rate

    while (ros::ok())
    {
    	ros::spinOnce();   //Check for new messages
        if(! mapHandler.HasMap())
        {
            continue;
        }
        ROS_INFO("Got map");
        planner.GetPath({0, 0}, {1,2}, 300, 5);
    
        loop_rate.sleep(); //Maintain the loop rate
    }
    return 0;
}