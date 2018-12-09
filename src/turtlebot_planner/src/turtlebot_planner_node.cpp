//  ///////////////////////////////////////////////////////////
//
// turtlebot_planner_node.cpp
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <algorithm>

short sgn(int x) { return x >= 0 ? 1 : -1; }

struct Point
{
    float x;
    float y;
};

class Controller
{
    public:
        Controller()
        : m_pathIndex(0)
        , m_isPoseSet(false)
        , m_isPathSet(false)
        , m_theta(0)
        {
            m_Marker.header.frame_id = "/map";
            m_Marker.header.stamp = ros::Time();
            m_Marker.ns = "robot";
            m_Marker.id = 0;
            m_Marker.type = visualization_msgs::Marker::SPHERE;
            m_Marker.action = visualization_msgs::Marker::ADD;
            m_Marker.scale.x = 0.5;
            m_Marker.scale.y = 0.3;
            m_Marker.scale.z = 0.1;
            m_Marker.color.a = 1.0;
            m_Marker.color.r = 0.0;
            m_Marker.color.g = 1.0;
            m_Marker.color.b = 0.0;
        }

        void poseCallback(const gazebo_msgs::ModelStates &msg)
        {
          int i;
          for (i = 0; i < msg.name.size(); i++)
              if (msg.name[i] == "mobile_base")
                  break;

              tf2::Quaternion q;
              m_curPos.x = msg.pose[i].position.x;
              m_curPos.y = msg.pose[i].position.y;
              m_theta = tf::getYaw(msg.pose[i].orientation);

              m_Marker.header.stamp = ros::Time();
              m_Marker.pose.position.x = m_curPos.x;
              m_Marker.pose.position.y = m_curPos.y;
              m_Marker.pose.position.z = 0;
              q.setRPY(0, 0, m_theta);
              m_Marker.pose.orientation.w = q.getW();
              m_Marker.pose.orientation.x = q.getX();
              m_Marker.pose.orientation.y = q.getY();
              m_Marker.pose.orientation.z = q.getZ();

              if(m_posePublisher)
              {
                m_posePublisher->publish(m_Marker);
            }

            if (!m_isPoseSet)
                m_isPoseSet = true;
        }

        void addPath(std::vector<Point> path)
        {
            m_path = path;
            m_isPathSet = true;
        }

        void setPublisher(ros::Publisher *posePublisher)
        {
            m_posePublisher = posePublisher;
        }

        bool getControlOutput(float& controlOutput)
        {
            struct Point startPoint = m_path[m_path.size() - 1 - m_pathIndex];
            struct Point nextPoint = m_path[m_path.size() - 2 - m_pathIndex];

            float xDistToNext = nextPoint.x - m_curPos.x;
            float yDistToNext = nextPoint.y - m_curPos.y;
            float distToGoal = sqrt(xDistToNext * xDistToNext + yDistToNext * yDistToNext);

            if (distToGoal < 0.25)
            {
                m_pathIndex++;
                if (m_pathIndex >= (m_path.size() - 1))
                {
                    return false;
                }

                startPoint = m_path[m_path.size() - 1 - m_pathIndex];
                nextPoint = m_path[m_path.size() - 2 - m_pathIndex];
                
                while ((fabs(startPoint.x - nextPoint.x) < 0.00001) && (fabs(startPoint.y - nextPoint.y) < 0.00001))
                {
                    m_pathIndex++;
                    if (m_pathIndex >= (m_path.size() - 1))
                    {
                        return false;
                    }

                    startPoint = m_path[m_path.size() - 1 - m_pathIndex];
                    nextPoint = m_path[m_path.size() - 2 - m_pathIndex];
                }

                xDistToNext = nextPoint.x - m_curPos.x;
                yDistToNext = nextPoint.y - m_curPos.y;
                distToGoal = sqrt(xDistToNext * xDistToNext + yDistToNext * yDistToNext);
            }

            float dx = nextPoint.x - startPoint.x;
            float dy = nextPoint.y - startPoint.y;

            float dx1 = m_curPos.x - startPoint.x;
            float dy1 = m_curPos.y - startPoint.y;

            float dot_product = dy * dy1 + dx * dx1;
            float norm_squared = dx * dx + dy * dy;
            float distance = sqrt(norm_squared);


            float distanceDeCarrotte = 0.25;
            struct Point pointLaPlusProche;
            pointLaPlusProche.x = startPoint.x + dot_product / norm_squared * dx;
            pointLaPlusProche.y = startPoint.y + dot_product / norm_squared * dy;

            float distanceAParcourir = sqrt((nextPoint.x - pointLaPlusProche.x) * (nextPoint.x - pointLaPlusProche.x) + (nextPoint.y - pointLaPlusProche.y) * (nextPoint.y - pointLaPlusProche.y));
            struct Point pointDeCarrotte;
            pointDeCarrotte.x = pointLaPlusProche.x + std::min(distanceDeCarrotte, distanceAParcourir) / distance * dx;
            pointDeCarrotte.y = pointLaPlusProche.y + std::min(distanceDeCarrotte, distanceAParcourir) / distance * dy;

            float angleDErreur = std::fmod((atan2(pointDeCarrotte.y - m_curPos.y, pointDeCarrotte.x - m_curPos.x) - m_theta),(2 * M_PI));

            if (angleDErreur > M_PI)
            {
                angleDErreur -= 2*M_PI;
            }
            else if (angleDErreur < -1*M_PI)
            {
                angleDErreur += 2*M_PI;
            }
            ROS_INFO("Index %d", m_pathIndex);

            ROS_INFO("Error %f\n current theta %f\n line angle %f\n Start: %f %f\n End: %f %f\n Current: %f %f\n Distance to goal: %f\n", angleDErreur, m_theta, atan2(dy, dx), startPoint.x, startPoint.y, nextPoint.x, nextPoint.y, m_curPos.x, m_curPos.y, distToGoal);
            float kp = 3;

            controlOutput = kp * angleDErreur;

            return true;
        }

    private: 
        std::vector<struct Point> m_path;
        uint8_t m_pathIndex;
        struct Point m_curPos;
        float m_theta;
        bool m_isPoseSet;
        bool m_isPathSet;
        ros::Publisher *m_posePublisher;
        visualization_msgs::Marker m_Marker;

};

class PRM
{
    struct Node;
    struct Edge
    {
        uint32_t nodeIdx;
        float cost;
    };

    struct Node
    {
        float x;
        float y;
        std::vector<Edge> edges;
    };

    typedef std::pair<uint32_t, float> DistPair;
    public: 
        PRM(std::string& frameId)
        : m_startPublisher(NULL)
        , m_goalPublisher(NULL)
        , m_nodesPublisher(NULL)
        , m_edgesPublisher(NULL)
        , m_pathPublisher(NULL)
        {   
            m_startMarker.header.frame_id = frameId;
            m_startMarker.header.stamp = ros::Time();
            m_startMarker.ns = "start";
            m_startMarker.id = 1;
            m_startMarker.type = visualization_msgs::Marker::POINTS;
            m_startMarker.action = visualization_msgs::Marker::ADD;
            m_startMarker.scale.x = 0.4;
            m_startMarker.scale.y = 0.4;
            m_startMarker.scale.z = 0.02;
            m_startMarker.color.a = 1.0;
            m_startMarker.color.r = 0.5;
            m_startMarker.color.g = 1.0;
            m_startMarker.color.b = 1.0;

            m_goalMarker.header.frame_id = frameId;
            m_goalMarker.header.stamp = ros::Time();
            m_goalMarker.ns = "goal";
            m_goalMarker.id = 2;
            m_goalMarker.type = visualization_msgs::Marker::POINTS;
            m_goalMarker.action = visualization_msgs::Marker::ADD;
            m_goalMarker.scale.x = 0.4;
            m_goalMarker.scale.y = 0.4;
            m_goalMarker.scale.z = 0.02;
            m_goalMarker.color.a = 4.0;
            m_goalMarker.color.r = 1.0;
            m_goalMarker.color.g = 0.5;
            m_goalMarker.color.b = 0.5;

            m_nodesMarker.header.frame_id = frameId;
            m_nodesMarker.header.stamp = ros::Time();
            m_nodesMarker.ns = "nodes";
            m_nodesMarker.id = 3;
            m_nodesMarker.type = visualization_msgs::Marker::POINTS;
            m_nodesMarker.action = visualization_msgs::Marker::ADD;
            m_nodesMarker.scale.x = 0.06;
            m_nodesMarker.scale.y = 0.06;
            m_nodesMarker.color.a = 4.0;
            m_nodesMarker.color.r = 1.0;
            m_nodesMarker.color.g = 0.0;
            m_nodesMarker.color.b = 0.2;


            m_edgesMarker.header.frame_id = frameId;
            m_edgesMarker.header.stamp = ros::Time();
            m_edgesMarker.ns = "edges";
            m_edgesMarker.id = 4;
            m_edgesMarker.type = visualization_msgs::Marker::LINE_LIST;
            m_edgesMarker.action = visualization_msgs::Marker::ADD;
            m_edgesMarker.scale.x = 0.01;
            m_edgesMarker.scale.y = 0.01;
            m_edgesMarker.color.a = 0.5;
            m_edgesMarker.color.r = 0.6;
            m_edgesMarker.color.g = 0.0;
            m_edgesMarker.color.b = 0.1;

            m_pathMarker.header.frame_id = frameId;
            m_pathMarker.header.stamp = ros::Time();
            m_pathMarker.ns = "path";
            m_pathMarker.id = 5;
            m_pathMarker.type = visualization_msgs::Marker::LINE_LIST;
            m_pathMarker.action = visualization_msgs::Marker::ADD;
            m_pathMarker.scale.x = 0.1;
            m_pathMarker.scale.y = 0.1;
            m_pathMarker.color.a = 1.0;
            m_pathMarker.color.r = 0.0;
            m_pathMarker.color.g = 1.0;
            m_pathMarker.color.b = 0.0;
        }
        ~PRM(){}

        void AddMap(std::vector<int8_t> map, uint32_t width, uint32_t height, double resolution, Point origin)
        {
            uint32_t filterHeight = 8;
            uint32_t filterWidth = 8;
            // Populate map
            m_map.resize(height);
            m_collisionMap.resize(height);
            for (int i = 0; i < height; i++)
            {
                m_map[i].resize(width);
                m_collisionMap[i].resize(width);
                for(int j = 0; j < width; j++)
                {
                    m_map[i][j] = map[height*i + j];
                }
            }

            // Create expanded map
            for (int i = 0; i < height; i++)
            {
                for(int j = 0; j < width; j++)
                {
                    for(int k = filterHeight+i-filterHeight/2; k < filterHeight+i+filterHeight/2; k++)
                    {
                        for(int l = filterWidth+j-filterWidth/2; l < filterWidth+j+filterWidth/2; l++)
                        {
                            int h = k - filterHeight;
                            int w = l - filterWidth;
                            // if((h < 0) || (h > height-1) || (w < 0) || (w > width-1))
                            if((h < 0) || (h > height-1) || (w < 0) || (w > width-1))
                            {
                                continue;
                            }
                            if( i == 0 )
                            {
                                ROS_INFO("i %d j %d h %d w %d", i, j, h, w);
                            }

                            if(m_map[i][j] == OCCUPIED)
                            {
                                m_collisionMap[h][w] = OCCUPIED;
                            }
                            
                        }
                    }
                    
                }
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
        
        void GetPath(Point start, std::vector<Point> goals, uint32_t nNodes, std::vector<Point> &path)
        {  
            ROS_INFO("Get path entry");
            Node node;
            geometry_msgs::Point p;
            std::vector<std::pair<uint32_t, uint32_t>> wayPoints;

            // Populate nodes vector start and goal
            m_nodesMarker.points.resize(nNodes + goals.size() + 1);
            m_nodes.resize(nNodes + goals.size() + 1);
            m_startMarker.points.resize(1);
            m_startMarker.header.stamp = ros::Time();
            p.x = start.x;
            p.y = start.y;
            p.z = 0;
            m_startMarker.points[0] = p;
            m_nodesMarker.points[0] = p;
            node.x = start.x; 
            node.y = start.y;
            m_nodes[0] = node;

            // Build way points
            ROS_INFO("Building way points");
            m_goalMarker.header.stamp = ros::Time();
            m_goalMarker.points.resize(goals.size());
            uint32_t nGoals = 0;
            wayPoints.push_back(std::pair<uint32_t, uint32_t>(0, 1));
            for (int i = 0; i < goals.size(); i++)
            {
                p.x = goals[i].x;
                p.y = goals[i].y;
                p.z = 0;
                m_goalMarker.points[i] = p;
                m_nodesMarker.points[i+1] = p;
                node.x = p.x; 
                node.y = p.y;
                m_nodes[i+1] = node;
                nGoals++;
                if(i < goals.size() - 1)
                {
                    wayPoints.push_back(std::pair<uint32_t, uint32_t>(i+1, i+2));
                }   
            }
            m_nWayPoints = goals.size() + 1;

            if(m_startPublisher)
            {
                m_startPublisher->publish(m_startMarker);
            }

            if(m_goalPublisher)
            {
                m_goalPublisher->publish(m_goalMarker);
            }

            // Build Graph
            ROS_INFO("Find shortest path %d", wayPoints.size());
            BuildGraph(nNodes, 10);

            std::vector<Node> globalPath;
            // Build path to each way points
            for(int i = 0; i < wayPoints.size(); i++)
            {
                std::vector<Node> localPath;
                ShortestPath(wayPoints[i].first, wayPoints[i].second, localPath);
                globalPath.insert(globalPath.begin(), localPath.begin(), localPath.end());
            }

            ROS_INFO("Drawing points");
            m_pathMarker.header.stamp = ros::Time();
            for(int i = 0; i < globalPath.size()-1; i++)
            {
                geometry_msgs::Point p1, p2;
                p1.x = globalPath[i].x;
                p1.y = globalPath[i].y;
                p1.z = 0;
                p2.x = globalPath[i+1].x;
                p2.y = globalPath[i+1].y;
                p2.z = 0;
                m_pathMarker.points.push_back(p1);
                m_pathMarker.points.push_back(p2);
                ROS_INFO("Path 1 -> %.3f, %3f", p1.x, p1.y);
                ROS_INFO("Path 2 -> %.3f, %3f", p2.x, p2.y);

                path.push_back({p1.x, p1.y});
            }

            path.push_back({globalPath[globalPath.size() - 1].x, globalPath[globalPath.size() - 1].y});
            if(m_pathPublisher)
            {
                m_pathPublisher->publish(m_pathMarker);
            }
        }

        void SetPublishers(ros::Publisher *startPublisher,
                           ros::Publisher *goalPublisher,
                           ros::Publisher *nodesPublisher,
                           ros::Publisher *edgesPublisher,
                           ros::Publisher *pathPublisher)
        {
            m_startPublisher = startPublisher;
            m_goalPublisher = goalPublisher;
            m_nodesPublisher = nodesPublisher;
            m_edgesPublisher = edgesPublisher;
            m_pathPublisher = pathPublisher;
        }

    private:
        bool NodeCollides(Node node)
        {
            int mapX = (node.x - m_xMin) / m_mapResolution;
            int mapY  = (node.y - m_yMin) / m_mapResolution;
            mapX = std::max(0, std::min(mapX, m_mapWidth-1));
            mapY = std::max(0, std::min(mapY, m_mapHeight-1));
            bool collision = CheckCollision(mapX, mapY);
            
            return collision;
        }

        void Bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y)
        {
            int dx = abs(x1 - x0);
            int dy = abs(y1 - y0);
            int dx2 = x1 - x0;
            int dy2 = y1 - y0;

            const bool s = abs(dy) > abs(dx);

            if (s)
            {
                int dx2 = dx;
                dx = dy;
                dy = dx2;
            }

            int inc1 = 2 * dy;
            int d = inc1 - dx;
            int inc2 = d - dx;

            x.push_back(x0);
            y.push_back(y0);

            while (x0 != x1 || y0 != y1)
            {
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

        bool EdgeCollides(Node nodeA, Node nodeB)
        {
            bool collision = 0;
            std::vector<int> x;
            std::vector<int> y;

            int x0 = (nodeA.x - m_xMin) / m_mapResolution;
            int y0  = (nodeA.y - m_yMin) / m_mapResolution;
            x0 = std::max(0, std::min(x0, m_mapWidth-1));
            y0 = std::max(0, std::min(y0, m_mapHeight-1));

            int x1 = (nodeB.x - m_xMin) / m_mapResolution;
            int y1 = (nodeB.y - m_yMin) / m_mapResolution;
            x1 = std::max(0, std::min(x1, m_mapWidth-1));
            y1 = std::max(0, std::min(y1, m_mapHeight-1));

            Bresenham(x0, y0, x1, y1, x, y);

            for (int i = 0; i < x.size(); i++)
            {
                int mapX = x[i];
                int mapY = y[i];
                collision |= CheckCollision(mapX, mapY);
            }

            return collision;
        }

        bool CheckCollision(int mapX, int mapY)
        {
            return m_collisionMap[mapY][mapX] != 0;
        }
        void BuildGraph(uint32_t nNodes, uint32_t nNeighbours)
        {
            ROS_INFO("Build graph entry");
            CreateNodes(nNodes);
            if(m_nodesPublisher)
            {
                m_nodesPublisher->publish(m_nodesMarker);
                ROS_INFO("Publish nodes");
            }
            
            // Create Edges
            for (int i = 0; i < m_nodes.size(); i++)
            {
                CreateEdges(nNeighbours, i);
            }

            if(m_edgesPublisher)
            {
                m_edgesPublisher->publish(m_edgesMarker);
                ROS_INFO("Publish edge"); 
            }
            
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
            // m_nodesMarker.points.resize(nNodes+2);
            m_nodesMarker.header.stamp = ros::Time();

            while(nodesCount < nNodes)
            {
                Node newNode;
                newNode.x = xDistr(kENG); 
                newNode.y = yDistr(kENG);
               
                if(NodeCollides(newNode))
                {
                    continue;
                }
                m_nodes[m_nWayPoints + nodesCount] = newNode;

                // Points for visualize
                geometry_msgs::Point p;
                p.x = newNode.x;
                p.y = newNode.y;
                p.z = 0;
                m_nodesMarker.points[m_nWayPoints + nodesCount] = p;
                nodesCount++;
            }
        }

        void CreateEdges(uint32_t nNeighbours, uint32_t srcIdx)
        {
            // Find edges for a sampled set of nodes
            Node srcNode = m_nodes[srcIdx];
            Node destNode;
            uint32_t nFound = 0;
            // Vector of distanct from src to ecery node and correspoinding idx
            std::vector<DistPair> distances(m_nodes.size());

            for(int i = 0; i < m_nodes.size(); i++)
            {
                if(i == srcIdx)
                {
                    distances[i] = DistPair (i, INFINITY);
                    continue;
                }
                destNode = m_nodes[i];
                float dist = sqrt(pow(srcNode.x - destNode.x, 2) + pow(srcNode.y - destNode.y, 2));
                distances[i] = DistPair (i, dist);
            }

            // Sort based on distances
            std::sort(distances.begin(), distances.end(), DistCompare);
            for(int i = 0; i < m_nodes.size(); i++)
            {
                if (nFound == nNeighbours)
                {
                    continue;
                }

                destNode = m_nodes[distances[i].first];
                if (EdgeCollides(srcNode, destNode))
                {
                    continue;
                }

                Edge edge = {distances[i].first, distances[i].second};
                m_nodes[srcIdx].edges.push_back(edge);
                nFound++;
                
                // Points for visualizing line
                geometry_msgs::Point p1, p2;
                p1.x = srcNode.x;
                p1.y = srcNode.y;
                p1.z = 0;
                p2.x = destNode.x;
                p2.y = destNode.y;
                p2.z = 0;
                m_edgesMarker.points.push_back(p1);
                m_edgesMarker.points.push_back(p2);
            }
            
        }
        
        static bool DistCompare (const DistPair& A, 
                                  const DistPair& B) 
        { 
            return A.second < B.second; 
        }

        void ShortestPath(uint32_t startIdx, uint32_t goalIdx, std::vector<Node> &path)
        {   
            uint32_t closedCount = 0;
            uint32_t openCount = 0;
            std::vector<int> vertices;
            std::vector<bool> open(m_nodes.size());
            std::vector<bool> closed(m_nodes.size());
            std::vector<int> prev(m_nodes.size());
            std::vector<float> distances(m_nodes.size());
            std::vector<float> hDistances(m_nodes.size()); // distance with heuristics
            ROS_INFO("Init Shortest");
            for(int i = 0; i < m_nodes.size(); i++)
            {
                distances[i] = INFINITY;
                hDistances[i] = INFINITY;
                prev[i] = -100;
                open[i] = false;
                closed[i] = false;
            }            

            ROS_INFO("Start Shortest");
            open[startIdx] = true;
            openCount++;
            hDistances[startIdx] = DistBetweenNodes(m_nodes[startIdx], m_nodes[goalIdx]);
            distances[startIdx] = 0;
            while (openCount > closedCount) 
            {
                float minDist = INFINITY;
                int minIdx = -1;
                int elementIdx = -1;
                for(int i = 0; i < open.size(); i++)
                {
                    if(open[i] == false) // Node not in open set
                    {
                        continue;
                    }

                    float dist = hDistances[i];
                    if (dist <= minDist)
                    {
                        minDist = dist;
                        minIdx = i;
                    }       
                }

                open[minIdx] = false;
                closed[minIdx] = true;
                closedCount++;
                
                if (minIdx == goalIdx) // Target reached
                {
                    break;
                }

                std::vector<Edge> neighbours = (m_nodes[minIdx]).edges;
                for (int i = 0; i < neighbours.size(); i++)
                {
                    Edge edge = neighbours[i];
                    uint32_t neighIdx = edge.nodeIdx;
                    if (closed[neighIdx] == true) // Neighbor in closed set
                    {
                        continue;
                    }

                    float cost = edge.cost;
                    float dist = distances[minIdx] + cost;

                    if (open[neighIdx] == false) // New node discovered
                    {
                        open[neighIdx] = true;
                        openCount++;
                    } else if (dist > distances[neighIdx])
                    {
                        continue;
                    }
                    distances[neighIdx] = dist;
                    hDistances[neighIdx] = distances[neighIdx] + DistBetweenNodes(m_nodes[neighIdx], m_nodes[goalIdx]);
                    prev[neighIdx] = minIdx;
                }
            }

            // ROS_INFO("Building Path");
            uint32_t currentIdx = goalIdx;
            if (prev[currentIdx] == -1)
            {
                ROS_INFO("No path");
                return;
            }
            while (currentIdx != -100)
            {
                // ROS_INFO("Push back");
                path.push_back(m_nodes[currentIdx]);
                ROS_INFO("Find prev %d %.3f, %.3f", currentIdx, m_nodes[currentIdx].x, m_nodes[currentIdx].y);
                currentIdx = prev[currentIdx];
            }
            ROS_INFO("Finished Building");
        
        }

        float DistBetweenNodes(Node nodeA, Node nodeB)
        {
            return sqrt(pow(nodeA.x - nodeB.x, 2) + pow(nodeA.y - nodeB.y, 2));
        }

        std::vector<std::vector<int8_t>> m_map;
        std::vector<std::vector<int8_t>> m_collisionMap;
        std::vector<Node> m_nodes;
        uint32_t m_nWayPoints;
        uint32_t m_nNodes;
        int m_mapWidth;
        int m_mapHeight;
        double m_xMin;
        double m_xMax;
        double m_yMin;
        double m_yMax;
        double m_mapResolution;
        static const int8_t OCCUPIED = 100;
        static const int8_t UNOCCUPIED = 0;

        // Visualization
        ros::Publisher *m_startPublisher;
        ros::Publisher *m_goalPublisher;
        ros::Publisher *m_nodesPublisher;
        ros::Publisher *m_edgesPublisher;
        ros::Publisher *m_pathPublisher;
        visualization_msgs::Marker m_startMarker;
        visualization_msgs::Marker m_goalMarker;
        visualization_msgs::Marker m_nodesMarker;
        visualization_msgs::Marker m_edgesMarker;
        visualization_msgs::Marker m_pathMarker;
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
    ros::Publisher startPublisher = n.advertise<visualization_msgs::Marker>( "/start", 0);
    ros::Publisher goalPublisher = n.advertise<visualization_msgs::Marker>( "/goal", 0);
    ros::Publisher nodesPublisher = n.advertise<visualization_msgs::Marker>( "/nodes", 0);
    ros::Publisher edgesPublisher = n.advertise<visualization_msgs::Marker>( "/edges", 0);
    ros::Publisher pathPublisher = n.advertise<visualization_msgs::Marker>( "/path", 0);

    planner.SetPublishers(&startPublisher, &goalPublisher, &nodesPublisher, &edgesPublisher, &pathPublisher);

    // Controller Initialization
    Controller controller;
    ros::Subscriber pose_sub;
    pose_sub = n.subscribe("/gazebo/model_states", 1, &Controller::poseCallback, &controller);

    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    ros::Publisher posePublisher = n.advertise<visualization_msgs::Marker>( "/robotMeasurement", 0 );
    geometry_msgs::Twist vel;

    controller.setPublisher(&posePublisher);
    //Set the loop rate
    ros::Rate loop_rate(10);    //20Hz update rate
    bool pathFound = false;
    bool donePath = false;



    std::vector<Point> path;
    while (ros::ok())
    {
    	ros::spinOnce();   //Check for new messages
        if(!mapHandler.HasMap())
        {
            continue;
        }

        if(!pathFound)
        {
            std::vector<Point> goals = {{8,-4}, {8, 0}, {4, 0},};
            planner.GetPath({0, 0}, goals, 600, path);
            pathFound = true;

            controller.addPath(path);
        }
        else
        {
            float angularVelocity = 0;

            if (!donePath)
            {
                if (controller.getControlOutput(angularVelocity))
                {
                    // output control
                    vel.linear.x = 0.3;
                    vel.angular.z = angularVelocity;
                    velocityPublisher.publish(vel); // Publish the command velocity
                }
                else
                {
                    ROS_INFO("Done path\n");
                    // output control
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    velocityPublisher.publish(vel); // Publish the command velocity
                    donePath = true;
                }
            }
        }

        loop_rate.sleep(); //Maintain the loop rate
    }
    return 0;
}