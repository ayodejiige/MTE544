//  ///////////////////////////////////////////////////////////
//
// turtlebot_localizer_node.cpp
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>

// #define SIMULATION

static std::random_device rd; // Obtain a random number from hardware
static std::mt19937 kENG(rd()); // seed the generator

struct Vector
{
    double x;
    double y;
    double theta;
    Eigen::Matrix3d cov;
    
};
typedef Vector Position;
typedef Vector Velocity;

std::ostream& operator << (std::ostream& o,
                           const Vector& a)
{
    o << "x: " << a.x << "\ty: " << a.y <<  "\ttheta: " << a.theta;
    // o << "\ncovariance: " << std::endl << a.cov;
    return o;
}

Eigen::Vector3d StructToVector(const Vector& vector)
{
    // ROS_INFO("Converting x: %.3f, y: %.3f, theta:%.3f to vector", vector.x, vector.y, vector.theta);
    Eigen::VectorXd v(3);
    v << vector.x, vector.y, vector.theta;

    return v;
}

Vector VectorToStruct(const Eigen::VectorXd& vector)
{
    // ROS_INFO("Converting vector to struct");
    // std::cout << vector << endl;
    Vector v;
    v.x = vector(0);
    v.y = vector(1);
    v.theta = vector(2);

    // ROS_INFO("Converted vector to struct");
    return v;
}

class MotionModel
{
    public:
        MotionModel()
        : m_initialStateSet(false)
        , m_newInput(false)
        {
        }

        ~MotionModel(){}

        void SetInitialState(const Position& initialState)
        {
            m_currentState = initialState;
            m_initialStateSet = true;
        }

        bool IsInitialSet()
        {
            return m_initialStateSet;
        }

        void UpdateStateInternal(double timeDelta)
        {
            // ROS_INFO("Updating state:");

            m_currentState = UpdateState(m_currentState, timeDelta);
        }

        Position UpdateState(const Position& currentState, double timeDelta)
        {
            // Fix velocity
            Eigen::Vector3d newStateVector;
            Position oldState;
            Position newState;
            Eigen::Vector3d xPrev;
            Eigen::Vector3d v;
            Eigen::Vector3d e;
            Velocity velocity;

            velocity = m_currentVelocity;
            velocity.x = velocity.x * std::cos(currentState.theta);
            velocity.y = velocity.y * std::sin(currentState.theta);
            xPrev = StructToVector(currentState);
            v = StructToVector(velocity);

            // Error generator
            std::normal_distribution<> posDistr(0, 0.1);
            std::normal_distribution<> angDistr(0, 0.1);
            e(0) = posDistr(kENG);
            e(1) = posDistr(kENG);
            e(2) = angDistr(kENG);

            newStateVector = A*xPrev + timeDelta*v + e;
            newState = VectorToStruct(newStateVector);

            // Handle rollover outside 0 -> 2pi range
            if (newState.theta > 2*M_PI)
            {
                newState.theta -= 2*M_PI;
            } else if( newState.theta < 0)
            {
                newState.theta += 2*M_PI;
            }

            return newState;
        }

        void JoystickCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            m_newInput = true;
        }

        void VelocityCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            Velocity input = {0, 0, 0};

            input.x = msg->twist.twist.linear.x;
            input.y = msg->twist.twist.linear.x;
            input.theta = msg->twist.twist.angular.z;

            m_currentVelocity = input;
        }

        Position GetCurrentState()
        {
            // ROS_DEBUG("Getting current state");
            return m_currentState;
        }

        Position GetInput()
        {
            // ROS_DEBUG("Getting current state");
            return m_currentVelocity;
        }

        bool NewInput()
        {
            bool ans = m_newInput;
            m_newInput = false;

            return ans;
        }

    private:
        const Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        Position m_currentState;
        Velocity m_currentVelocity;
        visualization_msgs::MarkerArray m_markers;
        bool m_initialStateSet;
        bool m_newInput;
};

class MeasurementModel
{
    public:
        MeasurementModel(std::string frameId)
        : m_publisher(NULL)
        , m_currentMeasurement({0, 0, 0})
        , m_frameId(frameId)
        {
            // Initialize covariance
            m_covSim << std::pow(0.1, 2), 0, 0,
                        0, std::pow(0.1, 2), 0,
                        0, 0, 0.02;

            m_Marker.header.frame_id = m_frameId;
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

            // Initialize path marker
            m_pathMarker.header.frame_id = m_frameId;
            m_pathMarker.header.stamp = ros::Time();
            m_pathMarker.ns = "robot_path";
            m_pathMarker.id = 2;
            m_pathMarker.type = visualization_msgs::Marker::LINE_STRIP;
            m_pathMarker.action = visualization_msgs::Marker::ADD;
            m_pathMarker.scale.x = 0.03;
            m_pathMarker.scale.y = 0.0;
            m_pathMarker.color.a = 1.0;
            m_pathMarker.color.r = 0.0;
            m_pathMarker.color.g = 0.2;
            m_pathMarker.color.b = 1.0;
            
        }

        ~MeasurementModel()
        {
        }

        void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
        {
            Position measurement;
            measurement.x = msg->pose.pose.position.x;
            measurement.y = msg->pose.pose.position.y;
            measurement.theta = tf::getYaw(msg->pose.pose.orientation);
            measurement.cov = m_covSim;
            UpdateMeasurement(measurement);
        }

        void PoseCallbackSim(const gazebo_msgs::ModelStates::ConstPtr& msg)
        {
            int i;
            for (i = 0; i < msg->name.size(); i++)
                if (msg->name[i] == m_simMsgName)
                    break;

            Position measurement;
            measurement.x = msg->pose[i].position.x;
            measurement.y = msg->pose[i].position.y;
            measurement.theta = tf::getYaw(msg->pose[i].orientation);
            measurement.cov = m_covSim;
            UpdateMeasurement(measurement);
        }

        void UpdateMeasurement(const Position& measurement)
        {
            m_currentMeasurement = measurement;

            // Transform from -pi to pi -> 0 to 2pi
            if(m_currentMeasurement.theta < 0)
            {
                m_currentMeasurement.theta += M_PI * 2;
            }

            Visualize();
        }

        Position GetMeasurement()
        {
            return m_currentMeasurement;
        }

        void SetPublisher(const ros::Publisher *publisher, const ros::Publisher *pathPublisher)
        {
            m_publisher = publisher;
            m_pathPublisher = pathPublisher;
        }
    
    private:
        const ros::Publisher *m_publisher;
        const ros::Publisher *m_pathPublisher;
        const std::string m_simMsgName = "mobile_base";
        Eigen::Matrix3d m_covSim;
        Position m_currentMeasurement;
        visualization_msgs::Marker m_pathMarker;
        visualization_msgs::Marker m_Marker;
        std::string m_frameId;

        void Visualize()
        {
            tf2::Quaternion q;
            m_Marker.header.stamp = ros::Time();
            m_Marker.pose.position.x = m_currentMeasurement.x;
            m_Marker.pose.position.y = m_currentMeasurement.y;
            m_Marker.pose.position.z = 0;
            q.setRPY(0, 0, m_currentMeasurement.theta);
            m_Marker.pose.orientation.w = q.getW();
            m_Marker.pose.orientation.x = q.getX();
            m_Marker.pose.orientation.y = q.getY();
            m_Marker.pose.orientation.z = q.getZ();


            geometry_msgs::Point p;
            p.x = m_currentMeasurement.x;
            p.y = m_currentMeasurement.y;
            p.z = 0;
            m_pathMarker.points.push_back(p);
            if(m_publisher)
            {
                m_publisher->publish(m_Marker);
            }
            if(m_pathPublisher)
            {
                m_pathPublisher->publish(m_pathMarker);
            }
        }
};

class ParticleFilter
{
    struct Particle
    {
        Position pos;
        double weight;
    };
    public:
        ParticleFilter(std::string& frameId, 
                       uint32_t particleCount,
                       double xMin,
                       double xMax,
                       double yMin,
                       double yMax,
                       MotionModel *motionModel)
        : m_particlesPublisher(NULL)
        , m_estimatePublisher(NULL)
        , m_frameId(frameId)
        , m_xMin(xMin)
        , m_xMax(xMax)
        , m_yMin(yMin)
        , m_yMax(yMax)
        , m_motionModel(motionModel)
        , m_nParticles(particleCount)
        {
            tf2::Quaternion q;
            double xSum = 0;
            double ySum = 0;
            double thetaSum = 0;
            // Initialize particles
            std::uniform_real_distribution<> thetaDistr(0, 2*M_PI);
            std::uniform_real_distribution<> xDistr(m_xMin, m_xMax); 
            std::uniform_real_distribution<> yDistr(m_yMin, m_yMax); 

            m_particles.resize(m_nParticles);

            // Initialize points
            m_pointsMsg.poses.resize(m_nParticles);
            m_pointsMsg.header.frame_id = m_frameId;
            m_pointsMsg.header.stamp = ros::Time::now();
            
            // Initialize estimate markers
            m_estimateMarker.header.frame_id = m_frameId;
            m_estimateMarker.header.stamp = ros::Time();
            m_estimateMarker.ns = "robot_estimate";
            m_estimateMarker.id = 0;
            m_estimateMarker.type = visualization_msgs::Marker::SPHERE;
            m_estimateMarker.action = visualization_msgs::Marker::ADD;
            m_estimateMarker.scale.x = 0.8;
            m_estimateMarker.scale.y = 0.2;
            m_estimateMarker.scale.z = 0.1;
            m_estimateMarker.color.a = 1.0;
            m_estimateMarker.color.r = 1.0;
            m_estimateMarker.color.g = 0.0;
            m_estimateMarker.color.b = 0.3;
            
            // Initialize path marker
            m_pathMarker.header.frame_id = m_frameId;
            m_pathMarker.header.stamp = ros::Time();
            m_pathMarker.ns = "robot_estimate";
            m_pathMarker.id = 1;
            m_pathMarker.type = visualization_msgs::Marker::LINE_STRIP;
            m_pathMarker.action = visualization_msgs::Marker::ADD;
            m_pathMarker.scale.x = 0.03;
            m_pathMarker.scale.y = 0.0;
            m_pathMarker.color.a = 1.0;
            m_pathMarker.color.r = 1.0;
            m_pathMarker.color.g = 0.6;
            m_pathMarker.color.b = 0.1;

            for(uint32_t i = 0; i < m_nParticles; i++)
            {
                m_particles[i].pos.x = xDistr(kENG);
                m_particles[i].pos.y = yDistr(kENG);
                m_particles[i].pos.theta = thetaDistr(kENG);
                m_particles[i].weight = 1.0/m_nParticles;

                // std::cout << std::setprecision(4) << m_particles[i].pos << "\tweight: " << m_particles[i].weight << std::endl;

                xSum += m_particles[i].pos.x;
                ySum += m_particles[i].pos.y;
                thetaSum += m_particles[i].pos.theta;

                m_pointsMsg.poses[i].position.x = m_particles[i].pos.x;
                m_pointsMsg.poses[i].position.y = m_particles[i].pos.y;
                m_pointsMsg.poses[i].position.z = 0;
                q.setRPY(0, 0, m_particles[i].pos.theta);
                m_pointsMsg.poses[i].orientation.w = q.getW();
                m_pointsMsg.poses[i].orientation.x = q.getX();
                m_pointsMsg.poses[i].orientation.y = q.getY();
                m_pointsMsg.poses[i].orientation.z = q.getZ();
            }

            // Sort particles based on theta
            std::sort(m_particles.begin(), m_particles.end(), ThetaCompare);
            m_posEstimate = {
                xSum/m_nParticles,
                ySum/m_nParticles,
                m_particles[m_nParticles/2].pos.theta // median theta
            };
        }
        ~ParticleFilter(){}

        void SetPublishers(ros::Publisher *particlesPublisher, ros::Publisher *estimatePublisher, ros::Publisher *pathPublisher)
        {
            m_particlesPublisher = particlesPublisher;
            m_estimatePublisher = estimatePublisher;
            m_pathPublisher = pathPublisher;
        }

        Particle Run(const Position& measurement, 
                     const Velocity& input,
                     double timeDelta)
        {   
            tf2::Quaternion q;
            double xSum = 0;
            double ySum = 0;
            double thetaSum = 0;
            double beta = 0;
            double wMax = 0;
            uint32_t index;
            std::vector <Particle> particlePredictions;
            particlePredictions.resize(m_nParticles);

            // Prediction and likelihood
            for(uint32_t i = 0; i < m_nParticles; i++)
            {
                particlePredictions[i].pos = m_motionModel->UpdateState(m_particles[i].pos, timeDelta);
                // if(i == 0) std::cout << particlePredictions[i].pos.theta << std::endl;
                particlePredictions[i].weight = Likelihood(measurement, particlePredictions[i].pos, measurement.cov);
                // std::cout <<  particlePredictions[i].weight << std::endl;
                wMax = wMax > particlePredictions[i].weight ? wMax : particlePredictions[i].weight;
            }

            // Resampling
            m_pointsMsg.header.stamp = ros::Time::now();
            std::uniform_int_distribution<> indexDistr(0, m_nParticles-1);
            std::uniform_real_distribution<> wDistr(0, 2*wMax);
            index = indexDistr(kENG);
            for(uint32_t i = 0; i < m_nParticles; i++)
            {
                // std::cout << std::setprecision(4) << m_particles[i].pos << "\tweight: " << m_particles[i].weight << std::endl;
                beta += wDistr(kENG);
                while (beta > particlePredictions[index].weight)
                {
                    beta -= particlePredictions[index].weight;
                    index = (index+1)%m_nParticles;
                }

                m_particles[i] = particlePredictions[index];

                m_pointsMsg.poses[i].position.x = m_particles[i].pos.x;
                m_pointsMsg.poses[i].position.y = m_particles[i].pos.y;
                m_pointsMsg.poses[i].position.z = 0;
                
                q.setRPY(0, 0, m_particles[i].pos.theta);
                m_pointsMsg.poses[i].orientation.w = q.getW();
                m_pointsMsg.poses[i].orientation.x = q.getX();
                m_pointsMsg.poses[i].orientation.y = q.getY();
                m_pointsMsg.poses[i].orientation.z = q.getZ();

                xSum += m_particles[i].pos.x;
                ySum += m_particles[i].pos.y;
                thetaSum += m_particles[i].pos.theta;
            }

            // Sort particles based on theta
            std::sort(m_particles.begin(), m_particles.end(), ThetaCompare);

            m_posEstimate = {
                xSum/m_nParticles,
                ySum/m_nParticles,
                m_particles[m_nParticles/2].pos.theta // median theta
            };
            q.setRPY(0, 0, m_posEstimate.theta);

            m_estimateMarker.header.stamp = ros::Time();
            m_estimateMarker.pose.position.x = m_posEstimate.x;
            m_estimateMarker.pose.position.y = m_posEstimate.y;
            m_estimateMarker.pose.position.z = 0;
            m_estimateMarker.pose.orientation.w = q.getW();
            m_estimateMarker.pose.orientation.x = q.getX();
            m_estimateMarker.pose.orientation.y = q.getY();
            m_estimateMarker.pose.orientation.z = q.getZ();
            
            geometry_msgs::Point p;
            p.x = m_posEstimate.x;
            p.y = m_posEstimate.y;
            p.z = 0;
            m_pathMarker.points.push_back(p);

            // std::cout << "------------------------------------------------------------------" << std::endl;
            // std::cout << "E" << m_posEstimate << std::endl;
            // std::cout << "M" << measurement << std::endl;
        
            // Publish
            Publish();
        }

        void Publish()
        {
            if(m_particlesPublisher)
            {
                m_particlesPublisher->publish(m_pointsMsg);
            }
            if(m_estimatePublisher)
            {
                m_estimatePublisher->publish(m_estimateMarker);
            }
            if(m_pathPublisher)
            {
                m_pathPublisher->publish(m_pathMarker);
            }
        }

        Position GetPostion()
        {
            return m_posEstimate;
        }

    private:
        MotionModel *m_motionModel;
        uint32_t m_nParticles;
        double m_xMin;
        double m_xMax;
        double m_yMin;
        double m_yMax;
        Position m_posEstimate;
        const std::string m_particleType = "points";
        static constexpr uint32_t k = 3; // array dimension
        static constexpr double m_particleSize = 0.1;
        ros::Publisher *m_particlesPublisher;
        ros::Publisher *m_estimatePublisher;
        ros::Publisher *m_pathPublisher;
        geometry_msgs::PoseArray m_pointsMsg;
        visualization_msgs::Marker m_estimateMarker;
        visualization_msgs::Marker m_pathMarker;
        std::vector <Particle> m_particles;
        std::string m_frameId; 

        static bool ThetaCompare (const Particle& A, 
                                  const Particle& B) 
        { 
            return A.pos.theta < B.pos.theta; 
        }

        double Likelihood(const Position& measurement, 
                          const Position& prediction,
                          const Eigen::Matrix3d& cov)
        {

            Eigen::Vector3d mu = StructToVector(prediction);
            Eigen::Vector3d y = StructToVector(measurement);
            Eigen::Vector3d b = y - mu;
            double likelihood = exp(-0.5*(std::log(cov.determinant())+(b).transpose()*cov.inverse()*(b) + k*std::log(2*M_PI)));

            return likelihood;
        }
        
};


int main(int argc, char **argv)
{
    // Static variables
    std::string frame = "/odom";
    uint32_t nParticles = 150;
#ifdef SIMULATION
    double xMin = -3;
    double xMax = 3;
    double yMin = -3;
    double yMax = 3;
#else
    double xMin = -1;
    double xMax = 5;
    double yMin = -2;
    double yMax = 6;
#endif

    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    Position measurement;
    MotionModel motionModel;
    MeasurementModel measurementModel(frame);
    ParticleFilter filter(frame, nParticles, xMin, xMax, yMin, yMax, &motionModel);

   
    //Subscribe to the desired topics and assign callbacks
#ifdef SIMULATION
    ros::Subscriber poseSub = n.subscribe("/gazebo/model_states", 1, &MeasurementModel::PoseCallbackSim, &measurementModel);
#else
    ros::Subscriber poseSub = n.subscribe("/indoor_pos", 1, &MeasurementModel::PoseCallback, &measurementModel);
#endif
    ros::Subscriber joystickSub = n.subscribe("/cmd_vel_mux/input/teleop", 1, &MotionModel::JoystickCallback, &motionModel);
    ros::Subscriber velocitySub = n.subscribe("/odom", 1, &MotionModel::VelocityCallback, &motionModel);
    ros::Publisher particlesPublisher = n.advertise<geometry_msgs::PoseArray>("/particles_", 1, true);
    ros::Publisher measurementPublisher = n.advertise<visualization_msgs::Marker>( "/robotMeasurement_", 0 );
    ros::Publisher estimatePublisher = n.advertise<visualization_msgs::Marker>( "/robotEstimate_", 0 );
    ros::Publisher pathPublisher = n.advertise<visualization_msgs::Marker>( "/robotPath_", 0 );
    ros::Publisher truePathPublisher = n.advertise<visualization_msgs::Marker>( "/trueRobotPath_", 0 );
   
    filter.SetPublishers(&particlesPublisher, &estimatePublisher, &pathPublisher);
    filter.Publish();
    measurementModel.SetPublisher(&measurementPublisher, &truePathPublisher);
    
    // //Velocity control variable
    // geometry_msgs::Twist vel;

    //Set the loop rate
    double timeDelta = 0.0;
    double end;
    double begin = ros::Time::now().toSec();
    ros::Rate loop_rate(10);    //20Hz update rate
    Velocity input = {0, 0, 0};
    bool run = false;

    while (ros::ok())
    {
    	ros::spinOnce();   //Check for new messages
    
        input = motionModel.GetInput();
        run = std::fabs(input.x) > 0.005 || std::fabs(input.y) > 0.005 || std::fabs(input.theta) > 0.005;

        // Update measurement
        measurement = measurementModel.GetMeasurement();

        // Only run if robot is moving
        if (run)
        {
            // ROS_INFO("Run");
            end = ros::Time::now().toSec();
            timeDelta = end-begin;
            begin = end;
            // ROS_INFO("Running %.5f", timeDelta);
            filter.Run(measurement, input, timeDelta);
        }

        loop_rate.sleep(); //Maintain the loop rate
    }
    return 0;
}