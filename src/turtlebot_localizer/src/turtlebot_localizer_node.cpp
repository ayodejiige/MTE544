//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>

#define SIMULATION

static std::random_device rd; // obtain a random number from hardware
static std::mt19937 kENG(rd()); // seed the generator

struct Vector
{
    double x;
    double y;
    double theta;
    Eigen::Matrix3d cov;
    
};

std::ostream& operator << (std::ostream &o, const Vector &a)
{
    o << "x: " << a.x << "\ty: " << a.y <<  "\ttheta: " << a.theta;
    o << "\ncovariance: " << std::endl << a.cov;
    return o;
}

typedef Vector Position;
typedef Vector Velocity;


Eigen::Vector3d StructToVector(Vector vector)
{
    // ROS_INFO("Converting x: %.3f, y: %.3f, theta:%.3f to vector", vector.x, vector.y, vector.theta);
    Eigen::VectorXd v(3);
    v << vector.x, vector.y, vector.theta;

    return v;
}

Vector VectorToStruct(Eigen::VectorXd vector)
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
        {
        }

        ~MotionModel(){}

        void SetInitialState(Position initialState)
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

        Position UpdateState(Position currentState, double timeDelta)
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
            std::normal_distribution<> angDistr(0, 0.17);
            e(0) = posDistr(kENG);
            e(1) = posDistr(kENG);
            e(2) = angDistr(kENG);
            
            // No error
            // e(0) = 0.0;
            // e(1) = 0.0;
            // e(2) = 0.0;

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

        void VelocityCallback(const geometry_msgs::Twist &msg)
        {
            Velocity input = {0, 0, 0};

            input.x = msg.linear.x;
            input.y = msg.linear.x;
            input.theta = msg.angular.z;

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

    private:
        const Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        Position m_currentState;
        Velocity m_currentVelocity;
        visualization_msgs::MarkerArray m_markers;
        bool m_initialStateSet;
};

class MeasurementModel
{
    public:
        MeasurementModel()
        : m_currentMeasurement({0, 0, 0})
        , m_firstMeasurement(false)
        {
            // Initialize covariance
            m_covSim << std::pow(0.1, 2), 0, 0,
                        0, std::pow(0.1, 2), 0,
                        0, 0, std::pow(0.1, 2);
        }

        ~MeasurementModel()
        {
        }

        bool IsFirstMeasurementSet()
        {
            return m_firstMeasurement;
        }

        void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
        {
            Position measurement;
            measurement.x = msg->pose.pose.position.x;
            measurement.y = msg->pose.pose.position.y;
            measurement.theta = tf::getYaw(msg->pose.pose.orientation);
            measurement.cov(0,0) = (msg->pose.covariance)[0];
            measurement.cov(0,1) = (msg->pose.covariance)[1];
            measurement.cov(0,2) = (msg->pose.covariance)[5];
            measurement.cov(1,0) = (msg->pose.covariance)[6];
            measurement.cov(1,1) = (msg->pose.covariance)[7];
            measurement.cov(1,2) = (msg->pose.covariance)[11];
            measurement.cov(2,0) = (msg->pose.covariance)[30];
            measurement.cov(2,1) = (msg->pose.covariance)[31];
            measurement.cov(2,2) = (msg->pose.covariance)[35];
            updateMeasurement(measurement);
        }

        void PoseCallbackSim(const gazebo_msgs::ModelStates &msg)
        {
            int i;
            for (i = 0; i < msg.name.size(); i++)
                if (msg.name[i] == m_simMsgName)
                    break;

            Position measurement;
            measurement.x = msg.pose[i].position.x;
            measurement.y = msg.pose[i].position.y;
            measurement.theta = tf::getYaw(msg.pose[i].orientation);
            measurement.cov = m_covSim;
            updateMeasurement(measurement);
        }

        void updateMeasurement(Position measurement)
        {
            // Transform from -pi to pi -> 0 to 2pi
            if(measurement.theta < 0)
            {
                measurement.theta += M_PI * 2;
            }

            m_currentMeasurement = measurement;

            if (!m_firstMeasurement)
            {
                m_firstMeasurement = true;
            }
        }

        Position GetMeasurement()
        {
            return m_currentMeasurement;
        }
    
    private:
        const std::string m_simMsgName = "mobile_base";
        Eigen::Matrix3d m_covSim;
        Position m_currentMeasurement;
        bool m_firstMeasurement;
};

class ParticleFilter
{
    struct Particle
    {
        Position pos;
        double weight;
    };
    public:
        ParticleFilter(std::string frameId, uint32_t particleCount, double xMin, double xMax, double yMin, double yMax, MotionModel *motionModel)
        : m_particlesPublisher(NULL)
        , m_frameId(frameId)
        , m_xMin(xMin)
        , m_xMax(xMax)
        , m_yMin(yMin)
        , m_yMax(yMax)
        , m_motionModel(motionModel)
        , m_nParticles(particleCount)
        {
            // Initialize particles
            std::uniform_real_distribution<> thetaDistr(0, 2*M_PI);
            std::uniform_real_distribution<> xDistr(m_xMin, m_xMax); 
            std::uniform_real_distribution<> yDistr(m_yMin, m_yMax); 

            m_particles.resize(m_nParticles);
            
            // Initialize points
            m_points.header.frame_id = m_frameId;
            m_points.header.stamp = ros::Time::now();
            m_points.ns = m_particleType;
            m_points.action = visualization_msgs::Marker::ADD;
            m_points.pose.orientation.w = 1.0;
            m_points.id = 0;
            m_points.type = visualization_msgs::Marker::POINTS;
            m_points.scale.x = m_particleSize;
            m_points.scale.y = m_particleSize;
            m_points.color.g = 1.0f;
            m_points.color.a = 1.0;

            for(uint32_t i = 0; i < m_nParticles; i++)
            {
                m_particles[i].pos.x = xDistr(kENG);
                m_particles[i].pos.y = yDistr(kENG);
                m_particles[i].pos.theta = thetaDistr(kENG);
                m_particles[i].weight = 1.0/m_nParticles;

                // std::cout << std::setprecision(4) << m_particles[i].pos << "\tweight: " << m_particles[i].weight << std::endl;

                geometry_msgs::Point p;
                p.x = m_particles[i].pos.x;
                p.y = m_particles[i].pos.y;
                p.z = 0;
                m_points.points.push_back(p);
            }

        }
        ~ParticleFilter(){}

        void SetPublisher(ros::Publisher *particlesPublisher)
        {
            m_particlesPublisher = particlesPublisher;
        }

        void Resample()
        {

        }

        void PublishParticles()
        {
            if(m_particlesPublisher)
            {
                m_particlesPublisher->publish(m_points);
            }
        }

        Particle Run(Position measurement, Velocity input, double timeDelta)
        {   
            // double wSum = 0;
            double beta;
            double wMax = 0;
            uint32_t index;
            std::vector <Particle> particlePredictions;
            particlePredictions.resize(m_nParticles);

            // Prediction and likelihood
            for(uint32_t i = 0; i < m_nParticles; i++)
            {
                particlePredictions[i].pos = m_motionModel->UpdateState(m_particles[i].pos, timeDelta);
                particlePredictions[i].weight = Likelihood(measurement, particlePredictions[i].pos, measurement.cov);
                // std::cout <<  particlePredictions[i].weight << std::endl;
                wMax = wMax > particlePredictions[i].weight ? wMax : particlePredictions[i].weight;
            }

            // Resampling
            m_points.header.stamp = ros::Time::now();
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
                geometry_msgs::Point p;
                p.x = m_particles[i].pos.x;
                p.y = m_particles[i].pos.y;
                p.z = 0;
                m_points.points[i] = p;

            }

            // Publish
            PublishParticles();
        }

        double Likelihood(Position measurement, Position prediction, Eigen::Matrix3d cov)
        {

            Eigen::Vector3d mu = StructToVector(prediction);
            Eigen::Vector3d y = StructToVector(measurement);
            Eigen::Vector3d b = y - mu;
            double likelihood = exp(-0.5*(std::log(cov.determinant())+(b).transpose()*cov.inverse()*(b) + k*std::log(2*M_PI)));

            // std::cout << cov << std::endl;
            // std::cout << mu << std::endl;
            // std::cout << y << std::endl;
            // std::cout << b << std::endl;

            return likelihood;
        }
    private:
        const std::string m_particleType = "points";
        static constexpr uint32_t k = 3; // array dimension
        static constexpr double m_particleSize = 0.1;
        ros::Publisher *m_particlesPublisher;
        visualization_msgs::Marker m_points;
        uint32_t m_nParticles;
        MotionModel *m_motionModel;
        std::vector <Particle> m_particles;
        std::string m_frameId;   
        double m_xMin;
        double m_xMax;
        double m_yMin;
        double m_yMax;
};


int main(int argc, char **argv)
{
    // Static variables
    static std::string particleFrame = "/map";

    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    Position initialState;
    MotionModel motionModel;
    MeasurementModel measurementModel;
    ParticleFilter filter("/map", 1000, -10, 10, -10, 10, &motionModel);

    //Subscribe to the desired topics and assign callbacks
#ifdef SIMULATION
    ros::Subscriber poseSub = n.subscribe("/gazebo/model_states", 1, &MeasurementModel::PoseCallbackSim, &measurementModel);
#else
    ros::Subscriber poseSub = n.subscribe("/indoor_pos", 1, &MeasurementModel::PoseCallback, &measurementModel);
#endif
    ros::Subscriber velocitySub = n.subscribe("/cmd_vel_mux/input/teleop", 1, &MotionModel::VelocityCallback, &motionModel);
    ros::Publisher particlesPublisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    

    filter.SetPublisher(&particlesPublisher);
    filter.PublishParticles();

    //Setup topics to Publish from this node
    // ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    // pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    // marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    Velocity input = {0, 0, 0};
    bool run = false;

    while (ros::ok())
    {
    	ros::spinOnce();   //Check for new messages
        
        // motionModel.UpdateStateInternal(0.05);
        input = motionModel.GetInput();
        run = std::fabs(input.x) > 0.005 || std::fabs(input.y) > 0.005 || std::fabs(input.theta) > 0.005;

        // Only run if robot is moving
        if (run) filter.Run(measurementModel.GetMeasurement(), input, 0.05);

        // filter.PublishParticles();
        // ROS_INFO("Likelihood %.5f", filter.Likelihood({20, 0.5, M_PI/2}, {0.3, -0.5, M_PI/6}));
        loop_rate.sleep(); //Maintain the loop rate
    }
    return 0;
}