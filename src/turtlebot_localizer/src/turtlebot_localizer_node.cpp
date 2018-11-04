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
    o << "covariance: " << std::endl << a.cov;
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
            // ROS_INFO("Updating state:");

            Eigen::VectorXd newStateVector(3);
            Position oldState;
            Position newState;
            Eigen::Vector3d xPrev = StructToVector(currentState);
            Eigen::Vector3d v = StructToVector(m_currentVelocity);
            Eigen::Vector3d e;


            // Error generator
            std::random_device rd; // obtain a random number from hardware
            std::mt19937 eng(rd()); // seed the generator
            std::uniform_real_distribution<> predDistr(-0.1, 0.1);
            e(0) = predDistr(eng);
            e(1) = predDistr(eng);
            e(2) = predDistr(eng);
            

            newStateVector = A*xPrev + timeDelta*v;
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

            input.x = msg.linear.x * std::cos(m_currentState.theta);
            input.y = msg.linear.x * std::sin(m_currentState.theta);
            input.theta = msg.angular.z;

            m_currentVelocity = input;

            // std::cout << "state" << std::endl;
            // std::cout << m_currentState << std::endl;
            // std::cout << "input" << std::endl;
            // std::cout << m_currentVelocity << std::endl;
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
        const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
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
        }

        ~MeasurementModel()
        {
        }

        bool IsFirstMeasurementSet()
        {
            return m_firstMeasurement;
        }

        void PoseCallback(const gazebo_msgs::ModelStates &msg)
        {
            int i;
            for (i = 0; i < msg.name.size(); i++)
                if (msg.name[i] == "mobile_base")
                    break;

            Position measurement;
            measurement.x = msg.pose[i].position.x;
            measurement.y = msg.pose[i].position.y;
            measurement.theta = tf::getYaw(msg.pose[i].orientation);

            // Tranform from -pi to pi -> 0 to 2pi
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
            // Initialize covariance
            Q << std::pow(0.01, 2), 0, 0,
                 0, std::pow(0.01, 2), 0,
                 0, 0, std::pow(0.001, 2);

            // Initialize particles
            std::random_device rd; // obtain a random number from hardware
            std::mt19937 eng(rd()); // seed the generator
            std::uniform_real_distribution<> thetaDistr(-M_PI, M_PI);
            std::uniform_real_distribution<> xDistr(m_xMin, m_xMax); 
            std::uniform_real_distribution<> yDistr(m_yMin, m_yMax); 

            m_particles.resize(m_nParticles);
            
            // Initialize points
            m_points.header.frame_id = m_frameId;
            m_points.header.stamp = ros::Time::now();
            m_points.ns = "points";
            m_points.action = visualization_msgs::Marker::ADD;
            m_points.pose.orientation.w = 1.0;
            m_points.id = 0;
            m_points.type = visualization_msgs::Marker::POINTS;
            m_points.scale.x = m_particleSize;
            m_points.scale.y = m_particleSize;
            m_points.color.g = 1.0f;
            m_points.color.a = 1.0;

            for(std::vector<int>::size_type i = 0; i != m_particles.size(); i++)
            {
                m_particles[i].pos.x = xDistr(eng);
                m_particles[i].pos.y = yDistr(eng);
                m_particles[i].pos.theta = thetaDistr(eng);
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
            // visualization_msgs::Marker points;

            // std::random_device rd; // obtain a random number from hardware
            // std::mt19937 eng(rd()); // seed the generator
            // std::uniform_real_distribution<> thetaDistr(-M_PI, M_PI);
            // std::uniform_real_distribution<> xDistr(m_xMin, m_xMax); 
            // std::uniform_real_distribution<> yDistr(m_yMin, m_yMax); 

            // // Initialize points
            // points.header.frame_id = m_frameId;
            // points.header.stamp = ros::Time::now();
            // points.ns = "points";
            // points.action = visualization_msgs::Marker::ADD;
            // points.pose.orientation.w = 1.0;
            // points.id = 0;
            // points.type = visualization_msgs::Marker::POINTS;
            // points.scale.x = m_particleSize;
            // points.scale.y = m_particleSize;
            // points.color.g = 1.0f;
            // points.color.a = 1.0;

            // for(std::vector<int>::size_type i = 0; i != m_particles.size(); i++)
            // {
            //     m_particles[i].pos.x = xDistr(eng);
            //     m_particles[i].pos.y = yDistr(eng);
            //     m_particles[i].pos.theta = thetaDistr(eng);

            //     // std::cout << std::setprecision(4) << m_particles[i].pos << "\tweight: " << m_particles[i].weight << std::endl;

            //     geometry_msgs::Point p;
            //     p.x = m_particles[i].pos.x;
            //     p.y = m_particles[i].pos.y;
            //     p.z = 0;
            //     points.points.push_back(p);
            // }

            // m_points = points;
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
            std::vector <Position> particlePredictions;
            particlePredictions.resize(m_nParticles);

            // Prediction and likelihood
            for(std::vector<int>::size_type i = 0; i != m_particles.size(); i++)
            {
                particlePredictions[i] = m_motionModel->UpdateState(m_particles[i].pos, timeDelta);
            }

            // Resampling
            m_points.header.stamp = ros::Time::now();
            for(std::vector<int>::size_type i = 0; i != m_particles.size(); i++)
            {
                // std::cout << std::setprecision(4) << m_particles[i].pos << "\tweight: " << m_particles[i].weight << std::endl;

                geometry_msgs::Point p;
                p.x = particlePredictions[i].x;
                p.y = particlePredictions[i].y;
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
            double likelihood = exp(-0.5*(std::log(Q.determinant())+(b).transpose()*cov.inverse()*(b) + k*std::log(2*M_PI)));

            // std::cout << cov << std::endl;
            // std::cout << mu << std::endl;
            // std::cout << y << std::endl;
            // std::cout << b << std::endl;

            return likelihood;
        }
    private:
        Eigen::Matrix3d Q;
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
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    Position initialState;
    MotionModel motionModel;
    MeasurementModel measurementModel;
    ParticleFilter filter("/map", 100, -4, 4, -4, 4, &motionModel);

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber poseSub = n.subscribe("/gazebo/model_states", 1, &MeasurementModel::PoseCallback, &measurementModel);
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
        run = std::fabs(input.x) > 0.005 || std::fabs(input.y) > 0.005;
        std::cout << run << std::endl;

        // Only run if robot is moving
        if (run) filter.Run(measurementModel.GetMeasurement(), input, 0.05);

        // filter.PublishParticles();
        // ROS_INFO("Likelihood %.5f", filter.Likelihood({20, 0.5, M_PI/2}, {0.3, -0.5, M_PI/6}));
        loop_rate.sleep(); //Maintain the loop rate
    }
    return 0;
}