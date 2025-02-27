#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <mutex>

namespace rona_physical
{

class ViconObstacleLayerAS5 : public nav2_costmap_2d::CostmapLayer
{
public:
    void onInitialize() override
    {
        ros_node_ = rclcpp::Node::make_shared("vicon_obstacle_layerAS5");
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        declareParameter("topic", rclcpp::ParameterValue("/AS_5_neu/global_pose"));
        
        if (!ros_node_->get_parameter("topic", topic_) || topic_.empty())
        {
            topic_ = "/AS_5_neu/global_pose"; 
            RCLCPP_WARN(ros_node_->get_logger(), "⚠️ Parameter 'topic' is missing or empty. Using default: %s", topic_.c_str());
        }

        declareParameter("target_frame", rclcpp::ParameterValue("map"));
        if (!ros_node_->get_parameter("target_frame", target_frame_) || target_frame_.empty())
        {
            target_frame_ = "map";  
            RCLCPP_WARN(ros_node_->get_logger(), "⚠️ Parameter 'target_frame' is missing or empty. Using default: %s", target_frame_.c_str());
        }

        RCLCPP_INFO(ros_node_->get_logger(), "🔍 Subscribing to topic: %s", topic_.c_str());

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_, qos, std::bind(&ViconObstacleLayerAS5::obstacleCallback, this, std::placeholders::_1));

        current_ = true;

        executor_thread_ = std::thread([this]() {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(ros_node_);
            executor.spin();
        });

        RCLCPP_INFO(ros_node_->get_logger(), " Subscription created successfully.");
    }



    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        double expansion_x = 64.0;  // ⬆️ Increase obstacle width
        double expansion_y = 130.0;  // ⬆️ Increase obstacle height

        for (const auto& obs : obstacles_)
        {
            *min_x = std::min(*min_x, obs.x - expansion_x);
            *min_y = std::min(*min_y, obs.y - expansion_y);
            *max_x = std::max(*max_x, obs.x + expansion_x);
            *max_y = std::max(*max_y, obs.y + expansion_y);
        }
    }
    void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_x, int min_y, int max_x, int max_y) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        double obstacle_size = 0.5;  // ⬆️ Increase obstacle area

        for (const auto& obs : obstacles_)
        {
            unsigned int mx, my;
            if (master_grid.worldToMap(obs.x, obs.y, mx, my))
            {
                // Spread the obstacle over a larger area
                for (int dx = -10; dx <= 10; ++dx)
                {
                    for (int dy = -6; dy <= 6; ++dy)
                    {
                        unsigned int mark_x = mx + dx;
                        unsigned int mark_y = my + dy;

                        if (mark_x < master_grid.getSizeInCellsX() && mark_y < master_grid.getSizeInCellsY())
                        {
                            master_grid.setCost(mark_x, mark_y, nav2_costmap_2d::LETHAL_OBSTACLE);
                        }
                    }
                }

                // RCLCPP_INFO(ros_node_->get_logger(), "Set obstacle at costmap cells near (%d, %d)", mx, my);
            }
        }
    }


    void reset() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        obstacles_.clear();
        current_ = true;  
    }

    bool isClearable() override
    {
        return true;
    }

private:
    std::thread executor_thread_;
    void obstacleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // RCLCPP_INFO(ros_node_->get_logger(), "Received pose: x=%.2f, y=%.2f, frame=%s, time=%.2f",
        //             msg->pose.position.x, msg->pose.position.y, msg->header.frame_id.c_str(),
        //             msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

        try
        {
            // Lookup transform
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
                                            rclcpp::Duration::from_seconds(0.5));

            // Print transformation details
            // RCLCPP_INFO(ros_node_->get_logger(), "Transform from %s to %s: Translation(%.2f, %.2f, %.2f)",
            //             msg->header.frame_id.c_str(), target_frame_.c_str(),
            //             transform_stamped.transform.translation.x,
            //             transform_stamped.transform.translation.y,
            //             transform_stamped.transform.translation.z);

            // Apply transformation
            geometry_msgs::msg::PoseStamped transformed_pose;
            tf2::doTransform(*msg, transformed_pose, transform_stamped);

            // Store transformed obstacle
            obstacles_.push_back({transformed_pose.pose.position.x, transformed_pose.pose.position.y});
            
            // Print confirmation
            // RCLCPP_INFO(ros_node_->get_logger(), "Obstacle added: x=%.2f, y=%.2f, total obstacles=%lu",
            //             transformed_pose.pose.position.x, transformed_pose.pose.position.y, obstacles_.size());
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(ros_node_->get_logger(), "Transform failure: %s", ex.what());
        }
    }

    struct Obstacle
    {
        double x, y;
    };

    std::vector<Obstacle> obstacles_;
    std::mutex mutex_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string topic_;
    std::string target_frame_;
};

} // namespace rona_physical

PLUGINLIB_EXPORT_CLASS(rona_physical::ViconObstacleLayerAS5, nav2_costmap_2d::Layer)
