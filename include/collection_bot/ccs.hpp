
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ariac_msgs/msg/competition_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string>

#include "ariac_msgs/msg/order.hpp"
#include "ariac_msgs/msg/assembly_task.hpp"
#include "ariac_msgs/msg/kitting_task.hpp"
#include "ariac_msgs/msg/kitting_part.hpp"
#include "ariac_msgs/msg/bin_parts.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "ariac_msgs/srv/submit_order.hpp"


namespace order_
{   
    struct KittingPart{
    uint8_t quadrant;
    uint8_t color;
    uint8_t type;
    bool part_status{0};
    };

    struct KittingType{
        uint8_t quadrant;
        uint8_t agv_number;
        int tray_id;
        uint8_t destination;
        std::vector<KittingPart> parts;
    };

    struct AssemCombPart{
        uint8_t color;
        uint8_t type;
        geometry_msgs::msg::Pose pose;
        geometry_msgs::msg::PoseStamped pose_stamp;
        geometry_msgs::msg::Vector3 install_direction;
    };

    struct AssemCombType{
        uint8_t station;
        std::vector<uint8_t> agv_numbers;
        std::vector<AssemCombPart> parts;
    };

    class Orders{
    public:
        std::string id;
        uint8_t type;
        bool priority;
        KittingType kitting_type;
        bool order_status{0};
        AssemCombType AssemComb_type;

        std::map<int,std::string> order_type={
            {0,"KITTING"},
            {1,"ASSEMBLY"},
            {2,"COMBINED"}};
    };

}

class CompetitionARIAC : public rclcpp::Node
{
    public:

    CompetitionARIAC() : Node("competition_subscriber"){
        m_callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //subscription callback groups
        auto subscription_option1 = rclcpp::SubscriptionOptions();
        subscription_option1.callback_group = m_callback_group_1;
        auto subscription_option2 = rclcpp::SubscriptionOptions();
        subscription_option2.callback_group = m_callback_group_2;

        // Subscriber objects            
        comp_state_sub = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
                                                                    std::bind(&CompetitionARIAC::CompetitionStateCallback, this, std::placeholders::_1),subscription_option1);
        order_sub = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
                                                                    std::bind(&CompetitionARIAC::OrderCallback, this, std::placeholders::_1),subscription_option2);
        submit_order_client_ = create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
    };

    private:

        bool comp_start{false};
        bool order_submitted{false};
        bool priority_order = false;
        int priority_index = -1;
        int order_counter{0};
        int list_size{0};
        std::vector<order_::Orders> orders_list;
        std::vector<order_::Orders> orders_list_submit;
         
        rclcpp::CallbackGroup::SharedPtr m_callback_group_1;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_2;

        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr comp_state_sub;
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub;

        void CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
        void OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg); 
        
        void callServiceStart();
        void callServiceEnd();
        void callService_submit(std::string order);

        // ARIAC Services
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_comp_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_comp_client_;
        rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_client_;


};

