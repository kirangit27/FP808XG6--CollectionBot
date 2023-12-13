
#include <gtest/gtest.h>
#include "../include/ccs.hpp"

// Dummy Test
class CompetitionARIACTest : public ::testing::Test {
protected:
    void SetUp() override {

    }

    void TearDown() override {

    }
};

TEST_F(CompetitionARIACTest, OrderCallbackTest) {
    ariac_msgs::msg::Order test_order;
    test_order.id = "test_order_1";
    // Populate test_order with more data as needed

    CompetitionARIAC comp_ariac;
    comp_ariac.OrderCallback(std::make_shared<ariac_msgs::msg::Order>(test_order));

    // Check if the order is added to the orders_list
    EXPECT_EQ(comp_ariac.orders_list.size(), 1);
    EXPECT_EQ(comp_ariac.orders_list[0].id, "test_order_1");
}

TEST_F(CompetitionARIACTest, FloorRobotPickBinPartTest) {
    order_::KittingPart test_part;
    test_part.color = ariac_msgs::msg::Part::RED;
    test_part.type = ariac_msgs::msg::Part::BATTERY;
    // More attributes of test_part can be set as needed

    CompetitionARIAC comp_ariac;
    bool pick_result = comp_ariac.FloorRobotPickBinPart(test_part);

    EXPECT_TRUE(pick_result);
}