#include <gtest/gtest.h>
#include "lci_cali/input.hpp"

TEST(LCI_TEST, INPUT_TEST){
    std::shared_ptr<lci_cali::Input> input = std::make_shared<lci_cali::Input>("Input");
    rclcpp::spin(input);
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    // rclcpp::init(argc, argv);
    // testing::InitGoogleTest(&argc, argv);
    // rclcpp::shutdown();
    // return RUN_ALL_TESTS();

    rclcpp::init(argc,argv);
    std::shared_ptr<lci_cali::Input> input = std::make_shared<lci_cali::Input>("Input");
    rclcpp::spin(input);
    rclcpp::shutdown();
    return 0;
}
