#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <string>

#include "st3215_hardware/st3215_communication.hpp"

using namespace st3215_hardware;

class ST3215CommunicationTest : public ::testing::Test {
protected:
    void SetUp() override {
        comm_ = std::make_unique<ST3215Communication>("/dev/null", 1000000);
    }

    void TearDown() override {
        comm_.reset();
    }

    std::unique_ptr<ST3215Communication> comm_;
};

TEST_F(ST3215CommunicationTest, ConstructorTest) {
    EXPECT_FALSE(comm_->is_connected());
}

TEST_F(ST3215CommunicationTest, SetTimeoutTest) {
    EXPECT_TRUE(comm_->set_timeout(std::chrono::milliseconds(200)));
}

TEST_F(ST3215CommunicationTest, ConnectionTest) {
    EXPECT_FALSE(comm_->connect());
    EXPECT_FALSE(comm_->is_connected());
    
    comm_->disconnect();
    EXPECT_FALSE(comm_->is_connected());
}

TEST_F(ST3215CommunicationTest, ReadOperationsWithoutConnection) {
    EXPECT_EQ(comm_->read_position(1), std::nullopt);
    EXPECT_EQ(comm_->read_velocity(1), std::nullopt);
    EXPECT_EQ(comm_->read_effort(1), std::nullopt);
    EXPECT_EQ(comm_->read_temperature(1), std::nullopt);
    EXPECT_EQ(comm_->read_voltage(1), std::nullopt);
}

TEST_F(ST3215CommunicationTest, WriteOperationsWithoutConnection) {
    EXPECT_FALSE(comm_->write_position(1, 1.0));
    EXPECT_FALSE(comm_->write_velocity(1, 1.0));
    EXPECT_FALSE(comm_->write_effort(1, 1.0));
}

TEST_F(ST3215CommunicationTest, PingWithoutConnection) {
    EXPECT_FALSE(comm_->ping_servo(1));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}