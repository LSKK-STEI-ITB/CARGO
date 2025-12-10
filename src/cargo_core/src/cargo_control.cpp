#include "cargo_core/cargo_control.hpp"
#include "cargo_core/command_registry.hpp"


// TODO: Need safety check on each function at least to make sure it return a value. 
//       Idea is to make a state for each return of function. 

void cargo_core::arm_drone(rclcpp::Node::SharedPtr nh, const ParamMap &params){
  RCLCPP_INFO(nh->get_logger(), "--- Arming ---");
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client;
  client =
      nh->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  // Create request
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;
  mavros_msgs::msg::State curr_state;
  rclcpp::Duration dur(std::chrono::seconds(5));
  int count = 0;
  int threshold = 5;
  do {
    count++;
    if (count > threshold) {
      RCLCPP_FATAL(nh->get_logger(), "--- Arming Failed ---");
      return;
    }

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(nh, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (!response->success) {
        RCLCPP_WARN(nh->get_logger(), "---Arming failed, retrying...---");
      } else {
        RCLCPP_INFO(nh->get_logger(), "---Arming succeeded!---");
      }
      rclcpp::sleep_for(std::chrono::seconds(5));
      int gotTopic =
          get_topic_val(nh, curr_state, "/mavros/state", std::chrono::seconds(3));
    } else {
      RCLCPP_WARN(nh->get_logger(), "---Service call failed---");
    }
  } while (!curr_state.armed);
}


void cargo_core::get_alt(rclcpp::Node::SharedPtr nh, double &alt, std::string source) {
  if (source == "local_pose") {
    std::string topic_name = "/mavros/local_position/pose";
    geometry_msgs::msg::PoseStamped local_pose;
    bool got_topic =
        get_topic_val(nh, local_pose, topic_name, std::chrono::seconds(5));

    if (!got_topic) {
      RCLCPP_ERROR(nh->get_logger(), "Reading Altitude error...");
    }

    alt = local_pose.pose.position.z;
  }

  else if (source == "rangefinder") {
  }

  return;
}

void cargo_core::switch_mode(rclcpp::Node::SharedPtr nh,  const ParamMap &params) {
  const ParamValue &v = params.at("mode");  // get the variant
  std::string mode = std::get<std::string>(v);

  RCLCPP_INFO_STREAM(nh->get_logger(), "--- Changing mode to " << mode);
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setMode_client;
  setMode_client =
      nh->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->base_mode = 0;
  req->custom_mode = mode;
  mavros_msgs::msg::State curr_state;

  // The only way to exit this loop with no error is when desired mode is
  // reached
  uint8_t count = 0;
  do {
    count++;
    if (count > 5) {
      RCLCPP_FATAL(nh->get_logger(), "---Mode changing failed. Terminate.---");
      return;
    }

    auto future = setMode_client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(nh, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (!response->mode_sent) {
        RCLCPP_WARN(nh->get_logger(),
                    "---Mode changing failed, retrying...---");
      }
      rclcpp::sleep_for(std::chrono::seconds(5));
      int gotTopic =
          get_topic_val(nh, curr_state, "/mavros/state", std::chrono::seconds(5));
    } else {
      RCLCPP_FATAL_STREAM(nh->get_logger(), "Terminate in func " << __func__);
      return;
    }
  } while (boost::algorithm::to_lower_copy(curr_state.mode) !=
           boost::algorithm::to_lower_copy(mode));
}


template <class T> 
bool cargo_core::get_topic_val(rclcpp::Node::SharedPtr nh, T &return_val, const std::string &topic_name,
                   std::chrono::seconds retry_timeout){
    RCLCPP_INFO_STREAM(nh->get_logger(),
                     "Waiting for message from " << topic_name);
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    bool got_msg = false;
    std::shared_ptr<T> last_msg = nullptr;

    // Create a temporary subscription
    auto sub = nh->create_subscription<T>(topic_name, rclcpp::SensorDataQoS(),
                                        [&](typename T::SharedPtr msg) {
                                          last_msg = std::make_shared<T>(*msg);
                                          got_msg = true;
                                        });

    // Timeout logic
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(retry_timeout);

    while (!got_msg &&
            (std::chrono::steady_clock::now() - start_time) < timeout) {
        rclcpp::spin_some(nh);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!got_msg || !last_msg) {
        RCLCPP_FATAL_STREAM(nh->get_logger(),
                            "Cannot get message from " << topic_name);
        return false;
    }

    return_val = *last_msg;
    return true;
}


void cargo_core::takeoff(rclcpp::Node::SharedPtr nh, const ParamMap &params){

    const ParamValue &v = params.at("alt");  
    double alt = std::get<double>(v);
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client; 
    takeoff_client = nh->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff"); 

    while (!takeoff_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(nh->get_logger(), "Waiting for takeoff service...");
    }

    RCLCPP_WARN_STREAM(nh->get_logger(), "Takeoff height: " << alt);

    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = alt;
    request->min_pitch = 0;
    request->yaw = 0;
    request->latitude = 0;
    request->longitude = 0;

    double current_alt = 0;
    bool is_takeoff = false; // True only if service called
    int count =
        0; // Count how many retry if > 5 then cancel mission (something wrong)

    get_alt(nh, current_alt, "local_pose");

    double init_alt = current_alt;

    while (abs(init_alt + alt) - abs(current_alt) > 0.3 && count < 5) {
      auto future = takeoff_client->async_send_request(request);
      auto status =
          rclcpp::spin_until_future_complete(nh, future, std::chrono::seconds(3));
      if (status == rclcpp::FutureReturnCode::SUCCESS) {
        auto res = future.get();

        get_alt(nh, current_alt, "local_pose");

        // Once only, atleast we know it's called (otherwise it will spamming)
        if (!is_takeoff) {
          RCLCPP_INFO(nh->get_logger(),
                      "Takeoff service response: success=%d, result=%d",
                      res->success, res->result);
        }
      } else if (status == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(nh->get_logger(), "Takeoff service timed out, retrying ...");
        count++;
      } else {
        RCLCPP_ERROR(nh->get_logger(), "Takeoff service failed, retrying...");
        count++;
      }
    }

    if (alt - current_alt > 0.3) {
      RCLCPP_FATAL_STREAM(nh->get_logger(), "Terminate in function " << __func__);
    } else {
      RCLCPP_INFO_STREAM(nh->get_logger(),
                         "Target Altitude reached: " << current_alt);
    }
}


void cargo_core::land(rclcpp::Node::SharedPtr nh, const ParamMap &params){
    RCLCPP_INFO(nh->get_logger(), "--- Land Initiated ---");
    ParamMap land_params;
    land_params["mode"] = std::string("LAND");
    switch_mode(nh, land_params);
}

void cargo_core::initialize(rclcpp::Node::SharedPtr nh, const ParamMap &params){

  RCLCPP_INFO(nh->get_logger(), "-- Initialize UAV Mission --");
  // TODO: Wait for GPS

  bool GPSFound = false;
  bool Heading = false;
  double currentPoint_lat;
  double currentPoint_lon;
  double currentHeading;

  boost::function<void(const sensor_msgs::msg::NavSatFix &)>
      globalPositionCallback = [&](const sensor_msgs::msg::NavSatFix &msg) {
        currentPoint_lat = msg.latitude;
        currentPoint_lon = msg.longitude;
        GPSFound = true;
      };

  boost::function<void(const std_msgs::msg::Float64 &)> headingCallback =
      [&](const std_msgs::msg::Float64 &msg) {
        Heading = true;
        currentHeading = msg.data;
      };

  bool startMission = false;

  boost::function<void(const std_msgs::msg::Bool &)> startCb =
      [&](const std_msgs::msg::Bool &msg) {
        if (msg.data) {
          std::string topic_name = "/safety_node/active";
          size_t pub_count = nh->count_publishers(topic_name);
          if (pub_count > 0) {
            RCLCPP_WARN(nh->get_logger(), "---Starting mission---");
            startMission = true;
          } else {
            RCLCPP_ERROR(nh->get_logger(), "---Safety Node not running---");
          }
        }
      };

  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos);
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
  gps_sub = nh->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      globalPositionCallback);
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub;
  heading_sub = nh->create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      headingCallback);

  while (!GPSFound) {
    RCLCPP_WARN(nh->get_logger(), "Waiting for GPS...");
    rclcpp::spin_some(nh);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_WARN(nh->get_logger(), "---Mission ready to start mission---");

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_pub;
  start_pub = nh->create_subscription<std_msgs::msg::Bool>(
      "mission_center/start_mission", 10, startCb);
  while (!startMission) {
    rclcpp::spin_some(nh);
  }
  return;
}


void cargo_core::register_all_commands(){
    register_command("INITIALIZE", initialize);
    register_command("ARM", arm_drone);
    register_command("TAKEOFF", takeoff);
    register_command("LAND", land);
    register_command("SWITCH_MODE",switch_mode);
}
