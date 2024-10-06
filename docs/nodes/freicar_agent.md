# FreiCar Agent
*2024-10-06 tsteinke: this page describes the old agent template and is outdated. Please refer to the README and comments in the [new agent package](https://rlgit.informatik.uni-freiburg.de/freicar/base/-/tree/master/freicar_agent) for now.*

This node performs the bare minimum required for running an agent in the FreiCar software stack. This will be your starting point.
As mentioned in the overview, this node by default takes care of the following tasks:

- Sending `Track` requests at start and shutdown
- Publishing `FreiCarStatus` messages
- Publishing `FreiCarLocalization` messages
- Publishing a 3D model for visualization on RVIZ

In the following, we'll go through the important parts of the code.


# 1. FreiCarAgent Class
Instances of this class represent the agents in the environment.
```cpp
class FreiCarAgent
{
public:
	struct Settings {
		unsigned int type_code;
		int thread_sleep_ms;
		std::string name;
		std::string owner;
		std::string tf_name;
		std::string map_path;
	};
	~FreiCarAgent();
	FreiCarAgent(FreiCarAgent::Settings settings, std::shared_ptr<ros::NodeHandle> nh);
	void SendTrackRequest(bool track_enable);
	void StopAgentThread();
private:
	void Step(unsigned int thread_sleep_ms);

    void HaltCallback(const freicar_common::FreiCarHalt::ConstPtr &msg);
    void ResumeCallback(const std_msgs::String::ConstPtr &msg);
	void RGBCallback(const sensor_msgs::ImageConstPtr& msg);
	void DepthCallback(const sensor_msgs::ImageConstPtr& msg);

	void InitializeMap();
	void Initialize3DModel();

    void PublishStatus(const ros::Time& now);
	void PublishLocalization(const ros::Time& now, const map::Map& map_instance);

	// private members
    ...
};

```
## 1.1. Settings struct
This struct contains the parameters that are set via ROS or read from the host:

- **type_code**: could be `freicar::agent::type::REAL` or `freicar::agent::type::SIMULATED` or both (bitwise or), set via local ROS parameter
- **thread_sleep_ms**: global ROS parameter set by the 'freicar_setting` node
- **name**: local ROS parameter set by the launch file
- **owner**: "username@hostname"
- **tf_name**: name of the tf frames for this agent (same as name for simulated agents)
- **map_path**: initialize the map from this file if not received over the network

Note that if any of these parameters are missing, the node exits abruptly. E.g. `thread_sleep_ms` relies on `freicar_setting` for the `sim_sps` parameter. If that node is not running, intialization fails.

# 2. Public Functions
## 2.1. FreiCarAgent()
In the constructor, the static parts of messages are set, the publishers and subscribers are initialized and the main thread is started. `InitializeMap()` initializes the map as the name suggests. The code is very similar to the [code snippet](https://freicar-docs.readthedocs.io/nodes/freicar_map/#61-intializing-the-map) explained in FreiCar Map. Intitilization of future member variables should be performed here as well.
## 2.2. SendTrackRequest()
This function is responsible for sending a `Track` request to chaperone. As mentioned in the overview, chaperone is responsible for making sure the agents don't collide. Multiple assumptions are made when sending these requests:

1. The chaperone node is running. If not, the track request fails and the node exits.
2. The parallel `carla_proxy` node is manually SIGINT'd, if the track request fails for any reason. That way [hopefully] the agent is also removed from the simulator. Although this is not a serious problem, it helps avoid restarting the simulator every time this node fails.
3. This node is manually SIGINT'd, if the parallel `carla_prxoy` node crashes for any reason so that a stop-track request can be sent.

## 2.3. StopAgentThread()
This function is automatically called in the destructor. It sets the `running` flag to false and joins the thread. In normal circumstances, you don't need to directly call this function.

# 3. Private Functions
## 3.1. Step()
This is the main thread callback function for the agent. Its sleep period is set according to the global parameter `sim_sps`.
```cpp
void FreiCarAgent::Step(unsigned int thread_sleep_ms) {
	// transform structs
	tf2_ros::Buffer transform_buffer;
	tf2_ros::TransformListener tf2_listener(transform_buffer);
	auto& map_instance = map::Map::GetInstance();
	while (running_) {
		try {
			current_pose_ = transform_buffer.lookupTransform("map", settings_.tf_name, ros::Time(0));
		} catch (tf2::TransformException &ex) {
			ros::Duration(0.2).sleep();
			continue;
		}
        // your code here

		auto now = ros::Time::now();
		PublishStatus(now);
		PublishLocalization(now, map_instance);
		model_pub_.publish(model3d_);
		std::this_thread::sleep_for(std::chrono::milliseconds(thread_sleep_ms));
	}
}
```
In each iteration, the current pose is updated via `tf.lookupTransform`. The two functions `PublishStatus` and `PublishLocalization` take care of publishing necessary messages for each agent. The subsequent call also publishes a basic 3D model for visualization on RVIZ. Naturally this is not a necessary task and can be commented out.

## 3.2. Halt and Resume Callbacks
These messages are usually sent from the chaperone node. When a `Halt` message is received, the agent is expected to stop in the shortest distance possible. Currently, we only keep track of the suspensions using a flag, a string and an enum to indicate the state, the other agent involved in the suspension event and the type of the suspension respectively. The different types of suspension can be looked up in the [`HaltType` enum](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/include/freicar_common/shared/halt_type.h#L10). When the suspension is lifted, the agent is allowed to resume its activity.

## 3.3. RGBCallback
This is the callback function for RGB images. Currently, they are published in the `carla_proxy` node.
```cpp
void FreiCarAgent::RGBCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat rgb8_img = cv_ptr->image;
	// your code here
}
```
The image is cast to an RGB8 `cv::Mat`. At this point you can use the image for any downstream task.

## 3.4. DepthCallback
```cpp
void FreiCarAgent::DepthCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat float32_img = cv_ptr->image;
	// your code here
}
```
The image is cast to a float32 `cv::Mat`. Each pixel in the depth image corresponds to the same pixel in the RGB image. You should therefor keep track of the received images (e.g. using an `std::vector<>` member variable) if you want to process RGBD data.

### Note
We highly advise you to familiarize yourself with memory management in C++. OpenCV data structures use a scheme that is very similar to the standard shared pointer and they should be treated as such.
