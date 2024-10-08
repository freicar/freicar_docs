# FreiCar Map
*2024-10-06 tsteinke: this page is outdated. It describes the old map structure, which is still used in the carla system, but not for the real-world cars.*

A map typically contains all the relevant information about the environment. Our map has a hierarchical structure that provides access on multiple levels, depending on the requirement. The current structure can describe:

* lanes
* junctions
* road signs
* parking spaces


# 1. Compiling the Node
The following external dependencies are required for compiling the node. They are already installed in your docker image, so no need to do anything extra.

- Eigen
- nanoflann
- Thrift

The node also depends on `freicar_common`, a package that defines common messages, services and enums. 

## 1.1. Steps
`freicar_map` is a typical ROS node.It is already added to your catkin workspace as part of the `freicar_base` repo. You can compile `freicar_map` like any normal ROS  node:
```bash
cd ~/freicar_ws
catkin build freicar_map
```
The `CMakeFiles.txt` creates two libraries, namely `freicar_globalmap` and `freicar_map_framework`. The first one contains the core map structure, and the second one offers services that operate on top of it, such as planning. We later link against them to get the map functionalities in any arbitrary node.


# 2. Using the Map Framework
As mentioned, the map package creates two libraries. In a normal C++ project you'd have to link against them. Since ROS uses the catkin package system, you won't need to explicitly name each library that is created within the catkin system.

## 2.1. CMakeLists.txt
If you want to use the libraries from `freicar_map` or any other ROS node, you will need to find and link them using catkin.
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(new_ros_node)

# your C++ compilation flags
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# finding non-ROS libraries, e.g. Eigen
find_package (Eigen3 3 REQUIRED)

# finding ROS packages
find_package(catkin REQUIRED COMPONENTS
    # required for using ROS in C++
	roscpp 
	# other ros packages, e.g.
	std_msgs
	tf2
	tf2_ros
    # the local ROS package you want
	freicar_map
)
# dependecies of the current package
catkin_package(CATKIN_DEPENDS 
	roscpp
	std_msgs
	tf2
	tf2_ros
	freicar_map
)
# header files to consider
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

# your executable ROS node
add_executable(${PROJECT_NAME}_node src/main.cpp
									src/some_other_file.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
										   Eigen3::Eigen
)
```
The `catkin_package()` macro can perform multiple tasks. During the course, you will most likely only use it for declaring your dependencies. The listed dependencies will be `find_package`d, should another ROS package reference this one. When in doubt, you can repeat the same packages you've listed in `find_package(catkin ...)`, although it is not the best practice. It is slightly different in the case of ROS messages, as mentioned [here](https://answers.ros.org/question/261002/catkin_packagecatkin_depends-vs-find_packagecatkin-required-components/). You can also check out the [full documentation](https://docs.ros.org/en/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package).

The two libraries `freicar_globalmap` and `freicar_map_framework` are exported by the `freicar_map` package and are automatically included in `${catkin_LIBRARIES}`. The header directories are also considered as a part of `${catkin_INCLUDE_DIRS}`.

## 2.2. Including the Headers
The headers can be accessed through two main directories

- `freicar_map`
- `map_core`

You will most likely never need to include any core header, other than `map_core/freicar_map.h` which contains the map class itself.
```cpp
#include <freicar_map/planning/lane_star.h>
#include <freicar_map/logic/right_of_way.h>
...
```
# 3. Initialization
Access to the map is provided through a singleton object that should be initialized at the start of each program. There are two ways of initialzing:

1. Using a file
2. Receiving the map over TCP

Both of these approaches make use of the [Thrift library](https://thrift.apache.org/lib/cpp.html) and its serialization capabilities. The latter is currently only possible using the Map GUI that is accessible to the administrator. `ThriftMapProxy` provides an interface to high level thrift functionalities. You need an instance of this class to intialize the map. This [code snippet](#61-intializing-the-map) shows a typical initialization of the map singleton. It first tries to read the map from a file and starts a TCP server if it fails. The path is read from a private ROS parameter.

# 4. Map Structure
As mentioned, the map has a hierarchical structure. Almost all of the components inherit from `mapobjects::MapObject`. The base class contains a single member variable, namely its unique ID, or `uuid`. These IDs are generated once by the main map GUI and are persistant across usages. In the following, separate strucutres in the map are introduced.

## 4.1. LaneGroup
You can think of a lane group as all of the lanes in a section of a street. It provides access to the underlying lanes by `GetLeftLanes()` and `GetRightLanes()`. You most likely will not need to use the lane groups directly.

## 4.2. Lane
Lanes are distinguished by their uuids and are made out of `Point3D` objects: 3D points in space that define the curvature of the lane. Each lane also has a `Lane::Connection` to each of its neighbors. These connections can potentially define:

- `JUNCTION_STRAIGHT`: The next lane in a junction that goes straight
- `JUNCTION_LEFT`: The next lane in a junction that turns left
- `JUNCTION_RIGHT`: The next lane in a junction that turns right
- `STRAIGHT`: The next lane that's not part of a junction
- `OPPOSITE`: The opposite lane
- `ADJACENT_LEFT`: The adjacent lane to the left
- `ADJACENT_RIGHT`: The adjacent lane to the left
- `BACK`: The previous lane

**Note** that not all of these connections exist for all lanes. Most lanes in the current map have 2 or 3. Each lane has a hashmap that links the available connections to a lane pointer (`Lane*`).

As mentioned, `LaneGroup`s are sections of a street, no the entire street. Therefor `Lane`s which are themselves children of lanegroups, are a section of a lane in a street, not the entire length of the lane.
### LaneObject
Each lane can have multiple objects. Currently, the following categories are defined:

- `Roadsign`: traffic signs that belong to a lane, each contains a string
- `Parking`: parking spaces, contains `Point3D`s that define the perimeter
- `Crosswalk`: defined by its offset along the lane 
- `StopLine`: defined by its offset along the lane

### LaneMarking & LaneMarkingContainer
Each lane has two lane markings, on the left and on the right. Lane markings are also `MapObject`s, meaning they have a unique identifier. This way, lanes can share a lane marking, e.g. the left lane marking of a lane can be the right lane marking of another. You can access the LaneMarkingContainer of a lane, which in turn gives you access to the left and the right lane markings.

Each lane marking also a has type that provides abstract but useful information. E.g. one can figure out if it's allowed to overtake another car, by checking if the left lane marking has the type `DASHED`, and not `SOLID`. Although that should most likely be inferred by your perception system.
## 4.3. Junctions
Junctions are objects that are created during the post-processing of the map. Each junction has a unique integer ID to help you distinguish between them. It also has functions such as `GetIncomingLanes()` and `GetOutogingLanes()` that return the uuids of the lanes that lead into that junction, or lead out of it respectively.

The map class also creates associative containers that help you figure out junction related information about a lane. E.g. you can figure out if the current lane leads into a junction using `GetUpcomingJunctionID()`.

## 4.4. LanePoint3D
As mentioned, each lane is made up of `Point3D`s. To help search for the closest lane point on the map, we use a KD-tree and the `LanePoint3D` class. It inherits from `Point3D` and adds metadata to each point, such as the uuid of the lane it belongs to, its heading, its offset along the lane and its index in the `std::vector<Point3D>` in the `Lane` object.

# 5. Map Framework
Currently, two main features are built on top of `freicar::map::Map`, namely `freicar::planning` and `freicar::logic`.

## 5.1. Planning
We have implemented two planners so far:

- lane-star: A* on the lane level
- lane-follower: follows the lane and takes turns based on a given command

### Plan
Both planners return a `freicar::planning::Plan` object with equidistant nodes. `bool success` is a flag that indicates whether the planning was successful. You can directly access the plan steps, which contain:

- `std::string lane_uuid`: id of the lane it belongs to
- `mapobjects::Point3D position`: the 3D location
- `float lane_offset`: offset of that point in the lane
- `float plan_offset`: offset of that point in the plan
- `mapobjects::Lane::Connection path_description`: description of the relation of a point's lane w.r.t. last lane in the plan.

The plans are compatible, meaning you can append a plan from the lane-star planner directly to a plan generated by lane-follower. You however are responsible for making sure that it makes sense, i.e. your agent can follow that trajectory.

### LaneStar
For lane star, you need an object of the `LaneStar` class. You must define the maximum number of a-star steps before it gives up on finding a path. `100` will be more than enough for any application in this course. Afterwards, you can use `GetPlan()` to get a your desired plan. You will need to pass in the following parameters:

- `Point3D start`: where you want to start planning from
- `float start_heading`: the heading
- `Point3D goal`: your goal position
- `float goal_heading`: goal heading
- `float p2p_distance`: the desired distance between the steps in the plan

The headings are just there to make sure the correct lane is chosen. E.g. if your starting position is close to two different lanes, the heading helps choose the correct one.
For consecutive `GetPlan()` calls, you should use `ResetContainers()` first to clean up the search results from the last step.

### LaneFollower
This planner starts from a given position and follows along the lanes for a specified distance. If it reaches a junction, it'll use the given command to choose a direction to go.
For this planner, there is no object needed. You can call one of the two overloads of `freicar::planning::lane_follower::GetPlan()`:

- `Point3D current_position`: x, y, z
- [`PlannerCommand`](#76-planner-command) `command`: it should be `LEFT`, `RIGHT`, `STRAIGHT` or `RANDOM`
- `float distance`: the desired plan length
- `unsigned int step_count`: the number of desired steps in the plan

OR

- `std::string lane_uuid`: the current lane
- `float req_loffset`: the offset along that lane
- [`PlannerCommand`](#76-planner-command) `command`: same as before
- `float distance`: same as before
- `unsigned int step_count`: same as before

### JointPlanner
This class combines the mentioned planners (+ 2 others) into a single object using the [strategy pattern](https://sourcemaking.com/design_patterns/strategy/cpp/1). It's currently used internally to ease the switch between different planning modes. It does not offer anything extraordinary compared to the other two planners. This [code snippet](#62-jointplanner-example) shows a typical use case of the joint planner.

Not all functionalities are offered by all strategies. E.g. the lane follower does allow plan extension because it can simply use the old parameters and get a meaningful plan. lane-star on the other hand cannot provide an extension to a plan without randoming a new goal. In that case, the planner returns an empty plan with the `success` flag set to false.

## 5.2. Logic
Currently, the logic section only offers right-of-way calculation for junctions. To get whether an agent has right of way, it should send some data about itself and all the other perceived agents at the junction. This data includes:

- `std::string a_name`: the name (can be arbitrary)
- `std::string l_uuid`: the uuid of the lane the agent is on
- `float l_offset`: its offset along that lane
- `geometry_msgs::Vector3 velo`: its velocity vector
- `geometry_msgs::TransformStamped pose`: its 3D pose
- `Intent intent`: its intent at the junction: `GOING_STRAIGHT`, `GOING_LEFT`, `GOING_RIGHT` or `NONE` (unknown)

The function `GetRightOfWay` then returns whether the agent has the right of way, whether the junction is already occupied by another agent and the name of the agent that has won the right-of-way (in case of losing RoW, it could be useful).


# 6. Sample Code

## 6.1. Initializing the Map
```cpp
#include <freicar_map/thrift_map_proxy.h>

freicar::map::ThriftMapProxy map_proxy("127.0.0.1", 9091, 9090);
std::string map_path;
if (!ros::param::get("~map_path", map_path)) {
    ROS_ERROR("could not find parameter: map_path! map initialization failed.");
    return;
}
// if the map can't be loaded
if (!map_proxy.LoadMapFromFile(map_path)) {
    ROS_INFO("could not find thriftmap file: %s, starting map server...", map_path.c_str());
    map_proxy.StartMapServer();
    // stalling main thread until map is received
    while (freicar::map::Map::GetInstance().status() == freicar::map::MapStatus::UNINITIALIZED) {
        ROS_INFO("waiting for map...", );
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("map received!");
    // Thrift creats a corrupt file on failed reads, removing it
    remove(map_path.c_str());
    map_proxy.WriteMapToFile(map_path);
    ROS_INFO("saved new map");
}
ros::Duration(2.0).sleep();
// minumum distance of lane nodes = 22cm. experimental value
freicar::map::Map::GetInstance().PostProcess(0.22);
```
## 6.2. JointPlanner Example
```cpp
#include <freicar_map/planning/joint_planner.h>
using freicar::planning;
JointPlanner joint_planner;
joint_planner.SetStrategy(new strategy::LaneStarStrategy(start_point, start_heading,
                                                         goal_point, goal_heading,
                                                         p2p_distance, max_steps));
Plan new_plan = joint_planner.GetPlan();
...
if (divergence_detected) {
    Plan new_plan = joint_planner.Replan(current_x, current_y, current_z, current_heading);
    ...
} else if (should_change_strategy) {
    joint_planner.SetStrategy(new strategy::LaneFollowerStrategy(...));
    Plan new_plan = joint_planner.GetPlan();
    ...
}
```
## 6.3. At a Junction?
```cpp
#include <map_core/freicar_map.h>
#include <freicar_map/planning/lane_follower.h>
auto& map_instance = freicar::map::Map::GetInstance();
auto current_plan = freicar::planning::lane_follower::GetPlan(...);
size_t plan_index = 0;
...
std::string current_lane_uuid = /* some source */;
if (map_instance.GetUpcomingJunctionID(current_l_uuid) != -1) {
    std::cout << "on a lane that leads to a junction" << std::endl;
} else {
    auto* current_lane = map_instance.FindLaneByUuid(current_l_uuid);
    auto* next_lane = current_lane->GetConnection(mapobjects::Lane::STRAIGHT);
    auto next_l_uuid = next_lane->GetUuid()->GetUuidValue();
    if (next_lane && map_instance.GetUpcomingJunctionID(next_l_uuid) != -1) {
        std::cout << "two lanes away from a junction" << std::endl;
    } else {
        std::cout << "not really close to a junction" << std::endl;
    }
}
```

## 6.4. Random A-star Planning
```cpp
#include <freicar_map/planning/lane_star.h>
#include <map_core/freicar_map.h>

auto& map_instance = freicar::map::Map::GetInstance();
freicar::planning::lane_star::LaneStar planner(100);

mapobjects::Point3D start_point(current_x, current_y, current_z);
auto goal_point = map_instance.GetRandomLanePoint();
auto new_plan = planner.GetPlan(start_point, 0, goal_point.AsPoint3D(),
                                goal_point.GetHeading(), 0.15f);
...
```

# 7. Enums

## 7.1. Map Status
```cpp
enum MapStatus {
	UNINITIALIZED = 1,
	LOADED_FROM_FILE = 3,
	UPDATED_FROM_REMOTE = 4
};
```
## 7.2. LaneMarking Type
```cpp
enum LaneMarkingType {
	SOLID = 1,
	DASHED = 2,
	CENTER_SOLID = 3,
	CENTER_DASHED = 4,
};
```
## 7.3. Lane Type
```cpp
enum LaneType {
	NORMAL = 1,
	CAR_LANE = 2,
	PEDESTRIAN_LANE = 3,
};
```
## 7.4. LaneObject Type
```cpp
enum LaneObjectType {
	LO_STOPLINE = 1,
	LO_SIGN = 2,
	LO_CROSSWALK = 3,
	LO_PARKING = 4,
};
```
## 7.6. Lane Direction
```cpp
enum LaneDirection {
	RIGHT = 1,
	LEFT = 2
};
```
## 7.6. Planner Command
```cpp
enum PlannerCommand : unsigned char {
	LEFT = 1,		// go   left   if possible, fail at the junction otherwise
	RIGHT = 2,		// go   right  if possible, fail at the junction otherwise
	STRAIGHT = 3,	// go straight if possible, fail at the junction otherwise
	RANDOM = 4,		// choose a random direction
    // only used in joint planner
	POINT = 5,		// lane-star planner to a specific point
	DIRECT = 6,		// direct path planner to a specific point
	EMPTY = 7		// empty plan
};
```
# 7.7. Junction Type
```cpp
class Junction 
{
	enum Type {
		T_INTESERSECTION,
		CROSS_INTESERSECTION,
		ROUNDABOUT,
		NONE
	};
    ...
}
```
# 8. Full Reference

## 8.1. Map Class
Useful functions in the map class are listed below. Some functions are not mentioned because they do not have a use case after the initialization.

### `MapStatus status()`
Returns the status of the map. See [here](#71-map-status).
### `static Map& GetInstance()`
Returns a reference to the static map object. It's advised to create a single accessible reference during the initialization phase of your program to avoid multiple `map::Map::GetInstance()` calls.
### `mapobjects::Lane* FindLaneByUuid(mapobjects::Uuid lane_uuid)`
### `mapobjects::Lane* FindLaneByUuid(std::string lane_uuid)`
Both calls return a pointer to the requested lane, `nullptr` if invalid.
### `mapobjects::LaneGroup* FindLaneGroupByUuid(mapobjects::Uuid lanegroup_uuid)`
Returns a pointer to the requested lanegorup, `nullptr` if the uuid is invalid.
### `int GetUpcomingJunctionID(std::string lane_uuid_value)`
Returns the id of the junction, given that the specified lane directly precedes a junction; `-1` otherwise.
### `int GetPastJunctionID(std::string lane_uuid_value)`
Returns the id of the junction, given that the specified lane directly follows a junction; `-1` otherwise.
### `int GetCurrentJunctionID(std::string lane_uuid_value)`
Returns the id of the junction, given that the specified lane belongs to a junction; `-1` otherwise.
### `std::vector<mapobjects::Junction> GetJunctions()`
Returns a vector of all junction objects.
### `mapobjects::Junction GetJunctionByID(int junction_id)`
Returns the junction object, given its id. An invalid junction with the id of `-1` otherwise.
### `FindClosestLanePoints(float local_x, float local_y, float local_z, uint32 n)`
Returns a vector of pairs `std::vector<std::pair<mapobjects::LanePoint3D, float>>` which contains the `n` closest points and their distance to the given coordinates.
### `FindClosestLanePointsWithHeading(float local_x, float local_y, float local_z, uint32 count, float heading)`
Returns a vector of pairs `std::vector<std::pair<mapobjects::LanePoint3D, float>>` which contains the `n` closest points and their distance to the given coordinates. The vector is sorted based on the given heading.
### `mapobjects::LanePoint3D GetRandomLanePoint()`
Returns a random `LanePoint3D` in the map :)
### `mapobjects::Lane& GetRandomLane()`
Returns a (reference to a) random lane in the map.
### `void PostProcess(float density_m)`
A function that performs post-processing on the base map structure. This process includes converting to a unified coordinate system, densifying the lane points, creating the hashmaps used for lookups, creating the KD tree, etc. This function **MUST** be called after the map has been loaded or received. If not done, most functions calls will fail.
### `void PrintMap()`
Debug function that prints all the lanes and their connections.
### `void SendAsRVIZMessage(float point_size, float lanemarking_size, std::shared_ptr<ros::NodeHandle> n)`
This function publishes the map structure in an acceptable format to be visualized using RVIZ. The output that you normally see in RVIZ is a result of this function, called by the `freicar_map` node. Naturally only one node needs to visualize the map.

## 8.2. Map Objects
This namespace contains the objects that are part of the core map structure.
___
### `std::string Uuid::GetUuidValue()`
Returns the standard string of a Uuid object.
___
### operator overloads
The following operators are overloaded for the `Point3D` class. Some of them do not make sense (geometrically) for a 3D point but were added to avoid a back and forth switching to `Eigen`.
```cpp
Point3D Point3D::operator-(const Point3D &rhs);
Point3D Point3D::operator+(const Point3D &rhs);
Point3D Point3D::operator/(const float rhs);
Point3D Point3D::operator*(const float rhs);
bool Point3D::operator==(const Point3D &rhs);
bool Point3D::operator!=(const Point3D &rhs);
```
### `void Point3D::SetCoords(std::tuple<float, float, float> tuple)`
Sets the coordinates of a point using an std::tuple.
### `void Point3D::SetCoords(float x, float y, float z = 0)`
Sets the coordinates of a point using float values.
### `void Point3D::SetX(float)`
### `void Point3D::SetY(float)`
### `void Point3D::SetZ(float)`
These functions set the X, Y or Z coordinate respectively.
### `std::tuple<float, float, float> Point3D::GetCoords()`
Returns the coordinates as an std::tuple.
### `float Point3D::x()`
### `float Point3D::y()`
### `float Point3D::z()`
These functions return the X, Y or Z coordinate respectively.
### `float Point3D::ComputeDistance(const mapobjects::Point3D& point)`
Returns the Euclidean distance to the given point.
### `float Point3D::ComputeDistance(float x, float y, float z)`
Same as the last function, but using direct coordinates.
### `std::string Point3D::str()`
Returns the point as a string in the form of (x, y, z). Useful for debugging.
### `Point3D Point3D::Cross(const Point3D &point)`
### `int Point3D::GetRelativeDirection(Point3D forward, Point3D adjacent)`
### `float Point3D::GetNorm()`
### `float Point3D::GetSquaredNorm()`
### `Point3D Point3D::Normalized()`
These functions are used internally and are irrelevant for other applications. Same as some of the overloaded operators, they do not make sense for a 3D point but implemented to avoid switching to Eigen vectors.
___
### `std::vector<Uuid> LaneMarkingContainer::GetLeftLaneMarkings()`
### `std::vector<Uuid> LaneMarkingContainer::GetRightLaneMarkings()`
Returns a vector of left/right lane markings. Each vector however contains only one element. This was necessary to match the Thrift backend (I think).
___
### `bool LaneGroup::IsJunction()`
Returns whether the lanegroup belongs to a junction or not.
### `std::vector<Uuid> LaneGroup::GetLeftLanes()`
Return the left lanes in the lanegroup.
### `std::vector<Uuid> LaneGroup::GetRightLanes()`
Return the right lanes in the lanegroup.
___
### `Point3D Stopline::GetPosition()`
Returns the 3D position of the stopline
___
### `int Parking::GetNumberOfParkingLots()`
Returns the number of parking spaces.
### `float Parking::GetWidth()`
Returns parking slot width.
### `float Parking::GetHeight()`
Returns parking slot height (in a 2D sense).
### `std::vector<std::vector<Point3D>> Parking::GetOuterPoints()`
Returns the perimeter of the parking spaces.
___
### `std::string Roadsign::GetSignType()`
Returns the type of the sign. So far "RightOfWay" and "Stop" are used.
### `Point3D Roadsign::GetPosition()`
Returns the 3D coordinates of the road sign.
___
### `bool Lane::IsOffroad()`
There is a dummy lane in the structure with "?" as its uuid. Some agents report this as their current lane uuid to indicate they're not close to any road.
### `std::vector<const Roadsign*> Lane::GetRoadSigns()`
Returns a list of all road signs (by pointer).
### `bool Lane::HasRoadSign(std::string sign_name)`
Returns true if the lane has the given sign.
### `const Stopline* Lane::GetStopLine()`
Returns the stop line for the lane, `nullptr` if it doesn't exist.
### `Uuid Lane::GetParentUuid()`
Returns the Uuid of its parent lanegroup.
### `const Lane* Lane::GetConnection(Connection connection)`
Returns a Lane pointer for the specified connection, `nullptr` if it doesn't exist.
### `Connection Lane::GetConnectionType(const Lane &lane)`
Returns the connection between the current and the given lane, `mapobjects::Lane::Connection::NONE` if it doesn't exist.
### `LaneType Lane::GetLaneType()`
Returns the type. See [here](#73-lane-type)
### `LaneDirection Lane::GetLaneDirection()`
Returns the direction of the lane. See [here](#76-lane-direction).
### `Point3D Lane::GetHandlePoint()`
Returns the center point in the lane.
### `float Lane::GetWidth()`
Returns the width of the lane.
### `LaneMarkingContainer Lane::GetLaneMarkingContainer()`
Returns the LaneMarkingContainer for the lane.
### `std::vector<Uuid> Lane::GetOutConnections()`
Returns a list of all lanes that the current lane leads to.
### `std::vector<Uuid> Lane::GetInConnections()`
Returns a list of all lanes that lead to the current lane.
### `std::vector<Point3D> Lane::GetPoints()`
Returns a list of all the `Point3D`s that make up the lane.
### `int Lane::GetJunctionID()`
Returns the id of the junction the lane belongs to, `-1` if it doesn't belong to any junction.
### `bool Lane::IsJunctionLane()`
Checks whether junction_id is not equal to `-1`.
___
### `std::vector<Uuid> Junction::GetLanes()`
Returns the uuid of all the lanes that belong to the junction.
### `std::vector<Uuid> Junction::GetLaneGroups()`
Returns the uuid of all the lanegroups that belong to the junction.
### `std::vector<Uuid> Junction::GetIncomingLanes()`
Returns a list of all the lanes that enter the junction.
### `std::vector<Uuid> Junction::GetOutgoingLanes()`
Returns a list of all the lanes that exit the junction.
### `std::vector<Point3D> Junction::GetBoundaryPoints()`
Returns a list of the points that more or less define the perimeter of the junction.
### `int Junction::GetID()`
Returns the junction id.
### `Point3D Junction::GetHandlePoint()`
Returns the center point of the junction.
### `Type Junction::GetType()`
Returns the type. See [here](#77-junction-type).
___
### `std::string LanePoint3D::GetLaneUuid()`
Return the uuid of the lane that the point belongs to.
### `unsigned int LanePoint3D::GetIndex()`
Returns the index of the point in the vector of `Point3D` objects inside that lane.
### `float LanePoint3D::GetHeading()`
Although points technically do not have a heading, `LanePoint3D` can specify the direction of the lane at that point.
### `float LanePoint3D::GetLaneOffset()`
Returns the offset of the point along the lane.
### `Point3D LanePoint3D::AsPoint3D()`
If there's ever necessary to change a `LanePoint3D` to a `Point3D`, e.g. to pass it to a function that only accepts the latter, this function can be used.
___
## 8.3. Thrift Map Proxy
### `ThriftMapProxy(std::string remote_address, int remote_port = 9091, int local_server_port = 9090)`
The constructor for the thrift map proxy. The remote address is the IP of the machine that is running the Map GUI. The remote and local ports are usually `9091` and `9090` respectively.
### `bool LoadMapFromFile(std::string path)`
Tries to load the map from a file and returns a boolean to indicate whether it was successful or not.
### `void WriteMapToFile(std::string path)`
Writes the current map to a file. If you have received the map over TCP, it is recommended to save it. The file format that's used by the Map GUI and the C++ Thrift API are not similar. Be sure to save the map **before** applying the post processing step.
### `void StartMapServer()`
Starts a local server to receive the map from a remote machine.
### `void StopMapServer()`
Stops the aforementioned map server.
___
## 8.4. Planners
A detailed explanation of planners is provided in [section 5.1.](#51-planning)
