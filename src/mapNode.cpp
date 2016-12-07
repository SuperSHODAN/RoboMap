#include "ros/ros.h"
#include "RoboMap/mapData.h"
#include "RoboMap/mapPoint.h"
#include "visualization_msgs/Marker.h"
#include <vector>

#define MAP_WIDTH  50
#define MAP_HIEGHT 50
#define UNITS_PER_CELL 0.25
#define SCALAR_SCALE_CONSTANT 100000

struct MapCell{
	bool isWalkable = true;
	bool wasExplored = false;
	float value = 0;
	float x;
	float y;
};

class Map{
public:
	MapCell cells[MAP_WIDTH][MAP_HIEGHT];

	void recomputeWeights(int iterations){
		// reset all cells
		for(int x = 0; x < MAP_WIDTH; x++){
		for(int y = 0; y < MAP_HIEGHT; y++){
			MapCell cell = cells[x][y];
			if(!cell.wasExplored){
				//cell.value = 1;
				cells[x][y].value = 1;
			}
			else{
				//cell.value = 0
				cells[x][y].value = 0;	
			}
		}}

		for(int i = 0; i < iterations; i++){
			//MapCell tempCells[MAP_WIDTH][MAP_HIEGHT] = cells;
			for(int x = 1; x < MAP_WIDTH-1; x++){
			for(int y = 1; y < MAP_HIEGHT-1; y++){
				MapCell cell = cells[x][y];
				if(!cell.wasExplored){
					cells[x][y].value = 1;
				}
				else if (!cell.isWalkable){
					cells[x][y].value = 0;
				}
				else{
					// sum neighbors
					float sum = 0;
					sum += cells[x + 1][y + 1].value;
					sum += cells[x + 1][y + 0].value;
					sum += cells[x + 1][y - 1].value;

					sum += cells[x + 0][y + 1].value;
					sum += cells[x + 0][y + 0].value;
					sum += cells[x + 0][y - 1].value;

					sum += cells[x - 1][y + 1].value;
					sum += cells[x - 1][y + 0].value;
					sum += cells[x - 1][y - 1].value;

					sum = sum / 9.0f;

					cells[x][y].value = sum;
					if(cells[x][y].value > 1) cells[x][y].value = 1;
					cells[x][y].isWalkable = cell.isWalkable;
					cells[x][y].wasExplored = cell.wasExplored;
				}
			}}
			// swap
			//cells = tempCells;
		}
	}
};


bool sendMapData(RoboMap::mapData::Request& request,
				 RoboMap::mapData::Response& response);
void pointCallback(const RoboMap::mapPoint::ConstPtr& message);
void getLocalMap(int x, int y, int size, MapCell** localMap);

Map map;

int main(int argc, char** argv) {
	ros::init(argc, argv, "map");
	ROS_INFO("Starting map node...");

	ros::NodeHandle nh;
	ros::Rate loop_rate(40);

	ros::ServiceServer srv_map = nh.advertiseService("mapData", sendMapData);
	ros::Subscriber sub_map = nh.subscribe("mapPoint", 1, pointCallback);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);


	uint32_t shape = visualization_msgs::Marker::POINTS;

	// // create temp map
	// for(int i=0; i<MAP_HIEGHT; i++){
	// 	//map.cells[0][i].value = 1;
	// 	map.cells[1][i].wasExlored = false;
	// }
	// map.cells[3][14].isWalkable = false;
	// map.cells[3][15].isWalkable = false;

	// map.cells[3][3].isWalkable = false;
	// map.cells[3][4].isWalkable = false;
	// map.cells[3][5].isWalkable = false;

	map.recomputeWeights(50);

	int localMapSize = 10;
	MapCell **localMap = new MapCell*[localMapSize];
	for(int i = 0; i < localMapSize; ++i) {
	    localMap[i] = new MapCell[localMapSize];
	}
	getLocalMap(7, 10, localMapSize, localMap);


	while(ros::ok()) {
		visualization_msgs::Marker marker;
		
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = "/my_frame";
	    marker.header.stamp = ros::Time::now();

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    marker.ns = "basic_shapes";
	    marker.id = 0;

	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    marker.type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = 0;
	    marker.pose.position.y = 0;
	    marker.pose.position.z = 0;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 1;
	    marker.scale.y = 1;
	    marker.scale.z = 1;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 1.0;

	    marker.lifetime = ros::Duration();

	    for(int i=0; i < MAP_WIDTH; i++){
	    	for(int j=0; j < MAP_HIEGHT; j++){
	    		geometry_msgs::Point p;
	    		p.x = 0 - (MAP_WIDTH/2) + i;
	    		p.y = 0 - (MAP_HIEGHT/2) + j;
	    		p.z = 0;
	    		marker.points.push_back(p);

	    		std_msgs::ColorRGBA c;
	    		c.r = c.b = c.g = map.cells[i][j].value;
	    		// if(c.r == 0) c.r = 1;
	    		// if(map.cells[i][j].wasExplored){
	    		// 	c.r = 0;
	    		// 	c.g = 0;
	    		// 	c.b = 1;
	    		// }
	    		c.a = 1.0f;
	    		marker.colors.push_back(c);
	    	}
	    }

	    // for(int i=0; i < localMapSize; i++){
	    // 	for(int j=0; j < localMapSize; j++){
	    // 		geometry_msgs::Point p;
	    // 		p.x = 0 - (MAP_WIDTH/2) + i + 2;
	    // 		p.y = 0 - (MAP_HIEGHT/2) + j + 5;
	    // 		p.z = 3;
	    // 		marker.points.push_back(p);

	    // 		std_msgs::ColorRGBA c;
	    // 		c.r = c.b = c.g = localMap[i][j].value;
	    // 		if(c.r == 0) c.r = 1;
	    // 		c.a = 1.0f;
	    // 		marker.colors.push_back(c);
	    // 	}
	    // }

	    // Publish the marker
	    while (marker_pub.getNumSubscribers() < 1)
	    {
	      if (!ros::ok())
	      {
	        return 0;
	      }
	      ROS_WARN_ONCE("Please create a subscriber to the marker");
	      sleep(1);
	    }

	    marker_pub.publish(marker);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

bool sendMapData(RoboMap::mapData::Request& request,
				 RoboMap::mapData::Response& response)
{
	ROS_INFO("responding tpo sendData call");

	

	// // create the temp map
	int localMapSize = 3;
	MapCell **localMap = new MapCell*[localMapSize];
	for(int i = 0; i < localMapSize; ++i) {
	    localMap[i] = new MapCell[localMapSize];
	}

	// fill the map with data
	getLocalMap((int)request.latitude, (int)request.longitude, localMapSize, localMap);

	// convert to a 1D scalar array
	std::vector<double> scalarData;
	for(int i=0; i < localMapSize; i++){
		for(int j=0; j < localMapSize; j++){

			scalarData.push_back(localMap[i][j].value);
		}
	}
	
	response.intensity = scalarData;
	response.latitude  = (int)request.latitude  - localMapSize/2;
	response.longitude = (int)request.longitude - localMapSize/2;

	ROS_INFO("explored: %d, %d", (int)request.latitude, (int)request.longitude);
	map.cells[(int)request.latitude][(int)request.longitude].wasExplored = true;

	map.recomputeWeights(2);
	return true;
}

void pointCallback(const RoboMap::mapPoint::ConstPtr& message){
	ROS_INFO("lat:%f lon:%f", message->latitude, message->longitude);
	int x = (int) message->latitude;
	int y = (int) message->longitude;

	map.cells[x][y].isWalkable = false;
	map.cells[x][y].wasExplored = true;
	map.recomputeWeights(2);
}

void getLocalMap(int x, int y, int size, MapCell** localMap){
	int topLeftX = x - size/2;
	int topLeftY = y - size/2;

	for(int i=0; i < size; i ++){
		for(int j=0; j < size; j ++){
			int localX = topLeftX + i;
			int localY = topLeftY + j;

			localMap[i][j] = map.cells[localX][localY];
		}
	}
}