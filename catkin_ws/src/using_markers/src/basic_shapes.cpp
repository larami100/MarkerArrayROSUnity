#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  // Set our initial shape type to be a cube
  int shapes[] = {visualization_msgs::Marker::CUBE, visualization_msgs::Marker::SPHERE, visualization_msgs::Marker::CYLINDER, visualization_msgs::Marker::MESH_RESOURCE }; 

  while (ros::ok())
  {
    visualization_msgs::MarkerArray shapesMarkerArray;
    int numberShapes = sizeof(shapes)/sizeof(shapes[0]);
    shapesMarkerArray.markers.resize(numberShapes);
    
    // Set the frame ID and timestamp.
    for(int i=0, xPosition=0; i<numberShapes; i++, xPosition+=2) 
    {
        shapesMarkerArray.markers[i].header.frame_id = "my_frame";
        shapesMarkerArray.markers[i].header.stamp = ros::Time::now();

    	// Set the namespace and id for this marker.  This serves to create a unique ID
    	// Any marker sent with the same namespace and id will overwrite the old one
    	shapesMarkerArray.markers[i].ns = "basic_shapes";
    	shapesMarkerArray.markers[i].id = i;

    	// Set the marker type.  Initially this is ARROW and cycles between that and CUBE and SPHERE
    	shapesMarkerArray.markers[i].type = shapes[i];

    	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    	shapesMarkerArray.markers[i].action = visualization_msgs::Marker::ADD;

    	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    	shapesMarkerArray.markers[i].pose.position.x = xPosition;
    	shapesMarkerArray.markers[i].pose.position.y = 0;
    	shapesMarkerArray.markers[i].pose.position.z = 0;
    	shapesMarkerArray.markers[i].pose.orientation.x = 0.0;
    	shapesMarkerArray.markers[i].pose.orientation.y = 0.0;
    	shapesMarkerArray.markers[i].pose.orientation.z = 0.0;
    	shapesMarkerArray.markers[i].pose.orientation.w = 1.0;

    	// Set the scale of the marker -- 1x1x1 here means 1m on a side
    	shapesMarkerArray.markers[i].scale.x = 1.0;
    	shapesMarkerArray.markers[i].scale.y = 1.0;
    	shapesMarkerArray.markers[i].scale.z = 1.0;

    	// Set the color -- be sure to set alpha to something non-zero!
    	shapesMarkerArray.markers[i].color.r = 0.0f;
    	shapesMarkerArray.markers[i].color.g = 1.0f;
    	shapesMarkerArray.markers[i].color.b = 0.0f;
    	shapesMarkerArray.markers[i].color.a = 1.0;

    	shapesMarkerArray.markers[i].lifetime = ros::Duration();
        if(shapes[i] == visualization_msgs::Marker::MESH_RESOURCE)
        {
    	    shapesMarkerArray.markers[i].mesh_resource = "file:///home/larami100/catkin_ws/src/using_markers/meshes/turtle.dae"; 
        } 
    }
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
    marker_pub.publish(shapesMarkerArray);

    r.sleep();
  }
}
