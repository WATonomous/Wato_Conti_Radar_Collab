#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "jsk_recognition_msgs/BoundingBox.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include "dbscan.h"
#include "cluster.h"
#include "point.h"
#include "evaluate.h"
#include <pthread.h>

//msg
#include "radar_driver/RadarPacket.h"
#include "radar_driver/BoundingBox.h"
#include "radar_driver/BoundingBoxList.h"
#include "radar_driver/RadarDetection.h"
// pipelinemsg

/*
  Plan:

  1) Create a subscriber node to subscribe to the point cloud topic to take in the array of points
  2) Everytime a new message is taken in from subscriber, the algoCallbackFunction will be called
  3) The algoCallBackFunction will take in the array of points and call the algo function
  4) The output (array of strucs containg the bounding box info) will be passed to two functions, the rvizVisualize function and pipelineFunction

  5) rvizVisualize function will convert the array of strucs into the BoundingBoxArray message type then publish 
  6) pipelineFunction will convert the array of structurs into an array of pipeline messages (needs to be created) then publish

  To create:
  -2 publishers (1 for rviz and one for pipeline)
  -1 new pipeline message type
  
*/

#define MAX_PACKET_COUNT 10
#define BOUNDING_BOX_OFFSET_RIGHT -0.5
#define BOUNDING_BOX_OFFSET_LEFT 0.5

#define LATERAL_OVERLAP_THRESHOLD 5
#define LONGITUDINAL_OVERLAP_THRESHOLD  5

class BoundingBoxClustering{

private:
  ros::NodeHandle snh;
  ros::NodeHandle rviznh;
  ros::NodeHandle pipenh;  

  ros::Publisher rvizPub;
  ros::Publisher pipePub;
  ros::Subscriber sub_right;
  ros::Subscriber sub_left;

public:

  std::vector<radar_driver::RadarDetection> right_combined_detections;
  std::vector<radar_driver::RadarDetection> left_combined_detections;
  int right_packet_count = 0;
  int left_packet_count = 0;

  pthread_mutex_t rightMutex;
  pthread_mutex_t leftMutex;

  BoundingBoxClustering(){
    rightMutex = PTHREAD_MUTEX_INITIALIZER;
    leftMutex = PTHREAD_MUTEX_INITIALIZER;
    rvizPub = rviznh.advertise<jsk_recognition_msgs::BoundingBoxArray>("boundingboxtopic", 1000);
    pipePub = pipenh.advertise<radar_driver::BoundingBoxList>("pipelineBoundingBoxTopic", 1000);
    sub_right = snh.subscribe("radar_right/filtered_radar_packet", 1000, &BoundingBoxClustering::right_pointcloud_callback, this);
    sub_left = snh.subscribe("radar_left/filtered_radar_packet", 1000, &BoundingBoxClustering::left_pointcloud_callback, this);
  }

  static std::vector<dbscan::Cluster::BoundingBox> DBSCANcall(std::vector<radar_driver::RadarDetection> detections, double offset){
    //convert detections into just xyz struc array
    //call DBSCAN algo
    //return the output

    std::vector<dbscan::Point> points;

    double x, y, vel, azAng0;

    for (int i = 0; i < detections.size(); i++){
        x = detections[i].posX;
        y = detections[i].posY;
        vel = detections[i].VrelRad;
        azAng0 = detections[i].AzAng0;
        points.push_back(dbscan::Point(x, y, vel, azAng0));
    }

    //which function gets called to return the info? What is being returned? vector of bounding boxes?
    int min_points = 3;
    double epsilon = 0.5;
    dbscan::Dbscan dbscan_object(points, min_points, epsilon); // Instance of DBSCAN object loaded with the vector of points

    dbscan_object.CreateClusters();

    std::vector<dbscan::Cluster::BoundingBox> boundingBoxClusters; // vector to hold the bounding boxes that will come from the clusters

    for (int i = 0; i < dbscan_object.clusters.size(); i++){
      boundingBoxClusters.push_back(dbscan_object.clusters[i].ConstructBoundingBox()); //populating vector with the constructed bounding boxes formed by the cluster information
      boundingBoxClusters[i].y_center += offset; // sets offset of the bounding boxes
    }

    return boundingBoxClusters;
  }
  
  void addBox(double xCen,double yCen,double zCen, double xdim, double ydim, double zdim, jsk_recognition_msgs::BoundingBoxArray& boxArray) {
    jsk_recognition_msgs::BoundingBox box;
    box.header.frame_id ="base_link";
    box.header.stamp = ros::Time::now();
    box.pose.position.x = xCen;
    box.pose.position.y = yCen;
    box.pose.position.z = zCen;
    box.dimensions.x = xdim;
    box.dimensions.y = ydim;
    box.dimensions.z = zdim;
    box.label = boxArray.boxes.size()+1;
    boxArray.boxes.push_back(box);
  }

  void rvizPublish(std::vector<dbscan::Cluster::BoundingBox> rawBoundingBoxes){ //radar_driver::boundingBox::BoundingBox is meant to be the bounding box Struc
    jsk_recognition_msgs::BoundingBoxArray boxArray;
    boxArray.header.frame_id = "base_link";
    boxArray.header.stamp = ros::Time::now();

    //iterate through all bounding box strucs
    //call addBox function, unpacking the structure in the parameters

    double xCen, yCen, xdim, ydim;
    for (int i = 0; i < rawBoundingBoxes.size(); i++){

      // .names may be wrong, must refer to full implmentation in the algo
      // TODO: check reference frame for x and y here 
      xCen = rawBoundingBoxes[i].x_center;
      yCen = rawBoundingBoxes[i].y_center;
      xdim = rawBoundingBoxes[i].x_dist;
      ydim = rawBoundingBoxes[i].y_dist;
      
      addBox(xCen, yCen, 0, xdim, ydim, 3, boxArray);
    }
    

      rvizPub.publish(boxArray);

  }

  void pipelinePublish(std::vector<dbscan::Cluster::BoundingBox> rawBoundingBoxes){
    //initialize ros message type
    //interate through rawBoundingBoxes vector
        //populate pipeline msg with rawBoundingBoxes struc data

    //publish message under new topic
    radar_driver::BoundingBoxList msg;
    msg.boxes.resize(rawBoundingBoxes.size());

    for (int i = 0; i < rawBoundingBoxes.size(); i++){
      //Transferring data from the cluster data type to pipeline message
      
      radar_driver::BoundingBox box;
      box.x = rawBoundingBoxes[i].x_center;
      box.y = rawBoundingBoxes[i].y_center;
      box.length = rawBoundingBoxes[i].x_dist;
      box.width = rawBoundingBoxes[i].y_dist;
      msg.boxes.push_back(box);
      //msg.boxes[i].velocity = rawBoundingBoxes[i].velocity; //CURRENTLY NOT IMPLEMENTED IN THE BOUNDINGBOX STRUCTURE DATA TYPE
    }

    pipePub.publish(msg);

  }

  void left_pointcloud_callback(const radar_driver::RadarPacket::ConstPtr& detectionsMsg){
    pthread_mutex_lock(&leftMutex);
    left_combined_detections.insert(left_combined_detections.end(), (detectionsMsg->Detections).begin(), (detectionsMsg->Detections).end());
    left_packet_count++;
    pthread_mutex_unlock(&leftMutex);
  }

  void right_pointcloud_callback(const radar_driver::RadarPacket::ConstPtr& detectionsMsg){
    pthread_mutex_lock(&rightMutex);
    right_combined_detections.insert(right_combined_detections.end(), (detectionsMsg->Detections).begin(), (detectionsMsg->Detections).end());
    right_packet_count++;
    pthread_mutex_unlock(&rightMutex);
  }

  // void pointcloud_callback(const radar_driver::RadarPacket::ConstPtr& detectionsMsg){
  //   //call DBSCAN

  //   combined_detections.insert(combined_detections.end(), (detectionsMsg->Detections).begin(), (detectionsMsg->Detections).end());
  //   packet_count++;

  //   if (packet_count == MAX_PACKET_COUNT){
  //     std::vector<dbscan::Cluster::BoundingBox> rawBoundingBoxes = DBSCANcall(combined_detections);
  //     rvizPublish(rawBoundingBoxes, rvizPub);
  //     pipelinePublish(rawBoundingBoxes, pipePub);
  //     packet_count = 0;
  //     combined_detections.clear();
  //   }
  // }

  bool isOverlapping2D(const dbscan::Cluster::BoundingBox& box1, const dbscan::Cluster::BoundingBox& box2)
  {
    double box1_x_min = box1.x_center - box1.x_dist/2.0;
    double box1_x_max = box1.x_center + box1.x_dist/2.0;
    double box1_y_min = box1.y_center - box1.y_dist/2.0;
    double box1_y_max = box1.y_center + box1.y_dist/2.0;

    double box2_x_min = box2.x_center - box2.x_dist/2.0;
    double box2_x_max = box2.x_center + box2.x_dist/2.0;
    double box2_y_min = box2.y_center - box2.y_dist/2.0;
    double box2_y_max = box2.y_center + box2.y_dist/2.0;

    bool isOverlap_x = box1_x_max >= (box2_x_min + LONGITUDINAL_OVERLAP_THRESHOLD) && 
                      box2_x_max >= (box1_x_min + LONGITUDINAL_OVERLAP_THRESHOLD);

    bool isOverlap_y = box1_y_max >= (box2_y_min + LATERAL_OVERLAP_THRESHOLD) && 
                      box2_y_max >= (box1_y_min + LATERAL_OVERLAP_THRESHOLD);

    return isOverlap_x && isOverlap_y;
  }

  std::vector<dbscan::Cluster::BoundingBox> filter_duplicate_clusters(std::vector<dbscan::Cluster::BoundingBox> cluster_one, std::vector<dbscan::Cluster::BoundingBox> cluster_two)
  {
    std::vector<dbscan::Cluster::BoundingBox> filteredBoundingBoxes;

    for(int box_one = 0; box_one < cluster_one.size(); box_one++)
    {
      for(int box_two = 0; box_two < cluster_two.size(); box_two++)
      {
        if(isOverlapping2D(cluster_one[box_one], cluster_two[box_two]))
        {
          // Add handling for overlap case here, ie. merge overlap
          // For now, just erase one cluster
          cluster_two.erase(cluster_two.begin() + box_two);
        }
      }
    }

    for(int box_one = 0; box_one < cluster_one.size(); box_one++)
    {
      filteredBoundingBoxes.push_back(cluster_one[box_one]);
    }
    for(int box_two = 0; box_two < cluster_two.size(); box_two++)
    {
      filteredBoundingBoxes.push_back(cluster_two[box_two]);
    }
    
    return filteredBoundingBoxes;
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "radar_clustering");

  BoundingBoxClustering clustering_object;

  while(ros::ok())
  {
    if(clustering_object.left_packet_count >= MAX_PACKET_COUNT && clustering_object.right_packet_count > MAX_PACKET_COUNT)
    {
      pthread_mutex_lock(&clustering_object.rightMutex);
      pthread_mutex_lock(&clustering_object.leftMutex);

      std::vector<dbscan::Cluster::BoundingBox> rightRawBoundingBoxes = BoundingBoxClustering::DBSCANcall(clustering_object.right_combined_detections, BOUNDING_BOX_OFFSET_RIGHT);
      std::vector<dbscan::Cluster::BoundingBox> leftRawBoundingBoxes = BoundingBoxClustering::DBSCANcall(clustering_object.left_combined_detections, BOUNDING_BOX_OFFSET_LEFT);

      std::vector<dbscan::Cluster::BoundingBox> filteredBoundingBoxes = clustering_object.filter_duplicate_clusters(rightRawBoundingBoxes, leftRawBoundingBoxes);

      clustering_object.rvizPublish(filteredBoundingBoxes);

      clustering_object.pipelinePublish(filteredBoundingBoxes);

      clustering_object.right_packet_count = 0;
      clustering_object.left_packet_count = 0;
      clustering_object.right_combined_detections.clear();
      clustering_object.left_combined_detections.clear();
      pthread_mutex_unlock(&clustering_object.rightMutex);
      pthread_mutex_unlock(&clustering_object.leftMutex);
    }
    ros::spinOnce();
  }

  // ros::spin();

}
