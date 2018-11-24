#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher pub;
ros::Publisher pub_markers;
geometry_msgs::PoseStampedConstPtr pose_ptr_;
int IsPoseUpdated;
ros::Publisher marker_pub;
ros::Publisher goal_pub;

float goal_x;
float goal_y;
float goal_w;
float map_size = 300;
float map_scale = 0.05;

int finishcount = 0;
void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  visualization_msgs::Marker points;
  points.header.frame_id ="/map";
  points.header.stamp= ros::Time::now();
  points.ns="frontier_points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w=1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = map_scale;
  points.scale.y = map_scale;

  visualization_msgs::Marker cluster_points;
  cluster_points.header.frame_id ="/map";
  cluster_points.header.stamp= ros::Time::now();
  cluster_points.ns="frontier_cluster_points";
  cluster_points.action = visualization_msgs::Marker::ADD;
  cluster_points.pose.orientation.w=1.0;
  cluster_points.id = 0;
  cluster_points.type = visualization_msgs::Marker::POINTS;
  cluster_points.scale.x = map_scale;
  cluster_points.scale.y = map_scale;

  float map_start_x = 0.2*map_size*map_scale;
  float map_start_y = 0.5*map_size*map_scale;

  int size_x = msg->info.width;
  int size_y = msg->info.height;
  vector<int> FrontierList;
  //Find the frontiers
  for(int i=0;i<size_x*size_y;i++)
  {
    int x_ic = i%size_x;
    int y_ic = i/size_x;

    float x_c = float(x_ic)*0.05-map_start_x;
    float y_c = float(y_ic)*0.05-map_start_y;
    float distance = 2000;
    if(IsPoseUpdated==1)
    {
      distance = sqrt(pow(pose_ptr_->pose.position.x-x_c,2)+pow(pose_ptr_->pose.position.y-y_c,2));
    }
    if(distance>0.5)
    {
      if(msg->data[i]==0)
      {
        if(x_ic>2 && x_ic<size_x-3)
        {
          if(y_ic>2 && y_ic<size_x-3)
          {
            int nearones_5by5[25];
            int nearones_3by3[9];
            for(int j=0;j<25;j++)
            {
              nearones_5by5[j]=i+size_x*(j/5-2)+(j%5-2);
            }
            for(int j=0;j<9;j++)
            {
              nearones_3by3[j]=i+size_x*(j/3-1)+(j%3-1);
            }
            int IsFrontier = 0;
            for(int j=0;j<9;j++)
            {
              if(msg->data[nearones_3by3[j]]==-1)
              {
                IsFrontier = 1;
              }
              //Mark as a frontier if unexplored space is near
            }
            if(IsFrontier==1)
            {
              for(int j=0;j<25;j++)
              {
                if(msg->data[nearones_5by5[j]]==100)
                {
                  IsFrontier=0;
                }
                //Unmark if occupied space is near
              }
            }
            if(IsFrontier==1)
            {
              FrontierList.push_back(i);
            }
          }
        }
      }
    }
  }

  if(FrontierList.size()!=0)
  {
    goal_x = 0.0;
    goal_y = 0.0;
    goal_w = 1.0;
    ROS_INFO("Frontier Detected!");
    ROS_INFO("Frontier Size : %d", int(FrontierList.size()));
    //Frontier Detection is Done
    //Now, let's cluster the fucking frontiers
    vector<vector<int> > FrontierList_temp;
    FrontierList_temp.resize(2);
    for(int i=0;i<FrontierList.size();i++)
    {
      FrontierList_temp[0].push_back(FrontierList[i]);
      FrontierList_temp[1].push_back(i);
    }
    vector<vector<int> > cluster;


    if(FrontierList.size()>1)
    {
      vector<int> individualCluster;
      vector<int> node;
      node.push_back(0);
      individualCluster.push_back(0);
      while(1)
      {
        vector<int> next_node;
        for(int i=0;i<node.size();i++)
        {
          int baseNode_x = FrontierList[node[i]]%size_x;
          int baseNode_y = FrontierList[node[i]]/size_x;

          for(int j=0;j<FrontierList_temp[0].size();)
          {
            int target_x = FrontierList_temp[0][j]%size_x;
            int target_y = FrontierList_temp[0][j]/size_x;


            int distance = pow(target_x-baseNode_x,2)+pow(target_y-baseNode_y,2);

            if(distance<=2 && distance!=0)
            {

              individualCluster.push_back(FrontierList_temp[1][j]);
              next_node.push_back(FrontierList_temp[1][j]);
              FrontierList_temp[0].erase(FrontierList_temp[0].begin()+j);
              FrontierList_temp[1].erase(FrontierList_temp[1].begin()+j);
            }
            else
            {
              j++;
            }
          }
        }
        node.clear();
        node = next_node;
        next_node.clear();
        if(FrontierList_temp[0].size()==0)
        {
          cluster.push_back(individualCluster);
          individualCluster.clear();
          break;
        }
        else
        {
          if(node.size()==0)
          {
            cluster.push_back(individualCluster);
            individualCluster.clear();
            node.push_back(FrontierList_temp[1][0]);
            FrontierList_temp[0].erase(FrontierList_temp[0].begin());
            FrontierList_temp[1].erase(FrontierList_temp[1].begin());
          }
        }
      }
      //Find the biggest cluster
      vector<int> maxClusterIdx;
      maxClusterIdx.push_back(0);
      int maxClusterSize = int(cluster[0].size());
      for(int i=1;i<int(cluster.size());i++)
      {
        if(int(cluster[i].size())>maxClusterSize)
        {
          maxClusterSize = int(cluster[i].size());
          maxClusterIdx.clear();
          maxClusterIdx.push_back(i);
        }
        else if(int(cluster[i].size())==maxClusterSize)
        {
          maxClusterIdx.push_back(i);
        }
      }

      vector<int> destinationCluster;

      if(int(maxClusterIdx.size())==1)
      {
        destinationCluster = cluster[maxClusterIdx[0]];
        ROS_INFO("Checking Problems");
      }
      else
      {
        vector<float> distance2Clusters;

        ROS_INFO("Checking Problems : %d", int(maxClusterIdx.size()));
        for(int i=0;i<int(maxClusterIdx.size());i++)
        {

          float cluster_meanx = 0;
          float cluster_meany = 0;
          for(int j=0;j<int(cluster[maxClusterIdx[i]].size());j++)
          {
            float cluster_x = float(FrontierList[cluster[maxClusterIdx[i]][j]]%size_x);
            float cluster_y = float(FrontierList[cluster[maxClusterIdx[i]][j]]/size_x);
            cluster_meanx = cluster_meanx+cluster_x/float(cluster[maxClusterIdx[i]].size());
            cluster_meany = cluster_meany+cluster_y/float(cluster[maxClusterIdx[i]].size());
          }
          float distanceTemp = sqrt(pow(pose_ptr_->pose.position.x-cluster_meanx,2)+pow(pose_ptr_->pose.position.y-cluster_meany,2));
          distance2Clusters.push_back(distanceTemp);

        }
        int NearestDistanceID = 0;
        float ShortestDistance = distance2Clusters[0];
        for(int i=1;i<int(distance2Clusters.size());i++)
        {
          if(distance2Clusters[i]<ShortestDistance)
          {
            ShortestDistance = distance2Clusters[i];
            NearestDistanceID = i;
          }
        }

        destinationCluster = cluster[maxClusterIdx[NearestDistanceID]];
      }

      points.color.r = 1.0f;
      points.color.a = 1.0;
      for(int i=0;i<int(FrontierList.size());i++)
      {
        float x = float(FrontierList[i]%size_x)*0.05f-map_start_x;
        float y = float(FrontierList[i]/size_x)*0.05f-map_start_y;
        float z = 0.0f;
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        points.points.push_back(p);
      }
      cluster_points.color.g = 1.0f;
      cluster_points.color.a = 1.0;
      for(int i=0;i<int(destinationCluster.size());i++)
      {
        float x = float(FrontierList[destinationCluster[i]]%size_x)*0.05f-map_start_x;
        float y = float(FrontierList[destinationCluster[i]]/size_x)*0.05f-map_start_y;
        float z = 0.0f;
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        cluster_points.points.push_back(p);
        goal_x = goal_x+(float(FrontierList[destinationCluster[i]]%size_x)*0.05f-map_start_x)/(float(destinationCluster.size()));
        goal_y = goal_y+(float(FrontierList[destinationCluster[i]]/size_x)*0.05f-map_start_y)/(float(destinationCluster.size()));
      }
    }
    else
    {
      ROS_INFO("Only one frontier");
      points.color.r = 1.0f;
      points.color.a = 1.0;
      for(int i=0;i<FrontierList.size();i++)
      {
        float x = float(FrontierList[i]%size_x)*0.05f-map_start_x;
        float y = float(FrontierList[i]/size_x)*0.05f-map_start_y;
        float z = 0.0f;
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        points.points.push_back(p);
        goal_x = goal_x+(float(FrontierList[i]%size_x)*0.05f-map_start_x)/(float(FrontierList.size()));
        goal_y = goal_y+(float(FrontierList[i]/size_x)*0.05f-map_start_y)/(float(FrontierList.size()));
      }

    }
    FrontierList.clear();
    IsPoseUpdated=0;
    ROS_INFO("Cluster size = %d",int(cluster.size()));

    geometry_msgs::PoseStamped goal_position;
    goal_position.header.frame_id = "map";
    goal_position.header.stamp = ros::Time::now();
    goal_position.pose.position.x = goal_x;
    goal_position.pose.position.y = goal_y;
    goal_position.pose.orientation.w = goal_w;
    goal_pub.publish(goal_position);
    ROS_INFO("Sending goal");
  }
  else
  {
    geometry_msgs::PoseStamped goal_position;
    goal_position.header.frame_id = "map";
    goal_position.header.stamp = ros::Time::now();
    goal_position.pose.position.x = 0;
    goal_position.pose.position.y = 0;
    goal_position.pose.orientation.w = goal_w;
    goal_pub.publish(goal_position);
    ROS_INFO("No Frontier Detected");
  }
  marker_pub.publish(points);
  marker_pub.publish(cluster_points);
  points.points.clear();
  cluster_points.points.clear();
}

void PoseUpdate(const geometry_msgs::PoseStampedConstPtr& pose)
{
   pose_ptr_ = pose;
   IsPoseUpdated = 1;
}

int main(int argc, char **argv)
{
   IsPoseUpdated = 0;
   ros::init(argc, argv, "find_frontier_node");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("map", 1000, mapConvert);
   ros::Subscriber sub2 = nh.subscribe("slam_out_pose",1000, PoseUpdate);
   marker_pub = nh.advertise<visualization_msgs::Marker>("FrontierLocationMarker",1);
   goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
   ros::spin();
   return 0;
}
