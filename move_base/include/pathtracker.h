#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>


const float PREDISTANCE=8.0;

class PathTracker
{
public:
  int lastPrepoint=0,nowPrepoint=0;
  std::vector<geometry_msgs::PoseStamped> path_latest;
  geometry_msgs::PoseStamped car_pose;
  double car_x,car_y,car_a;
  double xe,ye;
  bool isValid=false;

  int calPrepoint();
  double calSteer();
  int calNearestPoint();
  void resetTracker();
};
