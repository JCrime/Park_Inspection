#include "pathtracker.h"

void PathTracker::resetTracker()
{
  lastPrepoint=0;
  nowPrepoint=0;
  path_latest.clear();
  isValid=false;
}

int PathTracker::calPrepoint()
{
  double dx,dy;
  for(size_t i=lastPrepoint;i < path_latest.size();++i)
  {
    dx=car_x-path_latest[i].pose.position.x;
    dy=car_y-path_latest[i].pose.position.y;

//    if(dx*dx+dy*dy < PREDISTANCE)
//    {
//      continue;
//    }
//    else
//    {
//      xe=-dx;
//      ye=-dy;
//      nowPrepoint=i;
//      break;
//    }
    if((dx*dx+dy*dy > PREDISTANCE) || (path_latest.size()-1) == i)
    {
      xe=-dx;
      ye=-dy;
      lastPrepoint=nowPrepoint=i;
      break;
    }
  }
  if(nowPrepoint>path_latest.size()-3)
  {
    isValid=false;
    std::cout<<"tracking work is over"<<std::endl;
  }
  //std::cout<<"No. "<<nowPrepoint<<" of "<<path_latest.size()<<std::endl;
  return nowPrepoint;
}

double PathTracker::calSteer()
{

  double L = 0.6;
  double k = 15;                     //写这个是为了方便调试。确定k值之后应该把窗口中的lineEdit删掉
  double l = sqrt(xe * xe + ye * ye);

  double ep=l*(sin(car_a)*(xe/l)-cos(car_a)*(ye/l));
  double D=atan(2*L*ep/k);

  double angle=D*180/3.1416;
  return angle;
}

int PathTracker::calNearestPoint()
{
  double dx,dy,ds,mind=100000;
  for(int i=0;i <= path_latest.size() - 1;++i)
  {
    dx=car_x-path_latest[i].pose.position.x;
    dy=car_y-path_latest[i].pose.position.y;
    ds=dx*dx+dy+dy;

    if(ds>mind)
    {
      lastPrepoint=i;
      break;
    }
    else
    {
      mind=ds;
    }
  }
  return lastPrepoint;

}



