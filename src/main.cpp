#include "VL_FUSE/VIS_LIDAR_FUSE.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  VIS_LIDAR_FUSE node_init;

  node_init.INITIALIZE();

  return 0;
}
