#include "optimization.h"
#include "online_backend.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string mode;
  nh.param<std::string>("backend/mode", mode, "batch");
  std::unique_ptr<optimization> batch;
  std::unique_ptr<online::Backend> incremental;
  if (mode == "online") incremental.reset(new online::Backend(nh));
  else if (mode == "batch") batch.reset(new optimization(nh));
  else {
    ROS_FATAL("backend/mode must be 'online' or 'batch'");
    return 1;
  }
  LIVMapper mapper(nh);
  if (incremental) {
    mapper.setBackendCallbacks(
        [&](const backend::KeyframeEvent& event) { incremental->onKeyframe(event); },
        [&](const backend::GnssEvent& event) { incremental->onGnss(event); });
  } else if (!batch->debug_mode) {
    mapper.setBackendCallbacks(
        [&](const backend::KeyframeEvent& event) { batch->onKeyframe(event); },
        [&](const backend::GnssEvent& event) { batch->onGnss(event); });
  }

  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}
