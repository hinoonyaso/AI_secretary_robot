#include <rclcpp/rclcpp.hpp>
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/ob_camera_node.h>
#include <memory>
#include <magic_enum/magic_enum.hpp>
#include <iostream>

using namespace std;


using namespace orbbec_camera;

shared_ptr<ob::Device> initializeDevice(shared_ptr<ob::Pipeline> pipeline) {
  auto device = pipeline->getDevice();
  if (!device) {
    cout << "No device found" << endl;
    return nullptr;
  }
  return device;
}

void listSensorProfiles(const shared_ptr<ob::Device>& device) {
  auto sensor_list = device->getSensorList();
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profile_list = sensor->getStreamProfileList();
    for (size_t j = 0; j < profile_list->count(); j++) {
      auto origin_profile = profile_list->getProfile(j);
      if (sensor->type() == OB_SENSOR_COLOR || sensor->type() == OB_SENSOR_DEPTH ||
          sensor->type() == OB_SENSOR_IR || sensor->type() == OB_SENSOR_IR_LEFT ||
          sensor->type() == OB_SENSOR_IR_RIGHT) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        cout << magic_enum::enum_name(sensor->type()) << " profile: " << profile->width()
                  << "x" << profile->height() << " " << profile->fps() << "fps "
                  << magic_enum::enum_name(profile->format()) << endl;
      } else if (sensor->type() == OB_SENSOR_ACCEL) {
        auto profile = origin_profile->as<ob::AccelStreamProfile>();
        cout << magic_enum::enum_name(sensor->type()) << " profile: " << profile->sampleRate()
                  << "  full scale_range " << profile->fullScaleRange() << endl;
      } else if (sensor->type() == OB_SENSOR_GYRO) {
        auto profile = origin_profile->as<ob::GyroStreamProfile>();
        cout << magic_enum::enum_name(sensor->type()) << " profile: " << profile->sampleRate()
                  << "  full scale_range " << profile->fullScaleRange() << endl;
      } else {
        cout << "Unknown profile: " << magic_enum::enum_name(sensor->type()) << endl;
      }
    }
  }
}

void printDeviceProperties(const shared_ptr<ob::Device>& device) {
  if (!device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
    cout << "Current device not support depth work mode!" << endl;
    return;
  }
  auto current_depth_mode = device->getCurrentDepthWorkMode();
  cout << "Current depth mode: " << current_depth_mode.name << endl;
  auto depth_mode_list = device->getDepthWorkModeList();
  cout << "Depth mode list: " << endl;
  for (uint32_t i = 0; i < depth_mode_list->count(); i++) {
    cout << "Depth_mode_list[" << i << "]: " << (*depth_mode_list)[i].name << endl;
  }
}

void printPreset(const shared_ptr<ob::Device>& device) {
  auto preset_list = device->getAvailablePresetList();
  if (!preset_list || preset_list->count() == 0) {
    return;
  }
  cout << "Preset list:" << endl;
  for (uint32_t i = 0; i < preset_list->count(); i++) {
    auto name = preset_list->getName(i);
    cout << "Preset list[" << i << "]: " << name << endl;
  }
}
int main() {
  auto pipeline = make_shared<ob::Pipeline>();
  auto device = initializeDevice(pipeline);
  if (!device) {
    return -1;  // Device initialization failed
  }
  listSensorProfiles(device);
  printDeviceProperties(device);
  printPreset(device);
  return 0;
}
