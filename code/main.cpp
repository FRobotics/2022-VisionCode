// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringExtras.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"

//#include <wpi/StringRef.h>
//#include <iostream>

#include "GripPipeline.cpp"

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

static const char* configFile = "/boot/frc.json";

namespace {

unsigned int team;
bool server = false;

struct CameraConfig {
  std::string name;
  std::string path;
  wpi::json config;
  wpi::json streamConfig;
};

struct SwitchedCameraConfig {
  std::string name;
  std::string key;
};

std::vector<CameraConfig> cameraConfigs;
std::vector<SwitchedCameraConfig> switchedCameraConfigs;
std::vector<cs::VideoSource> cameras;

void ParseError(std::string_view msg) {
  fmt::print(stderr, "config error in '{}': {}\n", configFile, msg);
}

bool ReadCameraConfig(const wpi::json& config) {
  CameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError(fmt::format("could not read camera name: {}", e.what()));
    return false;
  }

  // path
  try {
    c.path = config.at("path").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError(fmt::format("camera '{}': could not read path: {}", c.name, e.what()));
    return false;
  }

  // stream properties
  if (config.count("stream") != 0) c.streamConfig = config.at("stream");

  c.config = config;

  cameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadSwitchedCameraConfig(const wpi::json& config) {
  SwitchedCameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError(fmt::format("could not read switched camera name: {}",
                           e.what()));
    return false;
  }

  // key
  try {
    c.key = config.at("key").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError(fmt::format("switched camera '{}': could not read key: {}",
                           c.name, e.what()));
    return false;
  }

  switchedCameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadConfig() {
  // open config file
  std::error_code ec;
  wpi::raw_fd_istream is(configFile, ec);
  if (ec) {
    wpi::errs() << "could not open '" << configFile << "': " << ec.message()
                << '\n';
    return false;
  }

  // parse file
  wpi::json j;
  try {
    j = wpi::json::parse(is);
  } catch (const wpi::json::parse_error& e) {
    ParseError(fmt::format("byte {}: {}", e.byte, e.what()));
    return false;
  }

  // top level must be an object
  if (!j.is_object()) {
    ParseError("must be JSON object");
    return false;
  }

  // team number
  try {
    team = j.at("team").get<unsigned int>();
  } catch (const wpi::json::exception& e) {
    ParseError(fmt::format("could not read team number: {}", e.what()));
    return false;
  }

  // ntmode (optional)
  if (j.count("ntmode") != 0) {
    try {
      auto str = j.at("ntmode").get<std::string>();
      if (wpi::equals_lower(str, "client")) {
        server = false;
      } else if (wpi::equals_lower(str, "server")) {
        server = true;
      } else {
        ParseError(fmt::format("could not understand ntmode value '{}'", str));
      }
    } catch (const wpi::json::exception& e) {
      ParseError(fmt::format("could not read ntmode: {}", e.what()));
    }
  }

  // cameras
  try {
    for (auto&& camera : j.at("cameras")) {
      if (!ReadCameraConfig(camera)) return false;
    }
  } catch (const wpi::json::exception& e) {
    ParseError(fmt::format("could not read cameras: {}", e.what()));
    return false;
  }

  // switched cameras (optional)
  if (j.count("switched cameras") != 0) {
    try {
      for (auto&& camera : j.at("switched cameras")) {
        if (!ReadSwitchedCameraConfig(camera)) return false;
      }
    } catch (const wpi::json::exception& e) {
      ParseError(fmt::format("could not read switched cameras: {}", e.what()));
      return false;
    }
  }

  return true;
}

cs::UsbCamera StartCamera(const CameraConfig& config) {
  fmt::print("Starting camera '{}' on {}\n", config.name, config.path);
  cs::UsbCamera camera{config.name, config.path};
  auto server = frc::CameraServer::StartAutomaticCapture(camera);

  camera.SetConfigJson(config.config);
  camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

  if (config.streamConfig.is_object())
    server.SetConfigJson(config.streamConfig);

  return camera;
}

cs::MjpegServer StartSwitchedCamera(const SwitchedCameraConfig& config) {
  fmt::print("Starting switched camera '{}' on {}\n", config.name, config.key);
  auto server = frc::CameraServer::AddSwitchedCamera(config.name);

  nt::NetworkTableInstance::GetDefault()
      .GetEntry(config.key)
      .AddListener(
          [server](const auto& event) mutable {
            if (event.value->IsDouble()) {
              int i = event.value->GetDouble();
              if (i >= 0 && i < cameras.size()) server.SetSource(cameras[i]);
            } else if (event.value->IsString()) {
              auto str = event.value->GetString();
              for (int i = 0; i < cameraConfigs.size(); ++i) {
                if (str == cameraConfigs[i].name) {
                  server.SetSource(cameras[i]);
                  break;
                }
              }
            }
          },
          NT_NOTIFY_IMMEDIATE | NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

  return server;
}

// example pipeline
class MyPipeline : public frc::VisionPipeline {
 public:
  int val = 0;

  void Process(cv::Mat& mat) override {
    ++val;
  }
};
}  // namespace

int main(int argc, char* argv[]) {

  fmt::print("\nStarting vision program.  Starting wait.\n");

  std::this_thread::sleep_for(std::chrono::seconds(29));

  fmt::print("Wait complete.\n");

  if (argc >= 2) configFile = argv[1];

  // read configuration
  if (!ReadConfig()) return EXIT_FAILURE;

  // start NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  if (server) {
    fmt::print("Setting up NetworkTables server\n");
    ntinst.StartServer();
  } else {
    fmt::print("Setting up NetworkTables client for team {}\n", team);
    ntinst.StartClientTeam(team);
    ntinst.StartDSClient();
  }

  // -- TEAM 4150 -- if a camera named vision is not specified.
  int visionIndex = 100;

  // start cameras
  for (const auto& config : cameraConfigs) {
    cameras.emplace_back(StartCamera(config));
    // -- TEAM 4150 -- if camera is named Vision, then remember the index...
    if (cameras.back().GetName() == "Vision") {
      visionIndex = cameras.size() - 1; 
    }
  }
  
  // -- TEAM 4150  -- write which camera we are using.
  ntinst.GetEntry("/vision/Debug").SetDouble((double)visionIndex);
  if (visionIndex == 100) {
    ntinst.GetEntry("/vision/Debug").SetString("Vision Index Not Set!");
    visionIndex = 0;	// set to zero if camera named Vision not found.
  }
  

  // start switched cameras
  for (const auto& config : switchedCameraConfigs) StartSwitchedCamera(config);

  
  // TEAM 4150  -- provide the grip class with our network tables instance.
  grip::nt = &ntinst;

  // start image processing on camera 0 if present
  // TEAM 4150 -- changed to visionIndex.   was 1.
  if (cameras.size() >= visionIndex) {   
    std::thread([&] {
      frc::VisionRunner<grip::GripPipeline> runner(
			cameras[visionIndex], 
			new grip::GripPipeline(),
                        [&](grip::GripPipeline &pipeline) {
        // ---- do something with pipeline results
	grip::FetchVisionNetworkTable();
      });
      /* something like this for GRIP:
      frc::VisionRunner<MyPipeline> runner(cameras[0], new grip::GripPipeline(),
                                           [&](grip::GripPipeline& pipeline) {
        ...
      });
       */

      // TEAM 4150
      grip::hsvThresholdEntries[0] = ntinst.GetEntry("/vision/HSV/hueLow");      
      grip::hsvThresholdEntries[1] = ntinst.GetEntry("/vision/HSV/hueHigh");
      grip::hsvThresholdEntries[2] = ntinst.GetEntry("/vision/HSV/satLow");
      grip::hsvThresholdEntries[3] = ntinst.GetEntry("/vision/HSV/satHigh");
      grip::hsvThresholdEntries[4] = ntinst.GetEntry("/vision/HSV/valLow");
      grip::hsvThresholdEntries[5] = ntinst.GetEntry("/vision/HSV/valHigh");

      grip::outputEntries[0] = ntinst.GetEntry("/vision/averageX");
      grip::outputEntries[1] = ntinst.GetEntry("/vision/averageY");
      grip::outputEntries[2] = ntinst.GetEntry("/vision/averageWidth");
      grip::outputEntries[3] = ntinst.GetEntry("/vision/averageHeight");
      grip::outputEntries[4] = ntinst.GetEntry("/vision/stripYOffset");  //  "/vision/distance");
      grip::outputEntries[5] = ntinst.GetEntry("/vision/stripeCount");
      grip::outputEntries[6] = ntinst.GetEntry("/vision/watchdog");
      grip::outputEntries[7] = ntinst.GetEntry("/vision/stripHeightDiff");
      grip::outputEntries[8] = ntinst.GetEntry("/vision/stripXOffset");

      grip::filterEntries[0] = ntinst.GetEntry("/vision/filterHeight");
      grip::filterEntries[1] = ntinst.GetEntry("/vision/deviationThreshold");

      grip::FetchVisionNetworkTable();

      runner.RunForever();
    }).detach();
  }

  fmt::print("Startup complete.\n");

  // loop forever
  double n = 0;
  for (;;) {
	n++;
	grip::FetchVisionNetworkTable();
	std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
