/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"

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

bool ReadCameraConfig(const wpi::json& config) {
	CameraConfig c;

	// try to get the camera name
	try {
		c.name = config.at("name").get<std::string>();
	} catch (const wpi::json::exception& e) {
		std::cerr << "could not read camera name: " << e.what() << "\n";	
		return false;
	}
	
	// try to get the camera path
	try {
		c.path = config.at("path").get<std::string>();
	} catch (const wpi::json::exception& e) {
		std::cerr << "camera '" << c.name << "': could not read path: " << e.what() << "\n";
		return false;
	}

	// read stream properties
	if (config.count("stream") != 0) c. streamConfig = config.at("stream");

	c.config = config;
	
	cameraConfigs.emplace_back(std::move(c));
	return true; 
}

bool ReadSwitchedCameraConfig(const wpi::json& config) {
	SwitchedCameraConfig c;

	// get the camera name
	try {
		c.name = config.at("name").get<std::string>();
	} catch (const wpi::json::exception& e) {
		std::cerr << "could not read switched camera name: " << e.what() << "\n";
		return false;
	}	

	// get the camera key
	try {
		c.key = config.at("key").get<std::string>();
	} catch (const wpi::json::exception& e) {
		std::cerr << "switched camera: '" << c.name << "' could not read key: " << e.what() << "\n";
		return false;
	}

	return true;	
}
bool ReadConfig() {
	// attempt to open a stream to read config file

	std::error_code ec;
	wpi::raw_fd_istream is(configFile, ec);
 	// if an error exists print out error and exit	
	if (ec) {
		std::cerr << "could not open '" << configFile << "': " << ec.message() << "\n";
		return false;
	}
	
	// attempt to parse json config file
	wpi::json j;
	try {
		j = wpi::json::parse(is);
	} catch (const wpi::json::parse_error& e) {
		std::cerr << "error on byte " << e.byte << "; " << e.what() << "\n";
		return false;
	}

	// double check that top level is an object (entire data structure is encapsulated
	if (!j.is_object()) {
		std::cerr << "JSON file must be encapsulated as a full object\n";	
		return false;
	}

	// try to read team number
	try {
		team = j.at("team").get<unsigned int>();	
	} catch (const wpi::json::exception& e) {
		std::cerr << "could not read team number: " << e.what() << "\n";
		return false;
	}

	// read ntmode if it exists (ntmode is whether this is running as a server or client)
	if (j.count("ntmode") != 0) {
		try {
			auto str = j.at("ntmode").get<std::string>();
			wpi::StringRef s(str);  // turn raw ntmode into wpi ref. string	
			if (s.equals_lower("client")) {
				server = false;
			}else if (s.equals_lower("server")) {
				server = true;
			}else {
				std::cerr << "could not understand ntmode value '" << str << "'\n";
			}	
		} catch (const wpi::json::exception& e) {
			std::cerr << "could not read ntmode: " << e.what() << "\n";
			return false;
		}
	}

	// try to read camera config
	try {
		for (auto && camera : j.at("cameras")) {
			if (!ReadCameraConfig(camera)) return false;
		}
	} catch (const wpi::json::exception e) {
		std::cerr << "Error reading camera config: " << e.what() << "\n";
		return false;
	}

	std::cout << "This is a brain czech\n";
	if (j.count("switched cameras") != 0) {
		std::cout << "testing==============++!!!!!\n";
		try {
			for (auto&& camera : j.at("switched cameras")) {
				if (!ReadSwitchedCameraConfig(camera)) return false;
			}
		} catch (const wpi::json::exception& e) {
			std::cerr << "could not erad switched cameras: " << e.what() << "\n";
			return false;
		}
	}

	return true;
}
cs::UsbCamera StartCamera(const CameraConfig& config) {
	std::cout << "Starting camera '" << config.name << "' on " << config.path << "\n";
	auto inst = frc::CameraServer::GetInstance();
	cs::UsbCamera camera{config.name, config.path};
	auto server = inst->StartAutomaticCapture(camera);

	camera.SetConfigJson(config.config);
	camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

	if (config.streamConfig.is_object())
		server.SetConfigJson(config.streamConfig);

	return camera;
}
class MyPipeline : public frc::VisionPipeline {
	public:
		int val = 0;
		
		void Process(cv::Mat& mat) override {
			++val;	
		}
};

cs::MjpegServer StartSwitchedCamera(const SwitchedCameraConfig& config) {
	std::cout << "Starting Switched camera '" << config.name << "' on " << config.key << "\n";

	auto server = frc::CameraServer::GetInstance()->AddSwitchedCamera(config.name);

	/*  Okay so this block of code is legit insane and I'm going to break it down
	    Alright, so it basically lets you switch which camera is being streamed via either a key or a string identifier
	*/
	nt::NetworkTableInstance::GetDefault()
		.GetEntry(config.key)
		.AddListener(
			[server](const auto& event) mutable {
				if (event.value->IsDouble()) {
					int i = event.value->GetDouble();
					if (i >= 0 && i < cameras.size()) server.SetSource(cameras[i]);
				} else if (event.value->IsString()) {
					auto str = event.value->GetString();
					for (int i = 0 ; i < cameraConfigs.size() ; i++) {
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
} //  end of namespace
int main(int argc, char* argv[]) {
//  wpi::outs() << "uiaweuiawghuiowaegngwbnwuieg=============awpiuegaweiogaweuih\n";
//`  for (;;) {
//	  std::cout << "aowiejgiauwGHiu===awekgwegui===awjg\n";
//	std::this_thread::sleep_for(std::chrono::seconds(1));
//  }
  if (argc >= 2) configFile = argv[1];

  // read configuration
  if (!ReadConfig()) {
	wpi::outs() << "plz\n";
	std::cout << "plz\n";
	return EXIT_FAILURE;
  }

  // start NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  if (server) {
    wpi::outs() << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } else {
    wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
  }

	for (const auto& config : cameraConfigs) 
		cameras.emplace_back(StartCamera(config));

	//  start switched cameras
	for (const auto& config : switchedCameraConfigs) {
		StartSwitchedCamera(config);
		std::cout << "detected switched" << std::endl;
	}

  // start image processing on camera 0 if present
  if (cameras.size() >= 1) {
    std::thread([&] {
      frc::VisionRunner<MyPipeline> runner(cameras[0], new MyPipeline(),
                                           [&](MyPipeline &pipeline) {
        // do something with pipeline results
      });
      /* something like this for GRIP:
      frc::VisionRunner<MyPipeline> runner(cameras[0], new grip::GripPipeline(),
                                           [&](grip::GripPipeline& pipeline) {
        ...
      });
       */
      runner.RunForever();
    }).detach();
  }

  // loop forever
  for (;;) {
    std::cout << "OI\n";
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}
