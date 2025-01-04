// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <thread>

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QByteArray>

#include <franka/exception.h>
#include <franka/robot.h>

#include "kian_action_arc.h"
#include "network_handler.h"


void read_config_file(std::string& robot_ip, std::string& collector_ip, int& collector_port, double& speed_factor, double& uncertainty_variance_lower, double& uncertainty_variance_upper, std::map<std::string, std::vector<Vector7d>>& action_map)
{
  QString filePath = "config.json";
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      std::cout << "ERROR: Failed to open config file:" << file.errorString().toStdString() << std::endl;
      return;
  }
  QByteArray jsonData = file.readAll();
  file.close();
  QJsonParseError parseError;
  QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &parseError);
  if (parseError.error != QJsonParseError::NoError) {
    std::cout << "ERROR: Failed to parse JSON:" << parseError.errorString().toStdString() << std::endl;
    return;
  }
  QJsonObject jsonObj = jsonDoc.object();
  robot_ip = jsonObj.value("robot_ip").toString().toStdString();
  collector_ip = jsonObj.value("collector_ip").toString().toStdString();
  collector_port = jsonObj.value("collector_port").toInt();
  speed_factor = jsonObj.value("speed_factor").toDouble();
  uncertainty_variance_lower = jsonObj.value("uncertainty_variance_lower").toDouble();
  uncertainty_variance_upper = jsonObj.value("uncertainty_variance_upper").toDouble();

  // std::cout << "Robot IP: " << robot_ip << std::endl;
  // std::cout << "Collector IP: " << collector_ip << std::endl;

  // Read other keys like Arc, SLFW, etc. (actions)
  for (auto it = jsonObj.begin(); it != jsonObj.end(); ++it) {
      QString key = it.key();
      if (key != "robot_ip" && key != "collector_ip" && key != "collector_port" && key != "speed_factor" && key != "uncertainty_variance_lower" && key != "uncertainty_variance_upper") {
          QJsonArray jsonArray = it.value().toArray();
          std::vector<Eigen::Matrix<double, 7, 1, 0>> matrixVector;
          for (int i = 0; i < jsonArray.size(); ++i) {
              QJsonArray innerArray = jsonArray[i].toArray();
              Eigen::Matrix<double, 7, 1, 0> eigenMatrix;
              for (int j = 0; j < innerArray.size(); ++j) {
                  eigenMatrix(j) = innerArray[j].toDouble();
              }
              matrixVector.push_back(eigenMatrix);
          }
          action_map[key.toStdString()] = matrixVector;
      }
  }

  // for(const auto d : action_map["Arc2"])
  //   std::cout << d.transpose() << std::endl;

}


void robot_func(std::string* trigger, franka::RobotState* last_state, std::string robot_ip, double speed_factor, double uncertainty_variance_lower, double uncertainty_variance_upper, std::map<std::string, std::vector<Vector7d>>* action_map)
{
  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    MotionGenerator motion_generator(speed_factor, uncertainty_variance_lower, uncertainty_variance_upper, trigger, last_state, *action_map);
    robot.control(motion_generator);

  } catch (const franka::Exception& e){
    std::cout << e.what() << std::endl;
    return;
  }
}

int main(int argc, char** argv) {
  // if (argc != 3) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname/ip> <collector-ip>" << std::endl;
  //   return -1;
  // }
  std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  QCoreApplication app{argc, argv};


  std::string robot_ip;
  std::string collector_ip;
  int collector_port;
  double speed_factor;
  double uncertainty_variance_upper;
  double uncertainty_variance_lower;
  std::map<std::string, std::vector<Vector7d>> action_map; // maps the action to the vector of joint positions
  read_config_file(robot_ip, collector_ip, collector_port, speed_factor, uncertainty_variance_lower, uncertainty_variance_upper, action_map);

  std::string* trigger = new std::string{""};
  franka::RobotState* last_state = new franka::RobotState{};

  NetworkHandler network_handler{trigger, last_state, collector_ip, collector_port, action_map};



  std::thread robot_thread(robot_func, trigger, last_state, robot_ip, speed_factor, uncertainty_variance_lower, uncertainty_variance_upper, &action_map);


  
  app.exec();
  return 0;
}


#include "main.moc"
