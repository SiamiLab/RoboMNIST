#include <iostream>
#include <string>
#include <sstream>

#include <QCoreApplication>
#include <QObject>
#include <QUdpSocket>
#include <QByteArray>
#include <QString>
#include <QTimer>
#include <QNetworkInterface>
#include <QList>

#include <Eigen/Core>

#include <franka/robot.h>

using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;

class NetworkHandler : public QObject
{
    Q_OBJECT
public:
    explicit NetworkHandler(std::string* trigger, franka::RobotState* last_state, std::string collector_ip, int collector_port, const std::map<std::string, std::vector<Vector7d>>& action_map, QObject *parent = nullptr);

public slots:
    void processPendingDatagrams();
    void send_last_state();


private:
    QCoreApplication* app;
    QUdpSocket* udp_socket;
    QUdpSocket* udp_socket_sender;
    QTimer* timer;
    std::string* trigger;
    franka::RobotState* last_state;
    std::string collector_ip;
    int collector_port;
    const std::map<std::string, std::vector<Vector7d>> action_map;
};