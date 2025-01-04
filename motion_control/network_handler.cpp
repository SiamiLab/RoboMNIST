#include "network_handler.h"

NetworkHandler::NetworkHandler(std::string* trigger, franka::RobotState* last_state, std::string collector_ip, int collector_port, const std::map<std::string, std::vector<Vector7d>>& action_map, QObject *parent)
    : QObject(parent), trigger{trigger}, last_state{last_state}, collector_ip{collector_ip}, collector_port{collector_port}, action_map{action_map}
{
    // Find the network interface with the specific name
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    QNetworkInterface chosenInterface;

    foreach (const QNetworkInterface &interface, interfaces) {
        if (interface.humanReadableName() == "wlx9cc9ebf83870" || interface.humanReadableName() == "enx7cc2c648c947"){ // change this //wifi dongle: wlx9cc9ebf83870 - usb dongle: enx7cc2c648c947
            chosenInterface = interface;
            break;
        }
    }
    udp_socket_sender = new QUdpSocket{};
    udp_socket_sender->bind(chosenInterface.addressEntries().first().ip(), 0);
    

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &NetworkHandler::send_last_state);
    timer->setInterval(1); // 1khz same as robot loop
    timer->start();

    udp_socket = new QUdpSocket{};
    if (!udp_socket->bind(QHostAddress::Any, 12345))
        std::cout << "Failed to bind port!" << std::endl;
    connect(udp_socket, &QUdpSocket::readyRead, this, &NetworkHandler::processPendingDatagrams);
    
}

void NetworkHandler::send_last_state()
{
    std::stringstream ss;
    std::vector<double> arr{last_state->q.begin(), last_state->q.end()}; // store joint positions
    // store endefector position in base frame
    arr.push_back(last_state->O_T_EE[12]);
    arr.push_back(last_state->O_T_EE[13]);
    arr.push_back(last_state->O_T_EE[14]);
    // convert arr to string and send
    ss << arr[0];
    for (size_t i = 1; i < arr.size(); ++i) {
        ss << ',' << arr[i];
    }
    std::string result = ss.str();
    QByteArray datagram = QString::fromStdString(result).toUtf8();
    udp_socket_sender->writeDatagram(datagram.data(), datagram.size(), QHostAddress{QString::fromStdString(collector_ip)}, collector_port); // QHostAddress::Broadcast
}


void NetworkHandler::processPendingDatagrams()
{
    while (udp_socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udp_socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        udp_socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        // std::cout << "Received datagram:" << datagram.constData() << std::endl;
        // std::cout << "From:" << sender.toString().toStdString() << ":" << senderPort << std::endl;
        std::string trigger_tmp = QString(datagram).trimmed().toStdString();
        if (action_map.find(trigger_tmp) != action_map.end())
            *trigger = trigger_tmp;
        else
        {
            trigger->clear();
            std::cout << "ERROR: the requested action (" << trigger_tmp << ") is not defined!" << std::endl;
        }
        
    }
}
