#include "udp_utils.h"


void udp_send(json parameter, bool &running, Data2Send &data_send) {

    std::string remote_ip = parameter["remote_ip"];
    const char *remote_ip_c = remote_ip.c_str();

    uint16_t PORTSend = 5001; //parameter["remote_port"]

    int sockfd;

    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORTSend);
    servaddr.sin_addr.s_addr = inet_addr(remote_ip_c);

    const int NoDatatoSend = 36;

    while (running) {
        std::unique_lock<std::mutex> lck_send(data_send.mutex);
        cv_send.wait(lck_send, [] { return send_allowed; });

        double msg2send[NoDatatoSend];
        msg2send[0] = data_send.time_local;

        std::copy(data_send.q_robot.begin(), data_send.q_robot.end(), msg2send + 1);
        std::copy(data_send.dq_robot.begin(), data_send.dq_robot.end(), msg2send + 8);
        std::copy(data_send.tau_ext_robot.begin(), data_send.tau_ext_robot.end(), msg2send + 15);
        std::copy(data_send.f_ext_robot.begin(), data_send.f_ext_robot.end(), msg2send + 22);
        std::copy(data_send.E_robot_in.begin(), data_send.E_robot_in.end(), msg2send + 28);

        msg2send[35] = data_send.has_data_robot;

        sendto(sockfd, msg2send, sizeof(msg2send), MSG_CONFIRM, (const struct sockaddr *)&servaddr, sizeof(servaddr));

        send_allowed = false;
        lck_send.unlock();
    }

    close(sockfd);
}


void udp_recv(json parameter, bool &running, Data2Recv &data_recv) {

    
    uint16_t PORTRECV = 5001; //parameter["local_port"] 

    int sockfd;

    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information RECV
    cliaddr.sin_family = AF_INET;  
    cliaddr.sin_addr.s_addr = INADDR_ANY;
    cliaddr.sin_port = htons(PORTRECV);

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr *)&cliaddr, sizeof(cliaddr)) < 0) {
        std::cout << "bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // int n, n_rev;
    socklen_t len;
    len = sizeof(cliaddr);  // len is value/resuslt

    const int NoDatatoRecv = 36;

    double msg2recv[NoDatatoRecv];

    while (running) {

        recvfrom(sockfd, msg2recv, sizeof(msg2recv), MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);

        if (data_recv.mutex.try_lock()) {
            data_recv.time_remote = msg2recv[0];

            std::copy(msg2recv + 1, msg2recv + 8, data_recv.q_cloud.begin());
            std::copy(msg2recv + 8, msg2recv + 15, data_recv.dq_cloud.begin());
            std::copy(msg2recv + 15, msg2recv + 22, data_recv.tau_com_cloud.begin());
            std::copy(msg2recv + 22, msg2recv + 28, data_recv.f_com_cloud.begin());
            std::copy(msg2recv + 28, msg2recv + 35, data_recv.E_cloud_in.begin());

            data_recv.has_data_cloud = msg2recv[35];

            data_recv.mutex.unlock();
        } else {
            std::cout << "No receive lock in UDP thread" << std::endl;
        }

    }
    close(sockfd);
}
