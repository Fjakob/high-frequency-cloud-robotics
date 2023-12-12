#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>

// json import
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "data_types.h"

std::condition_variable cv_send;
bool send_allowed{false};

void udp_send(json parameter, bool &running, Data2Send &data_send);
void udp_recv(json parameter, bool &running, Data2Recv &data_recv);
