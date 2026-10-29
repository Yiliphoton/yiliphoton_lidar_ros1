
#pragma once

#ifdef _WIN32

#include "lidar_sdk/driver/input/socket/input_sock_win.hpp"

#else  //__linux__

#include "lidar_sdk/driver/input/socket/input_sock_unix.hpp"

#endif

