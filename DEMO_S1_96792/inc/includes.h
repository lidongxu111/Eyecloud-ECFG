/**
 * @file   includes.h
 * @author Zed
 * @email  zhangdi@eyecloud.tech
 * @date
 * @brief  Contains most of the required common header files
 */

#ifndef INCLUDE_H_
#define INCLUDE_H_

#define _WINDOWS

#include <map>
#include <string>
#include <math.h>
#include <time.h>
#include <vector>
#include <thread>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>

// #include "sysinfo.h"

#ifndef _WINDOWS
#include <netdb.h>
#include <assert.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <arpa/inet.h>
#include <semaphore.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#else
#include <io.h>
#include <direct.h>
#include <fcntl.h>
#include <windows.h>
#endif

#define BYTE uint8_t
#define INT8U uint8_t
#define INT16U uint16_t
#define INT16 int16_t
#define INT32U uint32_t
#define INT32 int32_t
#define UINT uint32_t
#define BOOL unsigned char

#endif
