
/*
	This is a demo written in C++to demonstrate how to call all interfaces in ecfg_sdk, as well as the order of interface calls.
	This example uses device S16-96792 and module imx735 as examples to obtain a full resolution image display.

	1. Initialize all deserialization chips.
	2. Initialize module configuration.
	3. Select the port that has been successfully initialized and process the preview display.
	4.30 minutes as a round of testing, load the device again to run, 
	and there is a necessary delay between the two tests to ensure that the device can be correctly counted after restarting.
 */

#include <includes.h>
#include <ecfg_sdk.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <fstream>
#include "cJSON.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui_c.h>

// During single channel operation, only one A or B is connected
//  #define DES_INIT_FILE_PATH "./config/imx735_max96793_3000_5760_20_12bit_4L_single_G2/imx735_max96793_3000_5760_20_12bit_4L_single_G2_init.json"
//  #define SER_SENSOR_A_INIT_FILE_PATH "./config/imx735_max96793_3000_5760_20_12bit_4L_single_G2/imx735_max96793_3000_5760_20_12bit_4L_single_G2_A.json"
//  #define SER_SENSOR_B_INIT_FILE_PATH "./config/imx735_max96793_3000_5760_20_12bit_4L_single_G2/imx735_max96793_3000_5760_20_12bit_4L_single_G2_B.json"

// Simultaneously accessing configuration files for A and B
#define DES_INIT_FILE_PATH "./config/imx735_max96793_3000_5760_20_12bit_4L/imx735_max96793_3000_5760_20_12bit_4L_init.json"
#define SER_SENSOR_A_INIT_FILE_PATH "./config/imx735_max96793_3000_5760_20_12bit_4L/imx735_max96793_3000_5760_20_12bit_4L_A.json"
#define SER_SENSOR_B_INIT_FILE_PATH "./config/imx735_max96793_3000_5760_20_12bit_4L/imx735_max96793_3000_5760_20_12bit_4L_B.json"

// Display frame count
#define MAX_RECV_FRAME 100

// Display the width and height of the canvas
#define SHOW_IMAGE_W (1280)
#define SHOW_IMAGE_H (720)

// Maximum number of supported access devices in DEMO
#define MAX_DEV_SIZE (8)

// Save the port ID of the camera that has been successfully initialized
int active_camera_mark[MAX_DEV_SIZE][MAX_MULTI_PORT_NUM] = {0};
// Devices that are currently active and running
static int active_device_id = 0;
// Save the current number of devices
static int max_dev_id = 0;
// Determine whether the running thread has exited
static bool is_running = false;

using namespace std;

// Reading voltage values from configuration files
int get_module_voltage()
{
	int voltage = 0;
	/*
	The voltage value that the module needs to be set is in the 'voltage' field of
	the 'config/xx_xx_xx/xx_xx_xx_init.json' file
	*/
	std::string file_name = DES_INIT_FILE_PATH;
	std::ifstream file(file_name);
	if (file.is_open())
	{
		std::string buff((std::istreambuf_iterator<char>(file)),
				 (std::istreambuf_iterator<char>()));
		cJSON *root = cJSON_Parse(buff.c_str());
		if (root != nullptr)
		{
			cJSON *voltage_item = cJSON_GetObjectItem(root, "voltage");
			if (voltage_item != nullptr)
			{
				voltage = voltage_item->valueint;
			}
		}
		cJSON_Delete(root);
	}
	else
	{
		return -1;
	}
	return voltage;
}

// IIC test example
void i2c_test(ECFG *ecfg, vector<string> device_list)
{
	/*
	Continuously read registers, starting from 0x8401 and ending at 0x83F8
	*/
	int size = 0x8401 - 0x83F8 + 1;
	int *data = new int[size];
	int reg = 0x83F8;
	int ret = ecfg->read_i2c("/dev/port1-sen", reg, data, size);
	if (ret >= 0)
	{
		for (int i = 0; i < size; i++)
		{
			PTRACE_LOG(LOG_INFO, "Read Register 0x%x success,value: 0x%x", reg + i, data[i]);
		}
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Read Register 0x%x fail,ret: %d", reg, ret);
	}
	delete[] data;

	/*
	Read register separately, address 0x1C, read, write, and read again
	*/
	reg = 0x1C;
	size = 1;
	int read_data[1] = {0};
	int write_data[1] = {0xFF};
	/*
	First read of 0x1C register value
	*/
	ret = ecfg->read_i2c("/dev/G1-des", reg, read_data, size);
	if (ret >= 0)
	{
		PTRACE_LOG(LOG_INFO, "Read register 0x%x success,value: 0x%x", reg, read_data[0]);
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Read register 0x%x fail,ret: %d", reg, ret);
	}

	/*
	First write to 0x1C register value
	*/
	ret = ecfg->write_i2c("/dev/G1-des", reg, write_data, size);
	if (ret >= 0)
	{
		PTRACE_LOG(LOG_INFO, "Write register 0x%x success,value: 0x%x", reg, write_data[0]);
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Write register 0x%x fail,ret: %d", reg, ret);
	}

	/*
	Second read of 0x1C register value
	*/
	ret = ecfg->read_i2c("/dev/G1-des", reg, read_data, size);
	if (ret >= 0)
	{
		PTRACE_LOG(LOG_INFO, "Read register 0x%x success,value: 0x%x", reg, read_data[0]);
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Read register 0x%x fail,ret: %d", reg, ret);
	}

	reg = 0x03f4;
	ret = ecfg->read_i2c("/dev/port1-sen", reg, read_data, size);
	if (ret >= 0)
	{
		PTRACE_LOG(LOG_INFO, "Read register 0x%x success,value: 0x%x", reg, read_data[0]);
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Read register 0x%x fail,ret: %d", reg, ret);
	}
}

int query_device_init_status(ECFG *ecfg, int device_index)
{
	std::vector<std::string> device_list;
	int ret = ecfg->get_i2c_dev_list(device_list);
	if (ret < 0 || device_list.size() <= 0)
	{
		PTRACE_LOG(LOG_ERROR, "Failed to obtain device node name for device%d!", device_index);
		return ret;
	}
	else
	{
		/*
		打印设备节点列表
		*/
		PTRACE_LOG(LOG_INFO, "Device%d Node List:", device_index);
		for (int i = 0; i < device_list.size(); i++)
		{
			PTRACE_LOG(LOG_INFO, "%s", device_list[i].c_str());
		}

		/*
		I2C read and write test
		*/
		// i2c_test(ecfg, device_list);
	}
	return 0;
}

int query_module_init_status(ECFG *ecfg, int device_index)
{
	std::vector<int> camera_status_list;
	int ret = ecfg->get_cam_status(camera_status_list);
	if (ret < 0 || camera_status_list.size() <= 0)
	{
		PTRACE_LOG(LOG_ERROR, "Get device%d module status fail!", device_index);
		return ret;
	}
	else
	{
		for (int i = 0; i < camera_status_list.size(); i++)
		{
			/*
			Judging the initialization status of the module, if the return value is ECFG-CONNECTED or ECFG-STREAM,
			the module initialization is successful,
			while others have not failed
			*/
			if (camera_status_list[i] == ECFG_CONNECTED || camera_status_list[i] == ECFG_STREAM)
			{
				PTRACE_LOG(LOG_INFO, "Device:%d Module:%2d Configuration successful", device_index, i + 1);

				// Mark as valid module
				active_camera_mark[device_index][i] = true;
			}
			else
			{
				PTRACE_LOG(LOG_ERROR, "Device:%d Module:%2d Configuration fail", device_index, i + 1);
			}
		}
	}
	return 0;
}

/*
	The necessary operation process for initializing a device and its modules.
 */
int initialize_device_and_module(ECFG *ecfg, int device_index)
{
	/*
	3.Open the device
	*/
	int ret = ecfg->open();
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Open device%d fail!", device_index);
		return ret;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Open device%d success!", device_index);
	}

	/*
	4.Set module power supply voltage
	*/
	int voltage = get_module_voltage();
	if (voltage < 0)
	{
		return voltage;
	}
	ret = ecfg->set_poc_voltage(voltage);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d set voltage fail! ret: %d", device_index, ret);
		return ret;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Device%d set voltage success! voltage:%dmv", device_index, voltage);
	}

	////////////////////////S16//////////////////////
	/*
	5.To power on the port,
	simply power on the port to which the module is connected.
	The parameter is port number, port numbers 1-16
	*/
	int port_id;
	for (port_id = 1; port_id <= MAX_MULTI_PORT_NUM; port_id++)
	{
		ret = ecfg->poc_power_on(port_id);
		if (ret < 0)
		{
			PTRACE_LOG(LOG_ERROR, "Device%d port%d power on fail!", device_index, port_id);
			return ret;
		}
		else
		{
			PTRACE_LOG(LOG_INFO, "Device%d port%d power on success!", device_index, port_id);
		}
	}

	/* 6. Configure the main control and deserialization chips of the box*/
	/* Every two ports share a deserialization chip, so they are assigned to the same group*/
	/*
		port_id 1 and 2   : group_id 1;
		port_id 3 and 4   : group_id 2;
		port_id 5 and 6   : group_id 3;
		port_id 7 and 8   : group_id 4;
		port_id 9 and 10  : group_id 5;
		port_id 11 and 12 : group_id 6;
		port_id 13 and 14 : group_id 7;
		port_id 15 and 16 : group_id 8;
		1 and 2, 3 and 4, 5 and 6, 7 and 8 groups are the same development board.
		The deserialization chips on the same board need to be initialized sequentially,
		and different development boards can be initialized simultaneously
	*/
	/* Pass in group_id and init.json file path in the init function*/
	// Multi threaded optimization, parallel processing for chip initialization, 4-thread parallel processing
	// Each S16 has four independent S4 combinations, and each S4 can be processed in parallel
	std::thread *des_init_thread[MAX_MULTI_PORT_NUM / MAX_SINGLE_PORT_NUM];
	int sub_dev_index = 0;
	for (sub_dev_index = 0; sub_dev_index < MAX_MULTI_PORT_NUM / MAX_SINGLE_PORT_NUM; sub_dev_index++)
	{
		des_init_thread[sub_dev_index] = new std::thread([=]()
								 {
			int ret;
			int sub_grp_id = sub_dev_index * MAX_SINGLE_GROUP_NUM + 1;
			ret = ecfg->init(sub_grp_id, DES_INIT_FILE_PATH);
			if (ret < 0)
			{
				PTRACE_LOG(LOG_ERROR, "Device%d initialize the deserialization chip%d fail!", device_index, sub_grp_id);
				return ret;
			}
			else
			{
				PTRACE_LOG(LOG_INFO, "Device%d initialize the deserialization chip%d success!", device_index, sub_grp_id);
			}

			sub_grp_id++;
			ret = ecfg->init(sub_grp_id, DES_INIT_FILE_PATH);
			if (ret < 0)
			{
				PTRACE_LOG(LOG_ERROR, "Device%d initialize the deserialization chip%d fail!", device_index, sub_grp_id);
				return ret;
			}
			else
			{
				PTRACE_LOG(LOG_INFO, "Device%d initialize the deserialization chip%d success!", device_index, sub_grp_id);
			} });
	}

	for (sub_dev_index = 0; sub_dev_index < MAX_MULTI_PORT_NUM / MAX_SINGLE_PORT_NUM; sub_dev_index++)
	{
		des_init_thread[sub_dev_index]->join();
	}

	/* 7.Configuration module*/
	/*
		Ports 1-4, 5-8, 9-12, and 13-16 are the same development board.
		Modules on the same board need to be initialized and called sequentially,
		while modules on different development boards can be initialized and called simultaneously.
	*/
	// Multi threaded optimization, parallel processing of DES initialization
	// Each S16 has four independent S4 combinations, and each S4 can be processed in parallel
	std::thread *camera_init_thread[MAX_MULTI_PORT_NUM / MAX_SINGLE_PORT_NUM];

	sub_dev_index = 0;
	for (sub_dev_index = 0; sub_dev_index < MAX_MULTI_PORT_NUM / MAX_SINGLE_PORT_NUM; sub_dev_index++)
	{
		camera_init_thread[sub_dev_index] = new std::thread([=]()
								    {
			int ret = 0;
			int port_id = sub_dev_index * MAX_SINGLE_PORT_NUM + 1;
			int port_sum = port_id + MAX_SINGLE_PORT_NUM;
			for (; port_id < port_sum;)
			{
				if (port_id % 2 == 1)
				{
					ret = ecfg->start(port_id, SER_SENSOR_A_INIT_FILE_PATH);
					if (ret < 0)
					{
						PTRACE_LOG(LOG_ERROR, "Device%d camera%2d initializate failed,ret = %d!", device_index, port_id, ret);
					}
					else
					{
						PTRACE_LOG(LOG_INFO, "Device%d camera%2d initializate success!", device_index, port_id);
					}
				}
				else
				{
					ret = ecfg->start(port_id, SER_SENSOR_B_INIT_FILE_PATH);
					if (ret < 0)
					{
						PTRACE_LOG(LOG_ERROR, "Device%d camera%2d initializate fail,ret = %d!", device_index, port_id, ret);
					}
					else
					{
						PTRACE_LOG(LOG_INFO, "Device%d camera%2d initializate success!", device_index, port_id);
					}
				}

				port_id++;
			} });
	}

	for (sub_dev_index = 0; sub_dev_index < MAX_MULTI_PORT_NUM / MAX_SINGLE_PORT_NUM; sub_dev_index++)
	{
		camera_init_thread[sub_dev_index]->join();
	}

	return 0;
}

int close_device_and_module(ECFG *ecfg, int device_index)
{
	int ret = 0;
	for (int port_id = 1; port_id <= MAX_MULTI_PORT_NUM; port_id++)
	{
		/* 11.Power off the module port*/
		ret = ecfg->poc_power_off(port_id);
		if (ret < 0)
		{
			PTRACE_LOG(LOG_ERROR, "Device%d Port%d power off fail!", device_index, port_id);
		}
	}

	/* 12.Close device*/
	ret = ecfg->close();
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d close fail!", device_index);
		return ret;
	}

	return 0;
}

/*
	Display the read stream information and obtain YUV420P format data
*/
void compressed_resolution_display(ECFG *ecfg, int device_index, int port_id)
{
	/* 
	Obtain module information on the specified port,
	such as the original image width, original height, bit depth, etc 
	*/
	CamCfgParaSpec_t cam_info = ecfg->get_cam_info(port_id);
	int width = cam_info.width;
	int height = cam_info.height;
	int bayer = cam_info.bayer;
	int cpu_temp = 0;

	// Run at half the standard frame rate
	float fps = cam_info.fps / 2;

	// Run at standard frame rate
	fps = -1;

	// Less USB transmission bandwidth, 2x reduction in width and height ratio, YUV420P format
	int compress_scale = 2;
	/*
		set_yuv_frame_preview() used to set YUV format for drawing
		Fps is the frame rate of the output stream. When entering -1 in fps, it indicates the full frame rate display. 
		You can also enter 10/20 to specify 10 or 20 frames to output the image.
		Compress_scale is the compression ratio of image width and height, 
		which reduces USB load by reducing image size.
	*/
	int ret = ecfg->set_yuv_frame_preview(port_id, fps, compress_scale);
	if (ret != 0)// Setting failed, return
	{
		printf("set_compress_resolution_preview fail,device_index %d port_id %d ret:%d\n", device_index, port_id, ret);
		return;
	}
	/*
		The bit depth of imx735 is 12, so the image size is width multiplied by height multiplied by 2,
		When transmitting each image data, a data header will be attached in front of it, which contains information about the image. 
		Therefore, the size of the image should be added to the size of the header.
		The received buffer size can be larger than the size of the image.
	*/
	int image_size = width * height * 2 + sizeof(frameSpecOut);
	unsigned char *image_buffer = new unsigned char[image_size];

	int show_frame_num = MAX_RECV_FRAME;
	cv::startWindowThread();
	while (show_frame_num-- && is_running)
	{
		// Not the current device, exit the test early
		if (active_device_id != device_index)
			break;

		// Obtain one frame of image data
		int ret = ecfg->get_frame_data(port_id, image_buffer, image_size);
		if (ret < 0)
		{
			PTRACE_LOG(LOG_ERROR, "Device%d Module%2d Get Image Failed!", device_index, port_id);
			os_sleep(1);
			continue;
		}
		else
		{
			frameSpecOut *image_info = (frameSpecOut *)image_buffer;
			int image_width = image_info->res[0];
			int image_height = image_info->res[1];
			int seq_no = image_info->seqNo;


			// Outliers in width and height values, not handled.
			if (image_width != width / compress_scale || image_height != height / compress_scale)
			{
				PTRACE_LOG(LOG_ERROR, "compressed_resolution_display Dev:%2d,port:%d Abnormal Width:%d - Height:%d - Seqno:%d", device_index, port_id, image_width, image_height, seq_no);
				os_sleep(1);
				continue;
			}

			float scale = 0.5;

			/* Build a cv::mat using yuv image data*/
			
			cv::Mat img_yuv;
			img_yuv.create(image_height * 3 / 2, image_width, CV_8UC1);
			/* YUV420P-->RGB */
			img_yuv.data = (unsigned char *)image_buffer + sizeof(frameSpecOut);
			cv::Mat img_bgr;
			cv::cvtColor(img_yuv, img_bgr, CV_YUV2BGR_I420);
			cv::Mat img_show;
			cv::resize(img_bgr, img_show, cv::Size(img_bgr.cols * scale, img_bgr.rows * scale), 0, 0, cv::INTER_LINEAR);

			char info[128];

			sprintf(info, "compressed:dev_id:%d,port:%d", device_index, port_id);
			cv::putText(img_show, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

			sprintf(info, "Width:%d - Height:%d - Seqno:%d", image_width, image_height, seq_no);
			cv::putText(img_show, info, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

			vector<double> vec_fps;
			/* Get module frame rate*/
			int ret = ecfg->get_frame_rate(vec_fps);
			if (ret >= 0 && vec_fps.size() > 0)
			{
				double fps = vec_fps[port_id - 1];
				sprintf(info, "Fps:%.2f", fps);
				cv::putText(img_show, info, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_vol;
			/* Get module voltage*/
			ret = ecfg->get_poc_voltage(vec_vol);
			if (ret >= 0 && vec_vol.size() > 0)
			{
				int voltage = vec_vol[port_id - 1];
				sprintf(info, "Voltage:%d", voltage);
				cv::putText(img_show, info, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_cur;
			/* Get module voltage*/
			ret = ecfg->get_poc_current(vec_cur);
			if (ret >= 0 && vec_cur.size() > 0)
			{
				int current = vec_cur[port_id - 1];
				sprintf(info, "Current:%d", current);
				cv::putText(img_show, info, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_dropped_frames;
			/* Get the number of lost frames in the module*/
			ret = ecfg->get_dropped_frames_cnt(vec_dropped_frames);
			if (ret >= 0 && vec_dropped_frames.size() > 0)
			{
				int dropped_frames_num = vec_dropped_frames[port_id - 1];
				sprintf(info, "Dropped Frames:%d", dropped_frames_num);
				cv::putText(img_show, info, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_error_frames;
			/* Get the number of module error frames*/
			ret = ecfg->get_error_frames_cnt(vec_error_frames);
			if (ret >= 0 && vec_error_frames.size() > 0)
			{
				int error_frames_num = vec_error_frames[port_id - 1];
				sprintf(info, "Error Frames:%d", error_frames_num);
				cv::putText(img_show, info, cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_temp;
			/* Get module temperature*/
			ret = ecfg->get_module_temperature(vec_temp);
			if (ret >= 0 && vec_temp.size() > 0)
			{
				int temp = vec_temp[port_id - 1];
				sprintf(info, "Module Temperature:%d", temp);
				cv::putText(img_show, info, cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_led_cur;
			/* Get LED current*/
			ret = ecfg->get_led_current(vec_led_cur);
			if (ret >= 0 && vec_led_cur.size() > 0)
			{
				int led_cur = vec_led_cur[port_id - 1];
				sprintf(info, "LED Current:%d", led_cur);
				cv::putText(img_show, info, cv::Point(10, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_led_vol;
			/* Get LED voltage*/
			ret = ecfg->get_led_voltage(vec_led_vol);
			if (ret >= 0 && vec_led_vol.size() > 0)
			{
				int led_vol = vec_led_vol[port_id - 1];
				sprintf(info, "LED Voltage:%d", led_vol);
				cv::putText(img_show, info, cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			if (seq_no % 30 == 0)
			{
				/* Get CPU temperature*/
				cpu_temp = port_id;
				vector<int> vec_cpu_temp;
				ret = ecfg->get_cpu_temp(vec_cpu_temp);
				if (ret >= 0 && vec_cpu_temp.size() > 0)
				{
					cpu_temp = vec_cpu_temp[port_id / 4];
				}
			}
			sprintf(info, "Cpu Temperature:%d", cpu_temp);
			cv::putText(img_show, info, cv::Point(10, 230), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

			char window_name[128];
			sprintf(window_name, "compress_resolution_device%d_port%d", device_index, port_id);
			cv::imshow(window_name, img_show);
			cv::waitKey(1);
		}
	}
	delete[] image_buffer;
	cv::destroyAllWindows();

	// Turn off port output
	ret = ecfg->stop_output_mode(port_id);
	int retryCnt = 100;
	while (ret != 0 && retryCnt > 0)
	{
		os_sleep(10);
		retryCnt--;
		printf("stop_output_mode_devid_%d,port_id_%d, ret:%d\n", device_index, port_id, ret);
		ret = ecfg->stop_output_mode(port_id);
	}
}

/*
	Display the read stream information and obtain Raw data
*/
void full_resolution_display(ECFG *ecfg, int device_index, int port_id)
{
	/* 
	Obtain module information on the specified port, 
	such as the original image width, original height, bit depth, etc
	*/
	CamCfgParaSpec_t cam_info = ecfg->get_cam_info(port_id);
	int width = cam_info.width;
	int height = cam_info.height;
	int bayer = cam_info.bayer;
	int cpu_temp = 0;

	// Run at half the standard frame rate
	float fps = cam_info.fps / 2;

	// Run at standard frame rate
	fps = -1;

	/*
		set_yuv_frame_preview() used to set RAW format for drawing
		Fps is the frame rate of the output stream. When entering -1 in fps, it indicates the full frame rate display. 
		You can also enter 10/20 to specify 10 or 20 frames to output the image.
		Compress_scale is the compression ratio of image width and height, 
		which reduces USB load by reducing image size.
	*/
	int ret = ecfg->set_full_resolution_preview(port_id, fps);
	if (ret != 0)// Setting failed, return
	{
		printf("set_full_resolution_preview fail,device_index %d port_id %d ret:%d\n", device_index, port_id, ret);
		return;
	}

	/*
		The bit depth of imx735 is 12, so the image size is width multiplied by height multiplied by 2,
		When transmitting each image data, a data header will be attached in front of it, which contains information about the image. 
		Therefore, the size of the image should be added to the size of the header.
		The received buffer size can be larger than the size of the image.
	*/
	int image_size = width * height * 2 + sizeof(frameSpecOut);
	unsigned char *image_buffer = new unsigned char[image_size];

	int show_frame_num = MAX_RECV_FRAME;
	cv::startWindowThread();
	while (show_frame_num-- && is_running)
	{
		// Not the current device, exit the test early
		if (active_device_id != device_index)
			break;

		// Obtain one frame of image data
		int ret = ecfg->get_frame_data(port_id, (unsigned char *)image_buffer, image_size);
		if (ret < 0)
		{
			PTRACE_LOG(LOG_ERROR, "Device%d module%2d get image data fail!", device_index, port_id);
			os_sleep(1);
			continue;
		}
		else
		{
			frameSpecOut *image_info = (frameSpecOut *)image_buffer;
			int image_width = image_info->res[0];
			int image_height = image_info->res[1];
			int seq_no = image_info->seqNo;
			float scale = 1.0 * SHOW_IMAGE_W / image_width;

			/* Build a cv::mat using yuv image data*/
			cv::Mat img_raw12(image_height, image_width, CV_16SC1, image_buffer + sizeof(frameSpecOut));
			cv::Mat img_raw8;
			cv::convertScaleAbs(img_raw12, img_raw8, 0.1);
			cv::Mat img_rgb;

			// Retrieve 'xx_xx_init.jsn' from JSON file in Bayer format'
			if (bayer == 0) // Image Format : GRBG
			{
				cv::cvtColor(img_raw8, img_rgb, cv::COLOR_BayerGR2BGR);
			}
			else if (bayer == 1) // Image Format : RGGB
			{
				cv::cvtColor(img_raw8, img_rgb, cv::COLOR_BayerRG2BGR);
			}
			else if (bayer == 2) // Image Format : GBRG
			{
				cv::cvtColor(img_raw8, img_rgb, cv::COLOR_BayerGB2BGR);
			}
			else // Image Format : BGGR
			{
				cv::cvtColor(img_raw8, img_rgb, cv::COLOR_BayerBG2BGR);
			}

			cv::Mat img_show;
			cv::resize(img_rgb, img_show, cv::Size(img_rgb.cols * scale, img_rgb.rows * scale), 0, 0, cv::INTER_LINEAR);

			char info[128];

			sprintf(info, "full:dev_id:%d,port:%d", device_index, port_id);
			cv::putText(img_show, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

			sprintf(info, "Width:%d - Height:%d - Seqno:%d", image_width, image_height, seq_no);
			cv::putText(img_show, info, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

			vector<double> vec_fps;
			/* Get module frame rate*/
			int ret = ecfg->get_frame_rate(vec_fps);
			if (ret >= 0 && vec_fps.size() > 0)
			{
				double fps = vec_fps[port_id - 1];
				sprintf(info, "Fps:%.2f", fps);
				cv::putText(img_show, info, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_vol;
			/* Get module voltage*/
			ret = ecfg->get_poc_voltage(vec_vol);
			if (ret >= 0 && vec_vol.size() > 0)
			{
				int voltage = vec_vol[port_id - 1];
				sprintf(info, "Voltage:%d", voltage);
				cv::putText(img_show, info, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_cur;
			/* Get module voltage*/
			ret = ecfg->get_poc_current(vec_cur);
			if (ret >= 0 && vec_cur.size() > 0)
			{
				int current = vec_cur[port_id - 1];
				sprintf(info, "Current:%d", current);
				cv::putText(img_show, info, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_dropped_frames;
			/* Get the number of lost frames in the module*/
			ret = ecfg->get_dropped_frames_cnt(vec_dropped_frames);
			if (ret >= 0 && vec_dropped_frames.size() > 0)
			{
				int dropped_frames_num = vec_dropped_frames[port_id - 1];
				sprintf(info, "Dropped Frames:%d", dropped_frames_num);
				cv::putText(img_show, info, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_error_frames;
			/* Get the number of module error frames*/
			ret = ecfg->get_error_frames_cnt(vec_error_frames);
			if (ret >= 0 && vec_error_frames.size() > 0)
			{
				int error_frames_num = vec_error_frames[port_id - 1];
				sprintf(info, "Error Frames:%d", error_frames_num);
				cv::putText(img_show, info, cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_temp;
			/* Get module temperature*/
			ret = ecfg->get_module_temperature(vec_temp);
			if (ret >= 0 && vec_temp.size() > 0)
			{
				int temp = vec_temp[port_id - 1];
				sprintf(info, "Module Temperature:%d", temp);
				cv::putText(img_show, info, cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_led_cur;
			/* Get LED current*/
			ret = ecfg->get_led_current(vec_led_cur);
			if (ret >= 0 && vec_led_cur.size() > 0)
			{
				int led_cur = vec_led_cur[port_id - 1];
				sprintf(info, "LED Current:%d", led_cur);
				cv::putText(img_show, info, cv::Point(10, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			vector<int> vec_led_vol;
			/* Get LED voltage*/
			ret = ecfg->get_led_voltage(vec_led_vol);
			if (ret >= 0 && vec_led_vol.size() > 0)
			{
				int led_vol = vec_led_vol[port_id - 1];
				sprintf(info, "LED Voltage:%d", led_vol);
				cv::putText(img_show, info, cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			}

			if (seq_no % 30 == 0)
			{
				/* Get CPU temperature*/
				cpu_temp = port_id;
				vector<int> vec_cpu_temp;
				ret = ecfg->get_cpu_temp(vec_cpu_temp);
				if (ret >= 0 && vec_cpu_temp.size() > 0)
				{
					cpu_temp = vec_cpu_temp[port_id / 4];
				}
			}
			sprintf(info, "Cpu Temperature:%d", cpu_temp);
			cv::putText(img_show, info, cv::Point(10, 230), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

			char window_name[128];
			sprintf(window_name, "full_resolution_device%d port%d", device_index, port_id);
			cv::imshow(window_name, img_show);
			cv::waitKey(1);
		}
	}
	delete[] image_buffer;
	cv::destroyAllWindows();

	// Turn off port output
	ret = ecfg->stop_output_mode(port_id);
	int retryCnt = 100;
	while (ret != 0 && retryCnt > 0)
	{
		os_sleep(10);
		retryCnt--;
		printf("stop_output_mode_devid_%d,port_id_%d, ret:%d\n", device_index, port_id, ret);

		ret = ecfg->stop_output_mode(port_id);
	}
}

int ecfg_run_thread(ECFG *ecfg, int device_index)
{
	/* Initialize devices, multiple devices can be initialized simultaneously or sequentially*/
	int ret = initialize_device_and_module(ecfg, device_index);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Initialize device%d fail!", device_index);
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Initialize device%d success!", device_index);
	}

	/* 
	8.Query the initialization status of the device and obtain the node names of each device in the current device
	*/
	ret = query_device_init_status(ecfg, device_index);

	/* 
	9.Query module loading status
	*/
	ret = query_module_init_status(ecfg, device_index);

	while (is_running)
	{
		// Multiple device testing to determine if it is the current active device
		if (active_device_id != device_index)
		{
			os_sleep(10);
			continue;
		}

		os_sleep(500);
		printf("***********start test device_index:%d\n", device_index);

		/////////////////////// Full resolution RAW preview display start///////////////////////////////
		for (int port_id = 0; port_id < MAX_MULTI_PORT_NUM; port_id++)
		{
			if (active_camera_mark[device_index][port_id] != 0)
			{
				full_resolution_display(ecfg, device_index, port_id + 1);
			}
		}
		///////////////////////Full resolution RAW preview display end///////////////////////////////

		/////////////////////// Compress Resolution YUV Preview Display start///////////////////////////////
		// for (int port_id = 0; port_id < MAX_MULTI_PORT_NUM; port_id++)
		// {
		// 	if (active_camera_mark[device_index][port_id] != 0)
		// 	{
		// 		compressed_resolution_display(ecfg, device_index, port_id + 1);
		// 	}
		// }
		///////////////////////Compress Resolution YUV Preview Display end///////////////////////////////

		printf("***********stop test device_index:%d\n", device_index);
		// Multiple device testing, switching to the next device
		active_device_id++;
		active_device_id = active_device_id % max_dev_id;
	}

	/* Turn off devices and modules, multiple devices can be turned off simultaneously or sequentially*/
	ret = close_device_and_module(ecfg, device_index);
	return ret;
}

void timer(int times)
{
	// Timed 30 minute device restart
	os_sleep(times * 60 * 1000); // 30 mins
	is_running = false;
}

int main()
{
	while (1)
	{
		/* 1.Set the device model to S16_96792*/
		DEVICE_MODEL device_model = S16_96792;

		/* 2.Get device*/
		vector<ECFG *> vec_ecfg;
		vec_ecfg = EcfgFactory::creatEcfg(device_model);
		if (vec_ecfg.size() == 0)
		{
			/*PTRACE.LOG is the log printing function in ecfg_sdk, which can be output to a log file*/
			PTRACE_LOG(LOG_ERROR, "No device found!");
			return 0;
		}

		int device_num = vec_ecfg.size();
		max_dev_id = device_num;
		PTRACE_LOG(LOG_INFO, "device_num %d", device_num);

		std::thread *ecfg_thread[MAX_DEV_NUM];
		is_running = true;

		// Multiple S16 devices are processed in parallel, with each S16 creating a test thread
		for (int device_index = 0; device_index < device_num; device_index++)
		{
			ecfg_thread[device_index] = new std::thread([=]()
								    { ecfg_run_thread(vec_ecfg[device_index], device_index); });
		}

		// Set a running time of 30 minutes per round
		int times = 30;
		// Set a timer, power off the device and restart it after the time has passed
		std::thread *timer_thread = new std::thread([=]()
							    { timer(times); });

		for (int device_index = 0; device_index < device_num; device_index++)
		{
			ecfg_thread[device_index]->join();
		}

		// After the device restarts, it takes time for the PC to enumerate the devices again.
		// Increase necessary delay to ensure that the device can be turned on normally during the next test.
		os_sleep(15 * 1000);
	}

	return 0;
}
