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

// imx735,BGGR 3000*5760_raw12@21.34fps
 #define DES_INIT_FILE_PATH "./config/max96793_3000_5760_20_12bit_4L/des_max96793_3000_5760_20_12bit_4L_init.json"
 #define SER_SENSOR_INIT_FILE_PATH "./config/max96793_3000_5760_20_12bit_4L/ser_sensor_max96793_3000_5760_20_12bit_4L_A.json"

// sc8238,BGGR 3840*2160_raw10@30fps
//#define DES_INIT_FILE_PATH "./config/sc8238_3840_2160_30_10bit_4L/des_sc8238_3840_2160_30_10bit_4L_init.json"
//#define SER_SENSOR_INIT_FILE_PATH "./config/sc8238_3840_2160_30_10bit_4L/ser_sensor_sc8238_3840_2160_30_10bit_4L_A.json"

#define DEMO_VERSION 2.0.5
#define MAX_RECV_FRAME (200)
using namespace std;

/*
	This is a demo written in C++.
	To show how to call ecfg_sdk's all interface and the oreder of the interface calls.
	This demo uses device 'S1-96792' and module 'imx735' as example,to get full resolution image display.
 */

/*
	Intialize the module configuration function.
	It's necessary to parse the Json configure file content and cache it to a buffer,then pass it to the device.
 */
int start_module(ECFG *ecfg, int port_id)
{
	//R2.0.1
	//R2.0.2
#if 0
	int size = 0;
	char *json_str = NULL;
	/*
		Port 'A' configure file is './config/xx_xx_xx/xx_xx_xx_A.json'.
		Please ensure the configure file name's format is correct.
	*/
	FILE *fp = fopen(SER_SENSOR_INIT_FILE_PATH, "r");

	if (fp == NULL)
	{
		PTRACE_LOG(LOG_ERROR, "Analysis Json File Failed!");
		return -1;
	}
	else
	{
		fseek(fp, 0, SEEK_END);
		size = ftell(fp);
		json_str = new char[size];
		memset(json_str, 0, size);
		rewind(fp);
		fread(json_str, 1, size, fp);
		fclose(fp);

		/*
			Function start() income port id and Json file's content.
		*/
		int ret = ecfg->start(port_id, json_str);
		if (ret < 0)
		{
			PTRACE_LOG(LOG_ERROR, "Configuration Module %d Failed!", port_id);
			return ret;
		}
		delete[] json_str;
	}
#else
	//VER >= R2.0.3
	/*
		Function start() income port id and Json file's content.
	*/
	int ret = ecfg->start(port_id, SER_SENSOR_INIT_FILE_PATH);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Configuration Module %d Failed!", port_id);
		return ret;
	}
#endif
	return 0;
}

int get_module_voltage()
{
	int voltage = 0;
	/*
		The module's voltage value to be setted is in file './config/xx_xx_xx/xx_xx_xx_init.json'.
		In Json field 'voltage'.
	 */

	std::ifstream file(DES_INIT_FILE_PATH);
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

int query_device_init_status(ECFG *ecfg, int device_index)
{
	std::vector<std::string> device_list;
	int ret = ecfg->get_i2c_dev_list(device_list);
	if (ret < 0 || device_list.size() <= 0)
	{
		PTRACE_LOG(LOG_ERROR, "Get Device%d Node List Failed!", device_index);
		return ret;
	}
	else
	{
		/* Printf Device Node List*/
		PTRACE_LOG(LOG_INFO, "Device%d Node List:", device_index);
		for (int i = 0; i < device_list.size(); i++)
		{
			PTRACE_LOG(LOG_INFO, "%s", device_list[i].c_str());
		}
	}
	return 0;
}

int query_module_init_status(ECFG *ecfg, int device_index)
{
	std::vector<int> camera_status_list;
	int ret = ecfg->get_cam_status(camera_status_list);
	if (ret < 0 || camera_status_list.size() <= 0)
	{
		PTRACE_LOG(LOG_ERROR, "Get Device%d Module Status List!", device_index);
		return ret;
	}
	else
	{
		if (camera_status_list.size() > 1)
		{
			/*
				Determine the initialization status of the module, with a return value of ECFG_CONNECTED or ECFG_STEAM indicates successful module initialization,
				while others do not fail
				*/
			if (camera_status_list[0] == ECFG_CONNECTED || camera_status_list[0] == ECFG_STREAM)
			{
				PTRACE_LOG(LOG_INFO, "Device:%d Module:%2d Configure Success", device_index, S1_DEV_PORT_1);
			}
			else
			{
				PTRACE_LOG(LOG_ERROR, "Device:%d Module:%2d Configure Failed", device_index, S1_DEV_PORT_1);
				return -1;
			}
		}
	}
	return 0;
}

/*
	The necessary operation to intialize and run a device and the modules on the device。
 */
int initialize_device_and_module(ECFG *ecfg, int device_index)
{
	/* 3.Open Device*/
	int ret = ecfg->open();
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Open Device%d Failed!", device_index);
		return ret;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Open Device%d Success!", device_index);
	}

	// Before set voltage,it's best to close port's power,prevent voltage surge.
	ret = ecfg->poc_power_off(S1_DEV_PORT_1);

	// Json Configure file include default voltage value.Analysis json file to get value.
	int voltage = get_module_voltage();
	if (voltage < 0)
	{
		printf("voltage = %d\n", voltage);
		return voltage;
	}

	/* 4.Set port and module's voltage*/
	ret = ecfg->set_poc_voltage(voltage);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Set Voltage %d Failed!,ret = %d", device_index, voltage,ret);
		return ret;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Device%d Set Voltage %d Success!", device_index, voltage);
	}

	// Necessary delay to ensure complete power outage during the previous run.
	os_sleep(100);
	/* 5.Power on the port with the port number as the parameter*/
	ret = ecfg->poc_power_on(S1_DEV_PORT_1);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Port%d Power On Failed!", device_index, S1_DEV_PORT_1);
		return ret;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Device%d Port%d Power On Success!", device_index, S1_DEV_PORT_1);
	}
	/* 6.Power on the Led with the port number as the parameter */
	ret = ecfg->ex_5v_power_on(S1_DEV_PORT_1);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Led%d Power On Failed!", device_index, S1_DEV_PORT_1);
		return ret;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Device%d Led%d Power On Success!", device_index, S1_DEV_PORT_1);
	}

	// Necessary delay, initialize after power on stable.
	os_sleep(100);

	/* 7.Configure the main control and deserialization chips of the box*/
	/* Every two ports share a deserialization chip, so they are assigned to the same group*/
	/* The deserialization chips on the same board need to be initialized sequentially,
	   and different development boards can be initialized simultaneously*/
	int grp_id = 1;

	/* Group Id and initialize json file path passed in the init() function */
	ret = ecfg->init(grp_id, DES_INIT_FILE_PATH);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Intialize Deserialization Chips %d Failed!", device_index, S1_DEV_PORT_1);
		return ret;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "Device%d Intialize Deserialization Chips %d Success!", device_index, S1_DEV_PORT_1);
	}

	/* 8.Configure Module*/
	/*
		Modules on the same board need to be initialized and called sequentially,
		while modules on different development boards can be initialized and called simultaneously
	*/
	ret = start_module(ecfg, S1_DEV_PORT_1);
	if (ret < 0)
		return ret;

	return 0;
}

int close_device_and_module(ECFG *ecfg, int device_index)
{
	/* 11.Send a stop streaming command to the module*/
	int ret = ecfg->stop_output_mode(S1_DEV_PORT_1);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Port%d Stop Streaming Failed!", device_index, S1_DEV_PORT_1);
		return ret;
	}

	/* 12.Stop module operation*/
	ret = ecfg->stop(S1_DEV_PORT_1);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Port%d Stop Failed!", device_index, S1_DEV_PORT_1);
		return ret;
	}

	/* 13.Power off the module port*/
	ret = ecfg->poc_power_off(S1_DEV_PORT_1);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Port%d Power Off Failed!!", device_index, S1_DEV_PORT_1);
		return ret;
	}

	/* 14.LED Power Off*/
	ret = ecfg->ex_5v_power_off(S1_DEV_PORT_1);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Led%d Power Off Failed!", device_index, S1_DEV_PORT_1);
		return ret;
	}

	/* 15.Close Device*/
	ret = ecfg->close();
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "Device%d Close Failed!", device_index);
		return ret;
	}
	return 0;
}

/*
	Display the read stream information.
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
	vector<int> cpu_temp;

	float fps = -1; // -1: full frame rate output
	int compress_scale = 4;  //The ISP output is 1/4 of the original resolution.
	//Sensor Resolution: 3000*5760
	//compress_scale = 1: 3000*5760@yuv420p
	//compress_scale = 2: 1500*2880@yuv420p
	//compress_scale = 4: 750*1440@yuv420p

	/*
		set_yuv_frame_preview() is used to set the compression resolution for outputting images,
		which is used to obtain smaller images and adapt to multi-channel image previews.
		Set the ports that need to be streamed out.
		Fps is the frame rate of the output stream.
		When entering -1 in fps, it indicates the full frame rate display.
		You can also enter 10/20 to specify 10 or 20 frames to output the image.
		'compress_scale' is the compressed image size,
		which is the original width and height of the image divided by compression_scale to obtain the compressed width and height.
		'compress_scale' is recommended to input 4-8,
	*/
	ecfg->set_yuv_frame_preview(port_id, fps, compress_scale);

	/*
		The bit depth of imx735 is 12, so the image size is width multiplied by height multiplied by 2.
		When transmitting each image data, a data header will be attached in front of it,
		which contains information about the image.
		Therefore, the size of the image should be added to the size of the header.
	*/
	int image_size = width * height * 2 + sizeof(frameSpecOut);
	unsigned char *image_buffer = new unsigned char[image_size];

	cv::startWindowThread();
	int show_frame_num = MAX_RECV_FRAME;
	while (show_frame_num--)
	{
		int ret = ecfg->get_frame_data(port_id, image_buffer, image_size);

		if (ret < 0)
		{
			PTRACE_LOG(LOG_ERROR, "Device%d Module%d Get Image Failed!", device_index, port_id);
		}
		else
		{
			frameSpecOut *image_info = (frameSpecOut *)image_buffer;
			int image_width = image_info->res[0];
			int image_height = image_info->res[1];
			int seq_no = image_info->seqNo;

			if (image_width != width / compress_scale || image_height != height / compress_scale)
			{
				PTRACE_LOG(LOG_ERROR, "Dev:%d Abnormal Width:%d - Height:%d - Seqno:%d", device_index, image_width, image_height, seq_no);
				os_sleep(1);
				continue;
			}

			float scale = 1.00;

			/* Build a cv::mat using yuv image data*/
			cv::Mat img_show;

			cv::Mat img_yuv;
			img_yuv.create(image_height * 3 / 2, image_width, CV_8UC1);
			/* YUV420P-->RGB */
			img_yuv.data = (unsigned char *)image_buffer + sizeof(frameSpecOut);
			cv::cvtColor(img_yuv, img_show, CV_YUV2BGR_I420);

			char info[128];
			sprintf(info, "Width:%d - Height:%d - Seqno:%d", image_width, image_height, seq_no);
			cv::putText(img_show, info, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

			vector<double> vec_fps;
			/* Get the current running frame rate of the module*/
			int ret = ecfg->get_frame_rate(vec_fps);
			if (ret >= 0 && vec_fps.size() > 0)
			{
				double fps = vec_fps[port_id - 1];
				sprintf(info, "Fps:%.2f", fps);
				cv::putText(img_show, info, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_vol;
			/* Obtain the current voltage value of the module during operation*/
			ret = ecfg->get_poc_voltage(vec_vol);
			if (ret >= 0 && vec_vol.size() > 0)
			{
				int voltage = vec_vol[port_id - 1];
				sprintf(info, "Voltage:%d", voltage);
				cv::putText(img_show, info, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_cur;
			/* Obtain the current value of the module during its current operation*/
			ret = ecfg->get_poc_current(vec_cur);
			if (ret >= 0 && vec_cur.size() > 0)
			{
				int current = vec_cur[port_id - 1];
				sprintf(info, "Current:%d", current);
				cv::putText(img_show, info, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_dropped_frames;
			/* Get the number of lost frames in the current runtime of the module*/
			ret = ecfg->get_dropped_frames_cnt(vec_dropped_frames);
			if (ret >= 0 && vec_dropped_frames.size() > 0)
			{
				int dropped_frames_num = vec_dropped_frames[port_id - 1];
				sprintf(info, "Dropped Frames:%d", dropped_frames_num);
				cv::putText(img_show, info, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_error_frames;
			/* Obtain the number of erroneous frames during the current runtime of the module*/
			ret = ecfg->get_error_frames_cnt(vec_error_frames);
			if (ret >= 0 && vec_error_frames.size() > 0)
			{
				int error_frames_num = vec_error_frames[port_id - 1];
				sprintf(info, "Error Frames:%d", error_frames_num);
				cv::putText(img_show, info, cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_temp;
			/* Obtain the module temperature at the current runtime of the module (. C)*/
			ret = ecfg->get_module_temperature(vec_temp);
			if (ret >= 0 && vec_temp.size() > 0)
			{
				int temp = vec_temp[port_id - 1];
				sprintf(info, "Module Temperature:%d", temp);
				cv::putText(img_show, info, cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_led_cur;
			/* Obtain the current value of LED during operation*/
			ret = ecfg->get_led_current(vec_led_cur);
			if (ret >= 0 && vec_led_cur.size() > 0)
			{
				int led_cur = vec_led_cur[port_id - 1];
				sprintf(info, "LED Current:%d", led_cur);
				cv::putText(img_show, info, cv::Point(10, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_led_vol;
			/* Obtain the voltage value of LED during operation*/
			ret = ecfg->get_led_voltage(vec_led_vol);
			if (ret >= 0 && vec_led_vol.size() > 0)
			{
				int led_vol = vec_led_vol[port_id - 1];
				sprintf(info, "LED Voltage:%d", led_vol);
				cv::putText(img_show, info, cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			if (seq_no % 30 == 0)
			{
				/* Obtain the temperature during CPU operation*/
				ret = ecfg->get_cpu_temp(cpu_temp);
			}

			if(cpu_temp.size()!=0)
			{
				sprintf(info, "Cpu Temperature:%d", cpu_temp[0]);
				cv::putText(img_show, info, cv::Point(10, 230), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			char window_name[128];
			sprintf(window_name, "device%d_port%d_ISP_YUV420P", device_index, port_id);
			cv::imshow(window_name, img_show);
			cv::waitKey(1);
		}
	}

	ecfg->stop_output_mode(port_id);

	delete[] image_buffer;

	cv::destroyAllWindows();
}

/*
	Display the read stream information.
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

	double alpha = 0.0;
	int bit_depth = cam_info.bit_depth;
	alpha = 1.0 / std::pow(2, bit_depth - 8);

	float fps = -1;
	vector<int> cpu_temp;

	/*
		set_full_resolution_preview() is used to set the full resolution output and obtain the original image output by the module.
		Fps is the frame rate of the output stream.
		When entering -1 in fps, it indicates the full frame rate display.
		You can also enter 10/20 to specify 10 or 20 frames to output the image.
	*/
	ecfg->set_full_resolution_preview(port_id, fps);

	/*
		The bit depth of imx735 is 12, so the image size is width multiplied by height multiplied by 2.
		When transmitting each image data, a data header will be attached in front of it, which contains information about the image.
		Therefore, the size of the image should be added to the size of the header.
	*/
	int image_size = width * height * 2 + sizeof(frameSpecOut);
	unsigned char *image_buffer = new unsigned char[image_size];

	cv::startWindowThread();
	int show_frame_num = MAX_RECV_FRAME;
	while (show_frame_num--)
	{
		int ret = ecfg->get_frame_data(port_id, image_buffer, image_size);

		if (ret < 0)
		{
			PTRACE_LOG(LOG_ERROR, "Device%d Module%d Get Image Failed!", device_index, port_id);
		}
		else
		{
			frameSpecOut *image_info = (frameSpecOut *)image_buffer;
			int image_width = image_info->res[0];
			int image_height = image_info->res[1];
			int seq_no = image_info->seqNo;

			if (image_width != width || image_height != height)
			{
				PTRACE_LOG(LOG_ERROR, "Dev:%d Abnormal Width:%d - Height:%d - Seqno:%d\n", device_index, image_width, image_height, seq_no);
				os_sleep(20);
				continue;
			}

			// w<960;h<800
			float scale = 1.0 * 960 / image_width;
			if (scale > 1.0 * 800 / image_height)
				scale = 1.0 * 800 / image_height;
			scale = 0.25;

			cv::Mat img_raw16(image_height, image_width, CV_16SC1, image_buffer + sizeof(frameSpecOut));
			cv::Mat img_raw8;
			cv::convertScaleAbs(img_raw16, img_raw8, alpha);
			cv::Mat img_rgb;

			// bayer analysis from file 'xx_xx_init.josn'
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
			sprintf(info, "Width:%d - Height:%d - Seqno:%d", image_width, image_height, seq_no);
			cv::putText(img_show, info, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

			vector<double> vec_fps;
			/* Get the current running frame rate of the module*/
			int ret = ecfg->get_frame_rate(vec_fps);
			if (ret >= 0 && vec_fps.size() > 0)
			{
				double fps = vec_fps[port_id - 1];
				sprintf(info, "Fps:%.2f", fps);
				cv::putText(img_show, info, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_vol;
			/* Obtain the current voltage value of the module during operation*/
			ret = ecfg->get_poc_voltage(vec_vol);
			if (ret >= 0 && vec_vol.size() > 0)
			{
				int voltage = vec_vol[port_id - 1];
				sprintf(info, "Voltage:%d", voltage);
				cv::putText(img_show, info, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_cur;
			/* Obtain the current value of the module during its current operation*/
			ret = ecfg->get_poc_current(vec_cur);
			if (ret >= 0 && vec_cur.size() > 0)
			{
				int current = vec_cur[port_id - 1];
				sprintf(info, "Current:%d", current);
				cv::putText(img_show, info, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_dropped_frames;
			/* Get the number of lost frames in the current runtime of the module*/
			ret = ecfg->get_dropped_frames_cnt(vec_dropped_frames);
			if (ret >= 0 && vec_dropped_frames.size() > 0)
			{
				int dropped_frames_num = vec_dropped_frames[port_id - 1];
				sprintf(info, "Dropped Frames:%d", dropped_frames_num);
				cv::putText(img_show, info, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_error_frames;
			/* Obtain the number of erroneous frames during the current runtime of the module*/
			ret = ecfg->get_error_frames_cnt(vec_error_frames);
			if (ret >= 0 && vec_error_frames.size() > 0)
			{
				int error_frames_num = vec_error_frames[port_id - 1];
				sprintf(info, "Error Frames:%d", error_frames_num);
				cv::putText(img_show, info, cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_temp;
			/* Obtain the module temperature at the current runtime of the module (. C)*/
			ret = ecfg->get_module_temperature(vec_temp);
			if (ret >= 0 && vec_temp.size() > 0)
			{
				int temp = vec_temp[port_id - 1];
				sprintf(info, "Module Temperature:%d", temp);
				cv::putText(img_show, info, cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_led_cur;
			/* Obtain the current value of LED during operation*/
			ret = ecfg->get_led_current(vec_led_cur);
			if (ret >= 0 && vec_led_cur.size() > 0)
			{
				int led_cur = vec_led_cur[port_id - 1];
				sprintf(info, "LED Current:%d", led_cur);
				cv::putText(img_show, info, cv::Point(10, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			vector<int> vec_led_vol;
			/* Obtain the voltage value of LED during operation*/
			ret = ecfg->get_led_voltage(vec_led_vol);
			if (ret >= 0 && vec_led_vol.size() > 0)
			{
				int led_vol = vec_led_vol[port_id - 1];
				sprintf(info, "LED Voltage:%d", led_vol);
				cv::putText(img_show, info, cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			if (seq_no % 30 == 0)
			{
				/* Obtain the temperature during CPU operation*/
				ret = ecfg->get_cpu_temp(cpu_temp);
			}

			if(cpu_temp.size()!=0)
			{
				sprintf(info, "Cpu Temperature:%d", cpu_temp[0]);
				cv::putText(img_show, info, cv::Point(10, 230), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
			}

			char window_name[128];
			sprintf(window_name, "device%d_port%d_NO_ISP_RAW", device_index, port_id);
			cv::imshow(window_name, img_show);
			cv::waitKey(1);
		}
	}

	ecfg->stop_output_mode(port_id);

	delete[] image_buffer;
	cv::destroyAllWindows();
}

int ecfg_run(int device_index, ECFG *ecfg)
{
	/* Initialize devices, multiple devices can be initialized simultaneously or sequentially*/
	int ret = initialize_device_and_module(ecfg, device_index);
	if (ret < 0)
	{
		PTRACE_LOG(LOG_ERROR, "@@@@@@@@@@@Initialize Device%d Failed!", device_index);
		goto out;
	}
	else
	{
		PTRACE_LOG(LOG_INFO, "@@@@@@@@@@@Initialize Device%d Success!", device_index);
	}

	/* 9.Query the initialization status of the device and obtain the node names of each device in the current device*/
	ret = query_device_init_status(ecfg, device_index);
	if (ret < 0)
	{
		goto out;
	}

	/* 10.Query module loading status*/
	ret = query_module_init_status(ecfg, device_index);
	if (ret < 0)
	{
		goto out;
	}

	while (1)
	{
		/* ISP mode, yuv420P */
		compressed_resolution_display(ecfg, device_index, S1_DEV_PORT_1);
		os_sleep(100);
		/* raw data mode: Full resolution image display*/
		full_resolution_display(ecfg, device_index, S1_DEV_PORT_1);
		os_sleep(100);
	}

out:
	/* Close devices and modules, multiple devices can be closed simultaneously or sequentially*/
	ret = close_device_and_module(ecfg, device_index);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

int main()
{
	/* 1.Set the device model*/
	DEVICE_MODEL device_model = S1_96792;

	/* 2.Acquisition device*/
	vector<ECFG *> vec_ecfg;
	vec_ecfg = EcfgFactory::creatEcfg(device_model);
	if (vec_ecfg.size() == 0)
	{
		/*PTRACE_ LOG is ecfg_sdk's log printing function ,it can output to log files*/
		PTRACE_LOG(LOG_ERROR, "No device found!");
		return 0;
	}

	std::thread *t_list[MAX_DEV_NUM];
	for (int device_index = 0; device_index < vec_ecfg.size(); device_index++)
	{
		t_list[device_index] = new std::thread([=]()
						       { ecfg_run(device_index, vec_ecfg[device_index]); });
	}

	for (int device_index = 0; device_index < vec_ecfg.size(); device_index++)
	{
		t_list[device_index]->join();
	}

	return 0;
}
