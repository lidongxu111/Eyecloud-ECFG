/**
 * @file   ecfg_sdk.h
 * @author Xiang
 * @email  xiangzhi@eyecloud.tech
 * @date
 * @brief  The Ecfg SDK is a software development package designed specifically
 * for running FrameGrabber series hardware.
 */

#ifndef _ECFG_SDK_H_
#define _ECFG_SDK_H_

#include "includes.h"

#define SDK_VERSION "1.3.24"

#define MAX_MULTI_PORT_NUM 16
#define MAX_MULTI_GROUP_NUM 8

#define MAX_DEV_NUM 32
#define MAX_SINGLE_PORT_NUM 4
#define MAX_SINGLE_GROUP_NUM 2

#define S1_DEV_PORT_1 1

#define S4_DEV_PORT_1 1
#define S4_DEV_PORT_2 2
#define S4_DEV_PORT_3 3
#define S4_DEV_PORT_4 4

#define S16_DEV_PORT_1 1
#define S16_DEV_PORT_2 2
#define S16_DEV_PORT_3 3
#define S16_DEV_PORT_4 4
#define S16_DEV_PORT_5 5
#define S16_DEV_PORT_6 6
#define S16_DEV_PORT_7 7
#define S16_DEV_PORT_8 8
#define S16_DEV_PORT_9 9
#define S16_DEV_PORT_10 10
#define S16_DEV_PORT_11 11
#define S16_DEV_PORT_12 12
#define S16_DEV_PORT_13 13
#define S16_DEV_PORT_14 14
#define S16_DEV_PORT_15 15
#define S16_DEV_PORT_16 16

typedef unsigned char UINT8;

using namespace std;

/**
 * @enum LOG_LEVEL
 * @brief the log level of the sdk
 */
enum LOG_LEVEL
{
        LOG_DEBUG,
        LOG_INFO,
        LOG_SHOW,
        LOG_WARNING,
        LOG_ERROR,
};

#ifdef _WINDOWS
#define PTRACE_LOG(level, fmt, ...) lib_message_log(level, fmt, __VA_ARGS__)
#else
#define PTRACE_LOG(level, fmt, args...) lib_message_log(level, fmt, ##args)
#endif

/**
 * @brief Function to record log information in local and print on the screen.
 */
void lib_message_log(int level, const char *fmt, ...);

/**
 * @brief Level of LOG system
 * 
 */
static int log_level = LOG_INFO;

/**
 * @brief Set the log level to be showed on the screen and saved locally.
 *
 * @param level LOG_LEVEL
 */
void set_log_level(int level);

#ifdef _WINDOWS

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

        /**
         * @brief Sleep function.
         *
         * @param ms millisecond
         */
        static void os_sleep(int ms);

        static void os_sleep(int ms)
        {
                Sleep(ms);
        }

#ifdef __cplusplus
}
#endif // __cplusplus

#else

void os_sleep(int ms);

#endif

/**
 * @enum usb_error
 * @brief The list of the usb error code.
 */
enum usb_error
{
        /** < Success (no error) */
        USB_SUCCESS = 0,

        /** < Input/output error */
        USB_ERROR_IO = -1,

        /** < Invalid parameter */
        USB_ERROR_INVALID_PARAM = -2,

        /**< Access denied (insufficient permissions) */
        USB_ERROR_ACCESS = -3,

        /**< No such device (it may have been disconnected) */
        USB_ERROR_NO_DEVICE = -4,

        /**< Entity not found */
        USB_ERROR_NOT_FOUND = -5,

        /**< Resource busy */
        USB_ERROR_BUSY = -6,

        /**< Operation timed out */
        USB_ERROR_TIMEOUT = -7,

        /**< Overflow */
        USB_ERROR_OVERFLOW = -8,

        /**< Pipe error */
        USB_ERROR_PIPE = -9,

        /**< System call interrupted (perhaps due to signal) */
        USB_ERROR_INTERRUPTED = -10,

        /**< Insufficient memory */
        USB_ERROR_NO_MEM = -11,

        /**< Operation not supported or unimplemented on this platform */
        USB_ERROR_NOT_SUPPORTED = -12,

        /* NB: Remember to update LIBUSB_ERROR_COUNT below as well as the
           message strings in strerror.c when adding new error codes here. */

        /**< Other error */
        USB_ERROR_OTHER = -99,

        /**< This function is not yet perfected*/
        NOT_SUPPORT = 0xffff,
};

/**
 * @enum I2CM_StatusType
 * @brief The list of the I2c communication abnormality.
 *
 */
typedef enum
{
        I2CM_STAT_OK = 0,
        I2CM_STAT_ERR = -1,
        I2CM_STAT_TIMEOUT = -2,
        I2CM_STAT_INVALID_MODULE = -3,
        I2CM_STAT_INVALID_PIN = -4,
        I2CM_STAT_INVALID_SPEED = -5,
        I2CM_STAT_SDA_STUCK_LOW = -6,
        I2CM_STAT_SCL_STUCK_LOW = -7,
        I2CM_STAT_TX_ABORT = -8,
        I2CM_STAT_FIFO_ERROR = -9,
        I2CM_STAT_ABRT_7B_ADDR_NOACK = -10,
        I2CM_STAT_ABRT_10ADDR1_NOACK = -11,
        I2CM_STAT_ABRT_10ADDR2_NOACK = -12,
        I2CM_STAT_ABRT_TXDATA_NOACK = -13,
        I2CM_STAT_ABRT_GCALL_NOACK = -14,
        I2CM_STAT_ABRT_GCALL_READ = -15,
        I2CM_STAT_ABRT_HS_ACKDET = -16,
        I2CM_STAT_ABRT_SBYTE_ACKDET = -17,
        I2CM_STAT_ABRT_HS_NORSTRT = -18,
        I2CM_STAT_ABRT_SBYTE_NORSTRT = -19,
        I2CM_STAT_ABRT_10B_RD_NORSTRT = -20,
        I2CM_STAT_ABRT_MASTER_DIS = -21,
        I2CM_STAT_ABRT_LOST = -22,
        I2CM_STAT_ABRT_SLV_FLUSH_TXFIFO = -23,
        I2CM_STAT_ABRT_SLV_ARBLOST = -24,
        I2CM_STAT_ABRT_SLVDR_INTX = -25,
        I2CM_STAT_ABRT_ERROR = -26, // TXABORT, but source didn't tell us why
        I2CM_STAT_NOT_INIT = -27,
        I2CM_STAT_VERIFY_FAIL = -28,
        I2CM_STAT_WRITE_UNDERFLOW = -29,
        I2CM_STAT_READ_UNDERFLOW = -30
} I2CM_StatusType;

/**
 * @enum ERROR_TYPE
 * @brief Specific steps' errors in ecfg_sdk functions.
 *
 */
typedef enum
{
        USB_ERROR = 1,
        I2C_ERROR = 2,
        PORT_START_ERROR = 3,
        SET_OUT_PUT_ERROR = 4,
        FG_CMD_GET_ERROR = 5,
        POC_POWER_ON_ERROR = 6,
        POC_POWER_OFF_ERROR = 7,
        GET_FRAME_DATE_ERROR = 8,
        PORT_CLOSE_ERROR = 9,
        GET_POC_VOLTAGE_ERROR = 10,
        GET_POC_CURRENT_ERROR = 11,
        DEVICE_INDEX_ERROR = 12,
        DEV_OPEN_ERROR = 13,
        DEV_CLOSE_ERROR = 14,
        GET_I2C_DEV_LISTS_ERROR = 15,
        GET_MONITOR_ERROR = 16,
        SET_I2C_ERROR = 17,
        SET_POC_VAL_ERROR = 18,
        SET_FAN_SPEED_ERROR = 19,
        EX_5V_POWER_ON_ERROR = 20,
        EX_5V_POWER_OFF_ERROR = 21,
        LED_ON_ERROR = 22,
        LED_OFF_ERROR = 23,
        JSON_PARSER_ERROR = 24,
        SET_EEROM_ERROR = 25,
        GET_EEPROM_ERROR = 26,
} ERROR_TYPE;

/**
 * @enum DEVICE_MODEL
 * @brief The list of EyeCloud FrameGrabber's existing product series.
 *
 */
typedef enum
{
        S16_9296,
        S4_9296,
        S1_9296,
        S4_9702,
        S1_9702,
        S16_96792,
        S4_96792,
        S1_96792,
        R1_9296,
        R1_96792,
        UNKOWN,
} DEVICE_MODEL;

/**
 * @enum API_ERROR
 * @brief Number of function with error.
 *
 */
typedef enum
{
        DEVICE = 0,
        OPEN = 1,
        CLOSE = 2,
        INIT = 3,
        GET_I2C_DEV_LIST = 4,
        SET_POC_VOLTAGE = 5,
        GET_POC_VOLTAGE = 6,
        GET_POC_CURRENT = 7,
        POC_POWER_ON = 8,
        POC_POWER_OFF = 9,
        START = 10,
        STOP = 11,
        GET_FRAME_RATE = 12,
        GET_DROPPED_FRAMES_CNT = 13,
        WRITE_I2C = 14,
        READ_I2C = 15,
        LED_ON = 16,
        LED_OFF = 17,
        SET_FAN_SPEED = 18,
        GET_FRAME_DATE = 19,
        EX_5V_POWER_ON = 20,
        EX_5V_POWER_OFF = 21,
        GET_CAM_INFO = 23,
        GET_CAM_MONITOR = 24,
        SET_EEPROM = 25,
        GET_EEPROM = 26,
        GET_ERROR_FRAMES_CNT = 27,
        GET_CAM_STATUS = 28,
        GET_CPU_TEMP = 29,
        GET_FRAME_COUNT = 30
} API_ERROR;

/**
 * @enum I2C_PROTO
 * @brief The byte order of i2c read and write.
 *
 */
typedef enum
{
        PROTO_16_8,  //! < 16b Register with  8b data
        PROTO_16_16, //! < 16b Register with 16b data
        PROTO_8_8,   //! <  8b Register with  8b data
        PROTO_8_16,  //! <  8b Register with 16b data
        PROTO_CMD,
} I2C_PROTO;

typedef enum
{
        LH,
        HL,
} ENDIANESS;

typedef enum
{
        SINGLE,
        CONTINUOUS
} TRANS_MODE;

/**
 * @struct I2cDev_t
 * @brief The data content that needs to be transmitted in i2c communicate.
 *
 */
typedef struct
{
        char name[16];
        unsigned int addr;
        I2C_PROTO type;
        TRANS_MODE mode;
        ENDIANESS endianess;
} I2cDev_t;

/**
 * @struct I2cDevMap_t
 * @brief The data content that needs to be transmitted in i2c communicate.
 *
 */
typedef struct
{
        char name[16];
        int bus_id;
        vector<I2cDev_t> list;
        int dev_id;
} I2cDevMap_t;

/**
 * @enum LED_TYPE
 * @brief The color of led.
 */
typedef enum
{
        BLUE,
        RED
} LED_TYPE;

/**
 * @enum ECFG_STATUS
 * @brief the status code of ecfg device.
 *
 */
typedef enum
{
        ECFG_DEFAULT,
        ECFG_BUSY,
        ECFG_CONNECTED,
        ECFG_STREAM,
        ECFG_DISCONNECTED
} ECFG_STATUS;

/**
 * @struct ProdInfo_t
 * @brief  Device's information.
 */
typedef struct
{
        /*firmware version*/
        char fw_ver[32];
        /*sdk version*/
        char sdk_ver[32];
        /*device's name*/
        char name[16];
} ProdInfo_t;

/**
 * @struct CamCfgParaSpec_t
 * @brief Camera's information.
 */
typedef struct
{
        int width;
        int height;
        float fps;
        int bit_depth;
        int nlane;
        int lanespeed;
        int vc_num;
        int bayer;
        std::string format;
        std::string serial_number;
        std::string sdk_version;
        std::string firmware_version;
        std::string grp_sn;
} CamCfgParaSpec_t;

/**
 * @struct FrameCnt_t
 * @brief Module runtime information.
 *
 */
typedef struct
{
        // The cumulative number of frames generated during module runtime
        unsigned int cnt;
        // Timestamp during module runtime.
        unsigned int ts;
        // Number of frames lost during module runtime
        unsigned int lost_cnt;
        // The number of frames that generate errors during module runtime.
        unsigned int error_cnt;
        // Frame rate during module runtime
        float fps;
} FrameCnt_t;

/**
 * @struct CamHwMonitor
 * @brief Module runtime information.
 *
 */
typedef struct
{
        unsigned char mipi[MAX_SINGLE_PORT_NUM];
        unsigned char des[MAX_SINGLE_PORT_NUM];
        // module running status.
        unsigned char cam[MAX_SINGLE_PORT_NUM];
        // module voltage.
        unsigned short vol[MAX_SINGLE_PORT_NUM];
        // module current.
        unsigned short cur[MAX_SINGLE_PORT_NUM];
        // led voltage.
        unsigned short led_vol[MAX_SINGLE_PORT_NUM];
        // led current.
        unsigned short led_cur[MAX_SINGLE_PORT_NUM];
        //module temperature
        int temp[MAX_SINGLE_PORT_NUM];
        FrameCnt_t fcnt[MAX_SINGLE_PORT_NUM];
} CamHwMonitor;

/**
 * @brief EEPROM VERSION 1
 * 
 */
typedef struct
{
        int version; // 0x01010101
        /*
                The calibration value is a signed floating-point value, 
                rounded to 4 decimal places To reduce data size, 
                it is defined as an integer, coef * 10000, 
                and then rounded to the nearest whole number
        */
        float dac_coef_a;
        float dac_coef_b;
        float adc_coef_a;
} vol_calibration_factor_v_1_t;

/**
 * @brief EEPROM VERSION 2
 * 
 */
typedef struct
{
        int version; // 0x01010102
        /*
                The calibration value is a signed floating-point value, 
                rounded to 4 decimal places To reduce data size, 
                it is defined as an integer, coef * 10000, 
                and then rounded to the nearest whole number
        */
        float dac_coef_a;
        float dac_coef_b;
        float dac_coef_c;
        float dac_coef_d;
        float adc_coef_a[4]; // 1.0011 - 6.2454;
        float adc_coef_b[4];
} vol_calibration_factor_v_2_t;

/**
 * @brief EEPROM VERSION 3
 * 
 */
typedef struct
{
        int version; // 0x01010104
        /*
                The calibration value is a signed floating-point value, 
                rounded to 4 decimal places To reduce data size, 
                it is defined as an integer, coef * 10000, 
                and then rounded to the nearest whole number
        */
        int cam_vol_coef_a[10];
        int cam_vol_coef_b[10];
        int cam_cur_coef_a[10];
        int cam_cur_coef_b[10];
        int led_vol_coef_a[10];
        int led_vol_coef_b[10];
        int led_cur_coef_a[10];
        int led_cur_coef_b[10];
} vol_calibration_factor_v_3_t;

/**
 * @brief EEPROM VERSION 4
 * 
 */
typedef struct
{
        int version; // 0x01010104
        /*
                The calibration value is a signed floating-point value, 
                rounded to 4 decimal places To reduce data size, 
                it is defined as an integer, coef * 10000, 
                and then rounded to the nearest whole number
        */
        float dac_coef_a;
        float dac_coef_b;
        float dac_coef_c;
        float dac_coef_d;
        float sys_coef_a;
        float sys_coef_b;
        float cam_vol_coef_a[4];
        float cam_vol_coef_b[4];
        float cam_cur_coef_a[4];
        float cam_cur_coef_b[4];
        float led_vol_coef_a[4];
        float led_vol_coef_b[4];
        float led_cur_coef_a[4];
        float led_cur_coef_b[4];
} vol_calibration_factor_v_4_t;

/**
 * @brief Select the corresponding structure based on the eeprom version number.
 * 
 */
typedef union
{
        int version;
        vol_calibration_factor_v_1_t vol_coef_v_1;
        vol_calibration_factor_v_2_t vol_coef_v_2;
        vol_calibration_factor_v_3_t vol_coef_v_3;
        vol_calibration_factor_v_4_t vol_coef_v_4;
        char res[276]; // Reserve 276 bytes
} vol_calibration_u;

/**
 * @brief Burn and store content in the machine before leaving the factory.
 * 
 */
typedef struct
{
        char grp_sn[16];        // yymmxxxx . Unique code for each S16 series device.
        int id;                 // Board ID . From 0~3.
        float adc_comp;         // ADC correction value.
        char serial_number[32]; // Device's serial number.
        char device_model[32];  // Equipment model.
        char struct_version[16];// EEPROM struct version.
        char reserve[24];       // Reserve 24 bytes
        vol_calibration_u vol_calib;
} eepromProgram_t; // Version3.0

/**
 * @enum FRAMETYPE
 * @brief The data format of the obtained camera stream.
 *
 */
typedef enum
{
        RAW10 = 18,
        NONE //! < reserve
} FRAMETYPE;

/**
 * @struct frameSpecOut
 * @brief The data content of the first saved frame in each obtained frame of data.
 *
 */
typedef struct
{
        FRAMETYPE type;      //! < image format of this frame
        unsigned int seqNo;  //! < frame index number of this frame
        unsigned int size;   //! < frame bytes size of the frame,include header and frame data
        unsigned int cam_id; //! < the index of camera on device
        unsigned int vc_id;  //! < the index of camera on device
        // unsigned long long time_stamp;
        unsigned int res[11]; //! < reserve,not used
} frameSpecOut;

/**
 * @struct raw_frame
 * @brief Include the data of frame and frame's header.
 *
 */
typedef struct raw_frame
{
        frameSpecOut *raw_info;
        int size;
        unsigned char *raw_frame_data;
} raw_frame;

/**
 * @brief Assist firmware in identifying device models.
 * 
 */
typedef enum
{
        Sx_9296 = 1,
        Sx_9702,
        Sx_96792A,
        Rx_9296,
        Rx_96792A,
} PRODUCT_LIST;

/**
 * @brief The virtual class of ECFG.
 *
 */
class ECFG
{
public:
        virtual ~ECFG(){};

        /**
         * @brief  Open the connected device,and generate handles.
         *
         * @return int
         * @retval 0  Open success.
         * @retval <0 Open failed.
         * @par sample:
         * @code
         * int ret = ecfg->open();
         * @endcode
         *
         */
        virtual int open() = 0;

        /**
         * @brief  Close the connected device,and release handles.
         *
         * @return int
         * @retval 0  Close success.
         * @retval <0  Close failed.
         * @par sample:
         * @code
         * int ret = ecfg->close();
         * @endcode
         *
         */
        virtual int close() = 0;

        /**
         * @brief Use a json config file to initialize a device's deserialization chip.
         *
         * @param grp_id The index of the specific device's deserialization chip,each device has 2 deserialization chips.
         *               S16_XXXX : 1~8
         *               S4_XXXX  : 1~2
         *               S1_XXXX  : 1
         * @param json_file The json config file to init the device's deserialization chip.
         * @return int
         * @retval 0 Init deserialization chip success.
         * @retval <0 Init deserialization chip failed.
         * @par sample:
         * @code
         * int ret = ecfg->init(0,"./config/init_test.json");
         * @endcode
         *
         */
        virtual int init(int grp_id, std::string json_file) = 0;

        /**
         * @brief Get the device lists through i2c. And return the port name of each device.
         *        Use the name to operate each device's port's i2c.
         *
         * @param array Vector to save all devices' i2c name.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<string> dev_lists;
         * int ret = ecfg->get_i2c_dev_list(dev_lists);
         * for(int i=0;i<dev_lists.size();i++ )
         *      cout << dev_lists[i] << endl;
         * @endcode
         * printf result:
         *   /dev/G1-des
         *   /dev/port1-ser
         *   /dev/port1-sen
         *   /dev/port1-mcu
         *   /dev/port2-ser
         *   /dev/port2-sen
         *   /dev/port2-mcu
         *   /dev/G2-des
         *   /dev/port3-ser
         *   /dev/port3-sen
         *   /dev/port3-mcu
         *   /dev/port4-ser
         *   /dev/port4-sen
         *   /dev/port4-mcu
         */
        virtual int get_i2c_dev_list(std::vector<std::string> &array) = 0;

        /**
         * @brief Set the device's voltage to the specific value.
         *
         * @param val The voltage value to be setted.The unit is millivolts.
         * @return int
         * @retval <0 Set failed.
         * @retval 0 Set success.
         * @par sample:
         * @code
         * int val = 9000;// (mV)
         * int ret = ecfg->set_poc_voltage(val);
         * @endcode
         *
         */
        virtual int set_poc_voltage(int val) = 0;

        /**
         * @brief Get the device's all ports' voltage value.Save in a vector array.
         *
         * @param array Vector to save all devices' dropped frames count.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<int> voltage_array;
         * int ret = ecfg->get_poc_voltage(&voltage_array);
         * @endcode
         *
         */
        virtual int get_poc_voltage(std::vector<int> &array) = 0;

        /**
         * @brief Get the device's all ports' current value.Save in a vector array.
         *
         * @param array Vector to save all devices' dropped frames count.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<int> current_array;
         * int ret = ecfg->get_poc_current(current_array);
         * @endcode
         *
         */
        virtual int get_poc_current(std::vector<int> &array) = 0;

        /**
         * @brief  Turn on the power on the device's specific port.
         *
         * @param port_id The index of the port on the device.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @return int
         * @retval 0 Power on success.
         * @retval <0 Power on failed.
         * @par sample:
         * @code
         * int port_id = 1;
         * int ret = ecfg->poc_power_on(port_id);
         * @endcode
         *
         */
        virtual int poc_power_on(int port_id) = 0;

        /**
         * @brief Turn off the power on the device's specific port.
         *
         * @param port_id The index of the port on the device.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @return int
         * @retval 0 Power off success.
         * @retval <0 Power off failed.
         * @par sample:
         * @code
         * int port_id = 4;
         * int ret = ecfg->poc_power_off(port_id);
         * @endcode
         *
         */
        virtual int poc_power_off(int port_id) = 0;

        /**
         * @brief Start the module on the specific port by the specific json config file.
         *
         * @param port_id The index of the camera on device.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @param json_file The json config file path to initialize the module.
         * @return int
         * @retval 0 Start success.
         * @retval <0 Start failed.
         * @par sample:
         * @code
         * int port_id = 2;
         * int ret = ecfg->start(port_id, "./config/start_test.json");
         * @endcode
         *
         */
        virtual int start(int port_id, char *json_file) = 0;

        /**
         * @brief Stop the module on the specific port .
         *
         * @param port_id The index of the port on the device.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @return int
         * @retval 0 Stop success.
         * @retval <0 Stop failed.
         * @par sample:
         * @code
         * int port_id = 3;
         * int ret = ecfg->stop(port_id);
         * @endcode
         *
         */
        virtual int stop(int port_id) = 0;

        /**
         * @brief Get the device's all [ports' frame rate.
         *
         * @param array Vector to save all devices' dropped frames count.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<double> frame_rate_array;
         * int ret = ecfg->get_frame_rate(frame_rate_array);
         * @endcode
         *
         */
        virtual int get_frame_rate(vector<double> &array) = 0;

        /**
         * @brief Get the count of frames dropped on device's all ports.
         *
         * @param array Vector to save all devices' dropped frames count.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<int> dropped_frames_array;
         * int ret = ecfg->get_dropped_frames_cnt(dropped_frames_array);
         * @endcode
         *
         */
        virtual int get_dropped_frames_cnt(std::vector<int> &array) = 0;

        /**
         * @brief Get the count of error frames on device's all ports.
         *
         * @param array Vector to save all devices' error frames count.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<int> error_frames_array;
         * int ret = ecfg->get_error_frames_cnt(error_frames_array);
         * @endcode
         *
         */
        virtual int get_error_frames_cnt(std::vector<int> &array) = 0;

        /**
         * @brief Get the count of all modules' frames on device's all ports.
         *
         * @param array Vector to save all ports' frames count.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<int> frames_array;
         * int ret = ecfg->get_frame_count(frames_array);
         * @endcode
         *
         */
        virtual int get_frame_count(vector<int> &array) = 0;

        /**
         * @brief Get each port's module's status on the device.
         *
         * @param array Vector to save all ports' module's status.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<int> cam_status_array;
         * int ret = ecfg->get_cam_status(cam_status_array);
         * @endcode
         *
         */
        virtual int get_cam_status(std::vector<int> &array) = 0;

        /**
         * @brief Get each port's module's temperature on the device.
         *
         * @param array Vector to save all ports' module's temperature.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * vector<int> cam_temp_array;
         * int ret = ecfg->get_module_temperature(cam_temp_array);
         * @endcode
         *
         */
        virtual int get_module_temperature(std::vector<int> &array) = 0;

        /**
         * @brief Set the register value on the specific i2c dev.
         *
         * @param i2c_dev The i2c device's name.
         * @param reg Register address.
         * @param data The array to save the register value.
         * @param size The size of data.
         * @return int
         * @retval 0 Set success.
         * @retval <0 Set failed.
         * @par sample:
         * @code
         * int i2c_array[4] = {0x10,0x11,0x12,0x13};
         * int register = 0x1c;
         * //dev_lists get from get_i2c_dev_list().
         * int ret = ecfg->write_i2c(dev_lists[2], register, i2c_array, 4);
         * @endcode
         *
         */
        virtual int write_i2c(std::string i2c_dev, int reg, int *data, int size) = 0;

        /**
         * @brief Get the register value on the specific i2c dev.
         *
         * @param i2c_dev The i2c device's name.
         * @param reg Register address.
         * @param data The array to save the register value.
         * @param size The size of array to get.
         * @return int
         * @retval 0 Read success.
         * @retval <0 Read failed.
         * @par sample:
         * @code
         * int i2c_array[10];
         * int register = 0x1c;
         * //dev_lists get from get_i2c_dev_list().
         * int ret = ecfg->read_i2c(dev_lists[2], register, i2c_array, 10);
         * @endcode
         *
         */
        virtual int read_i2c(std::string i2c_dev, int reg, int *data, int size) = 0;

        /**
         * @brief Get the register value on the specific i2c dev.
         *
         * @param i2c_dev The i2c device's name.
         * @param val The array to save the register value.
         * @param size The size of array to get.
         * @return int
         * @retval 0 Read success.
         * @retval <0 Read failed.
         * @par sample:
         * @code
         * int register = 0x1c;
         * //dev_lists get from get_i2c_dev_list().
         * int ret = ecfg->read_i2c_cmd(dev_lists[0], i2c_array, 10);
         * @endcode
         *
         */
        virtual int read_i2c_cmd(std::string i2c_dev, int *val, int size) = 0;

        /**
         * @brief Set the register value on the specific i2c dev.
         *
         * @param i2c_dev The i2c device's name.
         * @param data The array to save the register value.
         * @param size The size of data.
         * @return int
         * @retval 0 Set success.
         * @retval <0 Set failed.
         * @par sample:
         * @code
         * int i2c_array[4] = {0x10,0x11,0x12,0x13};
         * int register = 0x1c;
         * //dev_lists get from get_i2c_dev_list().
         * int ret = ecfg->write_i2c(dev_lists[0], i2c_array, 4);
         * @endcode
         *
         */
        virtual int write_i2c_cmd(std::string i2c_dev, int *val, int size) = 0;

        /**
         * @brief Open the led of specific port.
         *
         * @param type The color of the led.
         * @param port_id The index of the port's led.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @return int
         * @retval 0 Open success.
         * @retval <0 Open failed.
         * @par sample:
         * @code
         * int port_id = 2;
         * LED_TYPE type = BLUE;
         * int ret = ecfg->led_on(type, port_id);
         * @endcode
         *
         */
        virtual int led_on(LED_TYPE type, int port_id) = 0;

        /**
         * @brief Close the led of specific port.
         *
         * @param type The color of the led.
         * @param port_id The index of the port's led.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @return int
         * @retval 0 Close success.
         * @retval <0 Close failed.
         * @par sample:
         * @code
         * int port_id = 2;
         * LED_TYPE type = RED;
         * int ret = ecfg->led_off(type, port_id);
         * @endcode
         *
         */
        virtual int led_off(LED_TYPE type, int port_id) = 0;

        /**
         * @brief Set the operating speed of the fan.
         *
         * @param speed Thr speed value.
         * @return int
         * @retval 0 Set success.
         * @retval <0 Set failed.
         * @par sample:
         * @code
         * int speed = 100;
         * int ret = ecfg->set_fan_speed(speed);
         * @endcode
         *
         */
        virtual int set_fan_speed(int speed) = 0;

        /**
         * @brief Obtain data for one frame of the module on the specified port.
         *
         * @param port_id The index of the port.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @param frame The array to save the frame data.
         * @param size The size of the array.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get failed.
         * @par sample:
         * @code
         * int port_id = 1;
         * int size = width * height *2;
         * unsignde char *data = new unsigned char[size];
         * int ret = ecfg->get_frame_data(port_id, data, size);
         * @endcode
         *
         */
        virtual int get_frame_data(int port_id, unsigned char *frame, int size) = 0;

        /**
         * @brief Set the output format of the specified port's module to full resolution output.
         *
         * @param port_id The index of the port's module.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @param fps Set the number of frames for streaming.
         *            From 1~30(Determined based on the specific frame rate of the module).
         *            Set to -1 if not restricted.
         * @return int
         * @retval 0 Set success.
         * @retval <0 Set failed.
         * @par sample:
         * @code
         * int port_id = 1;
         * float fps = -1;
         * int ret = ecfg->set_full_resolution_preview(port_id, fps);
         * @endcode
         */
        virtual int set_full_resolution_preview(int port_id, float fps) = 0;

        /**
         * @brief Set the output format of the specified port's module to compress resolution output.
         *
         * @param port_id The index of the port's module.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @param fps Set the number of frames for streaming.
         *            From 1~30(Determined based on the specific frame rate of the module).
         *            Set to -1 if not restricted.
         * @param scale The compression rate.
         * @return int
         * @retval 0 Set success.
         * @retval <0 Set failed.
         * @par sample:
         * @code
         * int port_id = 1;
         * float fps = -1;
         * int scale = 6;
         * int ret = ecfg->set_compress_resolution_preview(port_id, fps, scale);
         * @endcode
         */
        virtual int set_compress_resolution_preview(int port_id, float fps, int scale) = 0;

        /**
         * @brief Set the output format of the specified port's module to YUV format output.
         *
         * @param port_id The index of the port's module.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @param fps Set the number of frames for streaming.
         *            From 1~30(Determined based on the specific frame rate of the module).
         *            Set to -1 if not restricted.
         * @param scale The compression rate.
         * @return int
         * @retval 0 Set success.
         * @retval <0 Set failed.
         * @par sample:
         * @code
         * int port_id = 1;
         * float fps = -1;
         * int scale = 1;
         * int ret = ecfg->set_yuv_frame_preview(port_id, fps, scale);
         * @endcode
         */
        virtual int set_yuv_frame_preview(int port_id, float fps, int scale) = 0;

        /**
         * @brief Stop the specified port's module's output.
         *
         * @param port_id The index of the port's module.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @return int
         * @retval 0 Set success.
         * @retval <0 Set failed.
         * @par sample:
         * @code
         * int port_id = 1;
         * int ret = ecfg->stop_output_mode(port_id);
         * @endcode
         */
        virtual int stop_output_mode(int port_id) = 0;

        /**
         * @brief Open the 5V expansion port.
         * 
         * @param port_id The index of the port's module.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         *
         * @return int
         * @retval 0 Open success.
         * @retval <0 Open failed.
         * @par sample:
         * @code
         * int port_id = 1;
         * int ret = ecfg->ex_5v_power_on(port_id);
         * @endcode
         *
         */
        virtual int ex_5v_power_on(int port_id) = 0;

        /**
         * @brief Close the 5V expansion port.
         * 
         * @param port_id The index of the port's module.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         *
         * @return int
         * @retval 0 Close success.
         * @retval <0 Close failed.
         * @par sample:
         * @code
         * int port_id = 2;
         * int ret = ecfg->ex_5v_power_off(port_id);
         * @endcode
         *
         */
        virtual int ex_5v_power_off(int port_id) = 0;

        /**
         * @brief Get the camera's information.
         *
         * @param port_id The index of the port's module.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * 
         * @return CamCfgParaSpec_t
         * @par sample:
         * @code
         * int port_id = 1;
         * CamCfgParaSpec_t cam_info = ecfg->get_cam_info(port_id);
         * @endcode
         *
         */
        virtual CamCfgParaSpec_t get_cam_info(int port_id) = 0;

        /**
         * @brief Update the firmware when using ECFG devices offline.
         *           Only for ECFG R1_9296/R1_96792
         *
         * @param fw_path The new firmware path.
         * @return int
         * @retval 0 Update success
         * @retval <0 Update fail
         * @code
         * string fw_path = "ECFG.mvcmd".
         * int ret = update_firmware(fw_path).
         * @endcode
         *
         */
        virtual int update_firmware(string fw_path) = 0;

        /**
         * @brief Get the led's current value
         *
         * @param array Vector to save all ports' led's current value.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get fail.
         * @code
         * vector<int> vec_led_cur;
         * int ret = ecfg->get_led_current(vec_led_cur);
         * @endcode
         *
         */
        virtual int get_led_current(std::vector<int> &array) = 0;

        /**
         * @brief Get the led's voltage value
         *
         * @param array Vector to save all ports' led's voltage value.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get fail.
         * @code
         * vector<int> vec_vol_cur;
         * int ret = ecfg->get_led_voltage(vec_vol_cur);
         * @endcode
         *
         */
        virtual int get_led_voltage(std::vector<int> &array) = 0;

        /**
         * @brief Get the cpu's temperature.
         *        S1 : 1 CPU
         *        S4 : 1 CPU
         *        S16 : 4 CPU
         *
         * @param array Vector to save all nccs' cpu's temperature.
         *              Save in the corresponding position of the vector array according to the index of the device.
         * @return int
         * @retval 0 Get success.
         * @retval <0 Get fail.
         * @code
         * vector<int> vec_cpu_temp;
         * int ret = ecfg->get_cpu_temp(vec_cpu_temp);
         * @endcode
         *
         */
        virtual int get_cpu_temp(std::vector<int> &array) = 0;

        /**
         * @brief Update the configuration files loaded internally when using ECFG devices offline.
         *           Only for ECFG R1_9296/R1_96792
         *
         * @param init_config_path Json config file path, to initialize a device's deserialization chip.
         * @param start_config_path Start the module on the specific port by the specific json config file.
         * @return int
         * @retval <0 Update fail.
         * @retval >=0 Update success.
         * @code
         * init_config_path = "./config/ar0820_3840_2160_30_12bit_4L/ar0820_3840_2160_30_12bit_4L_init.json"
         * start_config_path = "./config/ar0820_3840_2160_30_12bit_4L/ar0820_3840_2160_30_12bit_4L_A.json"
         * int ret = ecfg->update_config(init_config_path,start_config_path);
         * @endcode
         *
         */
        virtual int update_config(string init_config_path, string start_config_path) = 0;

        /**
         * @brief Perform signal quality testing on the ports of ECFG series devices.
         *
         * @param port_id The index of the port on the device.
         *                S16_XXXX : 1~16
         *                S4_XXXX  : 1~4
         *                S1_XXXX  : 1
         * @param link_margin_algorithm_num Input number# 1~5
         *      |----------------------------------------|
         *	|    Part No.  | Forward LM | Reverse LM |
         *	|--------------|------------|------------|
         *	| MAX9295A/B   |---Alg #1---|---Alg #3---|
         *	| MAX96717/K/F |---Alg #2---|---Alg #3---|
         *	| MAX96793(G2) |---Alg #2---|---Alg #3---|
         *	| MAX96793(G3) |---Alg #4---|---Alg #5---|
         *	|--------------|------------|------------|
         *	| MAX9296A/B   |--- Follow Serializer ---|
         *	| MAX96792A(G2)|--- Follow Serializer ---|
         *	|----------------------------------------|
         * @param t_des The name of device i2c's Dserializer(Get from func 'get_i2c_dev_list()').
         * @param t_ser The name of device i2c's Serializer(Get from func 'get_i2c_dev_list()').
         * @param lm The result of link_margin_algorithm.
         * @return int
         * @retval <0 Perform fail.
         * @retval >=0 Perform success.
         * @code
         * int port_id = 4;
         * int link_margin_algorithm_num = 1;
         * string t_des  = device_list[0];
         * string t_ser  = device_list[2];
         * float lm = 0;
         * int ret = ecfg->ecfg->link_margin(port_id,link_margin_algorithm_num,t_des,t_ser,lm);
         * @endcode
         *
         */
        virtual int link_margin(int port_id, int link_margin_algorithm_num, string t_des, string t_ser, float &lm) = 0;
};

/**
 * @brief Obtain the instantiated ecfg class.
 *
 */
class EcfgFactory
{
public:
        EcfgFactory();

        ~EcfgFactory();

        /**
         * @brief Download the firmware to the devices.
         *        Then boot the devices and get the num of devices.
         *        Return the instantiated ecfg class.
         *
         * @param device_model The device's model.
         * @return vector
         * @retval Vector of ecfg class.
         * @par sample:
         * @code
         * DEVICE_MODEL device_model = S16_9296;
         * vector<ECFG *> devices = EcfgFactory::creatEcfg(device_model);
         * ECFG *ecfg = devices[0];
         * @endcode
         */

        static vector<ECFG *> creatEcfg(DEVICE_MODEL device_model);
};

#endif /* _ECFG_SDK_H_ */
