/**
 * This file is written by WANG Guangfu from China.
 * Email: thuwgf@gmail.com
 * Date: 2022-07-14.
 * License: Apache License 2.0.
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <array>
#include <memory>
#include <sys/types.h>
#include <errno.h>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

// 115200 for JY61 ,9600 for others
// 115200 meaning 100Hz, 9600 meaning 20Hz.
#define BAUD 115200
#define Gravity 9.801 //in China Mainland most area can use this value.
#define DEGREE_TO_RAD 0.0174532925199 //Pi/180
#define DEGREE_TO_RAD_HALF 0.00872664626 //Pi/360
#define IMU_DEVICE_LOCATION "/dev/ttyTHS1" //for jetson user leave here as it is.
#define MPU_6050_TOPIC "mpu6050" //change mpu6050 topic name as your wish.
#define MPU_NODE_NAME "imu_6050_node" //change as you wish.
#define MPU_QUEUE_SIZE 10

template<typename T>
using SharedRef = std::shared_ptr<T>;

///@note here is cpp perfect forwarding.
template<typename T, typename... Args>
constexpr SharedRef<T> createSharedRef(Args &&...args) {
    return std::make_shared<T>(std::forward<Args>(args)...);
}

using ImgPubType = rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr;
using TimerPtrType = rclcpp::TimerBase::SharedPtr;
using ImuDataPatch = std::array<float, 9>;
using MilliSeconds = std::chrono::duration<double, std::milli>;
using std::cos;
using std::sin;

class MPU6050HW {
public:
    MPU6050HW() {
        bzero(mBuff, 1024);
        mRunning = false;
    };

    ~MPU6050HW() {
        if (mRunning) {
            close();
        }
    };

    void open(int baud = BAUD) {
        if (mRunning){
            fprintf(stderr, "Uart has been open: Handler: %d\n",mMPUHandle);
            return;
        }
        mMPUHandle = uart_open(mDeviceLoc);
        if (mMPUHandle == -1) {
            fprintf(stderr, "Uart open error\n");
            exit(EXIT_FAILURE);
        }

        if (uart_set(baud, 8, 'N', 1) == -1) {
            fprintf(stderr, "Uart set failed!\n");
            exit(EXIT_FAILURE);
        }
        mRunning = true;
    };

    void close() {
        mRunning = false;
        if (uart_close() == -1) {
            fprintf(stderr, "uart_close error\n");
            exit(EXIT_FAILURE);
        }
    };

    void getData(ImuDataPatch &data, bool withAngle = true) {
        int ret = recv_data(mBuff, 44);
        if (ret == -1) {
            fprintf(stderr, "uart read failed!\n");
            exit(EXIT_FAILURE);
        }
        for (int i = 0; i < ret; i++) {
            parse_data(mBuff[i]);
        }
        data[0] = mAcceleration[0];
        data[1] = mAcceleration[1];
        data[2] = mAcceleration[2];
        data[3] = mGyroscope[0];
        data[4] = mGyroscope[1];
        data[5] = mGyroscope[2];
        if (!withAngle)
            return;
        else {
            data[6] = mAngle[0];
            data[7] = mAngle[1];
            data[8] = mAngle[2];
        }
    };

private:
    int uart_open(const char *pathname) {
        mMPUHandle = ::open(pathname, O_RDWR | O_NOCTTY);
        if (-1 == mMPUHandle) {
            perror("Can't Open Serial Port");
            return (-1);
        } else
            printf("Open %s success!\n", pathname);
        if (isatty(STDIN_FILENO) == 0)
            printf("Standard input is not a terminal device\n");
        else
            printf("Isatty success!\n");
        return mMPUHandle;
    }

    int uart_set(int nSpeed, int nBits, char nEvent, int nStop) {
        struct termios newtio, oldtio;
        if (tcgetattr(mMPUHandle, &oldtio) != 0) {
            perror("SetupSerial 1");
            printf("Tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(mMPUHandle, &oldtio));
            return -1;
        }
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        switch (nBits) {
            case 7:
                newtio.c_cflag |= CS7;
                break;
            case 8:
                newtio.c_cflag |= CS8;
                break;
        }
        switch (nEvent) {
            case 'o':
            case 'O':
                newtio.c_cflag |= PARENB;
                newtio.c_cflag |= PARODD;
                newtio.c_iflag |= (INPCK | ISTRIP);
                break;
            case 'e':
            case 'E':
                newtio.c_iflag |= (INPCK | ISTRIP);
                newtio.c_cflag |= PARENB;
                newtio.c_cflag &= ~PARODD;
                break;
            case 'n':
            case 'N':
                newtio.c_cflag &= ~PARENB;
                break;
            default:
                break;
        }

        /*set baud*/

        switch (nSpeed) {
            case 2400:
                cfsetispeed(&newtio, B2400);
                cfsetospeed(&newtio, B2400);
                break;
            case 4800:
                cfsetispeed(&newtio, B4800);
                cfsetospeed(&newtio, B4800);
                break;
            case 9600:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
                break;
            case 115200:
                cfsetispeed(&newtio, B115200);
                cfsetospeed(&newtio, B115200);
                break;
            case 460800:
                cfsetispeed(&newtio, B460800);
                cfsetospeed(&newtio, B460800);
                break;
            default:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
                break;
        }
        if (nStop == 1)
            newtio.c_cflag &= ~CSTOPB;
        else if (nStop == 2)
            newtio.c_cflag |= CSTOPB;
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;
        tcflush(mMPUHandle, TCIFLUSH);

        if ((tcsetattr(mMPUHandle, TCSANOW, &newtio)) != 0) {
            perror("Com set error");
            return -1;
        }
        printf("Set done!\n");
        return 0;
    }

    int uart_close() {
        assert(mMPUHandle);
        ::close(mMPUHandle);
        return 0;
    }

    int send_data(char *send_buffer, int length) {
        length = write(mMPUHandle, send_buffer, length * sizeof(unsigned char));
        return length;
    }

    int recv_data(char *recv_buffer, int length) {
        length = read(mMPUHandle, recv_buffer, length);
        return length;
    }

    void parse_data(char chr) {
        static char chrBuf[100];
        static unsigned char chrCnt = 0;
        signed short sData[4];
        unsigned char i;

        time_t now;
        chrBuf[chrCnt++] = chr;
        if (chrCnt < 11)
            return;

        if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50)) {
            printf("Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
            memcpy(&chrBuf[0], &chrBuf[1], 10);
            chrCnt--;
            return;
        }

        memcpy(&sData[0], &chrBuf[2], 8);
        switch (chrBuf[1]) {
            case 0x51:
                for (i = 0; i < 3; i++)
                    mAcceleration[i] = (float) sData[i] / 32768.0 * 16.0;
                time(&now);
//                printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",
//                       asctime(localtime(&now)), mAcceleration[0], mAcceleration[1], mAcceleration[2]);
                break;
            case 0x52:
                for (i = 0; i < 3; i++)
                    mGyroscope[i] = (float) sData[i] / 32768.0 * 2000.0;
//                printf("w:%7.3f %7.3f %7.3f ", mGyroscope[0], mGyroscope[1], mGyroscope[2]);
                break;
            case 0x53:
                for (i = 0; i < 3; i++)
                    mAngle[i] = (float) sData[i] / 32768.0 * 180.0;
//                printf("A:%7.3f %7.3f %7.3f ", mAngle[0], mAngle[1], mAngle[2]);
                break;
        }
        chrCnt = 0;
    }

private:
    float mAcceleration[3];
    float mGyroscope[3];
    float mAngle[3];
    char mBuff[1024];
    int mMPUHandle;
    bool mRunning = false;
    const char *mDeviceLoc = IMU_DEVICE_LOCATION;
};

class MPU6050Node : public rclcpp::Node {
public:
    MPU6050Node() : Node(MPU_NODE_NAME) {
        mIMUDevice = createSharedRef<MPU6050HW>();
        mIMUDevice->open();
        // create publisher and setup timer callback. here 10 meaning queue size.
        mPublisher = this->create_publisher<sensor_msgs::msg::Imu>(MPU_6050_TOPIC, MPU_QUEUE_SIZE);
        mSeq = 0;
        MilliSeconds dur(mTimerDur);
        mTimer = this->create_wall_timer(dur, std::bind(&MPU6050Node::timer_callback, this));
    };

private:
    void timer_callback() {

        mIMUDevice->getData(mRawImuData);
        auto full_data = sensor_msgs::msg::Imu();
        // fill in data.
        full_data.header.stamp = this->get_clock()->now();
        full_data.linear_acceleration.x = mRawImuData[0] * mGravity;
        full_data.linear_acceleration.y = mRawImuData[1] * mGravity;
        full_data.linear_acceleration.z = mRawImuData[2] * mGravity;
        full_data.angular_velocity.x = mRawImuData[3] * DegreeToRad;
        full_data.angular_velocity.y = mRawImuData[4] * DegreeToRad;
        full_data.angular_velocity.z = mRawImuData[5] * DegreeToRad;
        float f_cos = cos(mRawImuData[6] * DegreeToRadHalf);
        float f_sin = sin(mRawImuData[6] * DegreeToRadHalf);
        float s_cos = cos(mRawImuData[7] * DegreeToRadHalf);
        float s_sin = sin(mRawImuData[7] * DegreeToRadHalf);
        float t_cos = cos(mRawImuData[8] * DegreeToRadHalf);
        float t_sin = sin(mRawImuData[8] * DegreeToRadHalf);
        full_data.orientation.w = f_cos * s_cos * t_cos + f_sin * s_sin * t_sin;
        full_data.orientation.x = f_sin * s_cos * t_cos - f_cos * s_sin * t_sin;
        full_data.orientation.y = f_cos * s_sin * t_cos + f_sin * s_cos * t_sin;
        full_data.orientation.z = f_cos * s_cos * t_sin - f_sin * s_sin * t_cos;

        RCLCPP_INFO(this->get_logger(), "MPU6050: \n\tSequence: %llu"
                                        "\n\tAcceleration(m/s^2): %6.3f %6.3f %6.3f; "
                                        "\n\tGyroscope(rad/s): %6.3f %6.3f %6.3f; "
                                        "\n\tEuler Angle(degree): %6.3f %6.3f %6.3f",
                    mSeq,
                    full_data.linear_acceleration.x,
                    full_data.linear_acceleration.y,
                    full_data.linear_acceleration.z,
                    full_data.angular_velocity.x,
                    full_data.angular_velocity.y,
                    full_data.angular_velocity.z,
                    mRawImuData[6], mRawImuData[7], mRawImuData[8]);
        mPublisher->publish(full_data);
        // usleep(1000);
        mSeq++;
    }

private:
    SharedRef<MPU6050HW> mIMUDevice;
    ImgPubType mPublisher;
    TimerPtrType mTimer;
    ImuDataPatch mRawImuData;

    const int mTimerDur = BAUD > 9700 ? 10 : 50;
    const double mGravity = Gravity;
    const double DegreeToRad = DEGREE_TO_RAD;
    const double DegreeToRadHalf = DEGREE_TO_RAD_HALF;
    unsigned long long mSeq = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050Node>());
    rclcpp::shutdown();
    return 0;
}
