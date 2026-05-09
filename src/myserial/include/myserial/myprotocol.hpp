#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include <robots_msgs/msg/game_status.hpp>
#include <robots_msgs/msg/robot_status.hpp>

#include <robots_msgs/msg/gimb_cmd_fram.hpp>
#include <robots_msgs/msg/gimbal_data.hpp>
#include <robots_msgs/msg/my_path.hpp>
#include <robots_msgs/msg/mode_cmd.hpp>
#include <robots_msgs/msg/chassis_odom.hpp>
#include <robots_msgs/msg/enemy_pose.hpp>

using namespace std;

namespace rm
{
    const uint8_t HEADER = 0x77;
    const uint8_t TAIL = 0x88;

    /**
     * @brief: 整体的发送通信结构体
     */
    // 42B
    struct __attribute__((__packed__)) WholeSendFrame
    {
        // 设置字段值
        void setAutoDriveMode(bool value) { _mode_cmd = (_mode_cmd & ~(1 << 0)) | ((value & 0x1) << 0); }
        void setGimbalMode(uint8_t value) { _mode_cmd = (_mode_cmd & ~(0b11 << 1)) | ((value & 0b11) << 1); }
        void setChassisMode(uint8_t value) { _mode_cmd = (_mode_cmd & ~(0b11 << 3)) | ((value & 0b11) << 3); }
        void setBuyBullet(bool value) { _mode_cmd = (_mode_cmd & ~(1 << 5)) | ((value & 0x1) << 5); }
        void setRebirth(bool value) { _mode_cmd = (_mode_cmd & ~(1 << 6)) | ((value & 0x1) << 6); }
        void setHiPower(bool value) { _mode_cmd = (_mode_cmd & ~(1 << 7)) | ((value & 0x1) << 7); }

        // 读取字段值
        bool getAutoDriveMode() const { return (_mode_cmd >> 0) & 0x1; }
        uint8_t getGimbalMode() const { return (_mode_cmd >> 1) & 0b11; }
        uint8_t getChassisMode() const { return (_mode_cmd >> 3) & 0b11; }
        bool getBuyBullet() const { return (_mode_cmd >> 5) & 0x1; }
        bool getRebirth() const { return (_mode_cmd >> 6) & 0x1; }


        uint8_t _sof = HEADER;

        // 1B
        uint8_t _priority = 0;
        /**
         * @brief: 控制底盘帧结构体
         */
        // 12B
        float _speed_x = 0;
        float _speed_y = 0;
        float _speed_w = 0;

        /**
         * @brief: 模式切换及向裁判系统发送数据
         */
        // 1B
        uint8_t _mode_cmd = 0;
        /*
            bit0: autodrivemode: 0-abnormal;1-normal;
            bit1-2: gimbalmode: 00:stop; 01:patrol&aim; 10:target_angle(single_shot); 11:small_buff
            bit3-4: chassismode: 00-stop; 01-spin_high; 10-spin_low; 11-follow
            bit5: buybullet
            bit6: rebirth
            bit7: Hi-power
        */

        /**
         * @brief: 路径点
         */
        //24 Byte
        uint8_t _intention = 0;
        uint16_t _start_x = 0;
        uint16_t _start_y = 0;
        int8_t _delta_x[9] = {0};
        int8_t _delta_y[9] = {0};
        int16_t _buff_yaw_diff_angle = 0;
        uint8_t _stair_mode = 0;  
        uint32_t _sentry_cmd = 0;

        uint8_t _eof = TAIL;

        void print()
        {
            cout << "================= WholeSendFrame 发送数据 =================" << endl;

            cout << "[底盘速度]" << endl;
            cout << "  Speed X: " << _speed_x << " m/s" << endl;
            cout << "  Speed Y: " << _speed_y << " m/s" << endl;
            cout << "  Speed W: " << _speed_w << " rad/s" << endl;

            cout << "[模式控制指令]" << endl;
            cout << "  Auto Drive Mode: " << (getAutoDriveMode() ? "ON" : "OFF") << endl;
            cout << "  Gimbal Mode: " << static_cast<int>(getGimbalMode()) << " (0:stop, 1:patrol&aim, 3:revolve)" << endl;
            cout << "  Chassis Mode: " << static_cast<int>(getChassisMode()) << " (0:stop, 1:spin, 3: follow)" << endl;
            cout << "  Buy Bullet: " << (getBuyBullet() ? "YES" : "NO") << endl;
            cout << "  Rebirth: " << (getRebirth() ? "YES" : "NO") << endl;
            cout << "  Hi-Power: " << (( _mode_cmd >> 7 ) & 0x1 ? "YES" : "NO") << endl;

            cout << "[路径点]" << endl;
            cout << "  Intention: " << static_cast<int>(_intention) << endl;
            cout << "  Start Point: (" << _start_x << ", " << _start_y << ")" << endl;

            cout << "  Delta X: ";
            for (int i = 0; i < 9; ++i)
                    cout << static_cast<int>(_delta_x[i]) << " ";
            cout << endl;

            cout << "  Delta Y: ";
            for (int i = 0; i < 9; ++i)
                    cout << static_cast<int>(_delta_y[i]) << " ";
            cout << endl;

            cout << "  Yaw diff: " << _buff_yaw_diff_angle << endl;
            cout << "  Stair Mode: " << static_cast<int>(_stair_mode) << endl;
            cout << "  Sentry Cmd: 0x" << hex << _sentry_cmd << dec << endl;

            cout << "===========================================================" << endl;
        }
        
        string to_string()
        {
            std::ostringstream oss;

            oss << "================= WholeSendFrame 发送数据 =================" << std::endl;

            oss << "[底盘速度]" << std::endl;
            oss << "  Speed X: " << _speed_x << " m/s" << std::endl;
            oss << "  Speed Y: " << _speed_y << " m/s" << std::endl;
            oss << "  Speed W: " << _speed_w << " rad/s" << std::endl;

            oss << "[模式控制指令]" << std::endl;
            oss << "  Auto Drive Mode: " << (getAutoDriveMode() ? "ON" : "OFF") << std::endl;
            oss << "  Gimbal Mode: " << static_cast<int>(getGimbalMode()) << " (0:stop, 1:patrol&aim, 3:revolve)" << std::endl;
            oss << "  Chassis Mode: " << static_cast<int>(getChassisMode()) << " (0:stop, 1:spin, 3:follow)" << std::endl;
            oss << "  Buy Bullet: " << (getBuyBullet() ? "YES" : "NO") << std::endl;
            oss << "  Rebirth: " << (getRebirth() ? "YES" : "NO") << std::endl;
            oss << "  Hi-Power: " << ((_mode_cmd >> 7) & 0x1 ? "YES" : "NO") << std::endl;

            oss << "[路径点]" << std::endl;
            oss << "  Intention: " << static_cast<int>(_intention) << std::endl;
            oss << "  Start Point: (" << _start_x << ", " << _start_y << ")" << std::endl;

            oss << "  Delta X: ";
            for (int i = 0; i < 9; ++i)
                    oss << static_cast<int>(_delta_x[i]) << " ";
            oss << std::endl;

            oss << "  Delta Y: ";
            for (int i = 0; i < 9; ++i)
                    oss << static_cast<int>(_delta_y[i]) << " ";
            oss << std::endl;

            oss << "  Yaw diff: " << _buff_yaw_diff_angle << endl;
            oss << "  Stair Mode: " << static_cast<int>(_stair_mode) << std::endl;
            oss << "  Sentry Cmd: 0x" << std::hex << _sentry_cmd << std::dec << std::endl;

            oss << "===========================================================" << std::endl;
            return oss.str();
        }
    };

    /**
     * @brief: slh: 整体的接收通信结构体
     */
    // 接收帧长度由 WHOLE_GET_LEN = sizeof(WholeGetFrame) 决定
    struct __attribute__((__packed__)) WholeGetFrame
    {
        uint8_t _sof = HEADER;

        // 6B
        /**
         * @brief: 接收底盘运动相关数据
         */
        float _speed_x = 0;  
        float _speed_y = 0;
        float _speed_w = 0;

        /**
         * @brief: 接收自瞄相关数据 
         */
        // 13B
        float _base_yaw = 0; // 大小yaw偏角
        float leg_length =0;
        uint8_t _enemy_id = 0;
        float _enemy_x = 0;
        float _enemy_y = 0; 
        //uint8_t _enemy_num = 0;
        /**
         * @brief: 接收来自裁判系统数据
         */
        // 4B 0x0001
        uint8_t _game_type = 0;
        uint8_t _game_process = 0;
        uint16_t _stage_remain_time = 0;

        // 8B 0x020B
        float our_hero_x = 0.0;
        float our_hero_y = 0.0;

        // 20B 0x0003
        uint16_t _my_HP = 400;
        uint16_t _my_outpost_HP = 1500;
        uint16_t _my_base_HP = 5000;
        uint16_t _enemy_1_robot_HP = 150;
        uint16_t _enemy_2_robot_HP = 150;
        uint16_t _enemy_3_robot_HP = 150;
        uint16_t _enemy_4_robot_HP = 150;
        uint16_t _enemy_7_robot_HP = 150;
        uint16_t _enemy_outpost_HP = 1500;
        uint16_t _enemy_base_HP = 5000;

        // 4B 0x0101
        uint32_t _event_data = 0;

        // 1B 0x0201
        uint8_t _robot_id = 0;

        // 12B 0x0203
        float _x = 0;     // 本机器人位置 x 坐标，单位：m
        float _y = 0;     // 本机器人位置 y 坐标，单位：m
        float _angle = 0; // 本机器人测速模块的朝向，单位：度。正北为 0 度         

        // 6B 0x0204
        uint8_t _recovery_buff = 0;      // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%
        uint8_t _defence_buff = 0;       // 机器人防御增益（百分比，值为 50 表示 50%防御增益
        uint8_t _vulnerability_buff = 0; // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
        uint16_t _attack_buff = 0;        // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益
        uint8_t _remaining_energy = 0;      //机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例
                                        /*
                                         bit 0：在剩余能量≥50%时为 1，其余情况为 0
                                         bit 1：在剩余能量≥30%时为 1，其余情况为 0
                                         bit 2：在剩余能量≥15%时为 1，其余情况为 0
                                         bit 3：在剩余能量≥5%时为 1，其余情况为 0
                                         Bit 4：在剩余能量≥1%时为 1，其余情况为 0
                                        */
        // 4B 0x0208
        uint16_t _projectile_allowance_17mm = 0; // 17mm 弹丸允许发弹量
        uint16_t _remaining_gold_coin = 0;       // 剩余金币数量

        // 4B 0x0209
        uint32_t _rfid_status = 0;

        // 6B 0x020D
        /*bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为 0，在
            哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的
            允许发弹量值。
            bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为 0，在哨兵
            机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允
            许发弹量的次数。
            bit 15-18：哨兵机器人成功远程兑换血量的次数，开局为 0，在哨兵机器人
            成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数。
            bit 19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为
            1，否则为 0。
            bit 20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为
            1，否则为 0。
            bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。*/
        uint32_t _sentry_info = 0;
        /*bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为 1，否则为 0。
            bit 1-11：队伍 17mm 允许发弹量的剩余可兑换数。
            bit 12-15：保留。*/
        uint16_t _sentry_info_2 = 0;

        // 8B 0x0303
        float _target_position_x = 0;
        float _target_position_y = 0;
        
        // 1B 底盘状态
        uint8_t _chassis_status = 0; 

        // 4B 底盘电容电压，单位：V
        float _capacitor_voltage = 0.0f;
        
        uint8_t _eof = TAIL;

        void print(void)
        {
            cout << "================= WholeGetFrame 接收数据 =================" << endl;

            cout << "[底盘运动数据]" << endl;
            cout << "speed:  " << static_cast<float>(_speed_x)/50
                    << "  " << static_cast<float>(_speed_y)/50
                    << "  " << static_cast<float>(_speed_w)/50 << " " << endl;

            cout << "[自瞄数据]" << endl;
            cout << "  Base Yaw: " << _base_yaw << " deg" << endl;
            cout << "  Enemy ID: " << static_cast<int>(_enemy_id)
                    << "  Pos: (" << static_cast<int>(_enemy_x)
                    << ", " << static_cast<int>(_enemy_y) << ")" << endl;

            cout << "[裁判系统 - 比赛状态]" << endl;
            cout << "  Game Type: " << static_cast<int>(_game_type)
                    << "  Process: " << static_cast<int>(_game_process)
                    << "  Remaining Time: " << _stage_remain_time << "s" << endl;

            cout << "[裁判系统 - HP 状态]" << endl;
            cout << "  My HP: " << _my_HP
                    << "  Outpost: " << _my_outpost_HP
                    << "  Base: " << _my_base_HP << endl;
            cout << "  Enemy Robots: "
                    << _enemy_1_robot_HP << ", "
                    << _enemy_2_robot_HP << ", "
                    << _enemy_3_robot_HP << ", "
                    << _enemy_4_robot_HP << ", "
                    << _enemy_7_robot_HP << endl;
            cout << "  Enemy Outpost: " << _enemy_outpost_HP
                    << "  Enemy Base: " << _enemy_base_HP << endl;

            cout << "[事件数据]" << endl;
            cout << "  Event Data: 0x" << hex << _event_data << dec << endl;

            cout << "[机器人状态]" << endl;
            cout << "  Robot ID: " << static_cast<int>(_robot_id)
                    << "  Pos: (" << _x << ", " << _y << ")  Angle: " << _angle << "°" << endl;

            cout << "[增益状态]" << endl;
            cout << "  Recovery: " << static_cast<int>(_recovery_buff) << "%"
                    << "  Defence: " << static_cast<int>(_defence_buff) << "%"
                    << "  Vulnerability: " << static_cast<int>(_vulnerability_buff) << "%"
                    << "  Attack: " << _attack_buff << "%"
                    << "  Energy BitFlag: 0x" << hex << static_cast<int>(_remaining_energy) << dec << endl;

            cout << "[资源信息]" << endl;
            cout << "  17mm Allowance: " << _projectile_allowance_17mm
                    << "  Gold Coin: " << _remaining_gold_coin << endl;

            cout << "  RFID Status: 0x" << hex << _rfid_status << dec << endl;

            cout << "[哨兵信息]" << endl;
            cout << "  Info1: 0x" << hex << _sentry_info
                    << "  Info2: 0x" << _sentry_info_2 << dec << endl;

            cout << "[目标位置]" << endl;
            cout << "  Target Pos: (" << _target_position_x
                    << ", " << _target_position_y << ")" << endl;
            
            cout << "[腿长信息]" << endl;
            cout << "  Leg Length: " << leg_length << endl;

            cout << "[底盘状态]" << endl;
            cout << "  Chassis Status: " << static_cast<int>(_chassis_status) << endl;
            cout << "  Capacitor Voltage: " << _capacitor_voltage << " V" << endl;

            cout << "==========================================================" << endl;
        }

        string to_string() {
            ostringstream oss;

            oss << "================= WholeGetFrame 接收数据 =================" << std::endl;

            oss << "[底盘运动数据]" << std::endl;
            oss << "  Speed X: " << static_cast<float>(_speed_x/1000) << std::endl;
            oss << "  Speed Y: " << static_cast<float>(_speed_y/1000) << std::endl;
            oss << "  Speed W: " << static_cast<float>(_speed_w/1000) << std::endl;

            oss << "[自瞄数据]" << std::endl;
            oss << "  Base Yaw: " << _base_yaw << " deg" << std::endl;
            oss << "  Enemy ID: " << static_cast<int>(_enemy_id) << " Pos: (" 
                << static_cast<float>(_enemy_x) << ", " << static_cast<float>(_enemy_y) << ")" << std::endl;

            oss << "[裁判系统 - 比赛状态]" << std::endl;
            oss << "  Game Type: " << static_cast<int>(_game_type) << " Process: " 
                << static_cast<int>(_game_process) << " Remaining Time: " << _stage_remain_time << "s" << std::endl;

            oss << "[裁判系统 - HP 状态]" << std::endl;
            oss << "  My HP: " << _my_HP << " Outpost: " << _my_outpost_HP 
                << " Base: " << _my_base_HP << std::endl;
            oss << "  Enemy Robots: " << _enemy_1_robot_HP << ", " << _enemy_2_robot_HP 
                << ", " << _enemy_3_robot_HP << ", " << _enemy_4_robot_HP 
                << ", " << _enemy_7_robot_HP << std::endl;
            oss << "  Enemy Outpost: " << _enemy_outpost_HP << " Enemy Base: " << _enemy_base_HP << std::endl;

            oss << "[事件数据]" << std::endl;
            oss << "  Event Data: 0x" << std::hex << _event_data << std::dec << std::endl;

            oss << "[机器人状态]" << std::endl;
            oss << "  Robot ID: " << static_cast<int>(_robot_id) << " Pos: (" << _x << ", " 
                << _y << ")  Angle: " << _angle << "°" << std::endl;

            oss << "[增益状态]" << std::endl;
            oss << "  Recovery: " << static_cast<int>(_recovery_buff) << "%" << " Defence: " 
                << static_cast<int>(_defence_buff) << "%" << " Vulnerability: " 
                << static_cast<int>(_vulnerability_buff) << "%" << " Attack: " 
                << _attack_buff << "%" << " Energy BitFlag: 0x" << std::hex 
                << static_cast<int>(_remaining_energy) << std::dec << std::endl;

            oss << "[资源信息]" << std::endl;
            oss << "  17mm Allowance: " << _projectile_allowance_17mm << " Gold Coin: " 
                << _remaining_gold_coin << std::endl;

            oss << "  RFID Status: 0x" << std::hex << _rfid_status << std::dec << std::endl;

            oss << "[哨兵信息]" << std::endl;
            oss << "  Info1: 0x" << std::hex << _sentry_info << " Info2: 0x" 
                << _sentry_info_2 << std::dec << std::endl;

            oss << "[目标位置]" << std::endl;
            oss << "  Target Pos: (" << _target_position_x << ", " << _target_position_y << ")" 
                << std::endl;

            oss << "[腿长信息]" << std::endl;
            oss << "  Leg Length: " << leg_length << std::endl;

            oss << "[底盘状态]" << std::endl;
            oss << "  Chassis Status: " << static_cast<int>(_chassis_status) << std::endl;
            oss << "  Capacitor Voltage: " << _capacitor_voltage << " V" << std::endl;

            oss << "==========================================================" << std::endl;

            return oss.str();
        }

    };

    const int WHOLE_SEND_LEN = sizeof(WholeSendFrame);
    const int WHOLE_GET_LEN = sizeof(WholeGetFrame);
}
