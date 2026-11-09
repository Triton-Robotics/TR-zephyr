#include "ref_serial.h"

#include <cstdint>

#include "../ui/ui_g.h"
#include "zephyr/kernel.h"

// If needed, these values could definitely be lowered, 1024 just seems like a
// safe option
#define REFEREE_READ_STACK_SIZE 4096
#define REFEREE_WRITE_STACK_SIZE 4096
#define THREAD_PRIORITY 8  // TODO: Figure out a better, more reasoned number

K_THREAD_STACK_DEFINE(referee_read_stack, REFEREE_READ_STACK_SIZE);
K_THREAD_STACK_DEFINE(referee_write_stack, REFEREE_WRITE_STACK_SIZE);

// -------------------------------------
// From South China University of Technology
// 华南理工大学广州学院-野狼战队-步兵代码
// ----------------------------------
// From South China University of Technology Guangzhou College - Wild Wolf Team
// - Infantry Code ----------------------
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.c/RM_JudgeSystem.c
// 2021 section

int Referee::JudgeSystem_USART_Receive_DMA()  // modified
{
    // b->enable_input(TRUE);
    // memset(JudgeSystem_rxBuff, 0, sizeof(JudgeSystem_rxBuff));
    ref.enable_output(true);
    // memset (JudgeSystem_rxBuff,0,JUDGESYSTEM_PACKSIZE);
    return ref.read(JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
}

void Referee::Judge_GetMessage(uint16_t Data_Length) {
    uint16_t n = 0;
    while (n < Data_Length) {
        // frame is the current byte the "cursor" is on
        uint8_t* frame = &JudgeSystem_rxBuff[n];
        if (frame[0] != JUDGE_FRAME_HEADER) {
            n++;
            continue;
        }

        // Constructs the command id header from buffer
        PacketHeader cmd_id = static_cast<PacketHeader>(frame[5] | (frame[6] << 8));

        // Searches through packets till the correct command id is found.
        // The correct packet is then stored in val
        const Packet* val = nullptr;
        for (const Packet& p : packets) {
            if (p.cmd_id == cmd_id) {
                val = &p;
                break;
            }
        }

        // Verify_CRC16_Check_Sum checks the data is uncorrupted
        // If corrupted, skip over it
        // Also skips if the cmd_id was not identified
        if (val == nullptr ||
            !Verify_CRC16_Check_Sum(frame, static_cast<uint16_t>(val->frame_len))) {
#if REF_DEBUG
            printf("%s_NO[%d]\n", val ? val->debug_print : "UNKNOWN", n);
#endif

            n++;
            continue;
        }
        last_rx_ms = k_uptime_get_32();
        // For the Power Heat it needs to be here otherwise it gets overwritten
        power_heat_data_t preData = power_heat_data;

        uint16_t frame_len = static_cast<uint16_t>(val->frame_len);

        // Frame len specifies the TOTAL amount of bytes per packet of data
        // Frame+7 bypasses the header + command ID prefix and skips to actual data
        // JUDGE_EXTRA is a const that contains the length of the extra useless data,
        // so we subtract to copy the right amount
        bool unpackStatusCode =
            unpack_payload(val->dest, frame + 7, frame_len - JUDGE_EXTRA, val->update_flag);
        if (!unpackStatusCode) printf("fail to parse server packet\n");

#if REF_DEBUG
        printf("%s[%d]\n", val ? val->debug_print : "UNKNOWN", n);
#endif

        // skips over the frame that was just accounted for
        n += frame_len;

        // This is smth that Judge_Power_Heat has to deal with it kinda ruins the cleaness of the
        // code :(
        if (val->cmd_id == PacketHeader::POWER_HEAT) {
            if (power_heat_data.shooter_17mm_barrel_heat == 0 &&
            preData.shooter_17mm_barrel_heat != 0) {
            power_heat_data.shooter_17mm_barrel_heat = preData.shooter_17mm_barrel_heat;
        }
            if (power_heat_data.shooter_42mm_barrel == 0 &&
                preData.shooter_42mm_barrel != 0) {
                power_heat_data.shooter_42mm_barrel = preData.shooter_42mm_barrel;
        }
        }
    }
}
// JudgeSystem.InfoUpdateFrame++;

void Referee::determine_ID() {
    TeamColor Color{is_red_or_blue()};
    switch (Color) {
        case TeamColor::BLUE:
        case TeamColor::RED: {
            // Calculate client ID
            Judge_SelfClient_ID = 0x0100 + robot_status.robot_id;
            break;
        }
        default:
            printf("UNKNOWN TEAM COLOR\n");
    }
}

void Referee::init_referee_struct_data() { memset(&robot_status, 0, sizeof(robot_status_t)); }

// Code from CSDN
// https://blog.csdn.net/zhang1079528541/article/details/115435696
void Referee::referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t* p_data,
                                       uint16_t len) {
    uint8_t tx_buff[MAX_SIZE];

    mutex_write_.lock();
    // 数据帧长度
    // Data frame length
    uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;

    // 存储数据的数组清零
    // Clear the data storage array to zero
    memset(tx_buff, 0, frame_length);

    /***** 帧头打包 *****/
    /***** Frame header packaging *****/

    // 数据帧起始字节
    // Data frame start byte
    tx_buff[0] = sof;

    // 数据帧中data的长度
    // Length of data field in data frame
    memcpy(&tx_buff[1], (uint8_t*)&len, sizeof(len));

    // 包序号
    // Packet sequence number
    tx_buff[3] = seq;

    // 帧头校验CRC8
    // Frame header check CRC8
    Append_CRC8_Check_Sum(tx_buff, frameheader_len);

    /***** 命令码打包 *****/
    /***** Command code packaging *****/
    memcpy(&tx_buff[frameheader_len], (uint8_t*)&cmd_id, cmd_len);

    /***** 数据打包 *****/
    /***** Data packaging *****/
    memcpy(&tx_buff[frameheader_len + cmd_len], p_data, len);

    // 一帧数据校验CRC16
    // Full frame data check CRC16
    Append_CRC16_Check_Sum(tx_buff, frame_length);

    if (seq == 0xff)
        seq = 0;
    else
        seq++;

    /***** 数据上传 *****/
    /***** Data upload *****/

    // USART_ClearFlag(UART4,USART_FLAG_TC);
    LL_USART_ClearFlag_TC(ref.usart_getter());

    /*
        for(i=0;i<frame_length;i++)
        {
        //   USART_SendData(UART4,tx_buff[i]);
        LL_USART_TransmitData8(USART3, tx_buff[i]);                        //
    // HAL_UART_Transmit(huart4, tx_buff[i], sizeof(tx_buff[i]), 10);

        // 等待之前的字符发送完成
        // Wait for previous characters to finish sending
        //   while (USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET);
                while(LL_USART_IsActiveFlag_TC(USART3) == RESET);
    }
    */
    mutex_write_.unlock();

    ref.write(tx_buff, frame_length);
}

void Referee::referee_data_pack_handle(uint8_t* packet, uint16_t len) {
    // Accquires thread resources
    mutex_write_.lock();
    /***** 数据上传 *****/
    /***** Data upload *****/
    LL_USART_ClearFlag_TC(ref.usart_getter());
    mutex_write_.unlock();

    ref.write(packet, len);
}

// Referee::Referee(PinName pin_tx, PinName pin_rx) : ref(pin_tx, pin_rx,
// 115200)
// {
//     memset(JudgeSystem_rxBuff, 0, JUDGESYSTEM_PACKSIZE);

//     this->readThread_.start(callback(this, &Referee::readThread));
//     this->writeThread_.start(callback(this, &Referee::writeThread));
// } 

Referee::Referee(const struct device* uart, USART_TypeDef* ll_usart)
    : ref(uart, ll_usart),
      packets{
          {PacketHeader::GAME_STATUS_DATA, PacketLength::GAME_STATUS_DATA, &game_status,
           &game_status.infoUpdateFlag, "GS"},
          {PacketHeader::GAME_RESULT_DATA, PacketLength::GAME_RESULT_DATA, &game_result,
           &game_result.infoUpdateFlag, "GR"},
          {PacketHeader::ROBOT_HP, PacketLength::ROBOT_HP, &ext_game_robot_HP,
           &ext_game_robot_HP.infoUpdateFlag, "HP"},
          {PacketHeader::EVENT_DATA, PacketLength::EVENT_DATA, &ext_even_data,
           &ext_even_data.infoUpdateFlag, "EV"},
          {PacketHeader::REFEREE_WARNING, PacketLength::REFEREE_WARNING, &referee_warning,
           &referee_warning.infoUpdateFlag, "RF"},
          {PacketHeader::DART_COUNTDOWN, PacketLength::DART_COUNTDOWN, &ext_dart_remaining_time,
           &ext_dart_remaining_time.infoUpdateFlag, "DT"},
          {PacketHeader::ROBOT_STATE, PacketLength::ROBOT_STATE, &robot_status,
           &robot_status.infoUpdateFlag, "RS"},
          {PacketHeader::POWER_HEAT, PacketLength::POWER_HEAT, &power_heat_data,
           &power_heat_data.infoUpdateFlag, "PH"},
          {PacketHeader::ROBOT_POSITION, PacketLength::ROBOT_POSITION, &robot_pos,
           &robot_pos.infoUpdateFlag, "RP"},
          {PacketHeader::ROBOT_BUFF, PacketLength::ROBOT_BUFF, &buff, &buff.infoUpdateFlag, "BF"},
          {PacketHeader::INJURY_STATE, PacketLength::INJURY_STATE, &hurt_data,
           &hurt_data.infoUpdateFlag, "IN"},
          {PacketHeader::REAL_TIME_SHOOT, PacketLength::REAL_TIME_SHOOT, &shoot_data,
           &shoot_data.infoUpdateFlag, "RE"},
          {PacketHeader::REMAINING_ROUNDS, PacketLength::REMAINING_ROUNDS, &projectile_allowance,
           &projectile_allowance.infoUpdateFlag, "RR"},
          {PacketHeader::ROBOT_RFID, PacketLength::ROBOT_RFID, &ext_rfid_status,
           &ext_rfid_status.infoUpdateFlag, "ID"},
          {PacketHeader::DART_CLIENT, PacketLength::DART_CLIENT, &ext_dart_client_cmd,
           &ext_dart_client_cmd.infoUpdateFlag, "DC"},
          {PacketHeader::ROBOT_COMMUNICATE, PacketLength::ROBOT_COMMUNICATE, Robot_Commute, nullptr,
           "RC"},
           {PacketHeader::GROUND_ROBOT_POS, PacketLength::GROUND_ROBOT_POS, &ground_robot_position,
           &ground_robot_position.infoUpdateFlag, "GP"},
          {PacketHeader::RADAR_MARK, PacketLength::RADAR_MARK, &radar_mark_data,
           &radar_mark_data.infoUpdateFlag, "RM"},
          {PacketHeader::SENTRY_INFO, PacketLength::SENTRY_INFO, &sentry_info,
           &sentry_info.infoUpdateFlag, "SI"},
          {PacketHeader::RADAR_INFO, PacketLength::RADAR_INFO, &radar_info,
           &radar_info.infoUpdateFlag, "RI"},
          {PacketHeader::MAP_INTERACTION, PacketLength::MAP_INTERACTION, &map_command,
           &map_command.infoUpdateFlag, "MC"},

          // unused
          // {PacketHeader::DART_LAUNCH, PacketLength::DART_LAUNCH, &ext_dart_status,
          // &ext_dart_status.infoUpdateFlag, "DL"}, {PacketHeader::AI_CHALLENGE_BUFF,
          // PacketLength::AI_CHALLENGE_BUFF, &ext_ICRA_buff_debuff_zone_status,
          // &ext_ICRA_buff_debuff_zone_status.infoUpdateFlag, "AI"}, {PacketHeader::SUPPLY_STATION,
          // PacketLength::SUPPLY_STATION, &ext_supply_projectile_action,
          // &ext_supply_projectile_action.infoUpdateFlag, "SS"}, {PacketHeader::AERIAL_ENERGY,
          // PacketLength::AERIAL_ENERGY, &aerial_robot_energy, &aerial_robot_energy.infoUpdateFlag,
          // "AE"},
      } {

    memset(JudgeSystem_rxBuff, 0, JUDGESYSTEM_PACKSIZE);

    k_thread_create(&m_read_tdata, referee_read_stack, REFEREE_READ_STACK_SIZE,
                    &Referee::readThreadEntry, this, NULL,
                    NULL,  // These 3 pointers are put into the readThreadEntry function, which
                           // basically just
                    THREAD_PRIORITY, 0,
                    K_NO_WAIT);  // Directs us to the particular thread we're talking about

    k_thread_create(&m_write_tdata, referee_write_stack, REFEREE_WRITE_STACK_SIZE,
                    &Referee::writeThreadEntry, this, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
}

void Referee::readThreadEntry(void* p1, void* p2, void* p3) {
    static_cast<Referee*>(p1)->readThread();
}

void Referee::writeThreadEntry(void* p1, void* p2, void* p3) {
    static_cast<Referee*>(p1)->writeThread();
}

bool Referee::readable() { return ref.readable(); }

void Referee::read() {
    if (ref.readable()) {
        int rad = JudgeSystem_USART_Receive_DMA();
        if (rad <= 0 || rad > JUDGESYSTEM_PACKSIZE) {
            if (enablePrintRefData) printf("[ERROR] bad ref read: %d\n", rad);
            return;  // or continue, depending on call site
        }
        mutex_read_.lock();
        memcpy(JudgeSystem_rxBuff, JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
        mutex_read_.unlock();
        Judge_GetMessage(static_cast<uint16_t>(rad));
        // int rad = JudgeSystem_USART_Receive_DMA();
        // mutex_read_.lock();
        // memcpy(JudgeSystem_rxBuff, JudgeSystem_rxBuff_priv,
        // JUDGESYSTEM_PACKSIZE); mutex_read_.unlock(); Judge_GetMessage(rad);

        if (enablePrintRefData) printDebug();
    } else {
        if (enablePrintRefData) printDebug();
    }
}
//This is just read again but in a loop with a slight delay
void Referee::readThread() {
    while(1){
        read();
        k_sleep(K_MSEC(5));
    }
    /* This is just read again but in a loop with a slight delay
    while (1) {
        // Un-nested read since all it does is call another fn
        // read();
        if (ref.readable()) {
            // int rad = JudgeSystem_USART_Receive_DMA();
            // mutex_read_.lock();
            // memcpy(JudgeSystem_rxBuff, JudgeSystem_rxBuff_priv,
            // JUDGESYSTEM_PACKSIZE); mutex_read_.unlock(); Judge_GetMessage(rad);
            int rad = JudgeSystem_USART_Receive_DMA();

            if (rad > 0 && rad <= JUDGESYSTEM_PACKSIZE) {
                mutex_read_.lock();
                memcpy(JudgeSystem_rxBuff, JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
                mutex_read_.unlock();
                Judge_GetMessage(static_cast<uint16_t>(rad));

                if (enablePrintRefData) printDebug();
            }

            else {
                if (enablePrintRefData) printDebug();
            }
        } else {
            if (enablePrintRefData) printDebug();
        }

        k_sleep(K_MSEC(5));
    }
        */
}

void Referee::writeThread() {
    while (get_robot_id() == 0) {
        k_yield();
    }

    // Some variables required to properrly send
    bool ui_dirty = false;
    bool prev_is_spinning = false;
    bool prev_is_flywheel_on = false;
    bool prev_is_cv_on = false;
    bool prev_is_aligned = false;

    UI mainUI(uint16_t(get_robot_id()),
              // robot_status.robot_id,
              [this](uint8_t* packet, uint16_t len) { referee_data_pack_handle(packet, len); });
    mainUI.ui_init_g();
    // uint64_t last_ref_update = now_us();

    bool was_connected = true;
    while (1) {
        uint32_t last = last_rx_ms;
        bool connected = last != 0 && (k_uptime_get_32() - last) < 1000;

        if (!connected) {
            was_connected = false;
            k_sleep(K_MSEC(100));
            continue;
        }
        if (!was_connected) {  // link came back after >1 s silence
            mainUI.ui_reinit_g();
            was_connected = true;
        }

        // uint64_t curr_time = now_us();
        // if (ref.readable()) {
        //     // if ref has not been able to be read from for more than 1 sec
        //     // but ref can currently be read from, re-init everything
        //     if(curr_time - last_ref_update > 1000000) {
        //         mainUI.ui_reinit_g();
        //     }
        //     last_ref_update = curr_time;
        // } else {
        //     // If cannot read, then not connected in server
        //     // which in that case do not need to try and send packets
        //     k_yield();
        //     continue;
        // }

        if (prev_is_spinning != is_spinning) {
            mainUI.set_spin_ui(is_spinning);
            prev_is_spinning = is_spinning;
            ui_dirty = true;
        }
        if (prev_is_flywheel_on != is_flywheel_on) {
            mainUI.set_flywheel_ui(is_flywheel_on);
            prev_is_flywheel_on = is_flywheel_on;
            ui_dirty = true;
        }
        if (prev_is_cv_on != is_cv_on) {
            mainUI.set_cv_ui(is_cv_on);
            prev_is_cv_on = is_cv_on;
            ui_dirty = true;
        }
        if (prev_is_aligned != is_aligned) {
            mainUI.set_alignment_ui(is_aligned);
            prev_is_aligned = is_aligned;
            ui_dirty = true;
        }

        if (ui_dirty) {
            mainUI.ui_update_g();
            ui_dirty = false;
        }

        // k_yield();
        k_sleep(K_MSEC(33));  // We only get new data every 30Hz, so we wait 1/30th of a second
    }
}

uint8_t Referee::get_robot_id() { return robot_status.robot_id; }

uint16_t Referee::get_remain_hp() { return robot_status.current_HP; }

GameProgress Referee::get_game_progress() { return game_status.game_progress; }

/**
 * @brief  Determine whether the robot is on the Red or Blue team
 * @param  void
 * @retval RED, BLUE, UNKNOWN
 * @attention  数据打包,打包完成后通过串口发送到裁判系统
 *             Package data; once packaged, send to the referee system via
 *             serial port
 */
TeamColor Referee::is_red_or_blue() {
    // Read current robot ID
    Judge_Self_ID = robot_status.robot_id;

    if (robot_status.robot_id >= 1 && robot_status.robot_id <= 11) {
        return TeamColor::RED;
    } else if (robot_status.robot_id >= 101 && robot_status.robot_id <= 111) {
        return TeamColor::BLUE;
    } else {
        return TeamColor::UNKNOWN;
    }
}

void Referee::printDebug() {
    if (!enablePrintRefData) return;

    if (!ref.readable()) {
        printf("REFEREE - Not readable!\n");
        return;
    }

    int rad = JudgeSystem_USART_Receive_DMA();
    if (rad > 0 && rad <= JUDGESYSTEM_PACKSIZE) {
        printf("[ERROR] bad ref read: %d\n", rad);
        return;
    }

    // does not include chassis voltage, current, power
    // if want to add, use reserved1-3 (respectively) in power_heat_data
    printf(
        "====REF DATA====\nRobot ID: %d\nColor: %s\nCurrent HP: %d\nMax HP: %d\nAngle: "
        "%.2f\n================\n",
        robot_status.robot_id, TeamColor_To_Str[static_cast<uint8_t>(is_red_or_blue())],
        robot_status.current_HP, robot_status.maximum_HP, static_cast<double>(robot_pos.angle));
}

bool Referee::unpack_payload(void* dest, const uint8_t* src, size_t payload_len,
                             uint8_t* update_flag) {
    if (!dest || !src) return false;

    memcpy(dest, src, payload_len);

    // Some structs do not have an update flag,
    // so check (just in case!)
    if (update_flag) {
        *update_flag = 1;
    }
    return true;
}