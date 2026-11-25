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
// ----------------------------------
// From South China University of Technology Guangzhou College - Wild Wolf Team
// - Infantry Code ----------------------
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.c/RM_JudgeSystem.c
// 2021 section

int Referee::JudgeSystem_USART_Receive_DMA()  // modified
{
    return ref.read(JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
}

uint16_t Referee::Judge_GetMessage(uint16_t Data_Length) {
    uint16_t n = 0;
    while (n < Data_Length) {
        // frame is the current byte the "cursor" is on
        uint8_t* frame = &JudgeSystem_rxBuff[n];
        if (frame[0] != JUDGE_FRAME_HEADER) {
            n++;
            continue;
        }
        //accounts for split frames
        if (n + LEN_HEADER + LEN_CMDID > Data_Length) return n;

        //Checks that the 0xA5 byte is actualy the header
        if (!Verify_CRC8_Check_Sum(frame, LEN_HEADER)) {
            n++;
            continue;
        }
        //pulls the packet length from the frame itself to avoid noise bugs
        uint16_t data_len = static_cast<uint16_t>(frame[1] | (frame[2] << 8));
        if (data_len > JUDGESYSTEM_PACKSIZE - JUDGE_EXTRA) {  // impossible length, treat as noise
            n++;
            continue;
        }
        uint16_t frame_len = data_len + JUDGE_EXTRA;
        // Constructs the command id header from buffer
        PacketHeader cmd_id = static_cast<PacketHeader>(frame[5] | (frame[6] << 8));

        //accounts for split frames
        if (n + frame_len > Data_Length) return n;


        if (!Verify_CRC16_Check_Sum(frame, frame_len)) {
#if REF_DEBUG
            printf("CRC16_NO[%d] cmd=0x%02X%02X len=%u\n", n, frame[6], frame[5], data_len);
#endif
            n++;
            continue;
        }

        // Searches through packets till the correct command id is found.
        // The correct packet is then stored in val
        const Packet* val = nullptr;
        for (const Packet& p : packets) {
            if (p.cmd_id == cmd_id) {
                val = &p;
                break;
            }
        }

        // skips if the cmd_id was not identified
        if (val == nullptr) {
#if REF_DEBUG
            printf("SKIP[%d] cmd=0x%02X%02X len=%u\n", n, frame[6], frame[5], data_len);
#endif

            n+=frame_len;
            continue;
        }
        last_rx_ms = k_uptime_get_32();
        // For the Power Heat it needs to be here otherwise it gets overwritten
        power_heat_data_t preData = power_heat_data;

        
        // Frame len specifies the TOTAL amount of bytes per packet of data
        // frame + LEN_HEADER + LEN_CMDID bypasses the header + command ID prefix and skips to actual data
        // JUDGE_EXTRA is a const that contains the length of the extra useless data,
        // so we subtract to copy the right amount
        uint16_t expected = static_cast<uint16_t>(val->frame_len) - JUDGE_EXTRA;
        uint16_t copy_len = (data_len < expected) ? data_len : expected;
        bool unpackStatusCode =
            unpack_payload(val->dest, frame + LEN_HEADER + LEN_CMDID, copy_len, val->update_flag);
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
    return Data_Length;
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
    // Data frame length
    uint16_t frame_length = LEN_HEADER + LEN_CMDID + len + LEN_TAIL;

    // Clear the data storage array to zero
    memset(tx_buff, 0, frame_length);

    /***** Frame header packaging *****/

    // Data frame start byte
    tx_buff[0] = sof;

    // Length of data field in data frame
    memcpy(&tx_buff[1], (uint8_t*)&len, sizeof(len));

    // Packet sequence number
    tx_buff[3] = seq;

    // Frame header check CRC8
    Append_CRC8_Check_Sum(tx_buff, LEN_HEADER);

    /***** Command code packaging *****/
    memcpy(&tx_buff[LEN_HEADER], (uint8_t*)&cmd_id, LEN_CMDID);

    /***** Data packaging *****/
    memcpy(&tx_buff[LEN_HEADER + LEN_CMDID], p_data, len);

    // Full frame data check CRC16
    Append_CRC16_Check_Sum(tx_buff, frame_length);

    if (seq == 0xff)
        seq = 0;
    else
        seq++;

    /***** Data upload *****/

    // USART_ClearFlag(UART4,USART_FLAG_TC);
    LL_USART_ClearFlag_TC(ref.usart_getter());

    /*
        for(i=0;i<frame_length;i++)
        {
        //   USART_SendData(UART4,tx_buff[i]);
        LL_USART_TransmitData8(USART3, tx_buff[i]);                        //
    // HAL_UART_Transmit(huart4, tx_buff[i], sizeof(tx_buff[i]), 10);

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
    /***** Data upload *****/
    LL_USART_ClearFlag_TC(ref.usart_getter());
    mutex_write_.unlock();

    // TEMP: confirm UI frames are being sent
    printf("UI tx %d bytes, cmd 0x%02X%02X\n", len, packet[6], packet[5]);
    ref.write(packet, len);
}


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
        //Accunts for split frames that were cut off
        // Append new bytes after whatever partial frame is left from last time
        uint16_t space = JUDGESYSTEM_PACKSIZE - rx_len;
        uint16_t add = (static_cast<uint16_t>(rad) < space) ? static_cast<uint16_t>(rad) : space;
        memcpy(JudgeSystem_rxBuff + rx_len, JudgeSystem_rxBuff_priv, add);
        uint16_t total = rx_len + add;

        uint16_t used = Judge_GetMessage(total);

        // Move the unfinished frame (if any) to the front for the next read
        rx_len = total - used;
        if (rx_len >= JUDGESYSTEM_PACKSIZE) rx_len = 0;  // safety: never stall on a full buffer
        memmove(JudgeSystem_rxBuff, JudgeSystem_rxBuff + used, rx_len);
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
}

void Referee::writeThread() {
    while (get_robot_id() == 0) {
        k_msleep(50);
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
 * @attention  Package data; once packaged, send to the referee system via
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

    // read() calls this every 5 ms, so only print once a second
    static uint32_t last_print_ms = 0;
    uint32_t now = k_uptime_get_32();
    if (now - last_print_ms < 1000) return;
    last_print_ms = now;

    // Only prints already-parsed data; never reads from the UART
    // does not include chassis voltage, current, power
    // if want to add, use reserved1-3 (respectively) in power_heat_data
    printf(
        "====REF DATA====\nRobot ID: %d\nColor: %s\nCurrent HP: %d\nMax HP: %d\n"
        "Power limit: %d\nBuffer energy: %d\nAngle: %.2f\nLast frame: %u ms ago\n"
        "================\n",
        robot_status.robot_id, TeamColor_To_Str[static_cast<uint8_t>(is_red_or_blue())],
        robot_status.current_HP, robot_status.maximum_HP,
        robot_status.chassis_power_limit, power_heat_data.buffer_energy,
        static_cast<double>(robot_pos.angle), now - last_rx_ms);
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