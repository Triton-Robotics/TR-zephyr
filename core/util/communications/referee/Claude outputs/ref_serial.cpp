#include "ref_serial.h"

#include <cstring>

#include "../ui/ui_g.h"
#include "zephyr/kernel.h"

// Referee system serial protocol, based on the SCUT Guangzhou "Wild Wolf" 2021 infantry code:
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.c/RM_JudgeSystem.c
//
// Frame layout: header (5) | cmd_id (2) | data (n) | CRC16 (2)
// header = SOF (1) | data_length (2) | seq (1) | CRC8 (1)

namespace {

constexpr size_t kReadStackSize = 4096;
constexpr size_t kWriteStackSize = 4096;
constexpr int kThreadPriority = 8;  // TODO: pick a reasoned priority

constexpr uint16_t kCmdIdOffset = LEN_HEADER;             // cmd_id follows the 5-byte header
constexpr uint16_t kDataOffset = LEN_HEADER + LEN_CMDID;  // data follows cmd_id

constexpr uint32_t kLinkTimeoutMs = 1000;  // no valid frame for this long = disconnected
constexpr int32_t kReadPeriodMs = 5;
constexpr int32_t kUiPeriodMs = 33;  // referee data arrives at ~30 Hz

}  // namespace

K_THREAD_STACK_DEFINE(referee_read_stack, kReadStackSize);
K_THREAD_STACK_DEFINE(referee_write_stack, kWriteStackSize);

// ---------------------------------------------------------------------------
// Construction / threads
// ---------------------------------------------------------------------------

Referee::Referee(const struct device* uart, USART_TypeDef* ll_usart) : ref(uart, ll_usart) {
    memset(JudgeSystem_rxBuff, 0, JUDGESYSTEM_PACKSIZE);

    // `this` is passed as p1 so the static entry point can call the member function.
    k_thread_create(&m_read_tdata, referee_read_stack, kReadStackSize, &Referee::readThreadEntry,
                    this, NULL, NULL, kThreadPriority, 0, K_NO_WAIT);
    k_thread_create(&m_write_tdata, referee_write_stack, kWriteStackSize,
                    &Referee::writeThreadEntry, this, NULL, NULL, kThreadPriority, 0, K_NO_WAIT);
}

void Referee::readThreadEntry(void* p1, void*, void*) { static_cast<Referee*>(p1)->readThread(); }

void Referee::writeThreadEntry(void* p1, void*, void*) {
    static_cast<Referee*>(p1)->writeThread();
}

void Referee::readThread() {
    while (true) {
        read();
        k_sleep(K_MSEC(kReadPeriodMs));
    }
}

void Referee::writeThread() {
    // Need a robot ID before the UI can address the client.
    while (get_robot_id() == 0) {
        k_yield();
    }

    UI mainUI(uint16_t(get_robot_id()),
              [this](uint8_t* packet, uint16_t len) { referee_data_pack_handle(packet, len); });
    mainUI.ui_init_g();

    bool was_connected = true;
    bool prev_is_spinning = false;
    bool prev_is_flywheel_on = false;
    bool prev_is_cv_on = false;
    bool prev_is_aligned = false;

    while (true) {
        uint32_t last = last_rx_ms;
        bool connected = last != 0 && (k_uptime_get_32() - last) < kLinkTimeoutMs;

        if (!connected) {
            was_connected = false;
            k_sleep(K_MSEC(100));
            continue;
        }
        if (!was_connected) {  // link came back after a timeout: client lost our UI
            mainUI.ui_reinit_g();
            was_connected = true;
        }

        bool ui_dirty = false;
        if (prev_is_spinning != is_spinning) {
            prev_is_spinning = is_spinning;
            mainUI.set_spin_ui(prev_is_spinning);
            ui_dirty = true;
        }
        if (prev_is_flywheel_on != is_flywheel_on) {
            prev_is_flywheel_on = is_flywheel_on;
            mainUI.set_flywheel_ui(prev_is_flywheel_on);
            ui_dirty = true;
        }
        if (prev_is_cv_on != is_cv_on) {
            prev_is_cv_on = is_cv_on;
            mainUI.set_cv_ui(prev_is_cv_on);
            ui_dirty = true;
        }
        if (prev_is_aligned != is_aligned) {
            prev_is_aligned = is_aligned;
            mainUI.set_alignment_ui(prev_is_aligned);
            ui_dirty = true;
        }
        if (ui_dirty) {
            mainUI.ui_update_g();
        }

        k_sleep(K_MSEC(kUiPeriodMs));
    }
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

bool Referee::readable() { return ref.readable(); }

int Referee::JudgeSystem_USART_Receive_DMA() {
    ref.enable_output(true);
    return ref.read(JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
}

void Referee::read() {
    if (!ref.readable()) {
        if (enablePrintRefData) printf("[REF] not readable\n");
        return;
    }

    int len = JudgeSystem_USART_Receive_DMA();
    if (len <= 0 || len > static_cast<int>(JUDGESYSTEM_PACKSIZE)) {
        if (enablePrintRefData) printf("[REF] bad read: %d\n", len);
        return;
    }

    mutex_read_.lock();
    memcpy(JudgeSystem_rxBuff, JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
    mutex_read_.unlock();

    Judge_GetMessage(static_cast<uint16_t>(len));

    if (enablePrintRefData) {
        printf("[REF] id %u %s hp %u/%u\n", robot_status.robot_id,
               is_red_or_blue() == TeamColor::RED ? "RED" : "BLUE", robot_status.current_HP,
               robot_status.maximum_HP);
    }
}

// Scans the receive buffer for frames, CRC-checks each one, and copies its payload into the
// matching struct. Bytes that don't start a valid frame are skipped one at a time.
void Referee::Judge_GetMessage(uint16_t Data_Length) {
    struct Packet {
        uint16_t cmd_id;
        uint16_t frame_len;    // full frame: header + cmd_id + data + CRC16
        void* dest;            // where the data section is copied
        uint8_t* update_flag;  // set to 1 on a successful copy (nullptr if none)
    };

    // NOTE: Robot_Buff copies into `buff` but flags `Buff` (kept from the original code).
    const Packet packets[] = {
        {Judge_Game_StatusData, JudgeLength_Game_StatusData, &game_status,
         &game_status.infoUpdateFlag},
        {Judge_Game_ResultData, JudgeLength_Game_ResultData, &game_result,
         &game_result.InfoUpdataFlag},
        {Judge_Robot_HP, JudgeLength_Robot_HP, ext_game_robot_HP.data.dataBuff,
         &ext_game_robot_HP.InfoUpdataFlag},
        {Judge_Dart_Launch, JudgeLength_Dart_Launch, ext_dart_status.data.dataBuff,
         &ext_dart_status.InfoUpdataFlag},
        {Judge_AI_ChallengeBuff, JudgeLength_AI_ChallengeBuff,
         ext_ICRA_buff_debuff_zone_status.data.dataBuff,
         &ext_ICRA_buff_debuff_zone_status.InfoUpdataFlag},
        {Judge_Event_Data, JudgeLength_Event_Data, ext_even_data.data.dataBuff,
         &ext_even_data.InfoUpdataFlag},
        {Judge_Supply_Station, JudgeLength_Supply_Station,
         ext_supply_projectile_action.data.dataBuff, &ext_supply_projectile_action.InfoUpdataFlag},
        {Judge_Referee_Warning, JudgeLength_Referee_Warning, &referee_warning,
         &referee_warning.InfoUpdataFlag},
        {Judge_Dart_Countdown, JudgeLength_Dart_Countdown, ext_dart_remaining_time.data.dataBuff,
         &ext_dart_remaining_time.InfoUpdataFlag},
        {Judge_Robot_State, JudgeLength_Robot_State, &robot_status, &robot_status.InfoUpdataFlag},
        {Judge_Power_Heat, JudgeLength_Power_Heat, &power_heat_data,
         &power_heat_data.InfoUpdataFlag},
        {Judge_Robot_Position, JudgeLength_Robot_Position, &robot_pos, &robot_pos.InfoUpdataFlag},
        {Judge_Robot_Buff, JudgeLength_Robot_Buff, &buff, &Buff.InfoUpdataFlag},
        {Judge_Aerial_Energy, JudgeLength_Aerial_Energy, aerial_robot_energy.data.dataBuff,
         &aerial_robot_energy.InfoUpdataFlag},
        {Judge_Injury_State, JudgeLength_Injury_State, &hurt_data, &hurt_data.InfoUpdataFlag},
        {Judge_RealTime_Shoot, JudgeLength_RealTime_Shoot, &shoot_data,
         &shoot_data.InfoUpdataFlag},
        {Judge_Remaining_Rounds, JudgeLength_Remaining_Rounds, &projectile_allowance,
         &projectile_allowance.InfoUpdataFlag},
        {Judge_Robot_RFID, JudgeLength_Robot_RFID, ext_rfid_status.data.dataBuff,
         &ext_rfid_status.InfoUpdataFlag},
        {Judge_Dart_Client, JudgeLength_Dart_Client, ext_dart_client_cmd.data.dataBuff,
         &ext_dart_client_cmd.InfoUpdataFlag},
        {Judge_Robot_Communicate, JudgeLength_Robot_Commute, Robot_Commute, nullptr},
    };

    uint16_t n = 0;
    while (n < Data_Length) {
        uint8_t* frame = &JudgeSystem_rxBuff[n];

        if (frame[0] != JUDGE_FRAME_HEADER || n + kDataOffset > Data_Length) {
            n++;
            continue;
        }

        const uint16_t cmd_id = frame[kCmdIdOffset] | (frame[kCmdIdOffset + 1] << 8);

        const Packet* pkt = nullptr;
        for (const Packet& p : packets) {
            if (p.cmd_id == cmd_id) {
                pkt = &p;
                break;
            }
        }

        if (pkt == nullptr || n + pkt->frame_len > Data_Length ||
            !Verify_CRC16_Check_Sum(frame, pkt->frame_len)) {
#if REF_DEBUG
            printf("[REF] skip cmd 0x%04x at %u\n", cmd_id, n);
#endif
            n++;
            continue;
        }

        // Workaround: the referee occasionally reports 0 barrel heat for one frame.
        // Keep the last nonzero value when that happens.
        const power_heat_data_t prev_heat = power_heat_data;

        memcpy(pkt->dest, frame + kDataOffset, pkt->frame_len - JUDGE_EXTRA);
        if (pkt->update_flag != nullptr) {
            *pkt->update_flag = 1;
        }

        if (cmd_id == Judge_Power_Heat) {
            auto keep_nonzero = [](uint16_t& now, uint16_t before) {
                if (now == 0 && before != 0) now = before;
            };
            keep_nonzero(power_heat_data.shooter_17mm_1_barrel_heat,
                         prev_heat.shooter_17mm_1_barrel_heat);
            keep_nonzero(power_heat_data.shooter_17mm_2_barrel_heat,
                         prev_heat.shooter_17mm_2_barrel_heat);
            keep_nonzero(power_heat_data.shooter_42mm_barrel_heat,
                         prev_heat.shooter_42mm_barrel_heat);
        }

        last_rx_ms = k_uptime_get_32();  // writeThread uses this to detect link loss

#if REF_DEBUG
        printf("[REF] cmd 0x%04x at %u\n", cmd_id, n);
#endif
        n += pkt->frame_len;
    }
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

// Builds a full frame around p_data and sends it.
// Based on https://blog.csdn.net/zhang1079528541/article/details/115435696
void Referee::referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t* p_data,
                                       uint16_t len) {
    uint8_t tx_buff[MAX_SIZE];
    const uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;
    if (frame_length > MAX_SIZE) return;

    mutex_write_.lock();
    memset(tx_buff, 0, frame_length);

    // Header
    tx_buff[0] = sof;
    memcpy(&tx_buff[1], &len, sizeof(len));
    tx_buff[3] = seq++;  // uint8_t wraps 0xFF -> 0
    Append_CRC8_Check_Sum(tx_buff, frameheader_len);

    // Command ID + data + CRC16 over the whole frame
    memcpy(&tx_buff[frameheader_len], &cmd_id, cmd_len);
    memcpy(&tx_buff[frameheader_len + cmd_len], p_data, len);
    Append_CRC16_Check_Sum(tx_buff, frame_length);

    LL_USART_ClearFlag_TC(ref.usart_getter());
    mutex_write_.unlock();

    ref.write(tx_buff, frame_length);
}

// Sends an already-built frame (used by the UI).
void Referee::referee_data_pack_handle(uint8_t* packet, uint16_t len) {
    mutex_write_.lock();
    LL_USART_ClearFlag_TC(ref.usart_getter());
    mutex_write_.unlock();

    ref.write(packet, len);
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------

uint8_t Referee::get_robot_id() { return robot_status.robot_id; }

uint16_t Referee::get_remain_hp() { return robot_status.current_HP; }

uint8_t Referee::get_game_progress() { return game_status.game_progress; }

// Red robots have IDs 1-11, blue robots 101-111.
TeamColor Referee::is_red_or_blue() {
    Judge_Self_ID = robot_status.robot_id;

    if (robot_status.robot_id >= 1 && robot_status.robot_id <= 11) {
        return TeamColor::RED;
    }
    if (robot_status.robot_id >= 101 && robot_status.robot_id <= 111) {
        return TeamColor::BLUE;
    }
    return TeamColor::UNKNOWN;
}

// Client ID is 0x0100 + robot ID for both teams.
void Referee::determine_ID() {
    if (is_red_or_blue() != TeamColor::UNKNOWN) {
        Judge_SelfClient_ID = 0x0100 + robot_status.robot_id;
    }
}

void Referee::init_referee_struct_data() { memset(&robot_status, 0, sizeof(robot_status_t)); }
