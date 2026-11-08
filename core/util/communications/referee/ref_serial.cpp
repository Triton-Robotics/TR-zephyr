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
    struct Packet{
        const uint16_t cmd_id; //the byte addresses of  the differnt possible packets
        const uint16_t frame_len; //the  byte length of the commands
        void* dest;//is where the data gets stored
        uint8_t* update_flag;//idk what this is but it gets set to 1 if the packet gets updated
        const char* debug_print;//2 letters corresponding to the cmds for debugging purposes
    };
    const Packet packets[] = {
    {Judge_Game_StatusData, JudgeLength_Game_StatusData, &game_status, &game_status.infoUpdateFlag, "GS"},
    {Judge_Game_ResultData, JudgeLength_Game_ResultData, &game_result, &game_result.infoUpdateFlag, "GR"},
    {Judge_Robot_HP, JudgeLength_Robot_HP, ext_game_robot_HP.data.dataBuff, &ext_game_robot_HP.infoUpdateFlag, "HP"},
    {Judge_Dart_Launch, JudgeLength_Dart_Launch, ext_dart_status.data.dataBuff, &ext_dart_status.infoUpdateFlag, "DL"},
    {Judge_AI_ChallengeBuff, JudgeLength_AI_ChallengeBuff, ext_ICRA_buff_debuff_zone_status.data.dataBuff, &ext_ICRA_buff_debuff_zone_status.infoUpdateFlag, "AI"},
    {Judge_Event_Data, JudgeLength_Event_Data, ext_even_data.data.dataBuff, &ext_even_data.infoUpdateFlag, "EV"},
    {Judge_Supply_Station, JudgeLength_Supply_Station, ext_supply_projectile_action.data.dataBuff, &ext_supply_projectile_action.infoUpdateFlag, "SS"},
    {Judge_Referee_Warning, JudgeLength_Referee_Warning, &referee_warning, &referee_warning.infoUpdateFlag, "RF"},
    {Judge_Dart_Countdown, JudgeLength_Dart_Countdown, ext_dart_remaining_time.data.dataBuff, &ext_dart_remaining_time.infoUpdateFlag, "DT"},
    {Judge_Robot_State, JudgeLength_Robot_State, &robot_status, &robot_status.infoUpdateFlag, "RS"},
    {Judge_Power_Heat, JudgeLength_Power_Heat, &power_heat_data, &power_heat_data.infoUpdateFlag, "PH"},
    {Judge_Robot_Position, JudgeLength_Robot_Position, &robot_pos, &robot_pos.infoUpdateFlag, "RP"},
    {Judge_Robot_Buff, JudgeLength_Robot_Buff, &buff, &Buff.infoUpdateFlag, "BF"},
    {Judge_Aerial_Energy, JudgeLength_Aerial_Energy, aerial_robot_energy.data.dataBuff, &aerial_robot_energy.infoUpdateFlag, "AE"},
    {Judge_Injury_State, JudgeLength_Injury_State, &hurt_data, &hurt_data.infoUpdateFlag, "IN"},
    {Judge_RealTime_Shoot, JudgeLength_RealTime_Shoot, &shoot_data, &shoot_data.infoUpdateFlag, "RE"},
    {Judge_Remaining_Rounds, JudgeLength_Remaining_Rounds, &projectile_allowance, &projectile_allowance.infoUpdateFlag, "RR"},
    {Judge_Robot_RFID, JudgeLength_Robot_RFID, ext_rfid_status.data.dataBuff, &ext_rfid_status.infoUpdateFlag, "ID"},
    {Judge_Dart_Client, JudgeLength_Dart_Client, ext_dart_client_cmd.data.dataBuff, &ext_dart_client_cmd.infoUpdateFlag, "DC"},
    {Judge_Robot_Communicate, JudgeLength_Robot_Commute, Robot_Commute, nullptr, "RC"},
    };
    uint16_t n =0;
    while(n<Data_Length){
        //frame is the current byte the "cursor" is on
        uint8_t* frame = &JudgeSystem_rxBuff[n];
        if(frame[0] != JUDGE_FRAME_HEADER){
            n++;
            continue;
        }
        // Constructs the command id header from buffer
        uint16_t cmd_id = frame[5] | (frame[6] << 8);
        //Searches throgh packets till the correct command id is found . val then points to the correct packet
        const Packet* val = nullptr;
        for(const Packet& p : packets){
            if(p.cmd_id == cmd_id){
                val = &p;
                break;
            }
        }
        
        //So Verify_CRC16_Check_Sum jsut checks the data is uncorrupted, so if it is we just skip over it, we also skip if the cmd_id was not identified
        if(!(Verify_CRC16_Check_Sum(frame,val->frame_len))||val==nullptr){
            //prints if REF_Depug is postive to show that it failed
            #if REF_DEBUG
                printf("%s_NO[%d]\n", val->debug_print, n);
            #endif
            n++;
            continue;
        }
        //For the Power Heat it needs to be here otherwise it gets overwritten
        power_heat_data_t preData = power_heat_data;
        //Andrew is gonan make a better function for this later but it just updates according to the nwe info
        memcpy(val->dest, frame+7, val->frame_len - JUDGE_EXTRA);
        //prints if REF_Depug is postive to show that it worked
        #if REF_DEBUG
            printf("%s[%d]\n", val->debug_print, n);
        #endif
        //skips over the frame that was just accounted for
        n += val->frame_len;
        //This chnages the update flag that all the vals have except for Judge_Robot_Communicate for some reason
        if(val->cmd_id!=Judge_Robot_Communicate){
            *val->update_flag = 1;
        }
        //This is smth that Judge_Power_Heat has to deal with it kinda ruins the cleaness of the code :(
        if(val->cmd_id==Judge_Power_Heat){
            if (power_heat_data.shooter_17mm_1_barrel_heat == 0 && preData.shooter_17mm_1_barrel_heat != 0) {
                power_heat_data.shooter_17mm_1_barrel_heat = preData.shooter_17mm_1_barrel_heat;
            }
            if (power_heat_data.shooter_17mm_2_barrel_heat == 0 && preData.shooter_17mm_2_barrel_heat != 0) {
                power_heat_data.shooter_17mm_2_barrel_heat = preData.shooter_17mm_2_barrel_heat;
            }
            if (power_heat_data.shooter_42mm_barrel_heat == 0 && preData.shooter_42mm_barrel_heat != 0) {
                power_heat_data.shooter_42mm_barrel_heat = preData.shooter_42mm_barrel_heat;
            }
        }
    }
}
//OLD UGLY ASS CODE
/*
    for (int n = 0; n < Data_Length;) {
        if (JudgeSystem_rxBuff[n] == JUDGE_FRAME_HEADER) {
            switch (JudgeSystem_rxBuff[n + 5] | JudgeSystem_rxBuff[n + 6] << 8) {
                // 比赛状态数据
                // Game status data
                case Judge_Game_StatusData:
                    // printf("ST[%d]\n", n);
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_Game_StatusData)) {
#if REF_DEBUG
                        printf("GS[%d]\n", n);
#endif
                        memcpy(&game_status, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Game_StatusData - JUDGE_EXTRA]));
                        n += JudgeLength_Game_StatusData;
                        game_status.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("GS_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 比赛结果
                // Game result
                case Judge_Game_ResultData:
                    // printf("GR[%d]\n", n);
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_Game_ResultData)) {
#if REF_DEBUG
                        printf("GR[%d]\n", n);
#endif
                        memcpy(&game_result, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Game_ResultData - JUDGE_EXTRA]));
                        n += JudgeLength_Game_ResultData;
                        game_result.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("GR_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 机器人血量数据
                // Robot HP data
                case Judge_Robot_HP:
                    // printf("HP[%d]\n", n);
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_HP)) {
#if REF_DEBUG
                        printf("HP[%d]\n", n);
#endif
                        memcpy(&ext_game_robot_HP.data.dataBuff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Robot_HP - JUDGE_EXTRA]));
                        n += JudgeLength_Robot_HP;
                        ext_game_robot_HP.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("HP_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 飞镖发射状态
                // Dart launch status
                case Judge_Dart_Launch:

                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Launch)) {
#if REF_DEBUG
                        printf("DL[%d]\n", n);
#endif
                        memcpy(&ext_dart_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Dart_Launch - JUDGE_EXTRA]));
                        n += JudgeLength_Dart_Launch;
                        ext_dart_status.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("DL_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // AI加成与惩罚
                // AI challenge buff and debuff
                case Judge_AI_ChallengeBuff:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_AI_ChallengeBuff)) {
#if REF_DEBUG
                        printf("AI[%d]\n", n);
#endif
                        memcpy(&ext_ICRA_buff_debuff_zone_status.data.dataBuff,
                               &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[11]));
                        n += JudgeLength_AI_ChallengeBuff;
                        ext_ICRA_buff_debuff_zone_status.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("AI_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 场地事件数据
                // Field event data
                case Judge_Event_Data:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Event_Data)) {
#if REF_DEBUG
                        printf("EV[%d]\n", n);
#endif
                        memcpy(&ext_even_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[4]));
                        n += JudgeLength_Event_Data;
                        ext_even_data.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("EV_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 补给站动作标识
                // Supply station action indicator
                case Judge_Supply_Station:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_Supply_Station)) {
#if REF_DEBUG
                        printf("SS[%d]\n", n);
#endif
                        memcpy(&ext_supply_projectile_action.data.dataBuff,
                               &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Supply_Station - JUDGE_EXTRA]));
                        n += JudgeLength_Supply_Station;
                        ext_supply_projectile_action.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("SS_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 裁判系统警告信息
                // Referee system warning information
                case Judge_Referee_Warning:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_Referee_Warning)) {
#if REF_DEBUG
                        printf("RF[%d]\n", n);
#endif
                        memcpy(&referee_warning, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Referee_Warning - JUDGE_EXTRA]));
                        n += JudgeLength_Referee_Warning;
                        referee_warning.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("RF_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 飞镖发射口倒计时
                // Dart launch countdown
                case Judge_Dart_Countdown:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_Dart_Countdown)) {
#if REF_DEBUG
                        printf("DT[%d]\n", n);
#endif
                        memcpy(&ext_dart_remaining_time.data.dataBuff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Dart_Countdown - JUDGE_EXTRA]));
                        n += JudgeLength_Dart_Countdown;
                        ext_dart_remaining_time.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("DT_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 比赛机器人状态
                // Game robot status
                case Judge_Robot_State:
                    // for(int i = 0; i < JudgeLength_Robot_State; i ++){
                    //     printf("|%2x", (uint8_t)*(JudgeSystem_rxBuff+n+i));
                    // }
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_State)) {
#if REF_DEBUG
                        printf("RS[%d]\n", n);
                        robot_status_t* gamering = (robot_status_t*)&JudgeSystem_rxBuff[n + 7];
// printf("G%u %u %u %u %u %u %u %x\n",
//     gamering->robot_id,
//     gamering->robot_level,
//     gamering->current_HP,
//     gamering->maximum_HP,
//     gamering->shooter_barrel_cooling_value,
//     gamering->shooter_barrel_heat_limit,
//     gamering->chassis_power_limit,
//     gamering->power_management_output);
#endif
                        memcpy(&robot_status, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Robot_State - JUDGE_EXTRA]));

                        n += JudgeLength_Robot_State;
                        robot_status.infoUpdateFlag = 1;

#if REF_DEBUG
// printf("R%u %u %u %u %u %u %u %x\n",
//     robot_status.robot_id,
//     robot_status.robot_level,
//     robot_status.current_HP,
//     robot_status.maximum_HP,
//     robot_status.shooter_barrel_cooling_value,
//     robot_status.shooter_barrel_heat_limit,
//     robot_status.chassis_power_limit,
//     robot_status.power_management_output);
#endif
                    } else {
#if REF_DEBUG
                        printf("RS_NO[%d|", n);
                        for (int i = 0; i < JudgeLength_Robot_State; i++) {
                            printf("%2x|", (uint8_t)*(JudgeSystem_rxBuff + n + i));
                        }
#endif
                        n++;
                    }
                    break;
                // 实时功率热量
                // Real-time power and heat
                case Judge_Power_Heat:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Power_Heat)) {
#if REF_DEBUG
                        printf("PH[%d]\n", n);
#endif
                        // WIERD BUG BYPASS
                        power_heat_data_t preData = power_heat_data;
                        memcpy(&power_heat_data, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Power_Heat - JUDGE_EXTRA]));

                        if (power_heat_data.shooter_17mm_1_barrel_heat == 0 &&
                            preData.shooter_17mm_1_barrel_heat != 0) {
                            power_heat_data.shooter_17mm_1_barrel_heat =
                                preData.shooter_17mm_1_barrel_heat;
                        }
                        if (power_heat_data.shooter_17mm_2_barrel_heat == 0 &&
                            preData.shooter_17mm_2_barrel_heat != 0) {
                            power_heat_data.shooter_17mm_2_barrel_heat =
                                preData.shooter_17mm_2_barrel_heat;
                        }
                        if (power_heat_data.shooter_42mm_barrel_heat == 0 &&
                            preData.shooter_42mm_barrel_heat != 0) {
                            power_heat_data.shooter_42mm_barrel_heat =
                                preData.shooter_42mm_barrel_heat;
                        }

                        n += JudgeLength_Power_Heat;
                        power_heat_data.infoUpdateFlag = 1;
#if REF_DEBUG
// printf("%u %u %u %u\n", power_heat_data.buffer_energy,
// power_heat_data.shooter_17mm_1_barrel_heat,
// power_heat_data.shooter_17mm_2_barrel_heat,
// power_heat_data.shooter_42mm_barrel_heat);
#endif
                    } else {
#if REF_DEBUG
                        printf("PH_NO[%d|", n);
// for(int i = 0; i < JudgeLength_Power_Heat; i ++){
//    printf("%2x|", (uint8_t)*(JudgeSystem_rxBuff+n+i));
// }
#endif
                        n++;
                    }
                    break;
                // 机器人位置
                // Robot position
                case Judge_Robot_Position:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_Robot_Position)) {
#if REF_DEBUG
                        printf("RP[%d]\n", n);
#endif
                        memcpy(&robot_pos, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Robot_Position - JUDGE_EXTRA]));
                        n += JudgeLength_Robot_Position;
                        robot_pos.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("RP_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 机器人增益
                // Robot buff
                case Judge_Robot_Buff:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Buff)) {
#if REF_DEBUG
                        printf("BF[%d]\n", n);
#endif
                        memcpy(&buff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Robot_Buff - JUDGE_EXTRA]));
                        n += JudgeLength_Robot_Buff;
                        Buff.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("BF_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 空中机器人能量状态
                // Aerial robot energy status
                case Judge_Aerial_Energy:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Aerial_Energy)) {
#if REF_DEBUG
                        printf("AE[%d]\n", n);
#endif
                        memcpy(&aerial_robot_energy.data.dataBuff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Aerial_Energy - JUDGE_EXTRA]));
                        n += JudgeLength_Aerial_Energy;
                        aerial_robot_energy.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("AE_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 伤害状态
                // Damage status
                case Judge_Injury_State:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Injury_State)) {
#if REF_DEBUG
                        printf("IN[%d]\n", n);
#endif
                        memcpy(&hurt_data, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Injury_State - JUDGE_EXTRA]));
                        n += JudgeLength_Injury_State;
                        hurt_data.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("IN_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 实时射击数据
                // Real-time shooting data
                case Judge_RealTime_Shoot:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_RealTime_Shoot)) {
#if REF_DEBUG
                        printf("RE[%d]\n", n);
#endif
                        memcpy(&shoot_data, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_RealTime_Shoot - JUDGE_EXTRA]));
                        n += JudgeLength_RealTime_Shoot;
                        shoot_data.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("RE_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 子弹剩余数
                // Remaining projectile allowance
                case Judge_Remaining_Rounds:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n,
                                               JudgeLength_Remaining_Rounds)) {
#if REF_DEBUG
                        printf("RR[%d]\n", n);
#endif
                        memcpy(&projectile_allowance, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Remaining_Rounds - JUDGE_EXTRA]));
                        n += JudgeLength_Remaining_Rounds;
                        projectile_allowance.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("RR_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 机器人RFID状态
                // Robot RFID status
                case Judge_Robot_RFID:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_RFID)) {
#if REF_DEBUG
                        printf("ID[%d]\n", n);
#endif
                        memcpy(&ext_rfid_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[JudgeLength_Robot_RFID - JUDGE_EXTRA]));
                        n += JudgeLength_Robot_RFID;
                        ext_rfid_status.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("ID_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                // 飞镖机器人客户端指令数据
                // Dart robot client command data
                case Judge_Dart_Client:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Client)) {
#if REF_DEBUG
                        printf("RC[%d]\n", n);
#endif
                        memcpy(&ext_dart_client_cmd.data.dataBuff, &JudgeSystem_rxBuff[n + 7],
                               sizeof(uint8_t[12]));
                        n += JudgeLength_Dart_Client;
                        ext_dart_client_cmd.infoUpdateFlag = 1;
                    } else {
#if REF_DEBUG
                        printf("DC_NO[%d]\n", n);
#endif
                        n++;
                    }  // 26
                    break;
                // 机器人信息交互(还有一种写法就是直接case内容ID 不case命令码)
                // Robot inter-communication (another approach is to directly case on the
                // content ID instead of the command code)
                case Judge_Robot_Communicate:
                    if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Commute)) {
                        memcpy(&Robot_Commute, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[26]));
                        n += JudgeLength_Robot_Commute;
                    } else {
#if REF_DEBUG
                        printf("RC_NO[%d]\n", n);
#endif
                        n++;
                    }
                    break;
                default:
                    // printf("[none %x]\n", JudgeSystem_rxBuff[n + 5] |
                    // JudgeSystem_rxBuff[n + 6] << 8);
                    n++;
                    break;
            }
        } else
            n++;
    }
*/
    // JudgeSystem.InfoUpdateFrame++;


void Referee::determine_ID() {
    TeamColor Color{is_red_or_blue()};
    if (Color == TeamColor::BLUE) {
        Judge_SelfClient_ID = 0x0100 + robot_status.robot_id;
    } else if (Color == TeamColor::RED) {
        // 计算客户端ID
        // Calculate client ID
        Judge_SelfClient_ID = 0x0100 + robot_status.robot_id;
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

Referee::Referee(const struct device* uart, USART_TypeDef* ll_usart) : ref(uart, ll_usart) {
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

game_status_t::GameProgress Referee::get_game_progress() { return game_status.game_progress; }

/**
 * @brief  判断自己红蓝方
 *         Determine whether the robot is on the Red or Blue team
 * @param  void
 * @retval RED   BLUE
 * @attention  数据打包,打包完成后通过串口发送到裁判系统
 *             Package data; once packaged, send to the referee system via 
 *             serial port
 */
TeamColor Referee::is_red_or_blue() {
    // 读取当前机器人ID
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
    printf("====REF DATA====\nRobot ID: %d\nColor: %s\nCurrent HP: %d\nMax HP: %d\nAngle: %.2f\n================\n",
        robot_status.robot_id,
        TeamColor_To_Str[static_cast<uint8_t>(is_red_or_blue())],
        robot_status.current_HP,
        robot_status.maximum_HP,
        static_cast<double>(robot_pos.angle)
    );
}