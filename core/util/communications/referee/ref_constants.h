#include <zephyr/kernel.h>
#include <cstdint>

// Screen resolution of your Robomaster Client
#define SCREEN_LENGTH 1920
#define SCREEN_WIDTH 1080

// Writing data stuff
// 上传数据最大的长度
// Maximum length of uploaded data
#define MAX_SIZE 128
// 帧头长度
// Frame header length
#define frameheader_len 5
// 命令码长度
// Command code length
#define cmd_len 2
// CRC16校验
// CRC16 checksum
#define crc_len 2

#define send_max_len 300  // 200

// -------------------------------------
// From South China University of Technology 华南理工大学广州学院-野狼战队-步兵代码
// ---------------------------------- 

// From South China University of Technology Guangzhou College - Wild Wolf Team - Infantry Code 
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.h/RM_JudgeSystem.h
// 2021 section

// 对应通信协议格式 frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
//  Corresponding communication protocol format:
//  frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,whole packet check)
#define LEN_HEADER 5 /*frame_header*/
#define LEN_CMDID 2  /*cmd_id*/
#define LEN_TAIL 2   /*frame_tail*/

// 起始字节，协议固定为0xA5
// Start byte, protocol fixed at 0xA5
#define JUDGE_FRAME_HEADER 0xA5
// 裁判系统包大小(354+35)
// Referee system packet size (354+35)
#define JUDGESYSTEM_PACKSIZE 389u


// although definitions work fine, they clutter up the namespace and are prone to integer interpretation errors
enum class TeamColor : uint8_t {
    UNKNOWN = 0,
    RED = 1,
    BLUE = 2,
};

// 信息传输
// Information transmission
#define TRUE 1
#define FALSE 0

/***********************************命令码ID*************************************/
/*********************************** Command Code ID *************************************/

#define Judge_Game_StatusData 0x0001   // 11
#define Judge_Game_ResultData 0x0002   // 1
#define Judge_Robot_HP 0x0003          // 32
#define Judge_Dart_Launch 0x0004       // UNUSED
#define Judge_AI_ChallengeBuff 0x0005  // UNUSED
#define Judge_Event_Data 0x0101        // 4
#define Judge_Supply_Station 0x0102    // 3
// #define       Judge_Request_Recharge             0x0103(对抗赛未开放)
// #define       Judge_Request_Recharge             0x0103(Not available in confrontation match)
#define Judge_Referee_Warning 0x0104  // 3
#define Judge_Dart_Countdown 0x0105   // 3
#define Judge_Robot_State 0x0201      // 13
#define Judge_Power_Heat 0x0202       // 16
#define Judge_Robot_Position 0x0203   // 16
#define Judge_Robot_Buff 0x0204       // 7
#define Judge_Aerial_Energy 0x0205
#define Judge_Injury_State 0x0206       // 1
#define Judge_RealTime_Shoot 0x0207     // 7
#define Judge_Remaining_Rounds 0x0208   // 6
#define Judge_Robot_RFID 0x0209         // 4
#define Judge_Dart_Client 0x020A        // 6
#define Judge_Robot_Communicate 0x0301  // 127
#define Judge_User_Defined 0x0302       // 30
#define Judge_Map_Interaction 0x0303    // 15
#define Judge_KeyMouse_Message 0x0304   // 12
#define Judge_Client_Map 0x0305         // 24

/***************************DATA_Length*************************/

#define JUDGE_EXTRA 9
/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

#define JudgeLength_Game_StatusData 20
#define JudgeLength_Game_ResultData 10
#define JudgeLength_Robot_HP 41
#define JudgeLength_Dart_Launch 12       // UNUSED
#define JudgeLength_AI_ChallengeBuff 20  // UNUSED
#define JudgeLength_Event_Data 13
#define JudgeLength_Supply_Station 12
// #define       JudgeLength_Request_Recharge       11(对抗赛未开放)
// #define       JudgeLength_Request_Recharge       11(Not available in confrontation match)
#define JudgeLength_Referee_Warning 12
#define JudgeLength_Dart_Countdown 12
#define JudgeLength_Robot_State 22
#define JudgeLength_Power_Heat 25
#define JudgeLength_Robot_Position 25
#define JudgeLength_Robot_Buff 15
#define JudgeLength_Aerial_Energy 11
#define JudgeLength_Injury_State 10
#define JudgeLength_RealTime_Shoot 16
#define JudgeLength_Remaining_Rounds 15
#define JudgeLength_Robot_RFID 13
#define JudgeLength_Dart_Client 21
#define JudgeLength_Robot_Commute 35
#define JudgeLength_Robot_Map 26

// needed for anonymous structs defined below
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

// -------------------------------------
// BELOW: from SCUT code line 733-1102

/* 自定义帧头 */
/* Custom frame header */
typedef struct {
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;

} xFrameHeader;

/* ID: 0x0001    Byte: 11     比赛状态数据 */
/* ID: 0x0001    Byte: 11     Game status data */
typedef struct {
    union {
        uint8_t dataBuff[11];
        struct {
            // 比赛类型
            // Game type
            uint8_t game_type : 4;
            // 当前比赛阶段
            //  Current game stage
            uint8_t game_progress : 4;
            // 当前阶段剩余时间  单位s
            //  Current stage remaining time (seconds)
            uint16_t stage_remain_time;
        };
    } data;
    uint8_t infoUpdateFlag;

} ext_map_command_t;

/* ID: 0x0001    Byte: 11     比赛状态数据 */
/* ID: 0x0001    Byte: 11     Game status data */
typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
    uint8_t infoUpdateFlag;
} game_status_t;

/* ID: 0x0002    Byte: 1       比赛结果数据 */
/* ID: 0x0002    Byte: 1       Game result data */
typedef struct {
    uint8_t winner;
    uint8_t infoUpdateFlag;
} game_result_t;

/* ID: 0x0003     Byte: 32     比赛机器人血量数据 */
/* ID: 0x0003     Byte: 32     Game robot HP data */
typedef struct {
    union {
        uint8_t dataBuff[32];
        struct {
            // 红 1 英雄机器人血量，未上场以及罚下血量为 0
            //  Red 1 Hero robot HP (0 if not on field or penalized)
            uint16_t red_1_robot_HP;
            // 红 2 工程机器人血量
            //  Red 2 Engineer robot HP
            uint16_t red_2_robot_HP;
            // 红 3 步兵机器人血量
            //  Red 3 Infantry robot HP
            uint16_t red_3_robot_HP;
            // 红 4 步兵机器人血量
            //  Red 4 Infantry robot HP
            uint16_t red_4_robot_HP;
            // 红 5 步兵机器人血量
            //  Red 5 Infantry robot HP
            uint16_t red_5_robot_HP;
            // 红 7 步兵机器人血量
            //  Red 7 Sentry/Infantry robot HP
            uint16_t red_7_robot_HP;
            // 红方前哨战血量
            //  Red outpost HP
            uint16_t red_outpost_HP;
            // 红方基地血量
            //  Red base HP
            uint16_t red_base_HP;
            uint16_t blue_1_robot_HP;
            uint16_t blue_2_robot_HP;
            uint16_t blue_3_robot_HP;
            uint16_t blue_4_robot_HP;
            uint16_t blue_5_robot_HP;
            uint16_t blue_7_robot_HP;
            uint16_t blue_outpost_HP;
            uint16_t blue_base_HP;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_robot_HP_t;

/* ID: 0x0004      Byte: 3    飞镖发射状态 */
/* ID: 0x0004      Byte: 3    Dart launch status */
typedef struct {
    union {
        uint8_t dataBuff[3];
        struct __packed {
            // 发射飞镖的队伍：1：红方飞镖2：蓝方飞镖
            //  Team launching dart: 1 = Red team dart, 2 = Blue team dart
            uint8_t dart_belong;
            // 发射时的剩余比赛时间，单位 s
            //  Remaining game time at launch (seconds)
            uint16_t stage_remaining_time;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_dart_status_t;

/* ID: 0x0005      Byte: 11      人工智能挑战赛加成与惩罚 */
/* ID: 0x0005      Byte: 11      AI Challenge Buff and Debuff */
typedef struct {
    union {
        uint8_t dataBuff[11];
        struct __packed {
            uint8_t F1_zone_status : 1;
            uint8_t F1_zone_buff_debuff_status : 3;
            uint8_t F2_zone_status : 1;
            uint8_t F2_zone_buff_debuff_status : 3;
            uint8_t F3_zone_status : 1;
            uint8_t F3_zone_buff_debuff_status : 3;
            uint8_t F4_zone_status : 1;
            uint8_t F4_zone_buff_debuff_status : 3;
            uint8_t F5_zone_status : 1;
            uint8_t F5_zone_buff_debuff_status : 3;
            uint8_t F6_zone_status : 1;
            uint8_t F6_zone_buff_debuff_status : 3;
            uint16_t red1_bullet_left;
            uint16_t red2_bullet_left;
            uint16_t blue1_bullet_left;
            uint16_t blue2_bullet_left;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101     Byte: 4       场地事件数据 */
/* ID: 0x0101     Byte: 4       Field event data */
typedef struct {
    union {
        uint8_t dataBuff[4];
        struct {
            uint32_t event_type;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_event_data_t;

/* ID: 0x0102     Byte: 4       场地补给站动作标识数据 */
/* ID: 0x0102     Byte: 4       Supply station action indicator data */
typedef struct {
    union {
        uint8_t dataBuff[4];
        struct {
            // 补给站口 ID
            //  Supply station aperture ID
            uint8_t supply_projectile_id;
            // 补弹机器人 ID
            //  Supply recipient robot ID
            uint8_t supply_robot_id;
            // 出弹口开闭状态
            //  Projectile exit opening/closing status
            uint8_t supply_projectile_step;
            // 补弹数量
            //  Number of projectiles supplied
            uint8_t supply_projectile_num;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_supply_projectile_action_t;

/* ID: 0X0104          Byte: 2       裁判警告数据 */
/* ID: 0X0104          Byte: 2       Referee warning data */
typedef struct {
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
    uint8_t infoUpdateFlag;
} referee_warning_t;

/* ID: 0x0105          Byte: 1       飞镖发射口倒计时  */
/* ID: 0x0105          Byte: 1       Dart launch gate countdown */
typedef struct {
    union {
        uint8_t dataBuff[1];
        struct {
            // 15s 倒计时
            //  15s countdown
            uint8_t dart_remaining_time;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_dart_remaining_time_t;

/* ID: 0X0201          Byte: 27      机器人状态数据 */
/* ID: 0X0201          Byte: 27      Robot status data */
typedef struct {
    union {
        uint8_t dataBuff[27];
        struct {
            uint8_t robot_id;

            uint8_t robot_level;
            // 机器人剩余血量
            //  Robot remaining HP
            uint16_t current_HP;
            // 机器人上限血量
            //  Robot maximum HP
            uint16_t maximum_HP;
            // 机器人 1 号 17mm 枪口每秒冷却值
            //  Robot No. 1 17mm barrel cooling value per second
            uint16_t shooter_barrel_cooling_value;
            // 机器人 1 号 17mm 枪口热量上限
            //  Robot No. 1 17mm barrel heat limit
            uint16_t shooter_barrel_heat_limit;
            // 机器人 1 号 17mm 枪口上限速度 单位 m/s
            //  Robot No. 1 17mm barrel maximum speed (m/s)
            uint16_t chassis_power_limit;
            uint8_t power_management_output;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_robot_status_t;

typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_output;
    uint8_t infoUpdateFlag;
} robot_status_t;

/* ID: 0X0202          Byte: 16      实时功率热量数据 */
/* ID: 0X0202          Byte: 16      Real-time power and heat data */
typedef struct {
    uint16_t reserved1;  // REMOVED chassis_voltage
    uint16_t reserved2;  // REMOVED chassis_current
    float reserved3;     // REMOVED chassis_power
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
    uint8_t infoUpdateFlag;
} power_heat_data_t;

/* ID: 0X0203          Byte: 16      机器人位置数据  */
/* ID: 0X0203          Byte: 16      Robot position data */
typedef struct {
    float x;
    float y;
    float angle;
    uint8_t infoUpdateFlag;
} robot_pos_t;

/* ID: 0X0204          Byte: 1       机器人增益数据 */
/* ID: 0X0204          Byte: 1       Robot buff data */
typedef struct {
    union {
        uint8_t dataBuff[1];
        struct __packed {
            /*bit 0：机器人血量补血状态
              bit 0: Robot HP recovery status
              bit 1：枪口热量冷却加速
              bit 1: Barrel heat cooling acceleration
              bit 2：机器人防御加成
              bit 2: Robot defense buff
              bit 3：机器人攻击加成
              bit 3: Robot attack buff*/
            uint8_t power_rune_buff;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_buff_t;

typedef struct {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} buff_t;

/* ID: 0X0205          Byte: 2       空中机器人能量状态数据 */
/* ID: 0X0205          Byte: 2       Aerial robot energy status data */
typedef struct {
    union {
        uint8_t dataBuff[2];
        struct __packed {
            // 可攻击时间 单位 s。30s 递减至 0
            //  Attackable time (seconds), decrements from 30s to 0
            uint8_t attack_time;
        };
    } data;
    uint8_t infoUpdateFlag;
} aerial_robot_energy_t;

/* ID: 0X0206          Byte: 1       伤害状态数据 */
/* ID: 0X0206          Byte: 1       Damage status data */
typedef struct {
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
    uint8_t infoUpdateFlag;
} hurt_data_t;

/* ID: 0X0207          Byte: 7       实时射击数据 */
/* ID: 0X0207          Byte: 7       Real-time shooting data */
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
    uint8_t infoUpdateFlag;
} shoot_data_t;

/* ID: 0X0208          Byte: 6       子弹剩余发送数 */
/* ID: 0X0208          Byte: 6       Remaining projectile launch count */
typedef struct {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint8_t infoUpdateFlag;
} projectile_allowance_t;

/* ID: 0X0209          Byte: 4       机器人RFID状态 */
/* ID: 0X0209          Byte: 4       Robot RFID status */
/*bit 0：基地增益点 RFID 状态；
bit 0: Base buff point RFID status;
bit 1：高地增益点 RFID 状态；
bit 1: Elevated ground buff point RFID status;
bit 2：能量机关激活点 RFID 状态；
bit 2: Power Rune activation point RFID status;
bit 3：飞坡增益点 RFID 状态；
bit 3: Launch ramp buff point RFID status;
bit 4：前哨岗增益点 RFID 状态；
bit 4: Outpost buff point RFID status;
bit 5：资源岛增益点 RFID 状态；
bit 5: Resource island buff point RFID status;
bit 6：补血点增益点 RFID 状态；
bit 6: HP recovery point buff point RFID status;
bit 7：工程机器人补血卡 RFID 状态；
bit 7: Engineer robot HP recovery card RFID status;
RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效果。
RFID status does not entirely represent the corresponding buff or penalty status; for example, if
the enemy has occupied an elevated ground buff point, the corresponding buff effect cannot be
obtained.*/
typedef struct {
    union {
        uint8_t dataBuff[4];
        struct {
            uint32_t rfid_status;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_rfid_status_t;

/* ID: 0x020A          Byte: 12      飞镖机器人客户端指令书 */
/* ID: 0x020A          Byte: 12      Dart robot client command book */
typedef struct {
    union {
        uint8_t dataBuff[12];
        struct __packed {
            // 当前飞镖发射口的状态
            //  Current dart launch gate status
            uint8_t dart_launch_opening_status;
            // 飞镖的打击目标，默认为前哨站
            //  Dart attack target (default is outpost)
            uint8_t dart_attack_target;
            // 切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
            //  Remaining game time when switching target (seconds), defaults to 0 if never switched
            uint16_t target_change_time;
            // 检测到的第一枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
            //  Detected 1st dart speed, unit: 0.1 m/s/LSB (0 if not detected)
            uint8_t first_dart_speed;
            // 检测到的第二枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
            //  Detected 2nd dart speed, unit: 0.1 m/s/LSB (0 if not detected)
            uint8_t second_dart_speed;
            // 检测到的第三枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
            //  Detected 3rd dart speed, unit: 0.1 m/s/LSB (0 if not detected)
            uint8_t third_dart_speed;
            // 检测到的第四枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
            //  Detected 4th dart speed, unit: 0.1 m/s/LSB (0 if not detected)
            uint8_t fourth_dart_speed;
            // 最近一次的发射飞镖的比赛剩余时间，单位秒，初始值为 0。
            //  Remaining game time of the most recent dart launch (seconds), initial value is 0
            uint16_t last_dart_launch_time;
            // 最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
            //  Remaining game time when operator confirmed launch command (seconds), initial value
            //  is 0
            uint16_t operate_launch_cmd_time;
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_dart_client_cmd_t;

// ----------------------- Line 1163

/* ID: 0X0301          Byte: n       机器人间交互数据 */
/* ID: 0X0301          Byte: n       Robot-to-robot interaction data */
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* data */
typedef struct {
    // 数据段n小于113
    //  Data segment n is less than 113
    uint8_t data[20];
} robot_interactive_data_t;

/*
        客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0x0100   0x0101  0x0102  0x0103  0x0110
   0x0104 Client custom data: cmd_id: 0x0301. Content ID: 0x0100, 0x0101, 0x0102, 0x0103, 0x0110,
   0x0104 发送频率：上限 10Hz Transmission frequency: max 10Hz


*/

/* 客户端删除图形 机器人间通信：0x0301 */
/* Client delete graphic | Robot communication: 0x0301 */
typedef struct {
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

// my copy
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    ext_client_custom_graphic_delete_t graphic_custom;  // ADDED
} ext_student_interactive_header_data_delete_t;

/* 图形数据 */
/* Graphic data */
typedef struct {
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} graphic_data_struct_t;

typedef struct {
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    int32_t data;
} ClientData_struct_t;

/* 客户端绘制一个图形 机器人间通信：0x0301 */
/* Client draw one graphic | Robot communication: 0x0301 */
typedef struct {
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

/* 客户端绘制二个图形 机器人间通信：0x0301 */
/* Client draw two graphics | Robot communication: 0x0301 */
typedef struct {
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

/* 客户端绘制五个图形 机器人间通信：0x0301 */
/* Client draw five graphics | Robot communication: 0x0301 */
typedef struct {
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

/* 客户端绘制七个图形 机器人间通信：0x0301 */
/* Client draw seven graphics | Robot communication: 0x0301 */
typedef struct {
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

/* 客户端绘制字符 机器人间通信：0x0301 */
/* Client draw characters | Robot communication: 0x0301 */
typedef struct {
    graphic_data_struct_t grapic_data_struct;
    char data[30];
} ext_client_custom_character_t;

// MY ADDED STRUCTS ------------------

/* ID: 0X0301          Byte: n       机器人间交互数据 */
/* ID: 0X0301          Byte: n       Robot-to-robot interaction data */
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    ext_client_custom_graphic_seven_t graphic_custom;  // ADDED
} ext_student_interactive_header_data_graphic_t;

// my copy
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    ext_client_custom_character_t graphic_custom;  // ADDED
} ext_student_interactive_header_data_character_t;

// my copy
typedef struct __packed {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    robot_interactive_data_t data;  // ADDED
} ext_student_interactive_header_data_robot_comm_t;

// ------------------- Line 1330

// /*机器人交互信息：0x0301*/
// /* Robot interactive information: 0x0301 */
// typedef __packed struct
// {
// 	//帧头
// 	// Frame header
// 	xFrameHeader   							txFrameHeader;
// 	//命令码
// 	// Command code
// 	uint16_t								CmdID;
// 	//数据段头结构
// 	// Data segment header structure
// 	ext_student_interactive_header_data_t   dataFrameHeader;
// 	//数据段
// 	// Data segment
// 	robot_interactive_data_t  	 			interactData;
// 	//帧尾
// 	// Frame tail
// 	uint16_t		 						FrameTail;
// }ext_CommunatianData_t;

// //帧头  命令码   数据段头结构  数据段   帧尾
// // Frame header | Command code | Data segment header | Data segment | Frame tail

// /*客户端结构体*/
// /* Client structure */
// //上传客户端
// // Upload to client
// typedef __packed struct
// {
// 	//帧头
// 	// Frame header
// 	xFrameHeader   							txFrameHeader;
// 	//命令码
// 	// Command code
// 	uint16_t		 						CmdID;
// 	//数据段头结构
// 	// Data segment header structure
// 	ext_student_interactive_header_data_t   dataFrameHeader;
// 	//数据段
// 	// Data segment
// 	graphic_data_struct_t cilentData[7];
// 	//帧尾
// 	// Frame tail
// 	uint16_t		 						FrameTail;
// }ext_SendClientData_t;

// typedef __packed struct
// {
// 	//帧头
// 	// Frame header
// 	xFrameHeader   							txFrameHeader;
// 	//命令码
// 	// Command code
// 	uint16_t		 						CmdID;
// 	//数据段头结构
// 	// Data segment header structure
// 	ext_student_interactive_header_data_t   dataFrameHeader;
// 	//数据段
// 	// Data segment
// 	graphic_data_struct_t cilentData[5];
// 	//帧尾
// 	// Frame tail
// 	uint16_t		 						FrameTail;
// }ext_ShowCrossHair_t;

#pragma GCC diagnostic pop