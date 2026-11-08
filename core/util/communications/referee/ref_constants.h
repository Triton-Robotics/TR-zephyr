#pragma once
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
// used for printing telemetry/debug data
constexpr const char* TeamColor_To_Str[] = {"UNKNOWN", "RED", "BLUE"};


// unused but required for the game status packet
enum class GameType : uint8_t {
    RMUC = 1, // RoboMaster University Championship
    RMUT = 2, // RoboMaster University Technical Challenge
    RMUA = 3, // ICRA RoboMaster University AI Challenge
    RMUL = 4, // RoboMaster University League 3V3 Match 
    RMUL_STANDARD = 5, // RMUL Standard Match
};

enum class GameProgress : uint8_t {
    SET_UP_PERIOD = 1,
    REF_INITIALIZATION_PERIOD = 2,
    COUNTDOWN = 3,
    COMPETITION = 4,
    COMPETITION_RESULT = 5,
};

enum class Result : uint8_t {
    DRAW = 0,
    RED_WIN = 1,
    BLUE_WIN = 2,
};

enum class PenaltyLevels : uint8_t {
    BOTH_YELLOW = 1, // both teams receive yellow card
    YELLOW_CARD = 2, // own robot received yellow
    RED_CARD = 3,
    FORFEITURE = 4,
};  

enum class ProjectileType : uint8_t {
    MM_17 = 1,
    MM_42 = 2,
};

enum class ShooterId : uint8_t {
    SHOOTER_17MM_1 = 1,
    SHOOTER_17MM_2 = 2,
    SHOOTER_42MM = 3,
};

enum class DartStatus: uint8_t {
    OPENED = 0,
    CLOSED = 1,
    OPENING_OR_CLOSING = 2,
};



// 信息传输
// Information transmission
#define TRUE 1
#define FALSE 0

/***********************************命令码ID*************************************/
/*********************************** Command Code ID *************************************/

enum class PacketHeader : uint16_t {
    GAME_STATUS_DATA = 0x0001,   // 11
    GAME_RESULT_DATA = 0x0002,   // 1
    ROBOT_HP = 0x0003,          // 32
    EVENT_DATA = 0x0101,        // 4
    REFEREE_WARNING = 0x0104,   // 3
    DART_COUNTDOWN = 0x0105,    // 3
    ROBOT_STATE = 0x0201,       // 13
    POWER_HEAT = 0x0202,        // 16
    ROBOT_POSITION = 0x0203,    // 16
    ROBOT_BUFF = 0x0204,        // 7
    INJURY_STATE = 0x0206,      // 1
    REAL_TIME_SHOOT = 0x0207,    // 7
    REMAINING_ROUNDS = 0x0208,  // 6
    ROBOT_RFID = 0x0209,        // 4
    DART_CLIENT = 0x020A,       // 6
    ROBOT_COMMUNICATE = 0x0301, // 127
    USER_DEFINED = 0x0302,      // 30
    MAP_INTERACTION = 0x0303,   // 15
    KEY_MOUSE_MESSAGE = 0x0304, // 12
    CLIENT_MAP = 0x0305,        // 24

    // DART_LAUNCH = 0x0004,       // UNUSED
    // AI_CHALLENGE_BUFF = 0x0005, // UNUSED
    // SUPPLY_STATION = 0x0102,    // UNUSED
    // REQUEST_RECHARGE = 0x0103,  // (Not available in confrontation match)
    // AERIAL_ENERGY = 0x0205,     // UNUSED
};


/***************************DATA_Length*************************/

#define JUDGE_EXTRA 9
/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

enum class PacketLength : uint16_t {
    GAME_STATUS_DATA = 20,
    GAME_RESULT_DATA = 10,
    ROBOT_HP = 41,
    EVENT_DATA = 13,
    REFEREE_WARNING = 12,
    DART_COUNTDOWN = 12,
    ROBOT_STATE = 22,
    POWER_HEAT = 25,
    ROBOT_POSITION = 25,
    ROBOT_BUFF = 15,
    INJURY_STATE = 10,
    REAL_TIME_SHOOT = 16,
    REMAINING_ROUNDS = 15,
    ROBOT_RFID = 13,
    DART_CLIENT = 21,
    ROBOT_COMMUNICATE = 35,

    // DART_LAUNCH = 12,          // UNUSED
    // AI_CHALLENGE_BUFF = 20,     // UNUSED
    // SUPPLY_STATION = 12,       // UNUSED
    // REQUEST_RECHARGE = 11,     // (Not available in confrontation match)
    // AERIAL_ENERGY = 11,        // UNUSED
    // ROBOT_MAP = 26,            // UNUSED
};

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

/* 
    ID: 0x0001    
    Byte: 11    
    General game status info 
*/
typedef struct __packed {
    GameType game_type : 4;
    GameProgress game_progress : 4;
    uint16_t stage_remain_time; // remaining time in round (in sec) 
    uint64_t SyncTimeStamp;     // unix timestamp to sync with server
    uint8_t infoUpdateFlag;
} game_status_t;
static_assert(offsetof(game_status_t, infoUpdateFlag) == 11, "game_status_t size must be 11 bytes");


/* 
    ID: 0x0002    
    Byte: 1     
    Game result data 
*/
typedef struct __packed {
    Result winner;
    uint8_t infoUpdateFlag;
} game_result_t;
static_assert(offsetof(game_result_t, infoUpdateFlag) == 1, "game_result_t size must be 1 byte");


/* 
    ID: 0x0003     
    32 byte length

    Retrieves the hp of every robot in the round from the server and stores it here.
    Although unused in this codebase right now, can be used to track states of every other robot.
    DO NOT DELETE; required for correctly parsing the incoming server packet
*/
typedef struct __packed {
    //  Red 1 Hero robot HP (0 if not on field or ejected/penalized)
    uint16_t red_1_robot_HP;
    //  Red 2 Engineer robot HP
    uint16_t red_2_robot_HP;
    //  Red 3 Infantry robot HP
    uint16_t red_3_robot_HP;    
    //  Red 4 Infantry robot HP
    uint16_t red_4_robot_HP;
    //  Red 5 Infantry robot HP, (no longer used)
    uint16_t _reserved1;
    //  Red 7 Sentry robot HP
    uint16_t red_7_robot_HP;
    //  Red outpost HP
    uint16_t red_outpost_HP;
    //  Red base HP
    uint16_t red_base_HP;

    // same convention follows for blue
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t _reserved2; // Blue 5 infantry (no longer used)
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;

    uint8_t infoUpdateFlag;
} ext_game_robot_HP_t;
static_assert(offsetof(ext_game_robot_HP_t, infoUpdateFlag) == 32, "ext_game_robot_HP_t size must be 32 bytes");


/* 
    ID: 0x0101     
    Byte: 4       
    Field event data 
*/
typedef struct __packed {
    uint32_t event_type;
    uint8_t infoUpdateFlag;
} ext_event_data_t;
static_assert(offsetof(ext_event_data_t, infoUpdateFlag) == 4, "ext_event_data_t size must be 4 bytes");


/* 
    ID: 0X0104          
    Byte: 3      
    Referee warning data 
*/
typedef struct __packed {
    PenaltyLevels level;
    uint8_t offending_robot_id;
    uint8_t count;
    uint8_t infoUpdateFlag;
} referee_warning_t;
static_assert(offsetof(referee_warning_t, infoUpdateFlag) == 3, "referee_warning_t size must be 3 bytes");


/* 
    ID: 0x0105          
    Byte: 3      
    Dart launch gate countdown 
*/
typedef struct __packed {
    uint8_t dart_remaining_time;
    uint16_t dart_info;
    uint8_t infoUpdateFlag;
} ext_dart_remaining_time_t;
static_assert(offsetof(ext_dart_remaining_time_t, infoUpdateFlag) == 3, "ext_dart_remaining_time_t size must be 3 bytes");


/* 
    ID: 0X0201          
    Byte: 13      
    Robot status data 
*/
typedef struct __packed {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    //  Robot No. 1 17mm barrel cooling value per second
    uint16_t shooter_barrel_cooling_value;
    //  Robot No. 1 17mm barrel heat limit
    uint16_t shooter_barrel_heat_limit;
    //  Robot No. 1 17mm barrel maximum speed (m/s)
    uint16_t chassis_power_limit;

    // value of 0 indicates no output
    // value of 1 indicates 24V output
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t infoUpdateFlag;
} robot_status_t;
static_assert(offsetof(robot_status_t, infoUpdateFlag) == 13, "robot_status_t size must be 13 bytes");


/* 
    ID: 0X0202          
    Byte: 16      
    Real-time power and heat data 
*/
typedef struct __packed {
    uint16_t reserved1;  // REMOVED chassis_voltage
    uint16_t reserved2;  // REMOVED chassis_current
    float reserved3;     // REMOVED chassis_power
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
    uint8_t infoUpdateFlag;
} power_heat_data_t;
static_assert(offsetof(power_heat_data_t, infoUpdateFlag) == 16, "power_heat_data_t size must be 16 bytes");


/* 
    ID: 0X0203          
    Byte: 12      
    
    Robot position data. Units are in m (for x,y) and degrees for angle.
    True north is 0 degrees
*/
typedef struct __packed {
    float x;
    float y;
    float angle;
    uint8_t infoUpdateFlag;
} robot_pos_t;
static_assert(offsetof(robot_pos_t, infoUpdateFlag) == 12, "robot_pos_t size must be 12 bytes");


/* 
    ID: 0X0204          
    Byte: 7       
    Robot buffs data
*/
typedef struct __packed {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
    uint8_t infoUpdateFlag;
} buff_t;
typedef buff_t ext_buff_t; // used for not breaking our serial defines
static_assert(offsetof(buff_t, infoUpdateFlag) == 7, "buff_t size must be 7 bytes");


/* 
    ID: 0X0206          
    Byte: 1       
    Damage status data 
*/
typedef struct __packed {
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
    uint8_t infoUpdateFlag;
} hurt_data_t;
static_assert(offsetof(hurt_data_t, infoUpdateFlag) == 1, "hurt_data_t size must be 1 byte");



/* 
    ID: 0X0207          
    Byte: 7       
    Real-time shooting data
*/
typedef struct __packed {
    ProjectileType bullet_type;
    ShooterId shooter_number;
    uint8_t launching_frequency; // Hz
    float initial_speed; // m/s
    uint8_t infoUpdateFlag;
} shoot_data_t;
static_assert(offsetof(shoot_data_t, infoUpdateFlag) == 7, "shoot_data_t size must be 7 bytes");


/* 
    ID: 0X0208          
    Byte: 6       
    Remaining projectile launch count 
*/
typedef struct __packed {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress; // the reserve 17mm projectile allowance provided by the Fortress Buff Point
    uint8_t infoUpdateFlag;
} projectile_allowance_t;
static_assert(offsetof(projectile_allowance_t, infoUpdateFlag) == 8, "projectile_allowance_t size must be 8 bytes");



/* 
    ID: 0X0209          
    Byte: 4       
    Robot RFID status 
    
    bit 0：基地增益点 RFID 状态；
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
    obtained.
*/
typedef struct __packed {
    uint32_t rfid_status;
    uint8_t infoUpdateFlag;
} ext_rfid_status_t;
static_assert(offsetof(ext_rfid_status_t, infoUpdateFlag) == 4, "ext_rfid_status_t size must be 4 bytes");


/* 
    ID: 0x020A          
    Byte: 12      
    Dart robot client commands 
*/
typedef struct __packed {
    DartStatus dart_launch_opening_status;
    uint8_t reserved;
    // remaining competition time when attack target changed. Default: 0sec (no target change)
    uint16_t target_change_time; 
    // Remaining competition time when the Operator confirms the launch command for the last time. Default: 0sec
    uint16_t latest_launch_cmd_time;
    uint8_t infoUpdateFlag;
} ext_dart_client_cmd_t;
static_assert(offsetof(ext_dart_client_cmd_t, infoUpdateFlag) == 6, "ext_dart_client_cmd_t size must be 6 bytes");

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