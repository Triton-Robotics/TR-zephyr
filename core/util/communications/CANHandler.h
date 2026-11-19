///////////////////////////////////////////////////////////////////////
//                        ____               __       __  __  _ _____                   ____               ____     ___   __   _ ___    __  _          
// _______ ___ _____ ___ / __/__  ___ ____ _/ /  ___ / /_/ /_(_) ___/__  __ _  ___ ___ / __ \___  _______ /  _/__  / _ | / /  (_) _/__ / /_(_)_ _  ___ 
/// __/ _ `/ // (_-</ -_)\ \/ _ \/ _ `/ _ `/ _ \/ -_) __/ __/ / /__/ _ \/  ' \/ -_|_-</ /_/ / _ \/ __/ -_)/ // _ \/ __ |/ /__/ / _/ -_) __/ /  ' \/ -_)
//\__/\_,_/\_,_/___/\__/___/ .__/\_,_/\_, /_//_/\__/\__/\__/_/\___/\___/_/_/_/\__/___/\____/_//_/\__/\__/___/_//_/_/ |_/____/_/_/ \__/\__/_/_/_/_/\__/ 
//                        /_/        /___/                                                                                                             
//
///////////////////////////////////////////////////////////////////////
// Yes the PKnessness Graffeti is indeed necessary
#pragma once
// #include "CANMsg.h"
#include <cerrno>
#include <stdint.h>
//Those zephyr heading files provide the API used below
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <vector>
#include <functional>
//The whole idea of this handler is to deal with multiple motors and devices, comprehensively.
//This Can handler used provided API from zephyr, which are in the form of usable zephyr functions like can_start(), can_send()...

//the rules for callback assignments..
struct ExactCallback {
    uint32_t id;
    std::function<void(const can_frame*)> func;
};//particular CAN ID---> particular callback function.

struct RangeCallback {
    uint32_t start_id;
    uint32_t end_id;
    std::function<void(const can_frame*)> func;// std::function is a type of function, inside: the function has no output and has parameter can_frame pointer
    //the function can be assigned later
};// All the msg with CAN ID in this range ---> particular callback function.

class CANHandler{
    
public:

    ~CANHandler() {
    if (filter_id >= 0) {
        can_remove_rx_filter(dev_, filter_id);
    }
}
    enum CANBus{CANBUS_1, CANBUS_2, NOBUS}; // Somewhat vestigal, makes porting mbedOS code a little easier - Dil 

    //Constructor here
    explicit CANHandler(const struct device* can_dev)
        : dev_(can_dev)//send the device pointer to the dev_
    {
        k_msgq_init(&rx_msgq_, rx_msgq_buf_, sizeof(struct can_frame), RX_QUEUE_DEPTH);//Using zephyr API to initialize the message queue.
    }
    
    int init() {
        if (!device_is_ready(dev_)) {
            return -ENODEV;
        }//using API to check if device is ready

        const struct can_filter catch_all = {.id =0, .mask = 0, .flags = 0};//set up the filter: only standartd message are allowed into the message queue.  catch_all is the actual filter

        filter_id = can_add_rx_filter_msgq(dev_, &rx_msgq_, &catch_all);//message come form which device. which message queue to put the filtered message. 
        //This function is the actual application of the previous filter.
        if (filter_id < 0) {
            return filter_id;
        }
        return can_start(dev_);//start the device. CAN controller
    }

    void registerCallback(uint32_t id, std::function<void(const can_frame*)> func) {//for particular CAN ID, particular callback function
        exact_.push_back({id, std::move(func)});//ExactCallBack structure pushed into.
    }

    void registerCallback(uint32_t start_id, uint32_t end_id, std::function<void(const can_frame*)> func) {
        range_.push_back({start_id, end_id, std::move(func)});
    }

    void readAllCan() {
        struct can_frame raw;//a piece of CAN message
        //CANMsg rxMsg;
        struct can_frame rxMsg;

        while (k_msgq_get(&rx_msgq_, &raw, K_NO_WAIT) == 0) {//To read message from the message queue, if successful return 0: continue to pull out the messages.
            // rxMsg.msg = raw;
            rxMsg = raw;

            for (auto &cb : exact_) {//&cb is the rules from the exact_ vector. they are different structures with particular id and function
                // if (rxMsg.msg.id == cb.id) {
                if (rxMsg.id == cb.id) {//if the CAN ID corresponds, use the proper function
                    cb.func(&rxMsg);
                }
            }
            for (auto &cb : range_) {
                // if (rxMsg.msg.id >= cb.start_id && rxMsg.msg.id <= cb.end_id) {
                if (rxMsg.id >= cb.start_id && rxMsg.id <= cb.end_id) {//if the CAN ID in this range
                    cb.func(&rxMsg);
                }
            }
        }
    }

    bool rawSend(uint32_t id, const uint8_t bytes[], uint8_t length = 8) {//three input value for the CAN message sent
        // CANMsg tx(id, bytes, length);
        struct can_frame tx = {
            .id = id,
            .dlc = length,
            .flags = 0,
        };
        memcpy(tx.data,bytes,length);//constructing a CAN message in the memory
        int ret = can_send(dev_, &tx, K_MSEC(100), nullptr, nullptr);//The device determines the CAN controller
        return ret == 0;
    }

    enum can_state getState() const {
    enum can_state state;
    struct can_bus_err_cnt err_cnt;
    can_get_state(dev_, &state, &err_cnt);
    return state;
}


private:
    static constexpr int RX_QUEUE_DEPTH = 128;//largest number of queue

    const struct device* dev_;//pointer towards the zephyr CAN device
    int filter_id = -1;

    struct k_msgq rx_msgq_;//The actual message queue. It stores CAN frames(which are intact CAN message)
    char rx_msgq_buf_[RX_QUEUE_DEPTH * sizeof(struct can_frame)] __aligned(4);

    std::vector<ExactCallback> exact_;//see the two structures above
    std::vector<RangeCallback> range_;
};