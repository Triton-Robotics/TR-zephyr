/* CAN message container.
 * Provides "<<" (append) and ">>" (extract) operators to simplify
 * adding/getting data items to/from a CAN message.
 * Usage is similar to the C++ io-stream operators.
 * Data length of CAN message is automatically updated when using "<<" or ">>" operators.
 */
#pragma once
#include <zephyr/drivers/can.h>
#include <cstring>
#include <cstdint> 
 

// I wrote this really early into porting to Zephyr. It's honestly kind of
// vestigial, since can_frame is really quite robust - Dil 


class CANMsg  
{
public:
    struct can_frame msg; //can_frame is a struct from Zephyr with fields id, dlc, data, and flags

    // Clears CAN message content    
    void clear() {
        std::memset(&msg, 0, sizeof(msg));
    };

    // Data Length Code to Bytes, trivial 1:1 conversion on F446RE, but when we move to H533RE it
    // can let us do the non-linear mapping; talk with auto/electronics/Uncs to see if that's
    // ever useful - Dil 
    uint8_t len() const {return can_dlc_to_bytes(msg.dlc); } 

    // Creates Empty CAN Msg
    CANMsg() { clear();}
 
    // Creates CAN message with specific content.
    CANMsg(uint32_t id, const uint8_t *data_in, uint8_t len = 8, bool extended = false) {
        clear();
        msg.id = id;
        msg.dlc = can_bytes_to_dlc(len);
        std::memcpy(msg.data, data_in, len);
        if (extended) {
            msg.flags |= CAN_FRAME_IDE; // Identifer Extension in Zephyr, not really useful for
                                        // our hardware right now  
        }
    }
 
    // Append operator: Appends data (value) to CAN message
    template<class T>
    CANMsg &operator<<(const T val) {
        uint8_t current_len = len();
        __ASSERT(sizeof(T) <= current_len, "CAN frame underflow"); // Check for space
        memcpy(&msg.data[current_len], &val, sizeof(T)); 
        msg.dlc = can_bytes_to_dlc(current_len + sizeof(T)); // Convert new byte # to dlc
        return *this;
    }
 
    // Extract operator: Extracts data (value) from CAN message
    template<class T>
    CANMsg &operator>>(T& val) {
        uint8_t current_len = len();
        //printf("Size:%d\tmax length:%d\n", sizeof(T), current_len);
        __ASSERT(sizeof(T) <= current_len, "CAN frame underflow");
        std::memcpy(&val, msg.data, sizeof(T));
        uint8_t remaining = current_len - sizeof(T);
        std::memmove(msg.data, msg.data + sizeof(T), remaining);
        msg.dlc = can_bytes_to_dlc(remaining);
        return *this;
    }
};
 
 
            
