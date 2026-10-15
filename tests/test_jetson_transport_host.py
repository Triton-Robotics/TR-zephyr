"""Host regression for actual transport sources with a simulated Zephyr UART.

Run: python3 tests/test_jetson_transport_host.py
Does not replace a Zephyr firmware build or test real interrupt timing.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <deque>
#include <vector>
#include <stdexcept>
#include <sys/types.h>
struct device {};
struct USART_TypeDef {};
struct spi_dt_spec {};
struct k_mutex { int locks = 0; };
struct k_sem {};
struct k_thread { void (*entry)(void*,void*,void*){}; void* arg{}; };
struct ring_buf { std::deque<uint8_t> bytes; size_t capacity{}; };
inline bool in_isr=false, tx_on=false, rx_on=false, drain_on_unlock=false;
inline bool rx_pending=false;
inline std::deque<uint8_t> rx_bytes;
inline std::vector<uint8_t> wire;
inline void (*callback)(const device*,void*) = nullptr;
inline void* callback_arg=nullptr;
inline const device* uart_device=nullptr;
inline uint64_t clock_us=1000;
inline int sleep_ms=0, suspended=0;
struct EndIteration {};
#define K_FOREVER -1
#define K_NO_WAIT 0
#define K_MSEC(x) (x)
#define K_THREAD_STACK_DEFINE(name,size) char name[size]
#define PM_DEVICE_ACTION_RESUME 0
#define PM_DEVICE_ACTION_SUSPEND 1
inline void require(bool ok) { if(!ok) throw std::runtime_error("transport assertion failed"); }
inline void k_mutex_init(k_mutex*) {}
inline void k_mutex_lock(k_mutex* m,int) { require(!in_isr); ++m->locks; }
inline void k_mutex_unlock(k_mutex* m) { --m->locks; }
inline void k_sem_init(k_sem*,int,int) {}
inline void k_sem_take(k_sem*,int) { throw std::runtime_error("unexpected blocked write"); }
inline void k_sem_give(k_sem*) {}
inline void ring_buf_init(ring_buf* b,size_t n,uint8_t*) { b->capacity=n; }
inline bool ring_buf_is_empty(ring_buf* b) { return b->bytes.empty(); }
inline size_t ring_buf_space_get(ring_buf* b) { return b->capacity-b->bytes.size(); }
inline size_t ring_buf_put(ring_buf* b,const uint8_t* p,size_t n) {
 size_t i=0;while(i<n && ring_buf_space_get(b)) b->bytes.push_back(p[i++]);return i;
}
inline size_t ring_buf_get(ring_buf* b,uint8_t* p,size_t n) {
 size_t i=0;while(i<n && !b->bytes.empty()){p[i++]=b->bytes.front();b->bytes.pop_front();}return i;
}
inline bool k_is_in_isr(){return in_isr;}
inline int uart_irq_update(const device*){return 1;}
inline bool uart_irq_rx_ready(const device*){return rx_pending;}
inline bool uart_irq_tx_ready(const device*){return tx_on;}
inline int uart_fifo_read(const device*,uint8_t* p,int) {
 if(rx_bytes.empty()){rx_pending=false;return 0;}*p=rx_bytes.front();rx_bytes.pop_front();return 1;
}
inline int uart_fifo_fill(const device*,const uint8_t* p,int){wire.push_back(*p);return 1;}
inline void uart_poll_out(const device*,uint8_t c){wire.push_back(c);}
inline void uart_irq_rx_enable(const device*){rx_on=true;}
inline void uart_irq_rx_disable(const device*){rx_on=false;}
inline void uart_irq_tx_enable(const device*){tx_on=true;}
inline void uart_irq_tx_disable(const device*){tx_on=false;}
inline void pm_device_busy_set(const device*){}
inline void pm_device_busy_clear(const device*){}
inline int pm_device_action_run(const device*,int action){suspended += action==1;return 0;}
inline void uart_irq_callback_user_data_set(const device* d,void(*cb)(const device*,void*),void* arg){
 uart_device=d;callback=cb;callback_arg=arg;
}
inline void irq(){in_isr=true;callback(uart_device,callback_arg);in_isr=false;}
inline unsigned int irq_lock(){return 0;}
inline void irq_unlock(unsigned int){if(drain_on_unlock)while(tx_on)irq();}
inline void k_sleep(int ms){sleep_ms=ms;throw EndIteration{};}
inline void k_yield(){throw EndIteration{};}
inline uint64_t now_us(){return clock_us;}
inline void k_thread_create(k_thread* t,char*,int,void(*f)(void*,void*,void*),
 void* a,void*,void*,int,int,int){t->entry=f;t->arg=a;}
class Mutex {public:void lock(){} void unlock(){}};
'''
HARNESS = r'''
#include "fake.h"
#include <memory>
#include <string>
// Expose the two thread entry records to run one iteration without real threads.
#define private public
#include "util/communications/jetson/Jetson.h"
#undef private
#include <iostream>
int main(){
 device d;
 Jetson transport(&d);
 require(transport.read().stamp_us==0);
 Jetson::WriteState state{};
 state.pitch_angle_rads=.25f;state.yaw_angle_rads=.5f;state.activate_CV=1;
 transport.write(state);
 auto tick=[&](k_thread& t){try{t.entry(t.arg,nullptr,nullptr);}catch(EndIteration&){};};
 // Simulate TX IRQ preempting enable_output immediately when IRQs are restored.
 drain_on_unlock=true;
 tick(transport.m_write_tdata);
 require(wire.size()==40 && sleep_ms==10 && rx_on && !tx_on && suspended==0);
 require(wire[0]==0xbb && wire[6]==0xaa && wire[36]==0xee);
 for(auto pair: {std::pair<size_t,size_t>{0,6},{6,30},{36,4}}){
  unsigned sum=0;for(size_t i=pair.first+1;i<pair.first+pair.second;i++)sum+=wire[i];
  require((sum&255)==0);
 }
 tick(transport.m_write_tdata);
 require(wire.size()==80); // output must restart after the preceding idle IRQ
 // Validate command reception and ensure bad input cannot refresh its age.
 auto receive=[&](std::vector<uint8_t> bytes){
  rx_bytes.assign(bytes.begin(),bytes.end());rx_pending=true;irq();tick(transport.m_read_tdata);
 };
 std::vector<uint8_t> command(11);command[0]=0xcc;
 float yaw=.2f,pitch=.3f;
 std::memcpy(command.data()+1,&yaw,4);std::memcpy(command.data()+5,&pitch,4);
 command[10]=Jetson::calculateLRC(reinterpret_cast<char*>(command.data()+1),9);
 receive(command);require(transport.read().stamp_us==1000);
 require(transport.read().desired_pitch_rads==pitch);
 clock_us=2000;command[10]^=1;receive(command);
 require(transport.read().stamp_us==1000);
 require(rx_on && suspended==0);
 std::cout<<"PASS: telemetry pacing/packets, idle IRQ, TX restart, RX and valid-command timestamp\n";
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p / "fake.h").write_text(STUB)
    for name in ["stm32f446xx.h", "zephyr/kernel.h", "zephyr/drivers/uart.h",
                 "zephyr/pm/device.h", "zephyr/device.h",
                 "util/algorithms/mbedMutex.cpp", "util/algorithms/general_functions.h"]:
        header = p / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "fake.h"\n')
    (p / "main.cpp").write_text(HARNESS)
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-I", str(p), "-I", str(ROOT), "-I", str(ROOT / "core"),
                    str(p / "main.cpp"), str(ROOT / "core/util/communications/mbedSerial.cpp"),
                    str(ROOT / "core/util/communications/jetson/Jetson.cpp"), "-o", str(p / "test")], check=True)
    subprocess.run([str(p / "test")], check=True)
