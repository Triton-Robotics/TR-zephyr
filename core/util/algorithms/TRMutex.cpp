//Not going to lie, I started this, but actually have no clue what I'm doing, so I'm leaving it for now 


// #include "TRMutex.h"
// #include "zephyr/kernel.h"

 
// // K_MUTEX_DEFINE statically initializes the mutex via macro (not a C++ constructor),
// // so there's no static-init-order hazard the way there would be with a plain global object.
// K_MUTEX_DEFINE(print_mutex);
// static std::queue<char> buffer;

// // Push a null-terminated string into the buffer as a single locked operation, so
// // concurrent callers can't have their characters interleaved.
// static void push_locked(const char* s) {
//     k_mutex_lock(&print_mutex, K_FOREVER);
//     for (int i = 0; s[i] != '\0'; i++) {
//         buffer.push(s[i]);
//     }
//     k_mutex_unlock(&print_mutex);
// }


// TRMutex::TRMutex() {
//     // print_code_thread.start(loop);
//     priorityIndicator = DEFAULT;

//     thread_id = k_thread_create( //This returns the id of the new thread, except I don't understand it at all??? 
//         &thread_data, //Pointer to thread
//         thread_stack, //Pointer to stack space
//         K_THREAD_STACK_SIZEOF(thread_stack), //Stack size
//         loop, //Thread entry function, Claude said to loop???
//         this, nullptr,nullptr, // 3 entry point parameters, wtf? I used Claude for this part
//         THREAD_PRIORITY, // In the name
//         0, // thread options, pretty sure zero is default, not sure what this does tbh
//         K_NO_WAIT // Scheduling delay, pretty sure we don't have one 
//     );
// }

// void TRMutex::updatePriority(priorityLevels desiredLevel) {
//     priorityIndicator = desiredLevel;
// }

// //----print no line

// //string/char inputs
// void TRMutex::print(char statement[], priorityLevels priority) {
//     if (priority <= priorityIndicator) {
//         int i=0;
//         while(statement[i] != '\0') {
//             buffer.push(statement[i]);
//             i++;
//         }
//     }       
// }

// //ints
// void TRMutex::print(int integer, priorityLevels priority) {
//     if (priority <= priorityIndicator) { 
//         char temp[11];
//         sprintf(temp, "%d", integer);
//         print(temp, priority);
//     }
// }

// //duplicate from printff method in main.h
// // void TRMutex::printff(const char* format, priorityLevels priority, ...) {
// //     if (priority <= priorityIndicator) {
// //         char temp[50];
// //         va_list args;
// //         va_start (args, format);
// //         vsnprintf (temp, 50, format, args);
// //         print(temp, priority);
// //         va_end (args);
// //     }
// // }

// //------println 

// //string/char inputs
// void TRMutex::println(char statement[], priorityLevels priority) {
//     if (priority <= priorityIndicator) {
//         int i=0;
//         while(statement[i] != '\0') {
//             buffer.push(statement[i]);
//             i++;
//         }
//         buffer.push('\n');
//     }
// }


// //ints
// void TRMutex::println(int integer, priorityLevels priority) {
//     if (priority <= priorityIndicator) {
//         char temp[11];
//         sprintf(temp, "%d", integer);
//         print(temp, priority);
//         buffer.push('\n');
//     }
// }

// void TRMutex::loop() {
//     while (true) {

//         if (!buffer.empty()) {
//             MUTEX.lock();
//             printf("%c", buffer.front());
//             buffer.pop();
//             MUTEX.unlock();
//         }
        
//         k_msleep(1);
//     }
// }


