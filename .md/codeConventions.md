
It is important to not only pay attention to errors, but also compiler and IDE warnings.

## GitHub
+ Always `pull` before every work session.
+ `commit` and `push` often. Ideally, every work session. This way changes are easy to track and do not pile up.
+ No force `push`ing. Contact your leads/TAs if you are having conflicts.
<br>

## Memory
Memory becomes especially important when running on an embedded device.
+ Watch for memory leaks.
+ Use [references](https://www.geeksforgeeks.org/references-in-cpp/) if you are passing too much data throughout functions.
<br>

## Code comments
+ Ideally, your code should be concise and readable enough to not warrant needing any comments; however, given the inherent complexity of our codebase, there will inevitably be times where inline comments are indeed necessary. Something to consider is that there will be a lot of people varying in backgrounds and expertise, and so generally comments should be targeted towards the average member: generally familiar with C++ and the codebase, but not intimately familiar with the particular file you're writing.  
+ When it comes to comments in class headers, ideally you should use doxygen to document the functions, as shown below; however for simpler functions, a simple inline comment is fine. 

```C++
/**
    @brief A helper function that calls a particular SerialBase's irq_handler safely. 
    
    @details Needed because Zephyr's UART Drivers (specifically uart_irq_callback_user_data_set(uart_dev,&irq_trampoline, this)) 
    needs a static function. However, irq_handler itself can't be static, since it needs to act on a particular instance of SerialBase, so
    we need this static helper function to bounce us to the right SerialBase Object 
     */
    static void irq_trampoline(const struct device *dev, void *user_data) {
        static_cast<SerialBase *>(user_data)->irq_handler();
    }

    /**
    @brief Checks if Uart RX and TX are ready and then writes to ring buffer, or transfers data out respectively

    @warning Always call this through irq_trampoline. Otherwise you'll get an invalid input error. Read irq_trampoline's comments for
    the details
     */
    void irq_handler();
```

<br>

## Variables
+ Use `camelCase` for variable naming.
+ Initialize variables the closest to the scope they will be used in.
+ Prefer [list initialization](https://stackoverflow.com/questions/18222926/what-are-the-advantages-of-list-initialization-using-curly-braces) over any other alternative:
```C++
int value {10};
```
+ Reduce the need for typecasting by correctly declaring the type of a variable. When necessary, do not use C-style casting `(int) var`, `int(var)`, use C++ `static_cast` instead.
```C++
float error {3.029};
int floorError {static_cast<int>(error)};
```
+ Global variables should only be used for physical sensors and actuator `objects` e.g., motors, can busses, remotes, etc.
+ Motor objects should have the `m_` prefix.
+ Avoid pointers unless absolutely necessary. [References](https://www.geeksforgeeks.org/references-in-cpp/) should be your default.
+ Constants should be uppercase and defined with `constexpr`.
```C++
constexpr int TIMEOUT_MS {400};
```
+ `static` variables should be prepended with the `s_` prefix.<br>
```C++
static int s_count;
```
+ Variables that hold data representing time should be postpended with the `_s`, `_m`, `_u`,... suffixes.
```C++
unsigned long dt_s;          // seconds
unsigned long timeStart_m;   // milliseconds
unsigned long elapsed_u;     // microseconds
```
+ Create temporary local (objects/variables)'s pointers/references when making several modifications to an array.
```C++
// ✅✅✅

    if(s_motorsExist[canBus][canID_0 / 4][canID_0 % 4]){
        DJIMotor* m_motor = s_allMotors[canBus][canID_0 / 4][canID_0 % 4];

        m_motor -> motorData[ANGLE]        = (int16_t)(receivedBytes[0] << 8 | receivedBytes[1]);
        m_motor -> motorData[VELOCITY]     = (int16_t)(receivedBytes[2] << 8 | receivedBytes[3]);
        m_motor -> motorData[TORQUE]       = (int16_t)(receivedBytes[4] << 8 | receivedBytes[5]);
        m_motor -> motorData[TEMPERATURE]  = (int16_t) receivedBytes[6];

        if(debug)
            m_motor -> printAllMotorData();

        m_motor -> timeSinceLastFeedback = us_ticker_read() / 1000 - m_motor -> timeOfLastFeedback;
        m_motor -> timeOfLastFeedback = us_ticker_read() / 1000;

        if(m_motor -> motorData[TEMPERATURE] > 80)
            printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n", msgID, m_motor -> motorData[TEMPERATURE], canBus + 1, m_motor -> motorID_0 + 1, m_motor -> name.c_str());

    }
```

```C++
// ❌❌❌

    if(s_motorsExist[canBus][canID_0 / 4][canID_0 % 4]){
        s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> motorData[ANGLE]        = (int16_t)(receivedBytes[0] << 8 | receivedBytes[1]);
        s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> motorData[VELOCITY]     = (int16_t)(receivedBytes[2] << 8 | receivedBytes[3]);
        s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> motorData[TORQUE]       = (int16_t)(receivedBytes[4] << 8 | receivedBytes[5]);
        s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> motorData[TEMPERATURE]  = (int16_t) receivedBytes[6];

        if(feedbackDebug)
            s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> printAllMotorData();

        s_allMotors [canBus][canID_0 / 4][canID_0 % 4] -> timeSinceLastFeedback = us_ticker_read() / 1000 - s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> timeOfLastFeedback;
        s_allMotors [canBus][canID_0 / 4][canID_0 % 4] -> timeOfLastFeedback = us_ticker_read() / 1000;
        
        if(s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> motorData[TEMPERATURE] > 80)
            printf("[WARNING] YOU HAVE A MOTOR [0x%x] ATTACHED THAT IS %d DEGREES CELSIUS ON BUS [%d] ID [%d], \"%s\" \n", msgID, s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> motorData[TEMPERATURE], canBus + 1, s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> motorID_0 + 1, s_allMotors[canBus][canID_0 / 4][canID_0 % 4] -> name.c_str());

    }
```
<br>

+ Avoid using doubles unless your Lead/TA explicitly tells you to. Floats are significantly faster to compute on our Nucleo (since it has an FPU), and generally the increased precision is superfluous. Be careful when using functions from the math library, as for example, functions like `fabs()`, `sin()`, and `sqrt()` all return doubles. Instead, append an f to the function to use the version that returns a float (i.e. `fabsf()`, `sinf()`, `sqrtf()`). Note that your clangd/compiler will have warnings when you build for implicit double promotions, which should help you catch these mistakes. 
## Functions
+ `static` functions should be prepended with the `s_` prefix.<br>
```C++
static void s_getFeedback(){/* ... */}
```
+ Use `inline` for very short and often called functions and operator overloading.
```C++
inline int operator>>(motorDataType data)                { return getData(data); }
inline void setSpeedPID(float kP, float kI, float kD)    { pidSpeed.setPID(kP, kI, kD); }
```
+ When needed, declare boolean debugging variables as default false parameters.
```C++
int calculateDerivative(int error, int lastError, int dt_m, bool debug = false){

    if(debug) {
        printf("e: %d\n", error);
        printf("le: %d\n", lastError);
        printf("de: %d\n", error - lastError);
        printf("dt (ms): %d\n", dt_m);
    }
    return (error - lastError) / dt_m;
}

int main(){

    calculateDerivative(6, 5, 1000);
    calculateDerivative(6, 5, 1000, true);

    return 0;

}
```
<br>

## Classes, Structs
+ When only data storage is needed use `struct`s. Only use `class`es when you need to store data and need member functions.
+ Constructor parameters should have the same name as member variables plus a `_` postpended
+ Prefer [constructor member initializer lists](https://www.learncpp.com/cpp-tutorial/constructor-member-initializer-lists/)
```C++
class Foo{
private:
    int x;
    int y;

public:
    explicit Foo(int x_ = 0, int y_ = 0) : x {x_}, y {y_} {
        printf("constructed\n");
    }
};
```
+ Try your best to follow the [Orthodox Canonical Class Form](https://www.francescmm.com/orthodox-canonical-class-form/). Note that a constructor with default arguments for all arguments also counts as a default construtor, since it can be called with no arguments.
+ Try your best to follow [Resource Acquisition is Initialization](https://en.wikipedia.org/wiki/Resource_acquisition_is_initialization):

"Resource allocation (or acquisition) is done during object creation (specifically initialization), by the constructor, while resource deallocation (release) is done during object destruction (specifically finalization), by the destructor. In other words, resource acquisition must succeed for initialization to succeed. Thus the resource is guaranteed to be held between when initialization finishes and finalization starts (holding the resources is a class invariant), and to be held only when the object is alive. Thus if there are no object leaks, there are no resource leaks."

<br>
<br>
<br

For any questions contact your lead on discord. 

