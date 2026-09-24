#include <Arduino.h>
#include "FIFObuf.h"
#include "DRV8214.h"

#define I2C_FREQUENCY 100000 // I2C frequency
#define SCL_PIN 1            // SCL pin
#define SDA_PIN 2            // SDA pin

#define IPROPI_RESISTOR 4700    // Value in Ohms of the resistor connected to IPROPI pi
#define NUM_RIPPLES 7           // NO. current ripples per output shaft revolution
#define RED_RATIO 64            // Reduction ratio - best guess is 1:16 so 16
//#define MAX_RPM 400             // will be around 1500 but set at 400 to be safe
#define MAX_RPM 800             // will be around 1500 but set at 400 to be safe
#define INTERNAL_MOTOR_R 2.5957 // 13.1 before we used the LCR bridge
#define NFAULT5 36              // GPIO pin 36 for nfault5 --> is low when stall detected
//#define MOTOR_CURRENT 0.8F      // Current supplied to the motor
#define MOTOR_CURRENT 1.5F      // Current supplied to the motor                    
#define MOTOR_VOLTAGE 0.0F      // I set this to 0 as we never use this since we use current controlled            
//#define MOTOR_VOLTAGE 5.0F      // I set this to 0 as we never use this since we use current controlled
#define DRIVER_ID_5 5           // Driver ID of motor driver 5
#define DRIVER_ID_4 4           // Driver ID of motor driver 4
#define DRIVER_ID_3 3           // Driver ID of motor driver 3
#define DRIVER_ID_2 2           // Driver ID of motor driver 2
#define DRIVER_ID_1 1           // Driver ID of motor driver 1

uint16_t MOTOR_SPEED = 0;       // I set this to 0 as we never use this since we use current controlled

// Timing
hw_timer_t *timer10ms = nullptr;
volatile bool tick10ms = false;

// ISR
void IRAM_ATTR onTimer10ms() {
  tick10ms = true;
}

// Grasps
enum graspName {
    PACK = -1,      // OPEN -> PACK for putting in a box.
    OPEN = 0,
    POWER = 1, 
    TRIPOD = 2,
    PINCH = 3,
    POINTER = 4,
    ROCK = 5,
    WAVE = 6,
};
unsigned long lastGrasp = 0;

// Motor 
enum motorDir {
    FORWARD, 
    REVERSE, 
    BRAKE,
    STATIC,
};
struct motorCommand {
    uint        motor;
    motorDir    direction;
    int         delay;
};
FIFObuf<motorCommand> motorQueue(100);

// DRV8214
DRV8214_Config cfg;
void setConfig()
{
    cfg.control_mode = PWM;                  // Control mode of the driver (PWM, PH_EN)
    cfg.I2CControlled = true;                // I2C Control of the driver (0: disabled, 1: enabled)
    cfg.regulation_mode = CURRENT_FIXED;     // Control mode of the driver (CURRENT_FIXED, CURRENT_CYCLES, SPEED, VOLTAGE)
    cfg.voltage_range = false;               // Expected applied supply voltage range to the attached motor (0: 0V-15.7V, 1: 0V-3.92V)
    cfg.ovp_enabled = false;                 // Overvoltage protection (0: disabled, 1: enabled)
    cfg.current_reg_mode = 3;                // We don't ever want to exceed given current values - Stall mode of the driver (0: no current regulation, 1: current regulation during inrush, 2: current regulation at all times, 3: current regulation at all times)
    cfg.stall_enabled = true;                // CORRECT - Stall detection (0: disabled, 1: enabled) --> NOT RELIABLE ENOUGH
    cfg.stall_behavior = true;               // DOESNT MATTER AS BUILT A BETTER STALL DETECTOR THAT BRAKES MOTOR - Stall behavior of the driver (0: outputs disable, 1: outputs continue to drive current)
    cfg.bridge_behavior_thr_reached = false; // N/A FOR US - Bridge behavior when ripple threshold is reached (0: H-bridge stays enabled, 1: H-bridge is disabled)
    cfg.soft_start_stop_enabled = false;     // Soft start/stop enabled if you dont want inrush current
    cfg.inrush_duration = 22;                // Measured in lab correctly (21.8 but has to be an int) - Inrush duration in ms (between 5ms and 6.7s)
    cfg.kmc = 5;                             // Did testing to find optimal value
    cfg.kmc_scale = 4;                       // Did testing to find optimal value
    cfg.verbose = false;                     // Enable verbose mode for debugging
    return;
}

// Initialize the driver with the I2C address, driver ID, and hardware dependant values
// address, driverID ,IPROPI resistor,ripples/rev,internal motor resisstance should be around 13.1 ohms, reduction ratio, max RPM = 12,000 - put 4,000 as limit for safety
DRV8214 motorDriver5(DRV8214_I2C_ADDR_ZZ, DRIVER_ID_5, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver4(DRV8214_I2C_ADDR_Z0, DRIVER_ID_4, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver3(DRV8214_I2C_ADDR_01, DRIVER_ID_3, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver2(DRV8214_I2C_ADDR_0Z, DRIVER_ID_2, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver1(DRV8214_I2C_ADDR_00, DRIVER_ID_1, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);

// Prototypes
void queueGrasp(graspName grasp);
void execMotorCommmand(motorCommand &cmd);

void setup()
{
    Serial.begin(115200);                        // SET BAUD RATE
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY); // for I2C communication
    delay(2000);
    setConfig();

    // Timer
    timer10ms = timerBegin(0, 80, true);
    timerAttachInterrupt(timer10ms, &onTimer10ms, true);
    timerAlarmWrite(timer10ms, 10000, true);
    timerAlarmEnable(timer10ms);

    Serial.print("STARTING UP");
    delay(1000);

    // Initialize motor driver
   // motorDriver5.init(cfg);
    motorDriver4.init(cfg);
    motorDriver3.init(cfg);
    motorDriver2.init(cfg);
    motorDriver1.init(cfg);

    motorDriver5.setInternalVoltageReference(3.3F);
    motorDriver4.setInternalVoltageReference(3.3F);
    motorDriver3.setInternalVoltageReference(3.3F);
    motorDriver2.setInternalVoltageReference(3.3F);
    motorDriver1.setInternalVoltageReference(3.3F);
    motorDriver5.getSenseResistor();
    delay(1000);

    motorDriver5.enableStallInterrupt();
    motorDriver4.enableStallInterrupt();
    motorDriver3.enableStallInterrupt();
    motorDriver2.enableStallInterrupt();
    motorDriver1.enableStallInterrupt();
    delay(2000);

    queueGrasp(OPEN);
    queueGrasp(PACK);
    queueGrasp(OPEN);
}

void loop()
{
    if (tick10ms)
    {
        tick10ms = false;
        static uint32_t delay = 0;
        while (delay <= 0 && motorQueue.size() > 0)
        {
            motorCommand mc = motorQueue.pop();
            delay = mc.delay;
            execMotorCommmand(mc);
        }
        if (delay >= 10)
        {
            delay = delay - 10;
        }
    }

    // rotate over grasps every 10 seconds
    static int grasp = 1;
    static bool graspOpen = false;
    if (millis() - lastGrasp > 10000)
    {
        lastGrasp = millis();
        if (graspOpen)
        {
            queueGrasp((graspName) grasp);
            grasp++;
            if (grasp > 6)
            {
                grasp = 1;
            }
        }
        else
        {
            queueGrasp(OPEN);
        }
        graspOpen = !graspOpen;
    }

    // serial input for grasps
    if (Serial.available())
    {
        int input = Serial.parseInt();
        while (Serial.available())
        {
            Serial.read();
        }
        switch (input)
        {
            case OPEN:
                queueGrasp(OPEN);
                break;
            case POWER:
                queueGrasp(POWER);
                break;
            case TRIPOD:
                queueGrasp(TRIPOD);
                break;
            case PINCH:
                queueGrasp(PINCH);
                break;
            case POINTER:
                queueGrasp(POINTER);
                break;
            case ROCK:
                queueGrasp(ROCK);
                break;
            case WAVE:
                queueGrasp(WAVE);
                break;
            default:
                break;
        }
    }
}

void queueGrasp(graspName grasp)
{
    switch (grasp) {
        case PACK:
            motorQueue.push(motorCommand {1, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  2000});
            motorQueue.push(motorCommand {1, BRAKE,  0});
            motorQueue.push(motorCommand {0, STATIC,  10});
        case OPEN:
            motorQueue.push(motorCommand {1, REVERSE, 0});
            motorQueue.push(motorCommand {2, REVERSE, 0});
            motorQueue.push(motorCommand {3, REVERSE, 0});
            motorQueue.push(motorCommand {4, REVERSE, 0});
            motorQueue.push(motorCommand {5, REVERSE, 0});
            motorQueue.push(motorCommand {0, STATIC,  2000});
            motorQueue.push(motorCommand {1, BRAKE,  0});
            motorQueue.push(motorCommand {2, BRAKE,  0});
            motorQueue.push(motorCommand {3, BRAKE,  0});
            motorQueue.push(motorCommand {4, BRAKE,  0});
            motorQueue.push(motorCommand {5, BRAKE,  0});
            motorQueue.push(motorCommand {0, STATIC,  10});
            break;  
        case POWER:
            motorQueue.push(motorCommand {2, FORWARD, 0});
            motorQueue.push(motorCommand {3, FORWARD, 0});
            motorQueue.push(motorCommand {4, FORWARD, 0});
            motorQueue.push(motorCommand {5, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  1500});
            motorQueue.push(motorCommand {1, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  1000});
            motorQueue.push(motorCommand {1, BRAKE,  0});
            motorQueue.push(motorCommand {2, BRAKE,  0});
            motorQueue.push(motorCommand {3, BRAKE,  0});
            motorQueue.push(motorCommand {4, BRAKE,  0});
            motorQueue.push(motorCommand {5, BRAKE,  0});
            motorQueue.push(motorCommand {0, STATIC,  10});
            break;
        case TRIPOD:
            motorQueue.push(motorCommand {1, FORWARD, 0});
            motorQueue.push(motorCommand {2, FORWARD, 0});
            motorQueue.push(motorCommand {3, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  810});
            motorQueue.push(motorCommand {1, BRAKE,  0});
            motorQueue.push(motorCommand {2, BRAKE,  0});
            motorQueue.push(motorCommand {3, BRAKE,  0});
            motorQueue.push(motorCommand {0, STATIC,  10});
            break;
        case PINCH:
            motorQueue.push(motorCommand {1, FORWARD, 0});
            motorQueue.push(motorCommand {2, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  810});
            motorQueue.push(motorCommand {1, BRAKE,  0});
            motorQueue.push(motorCommand {2, BRAKE,  0});
            motorQueue.push(motorCommand {0, STATIC,  10});
            break;
        case POINTER:
            motorQueue.push(motorCommand {1, FORWARD, 0});
            motorQueue.push(motorCommand {3, FORWARD, 0});
            motorQueue.push(motorCommand {4, FORWARD, 0});
            motorQueue.push(motorCommand {5, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  2000});
            motorQueue.push(motorCommand {1, BRAKE,  0});
            motorQueue.push(motorCommand {3, BRAKE,  0});
            motorQueue.push(motorCommand {4, BRAKE,  0});
            motorQueue.push(motorCommand {5, BRAKE,  0});
            motorQueue.push(motorCommand {0, STATIC,  10});
        case ROCK:
            motorQueue.push(motorCommand {3, FORWARD, 0});
            motorQueue.push(motorCommand {4, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  1500});
            motorQueue.push(motorCommand {1, FORWARD, 0});
            motorQueue.push(motorCommand {0, STATIC,  2000});
            motorQueue.push(motorCommand {1, BRAKE,  0});
            motorQueue.push(motorCommand {3, BRAKE,  0});
            motorQueue.push(motorCommand {4, BRAKE,  0});
            motorQueue.push(motorCommand {0, STATIC,  10});
            break;
        case WAVE:
            for (int i = 0; i < 3; i++)
            {
                motorQueue.push(motorCommand {2, FORWARD, 0});
                motorQueue.push(motorCommand {3, FORWARD, 0});
                motorQueue.push(motorCommand {4, FORWARD, 0});
                motorQueue.push(motorCommand {5, FORWARD, 0});
                motorQueue.push(motorCommand {0, STATIC,  500});
                motorQueue.push(motorCommand {2, BRAKE,  0});
                motorQueue.push(motorCommand {3, BRAKE,  0});
                motorQueue.push(motorCommand {4, BRAKE,  0});
                motorQueue.push(motorCommand {5, BRAKE,  0});
                motorQueue.push(motorCommand {0, STATIC,  10});
                motorQueue.push(motorCommand {2, REVERSE, 0});
                motorQueue.push(motorCommand {3, REVERSE, 0});
                motorQueue.push(motorCommand {4, REVERSE, 0});
                motorQueue.push(motorCommand {5, REVERSE, 0});
                motorQueue.push(motorCommand {0, STATIC,  500});
                motorQueue.push(motorCommand {2, BRAKE,  0});
                motorQueue.push(motorCommand {3, BRAKE,  0});
                motorQueue.push(motorCommand {4, BRAKE,  0});
                motorQueue.push(motorCommand {5, BRAKE,  0});
                motorQueue.push(motorCommand {0, STATIC,  10});
            }
            break;
        default:
            break;
    }
}

void execMotorCommmand(motorCommand &cmd)
{
    switch (cmd.direction) {
        case FORWARD:
            switch (cmd.motor) {
                case 1:
                    motorDriver1.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 2:
                    motorDriver2.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 3:
                    motorDriver3.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 4:
                    motorDriver4.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 5:
                    motorDriver5.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                default:
                    break;
            }
            break;
        case REVERSE:
            switch (cmd.motor) {
                case 1:
                    motorDriver1.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 2:
                    motorDriver2.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 3:
                    motorDriver3.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 4:
                    motorDriver4.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                case 5:
                    motorDriver5.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
                    break;
                default:
                    break;
            }
            break;
        case BRAKE:
            switch (cmd.motor) {
                case 1:
                    motorDriver1.brakeMotor();
                    break;
                case 2:
                    motorDriver2.brakeMotor();
                    break;
                case 3:
                    motorDriver3.brakeMotor();
                    break;
                case 4:
                    motorDriver4.brakeMotor();
                    break;
                case 5:
                    motorDriver5.brakeMotor();
                    break;
                default:
                    break;
            }
            break;
        case STATIC:
            break;
        default:
            break;
    }
}













void loop_()
{ // STALL DETECTION FOR ALL 5 MOTORS

    // Static arrays to store history for each motor
    static int ripple_history_1[2] = {0};
    static int ripple_history_2[2] = {0};
    static int ripple_history_3[2] = {0};
    static int ripple_history_4[2] = {0};
    static int ripple_history_5[2] = {0};

    static int history_index_1 = 0;
    static int history_index_2 = 0;
    static int history_index_3 = 0;
    static int history_index_4 = 0;
    static int history_index_5 = 0;

    static int sample_count_1 = 0;
    static int sample_count_2 = 0;
    static int sample_count_3 = 0;
    static int sample_count_4 = 0;
    static int sample_count_5 = 0;

    const int BRAKE_THRESHOLD = 1; // DIFFERENCE BETWEEN VALUES TO TRIGGER THE BRAKING

    // Get ripple counts for all motors
    int ripplecount_1 = motorDriver1.getRippleCount();
    int ripplecount_2 = motorDriver2.getRippleCount();
    int ripplecount_3 = motorDriver3.getRippleCount();
    int ripplecount_4 = motorDriver4.getRippleCount();
    int ripplecount_5 = motorDriver5.getRippleCount();

    delay(100);

    // Print all ripple counts
    //Serial.print("Motor 1: ");
    Serial.print(ripplecount_1);
    Serial.print(",");
    //Serial.print("Motor 2: ");
    Serial.print(ripplecount_2);
    Serial.print(",");
    //Serial.print("Motor 3: ");
    Serial.print(ripplecount_3);
    Serial.print(",");
    //Serial.print("Motor 4: ");
    Serial.print(ripplecount_4);
    Serial.print(",");
    //Serial.print("Motor 5: ");
    Serial.println(ripplecount_5);

    
    // Process Motor 2
    ripple_history_2[history_index_2] = ripplecount_2;
    history_index_2 = (history_index_2 + 1) % 2;
    if (sample_count_2 < 2)
        sample_count_2++;

    if (sample_count_2 >= 2)
    {
        int total_difference = 0;
        int oldest_index = history_index_2;

        for (int i = 0; i < sample_count_2 - 1; i++)
        {
            int current_index = (oldest_index + i) % 2;
            int next_index = (oldest_index + i + 1) % 2;
            total_difference += abs(ripple_history_2[next_index] - ripple_history_2[current_index]);
        }

        if (total_difference < BRAKE_THRESHOLD)
        {
            motorDriver2.brakeMotor();
            //Serial.println("BRAKE APPLIED TO MOTOR 2!");
        }
    }

    // Process Motor 3
    ripple_history_3[history_index_3] = ripplecount_3;
    history_index_3 = (history_index_3 + 1) % 2;
    if (sample_count_3 < 2)
        sample_count_3++;

    if (sample_count_3 >= 2)
    {
        int total_difference = 0;
        int oldest_index = history_index_3;

        for (int i = 0; i < sample_count_3 - 1; i++)
        {
            int current_index = (oldest_index + i) % 2;
            int next_index = (oldest_index + i + 1) % 2;
            total_difference += abs(ripple_history_3[next_index] - ripple_history_3[current_index]);
        }

        if (total_difference < BRAKE_THRESHOLD)
        {
            motorDriver3.brakeMotor();
            //Serial.println("BRAKE APPLIED TO MOTOR 3!");
        }
    }

    // Process Motor 4
    ripple_history_4[history_index_4] = ripplecount_4;
    history_index_4 = (history_index_4 + 1) % 2;
    if (sample_count_4 < 2)
        sample_count_4++;

    if (sample_count_4 >= 2)
    {
        int total_difference = 0;
        int oldest_index = history_index_4;

        for (int i = 0; i < sample_count_4 - 1; i++)
        {
            int current_index = (oldest_index + i) % 2;
            int next_index = (oldest_index + i + 1) % 2;
            total_difference += abs(ripple_history_4[next_index] - ripple_history_4[current_index]);
        }

        if (total_difference < BRAKE_THRESHOLD)
        {
            motorDriver4.brakeMotor();
            //Serial.println("BRAKE APPLIED TO MOTOR 4!");
        }
    }

    // Process Motor 5 (your original code)
    ripple_history_5[history_index_5] = ripplecount_5;
    history_index_5 = (history_index_5 + 1) % 2;
    if (sample_count_5 < 2)
        sample_count_5++;

    if (sample_count_5 >= 2)
    {
        int total_difference = 0;
        int oldest_index = history_index_5;

        for (int i = 0; i < sample_count_5 - 1; i++)
        {
            int current_index = (oldest_index + i) % 2;
            int next_index = (oldest_index + i + 1) % 2;
            total_difference += abs(ripple_history_5[next_index] - ripple_history_5[current_index]);
        }

        if (total_difference < BRAKE_THRESHOLD)
        {
            motorDriver5.brakeMotor();
            //Serial.println("BRAKE APPLIED TO MOTOR 5!");
        }
    }

    if(ripplecount_1>1000){
            motorDriver1.brakeMotor();
        }

    delay(10); // Small delay to prevent overwhelming the system


    if (Serial.available())
    {
        int input = Serial.parseInt();

        while (Serial.available())
        {
            Serial.read();
        }


        

        switch (input)
        {

        case 1:
            // CLOSES MOTOR 5
            motorDriver5.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); // Start continuous reverse motion
            break;

        case 2:
            // OPENS MOTOR 5
            motorDriver5.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 3:
            // BRAKES MOTOR 5
            motorDriver5.brakeMotor(); // Stop the motor
            break;

        case 4: // CLOSES  MOTOR 4
            motorDriver4.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 5:  // OPENS MOTOR 4
            motorDriver4.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); // Start continuous reverse motion
            break;

        case 6: // BRAKES MOTOR 4
            motorDriver4.brakeMotor(); // Stop the motor
            break;

        case 7: // CLOSES MOTOR 3
            motorDriver3.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 8:   // OPENS MOTOR 3
            motorDriver3.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); // Start continuous reverse motion
            break;

        case 9:  // BRAKES MOTOR 3
            motorDriver3.brakeMotor(); // Stop the motor
            break;

        case 10: // CLOSES MOTOR 2 
            motorDriver2.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 11:   // OPENS MOTOR 2
            motorDriver2.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); // Start continuous reverse motion
            break;

        case 12:                       // BRAKES MOTOR 2
            motorDriver2.brakeMotor(); // Stop the motor
            break;

        case 13: // CLOSES MOTOR 1 
            motorDriver1.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 14: // OPENS MOTOR 1
            motorDriver1.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); // Start continuous reverse motion
            break;

        case 15: // BRAKES MOTOR 1
            motorDriver1.brakeMotor(); // Stop the motor
            break;
        

        case 16: // CLOSES GRASP
            motorDriver5.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver3.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver2.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 17: // OPENS GRASP
            motorDriver5.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver3.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver2.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 18: // MIDDLE FINGER CODE
            motorDriver5.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver3.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver2.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(8000);
            motorDriver3.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(7000);
            motorDriver5.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver2.turnReverse(MOTOR_SPEED,MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 19: // 2 finger code
            motorDriver5.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver3.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver2.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(8000);
            motorDriver3.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver2.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(7000);
            motorDriver5.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 20: // ASL - LETTER A
            motorDriver5.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver3.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            motorDriver2.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(8000);
            motorDriver5.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver4.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            delay(10);
            motorDriver3.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            motorDriver2.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT);
            break;

        case 21:
            motorDriver1.brakeMotor();
            motorDriver2.brakeMotor();
            motorDriver3.brakeMotor();
            motorDriver4.brakeMotor();
            motorDriver5.brakeMotor();
            break;
            


        case 88: // RESET ALL RIPPLE COUNTS
            Serial.println("reset ripple:");
            motorDriver5.resetRippleCounter();
            motorDriver4.resetRippleCounter();
            motorDriver3.resetRippleCounter();
            motorDriver2.resetRippleCounter();
            motorDriver1.resetRippleCounter();
            break;



        case 30: // STORE RIPPLE COUNT FOR CHOSEN MOTOR IN AN ARRAY THAT PRINTS WHEN FULL
        {
            static int forwardRippleCounts[20];
            static int arrayIndex = 0;

            if (arrayIndex >= 20)
            {
                Serial.println("Array full! Showing all 10 stored values:");
                for (int i = 0; i < 20; i++)
                {
                    Serial.print("Index ");
                    Serial.print(i);
                    Serial.print(": ");
                    Serial.println(forwardRippleCounts[i]);
                }
                Serial.println("Reset ESP32 to clear array.");
            }
            else
            {
                int currentRipples = motorDriver5.getRippleCount();
                Serial.print("get ripple: ");
                Serial.println(currentRipples);

                forwardRippleCounts[arrayIndex] = currentRipples;
                Serial.print("Stored in forwardRippleCounts[");
                Serial.print(arrayIndex);
                Serial.print("]: ");
                Serial.println(forwardRippleCounts[arrayIndex]);

                arrayIndex++;
            }
            break;
        }

        default:
            Serial.println("you pressed something else");
            break;
        }
    }

}
