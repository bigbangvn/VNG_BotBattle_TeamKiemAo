/*
 * For VNG 11+1 Challenge
 * trongbangvp@gmail.com
 * sumobot ver 1.2 //use new sample code (new PIN MAP)
 * 2016-08-28
 * https://developer.mbed.org/compiler/#nav:/mbed_blinky/main.cpp;
 */

#include "mbed.h"
//#include "ble/BLE.h"
//#include "ble/services/iBeacon.h"
//#include "ble/services/UARTService.h"

//#define DEBUG
//#define HW_V2

DigitalOut  led1(p7);
#ifndef HW_V2
DigitalOut  DIR_L(p25);
DigitalOut  PWM_L(p23);
DigitalOut  DIR_R(p28);
DigitalOut  PWM_R(p24);
#else
DigitalOut  IN_1(p9);
DigitalOut  IN_2(p16);
DigitalOut  IN_3(p17);
DigitalOut  IN_4(p18);
#endif

//// Sensor
#define NOISE 8

#ifdef DEBUG
Serial      pc(p10, p11);
#endif
Ticker      ticker;
AnalogIn    analog_ir(p1);
DigitalIn   SEN_UP_R(p2);
DigitalIn   SEN_UP_L(p3);
DigitalIn   SEN_DOWN(p4);
DigitalIn   SEN_IR(p6);

//bangnt - add
AnalogIn    analog_ir_right(p5);

#define ON      0
#define OFF     1
#define VREF    3300.0  //mV

//// Motor
#define STOP    0
#define UP      1
#define DOWN    2
#define LEFT    3
#define RIGHT   4
uint8_t     g_dir_left = 0,     g_dir_right = 0;
uint8_t     g_time_pwm = 0,     g_pwm_left = 0,     g_pwm_right = 0;
uint16_t    g_time_led = 0;
uint8_t     g_sts_car = STOP;



//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * @param    dir: UP or DOWN; speed: 0-100.
 * BangNT: actual: control motor right, 0 -> up, 1 -> down. WTF???
 * @return   none.
 */
void motor_left(uint8_t dir, uint8_t speed)
{
#ifndef HW_V2
    DIR_L = dir;
    g_pwm_left = speed;
#else
    g_dir_left = dir;
    g_pwm_left = speed;
#endif
}
//////////////////////////////////////////////////
/*
 * @param    dir: UP or DOWN; speed: 0-100.
 * BangNT: fact: control motor left, 0 -> up, 1 -> down. WTF??
 * @return   none.
 */
void motor_right(uint8_t dir, uint8_t speed)
{
#ifndef HW_V2
    DIR_R = dir;
    g_pwm_right = speed;
#else
    g_dir_right = dir;
    g_pwm_right = speed;
#endif
}
//////////////////////////////////////////////////
/*
 * @param    speed: 0-100.
 * BangNT: rotate Counter Clockwise. Stupid function name !!
 * @return   none.
 */
void rotator_left(uint8_t speed)
{
#ifndef HW_V2
    DIR_L = 0;
    g_pwm_left = speed;
    DIR_R = 1;
    g_pwm_right = speed;
    g_sts_car = LEFT;
#else
    g_dir_left  = 0;
    g_dir_right = 1;
    g_pwm_left  = speed;
    g_pwm_right = speed;
#endif
}
//////////////////////////////////////////////////
/*
 * @param    speed: 0-100.
 * BangNT: rotate Clockwise (after install new battery, whether something changed, i don't remember). who the fuck named this function??
 * @return   none.
 */
void rotator_right(uint8_t speed)
{
#ifndef HW_V2
    DIR_L = 1;
    g_pwm_left = speed;
    DIR_R = 0;
    g_pwm_right = speed;
    g_sts_car = RIGHT;
#else
    g_dir_left  = 1;
    g_dir_right = 0;
    g_pwm_left  = speed;
    g_pwm_right = speed;
#endif
}
//////////////////////////////////////////////////
/*
 * @param    speed_left, speed_right: 0-100.
 * @return   none.
 */
void move_up(uint8_t speed_left, uint8_t speed_right)
{
#ifndef HW_V2
    DIR_L = 0;
    g_pwm_left = speed_left;
    DIR_R = 0;
    g_pwm_right = speed_right;
    g_sts_car = UP;
#else
    g_dir_left  = 0;
    g_dir_right = 0;
    g_pwm_left  = speed_left;
    g_pwm_right = speed_right;
#endif
}
//////////////////////////////////////////////////
/*
 * @param    speed_left, speed_right: 0-100.
 * @return   none.
 */
void move_down(uint8_t speed_left, uint8_t speed_right)
{
#ifndef HW_V2
    DIR_L = 1;
    g_pwm_left = speed_left;
    DIR_R = 1;
    g_pwm_right = speed_right;
    g_sts_car = DOWN;
#else
    g_dir_left  = 1;
    g_dir_right = 1;
    g_pwm_left  = speed_left;
    g_pwm_right = speed_right;
#endif
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
/*
 * @param    none.
 * @return   Distance (cm), 1-70 cm.
 */
uint16_t sensor_ir(void)
{
    uint16_t    adc;
    uint16_t    cm;
    uint16_t    sensor = 0, i;
    
    for(i=0; i<NOISE; i++) {
        adc = analog_ir.read_u16();
        adc = 750-adc;
        if      (adc > 60000) cm = 1;
        else if (adc > 600) cm = 0;
        else if (adc > 550) cm = adc/8;
        else if (adc > 500) cm = adc/10;
        else if (adc > 450) cm = adc/12;
        else if (adc > 400) cm = adc/14;
        else if (adc > 350) cm = adc/16;
        else if (adc > 300) cm = adc/18;
        else if (adc > 200) cm = adc/16;
        else if (adc > 200) cm = adc/14;
        else if (adc > 150) cm = adc/12;
        else if (adc > 100) cm = adc/10;
        else if (adc >  60) cm = adc/9;
        else if (adc >  30) cm = adc/8;
        else if (adc >   0) cm = adc/7;
        
        wait(0.001);
        sensor = sensor + cm;
        if(cm == 0) break;
        cm = sensor/NOISE;
    }
    
#ifdef DEBUG
    pc.printf("\r\n head ir sensor: %d adc, %d cm", adc, cm);
#endif
    return cm;
}

// 1 -> 30 cm: adc value 300 -> 600
bool sensor_ir_right_detected_enemy(void)
{
    uint16_t adc = analog_ir_right.read_u16();
    if(adc > 300 && adc < 600)
        return true;
    return false;
}

uint16_t sensor_ir_right(void)
{
    uint16_t    adc;
    uint16_t    cm;
    uint16_t    sensor = 0, i;
    
    for(i=0; i<NOISE; i++) {
        adc = analog_ir_right.read_u16();
        adc = 750-adc;
        if      (adc > 60000) cm = 1;
        else if (adc > 600) cm = 0;
        else if (adc > 550) cm = adc/8;
        else if (adc > 500) cm = adc/10;
        else if (adc > 450) cm = adc/12;
        else if (adc > 400) cm = adc/14;
        else if (adc > 350) cm = adc/16;
        else if (adc > 300) cm = adc/18;
        else if (adc > 200) cm = adc/16;
        else if (adc > 200) cm = adc/14;
        else if (adc > 150) cm = adc/12;
        else if (adc > 100) cm = adc/10;
        else if (adc >  60) cm = adc/9;
        else if (adc >  30) cm = adc/8;
        else if (adc >   0) cm = adc/7;
        
        wait(0.001);
        sensor = sensor + cm;
        if(cm == 0) break;
        cm = sensor/NOISE;
    }
    
#ifdef DEBUG
    pc.printf("\r\n Right ir sensor: %d adc, %d cm", adc, cm);
#endif
    return cm;
}

//////////////////////////////////////////////////
/*
 * @param    none.
 * @return   ON or OFF.
 */
uint8_t sensor_up_left(void)
{
    uint16_t i, sensor = 0;
    
    for(i=0; i<NOISE; i++) {
        wait(0.001);
        sensor = sensor + SEN_UP_L;
    }
    if(sensor > NOISE/2)   return OFF;
    else                   return ON;
}
//////////////////////////////////////////////////
/*
 * @param    none.
 * @return   ON or OFF.
 */
uint8_t sensor_up_right(void)
{
    uint16_t i, sensor = 0;
    
    for(i=0; i<NOISE; i++) {
        wait(0.001);
        sensor = sensor + SEN_UP_R;
    }
    if(sensor > NOISE/2)   return OFF;
    else                   return ON;
}
//////////////////////////////////////////////////
/*
 * @param    none.
 * @return   ON or OFF.
 */
uint8_t sensor_down(void)
{
    uint16_t i, sensor = 0;
    
    for(i=0; i<NOISE; i++) {
        wait(0.001);
        sensor = sensor + SEN_DOWN;
    }
    if(sensor > NOISE/2)   return OFF;
    else                   return ON;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void periodicCallback(void)
{
#ifndef HW_V2
    if(g_time_pwm < g_pwm_left)    PWM_L = 1; else PWM_L = 0;
    if(g_time_pwm < g_pwm_right)   PWM_R = 1; else PWM_R = 0;
#else
    if(g_dir_left == 1) {
        IN_1 = 0;
        if(g_time_pwm < g_pwm_left)    IN_2 = 1; else IN_2 = 0;
    }
    else {
        IN_2 = 0;
        if(g_time_pwm < g_pwm_left)    IN_1 = 1; else IN_1 = 0;
    }
    
    if(g_dir_right == 0) {
        IN_3 = 0;
        if(g_time_pwm < g_pwm_right)    IN_4 = 1; else IN_4 = 0;
    }
    else {
        IN_4 = 0;
        if(g_time_pwm < g_pwm_right)    IN_3 = 1; else IN_3 = 0;
    }
#endif
    g_time_pwm++; if(g_time_pwm >= 100) {g_time_pwm = 0;}
    g_time_led++; if(g_time_led >= 1000) {g_time_led = 0; led1 = !led1;}   //DIR_L = !DIR_L; DIR_R = !DIR_R;
}

/////////////////////
//SUMOBOT ALG
////////////////////

#define TIME_STEP 0.01
#define ENEMY_DISTANCE_UNKNOW 255
#define ENEMY_DISTANCE_MIN 1
#define ENEMY_DISTANCE_MAX 30
#define DISTANCE_ALWAYS_ATTACK 1 //maybe less than 8

#define USE_TWO_IR_SENSOR_AHEAD 0 //two ir sensor ahead to detect enemy more precisely, so we can increase roration speed
#define USE_MODE_HIGH_VOLTAGE 1

#define MOTOR_MAX_SPEED 100
static uint8_t TB_SPEED_ATTACK = MOTOR_MAX_SPEED;

#if USE_MODE_HIGH_VOLTAGE

static uint8_t TB_SPEED_ATTACK_MIN = 50;
static uint8_t TB_SPEED_ROTATE = 27; //35, 31, 27
static uint8_t TB_SPEED_ROTATE_NEAR_ENEMY = TB_SPEED_ROTATE + 6;
static uint8_t TB_SPEED_MOVE_BACKWARD = 50;
static uint8_t TB_SPEED_MOVE_FORWARD = 55;

#else

static uint8_t TB_SPEED_ATTACK_MIN = 50;
#if USE_TWO_IR_SENSOR_AHEAD
static uint8_t TB_SPEED_ROTATE = 60;
#else
static uint8_t TB_SPEED_ROTATE = 50;
#endif
static uint8_t TB_SPEED_MOVE_BACKWARD = 80;
static uint8_t TB_SPEED_MOVE_FORWARD = 90;

#endif


struct BotState {
    uint8_t vel_left;
    uint8_t vel_right;
    uint8_t enemy_distance;
    uint8_t sensor_up_left;
    uint8_t sensor_up_right;
    uint8_t sensor_down;
    BotState(): vel_left(0),
    vel_right(0),
    enemy_distance(ENEMY_DISTANCE_UNKNOW)
    {
        printf("BotState constructor\n");
    }
    void setVelocity(uint8_t left, uint8_t right)
    {
    }
};
static BotState _currentBotState;//not using

typedef enum BotStrategy {
    STRATEGY_MOVE_TO_SAFE_POS, //highest priority
    STRATEGY_MOVE_TO_EDGE,
    STRATEGY_ROTATE_SEARCH_ENEMY,
    STRATEGY_ATTACK,
    STRATEGY_SURVIVE_WHEN_BE_PUSHED_TO_EDGE
} BotStrategy;
static BotStrategy _currentStrategy = STRATEGY_ROTATE_SEARCH_ENEMY;
static BotStrategy _prevStrategy = _currentStrategy;
static float _currentStrategyTime = 0.0;

//Maximum time for a state
#define MAX_TIME_ROTATE_SEARCH_ENEMY 5.0

typedef enum AttackingMode {
    ATTACKING_MODE1, //just move forward to attack
    ATTACKING_MODE2  //move backward -> attack -> backward -> attack
} AttackingMode;

static AttackingMode _currentAttackingMode = ATTACKING_MODE1; //will be changed if it feels can't defeat the enemy
static float _contactEnemyTime = 0; //time contacted with enemy

typedef enum RotateDirection {
    DIRECTION_CW = 1,
    DIRECTION_CCW = -1
} RotateDirection;

static RotateDirection _rotateSearchEnemyDirection = DIRECTION_CCW;//1: rotate right (CW), -1 : rotate left (CCW)

uint8_t detected_enemy(uint8_t distance)
{
    if(distance >= ENEMY_DISTANCE_MIN && distance <= ENEMY_DISTANCE_MAX)
        return 1;
    return 0;
}

uint8_t detected_enemy_with_two_sensor(uint8_t distance1, uint8_t distance2)
{
    if(distance1 >= ENEMY_DISTANCE_MIN && distance1 <= ENEMY_DISTANCE_MAX)
        return 1;
    if(distance2 >= ENEMY_DISTANCE_MIN && distance2 <= ENEMY_DISTANCE_MAX)
        return 1;
    return 0;
}

void changeRotateSearchEnemyDirection()
{
    _rotateSearchEnemyDirection = (RotateDirection) -_rotateSearchEnemyDirection;
}

void rotateSearchEnemy(uint8_t rotationSpeed)
{
    if(_rotateSearchEnemyDirection == DIRECTION_CW)
        rotator_right(rotationSpeed);
    else
        rotator_left(rotationSpeed);
}
void maximizeAllSpeed()
{
    //TB_SPEED_ROTATE =        MOTOR_MAX_SPEED;
    TB_SPEED_MOVE_BACKWARD = MOTOR_MAX_SPEED;
    TB_SPEED_MOVE_FORWARD =  MOTOR_MAX_SPEED;
}
void stop()
{
    motor_left(0,0);
    motor_right(0,0);
    move_up(0, 0);
}

int main(void)
{
    wait(0.1);
    //Init Hardware
    SEN_UP_L.mode(PullUp);
    SEN_UP_R.mode(PullUp);
    SEN_DOWN.mode(PullUp);
    SEN_IR.mode(PullUp);
    led1 = 1;
#ifndef HW_V2
    DIR_L = 0; DIR_R = 0; PWM_L = 0;  PWM_R = 0;
#endif
#ifdef DEBUG
    pc.baud(9600);
    pc.printf("\n\r# Sumo Car\n\r");
#endif
    //Init interupt Timer
    ticker.attach(periodicCallback, 0.00015);
    wait(0.2);
    
    
#if 0//Test motor
    /*
     rotator_right(50);
     wait(3.0);
     rotator_left(50);
     wait(3.0);
     stop();
     
     motor_right(0, 50); //actual: left move up (forward)
     wait(3.0);
     motor_right(1, 50);//actual: left move down (backward)
     wait(3.0);
     stop();
     
     motor_left(0, 50); //actual: right move up
     wait(3.0);
     motor_left(1, 50);//actual: right move down
     wait(3.0);
     
     stop();
     */
    
    while(1)
    {
        uint8_t sen_distance = sensor_ir();
        if(sen_distance >= ENEMY_DISTANCE_MIN && sen_distance <= ENEMY_DISTANCE_MAX)
        {
            float threadholdMaxSpeed = 0.25 * (ENEMY_DISTANCE_MAX - ENEMY_DISTANCE_MIN);
            float distanceFloat = sen_distance;
            float percent = (distanceFloat - ENEMY_DISTANCE_MAX)/(threadholdMaxSpeed - ENEMY_DISTANCE_MAX);
            
            uint8_t attackingSpeed = TB_SPEED_ATTACK_MIN + percent * (TB_SPEED_ATTACK - TB_SPEED_ATTACK_MIN);
            
            move_up(attackingSpeed, attackingSpeed);
        } else
        {
            stop();
        }
        /*
         if(sen_distance_right > 1 && sen_distance_right < 30)
         {
         move_up(40,40);
         } else
         {
         stop();
         }
         */
    }
    
#else
    
    
    //main loop
    while(1)
    {
        if(_prevStrategy != _currentStrategy)
        {
            _prevStrategy = _currentStrategy;
            _currentStrategyTime = 0.0;
        }
        _currentStrategyTime += TIME_STEP;
        
        uint8_t sen_distance = sensor_ir();
        uint8_t sen_distance_right = sensor_ir_right();
        uint8_t sen_up_left = sensor_up_left();
        uint8_t sen_up_right = sensor_up_right();
        uint8_t sen_down  = sensor_down();
        
        _currentBotState.enemy_distance = sen_distance;
        _currentBotState.sensor_up_left = sen_up_left;
        _currentBotState.sensor_up_right = sen_up_right;
        _currentBotState.sensor_down = sen_down;
        
        //highest priority: keep on safe position
        if(_currentStrategy != STRATEGY_ATTACK &&
           _currentStrategy != STRATEGY_SURVIVE_WHEN_BE_PUSHED_TO_EDGE) //we have special behaviour for attacking/survive state. Otherwise, always try keep in safe pos
        {
            if(sen_up_left == OFF || sen_up_right == OFF || sen_down == OFF)
            {
                _currentStrategy = STRATEGY_MOVE_TO_SAFE_POS;
                _contactEnemyTime = 0; //reset
            }
        }
        
        //if contact too long -> we're weak -> in crease all speed to max
        if(_contactEnemyTime > 20)
        {
            maximizeAllSpeed();
        }
        
        switch (_currentStrategy) {
            case STRATEGY_MOVE_TO_SAFE_POS:
                if(sen_up_left == ON && sen_up_right == ON && sen_down == ON)
                {
                    stop();
#if USE_TWO_IR_SENSOR_AHEAD
                    if(detected_enemy_with_two_sensor(sen_distance, sen_distance_right))
#else
                        if(detected_enemy(sen_distance))
#endif
                        {
                            _currentStrategy = STRATEGY_ATTACK;
                        } else
                        {
                            _currentStrategy = STRATEGY_ROTATE_SEARCH_ENEMY;
                        }
                } else
                {
                    if(sen_down == OFF)
                    {
                        if(sen_up_left == ON && sen_up_right == ON)
                        {
                            move_up(TB_SPEED_MOVE_FORWARD, TB_SPEED_MOVE_FORWARD);
                        } else if(sen_up_left == ON)
                        {
                            //rotator_left(TB_SPEED_ROTATE); //change 'right' to 'left' after install new battery
                            //motor_left(1, TB_SPEED_ROTATE);
                            motor_right(0, TB_SPEED_ROTATE);
                        } else
                        {
                            //rotator_right(TB_SPEED_ROTATE); //change 'left' -> 'right' after install new battery
                            //motor_right(1, TB_SPEED_ROTATE);
                            motor_left(0, TB_SPEED_ROTATE);
                        }
                    } else //up left or up right off -> move down
                    {
                        move_down(TB_SPEED_MOVE_BACKWARD, TB_SPEED_MOVE_BACKWARD);
                    }
                }
                break;
            case STRATEGY_MOVE_TO_EDGE:
#if USE_TWO_IR_SENSOR_AHEAD
                if(detected_enemy_with_two_sensor(sen_distance, sen_distance_right))
#else
                    if(detected_enemy(sen_distance))
#endif
                    {
                        _currentStrategy = STRATEGY_ATTACK;
                    } else
                        if(sen_up_left == ON && sen_up_right == ON)
                        {
                            move_up(TB_SPEED_MOVE_FORWARD, TB_SPEED_MOVE_FORWARD);
                        } else
                        {
                            stop();
                            _currentStrategy = STRATEGY_MOVE_TO_SAFE_POS;
                        }
                break;
            case STRATEGY_ROTATE_SEARCH_ENEMY:
                if(_currentStrategyTime > MAX_TIME_ROTATE_SEARCH_ENEMY)
                {
                    changeRotateSearchEnemyDirection();
                    stop();
                    _currentStrategy = STRATEGY_MOVE_TO_EDGE;
                } else
#if USE_TWO_IR_SENSOR_AHEAD
                    if(detected_enemy_with_two_sensor(sen_distance, sen_distance_right))
#else
                        if(detected_enemy(sen_distance))
#endif
                        {
                            changeRotateSearchEnemyDirection();
                            stop();
                            _currentStrategy = STRATEGY_ATTACK;
                        } else
                        {
                            uint8_t rotationSpeed = TB_SPEED_ROTATE;
                            if(detected_enemy(sen_distance_right))
                            {
                                _rotateSearchEnemyDirection = DIRECTION_CW;
                                rotationSpeed = TB_SPEED_ROTATE_NEAR_ENEMY;
                            } else
                            {
                                //just keep rotating current direction
                            }
                            rotateSearchEnemy(rotationSpeed);
                        }
                break;
            case STRATEGY_ATTACK:
#if USE_TWO_IR_SENSOR_AHEAD
                if(detected_enemy_with_two_sensor(sen_distance, sen_distance_right))
#else
                    if(detected_enemy(sen_distance))
#endif
                    {
                        bool shouldContinueAttack = true;
                        if(sen_down == OFF) //Detect if we're attacking but was pushed to edge because the enemy is stronger
                        {
                            shouldContinueAttack = false;
                            _currentStrategy = STRATEGY_SURVIVE_WHEN_BE_PUSHED_TO_EDGE;
                        } else if(sen_up_left == OFF || sen_up_right == OFF)
                        {
                            if(sen_distance > DISTANCE_ALWAYS_ATTACK)
                            {
                                shouldContinueAttack = false;
                                _currentStrategy = STRATEGY_MOVE_TO_SAFE_POS;
                            } else
                            {
                                //just keep attacking because enemy is ahead (fix case enemy fake the black-line)
                            }
                        }
                        
                        if(shouldContinueAttack)
                        {
                            if(_currentAttackingMode == ATTACKING_MODE1) //mode 1
                            {
                                float threadholdMaxSpeed = 0.25 * (ENEMY_DISTANCE_MAX - ENEMY_DISTANCE_MIN);
                                float distanceFloat = sen_distance;
                                float percent = (distanceFloat - ENEMY_DISTANCE_MAX)/(threadholdMaxSpeed - ENEMY_DISTANCE_MAX);
                                
                                uint8_t attackingSpeed = TB_SPEED_ATTACK_MIN + percent * (TB_SPEED_ATTACK - TB_SPEED_ATTACK_MIN);
                                
                                move_up(attackingSpeed, attackingSpeed);
                                _contactEnemyTime += 0.01;
                                if(_contactEnemyTime > 30) //can't defeat enemy -> change attacking mode
                                {
                                    //_currentAttackingMode = ATTACKING_MODE2;
                                }
                            } else                                      //mode 2
                            {
                                if(_contactEnemyTime < 4.0)
                                {
                                    move_up(TB_SPEED_ATTACK, TB_SPEED_ATTACK);
                                    if(sen_distance < 5)
                                    {
                                        _contactEnemyTime += 0.01;
                                    }
                                } else
                                {
                                    move_down(TB_SPEED_MOVE_BACKWARD, TB_SPEED_MOVE_BACKWARD);
                                    if(sen_distance > 10)
                                    {
                                        _contactEnemyTime = 0; //start attack again
                                    }
                                }
                            }
                        }
                        
                    } else
                    {
                        _contactEnemyTime = 0;//reset
                        stop();
                        if(sen_down == OFF || sen_up_left == OFF || sen_up_right == OFF)
                        {
                            _currentStrategy = STRATEGY_MOVE_TO_SAFE_POS;
                        } else
                        {
                            _currentStrategy = STRATEGY_ROTATE_SEARCH_ENEMY;
                        }
                    }
                break;
            case STRATEGY_SURVIVE_WHEN_BE_PUSHED_TO_EDGE:
                maximizeAllSpeed();
                if(sen_up_left == OFF)
                {
                    rotator_right(MOTOR_MAX_SPEED);
                } else
                {
                    rotator_left(MOTOR_MAX_SPEED);
                }
                wait(0.5); //(should be tweak) Wait to survive this position.
                //Then try to move to safe position
                _currentStrategy = STRATEGY_MOVE_TO_SAFE_POS;
                break;
            default:
                _currentStrategy = STRATEGY_MOVE_TO_SAFE_POS;
                break;
        }
    }
#endif
    return 0;
}

