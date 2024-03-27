#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Macro functions
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10

// PINS
#define LED_BUILTIN 33
#define SERVO_LEFT_Front 12
#define SERVO_RIGHT_Front 13
#define SERVO_LEFT_Middle 14
#define SERVO_RIGHT_Middle 15
#define SERVO_LEFT_Back 16
#define SERVO_RIGHT_Back 17

// Robot Params
#define Rwidth 10.0f // example values
#define Rlength 20.0f
#define Rwheeldistance (Rlength / 2.0f)

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_SERVO_LEFT_Front LEDC_CHANNEL_2
#define PWM_SERVO_RIGHT_Front LEDC_CHANNEL_3
#define PWM_SERVO_LEFT_Middle LEDC_CHANNEL_4
#define PWM_SERVO_RIGHT_Middle LEDC_CHANNEL_5
#define PWM_SERVO_LEFT_Back LEDC_CHANNEL_6
#define PWM_SERVO_RIGHT_Back LEDC_CHANNEL_7

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

#define PWM_Servo_min  24.0f
#define PWM_Servo_max  300.0f

geometry_msgs__msg__Twist msg;

// Function forward declarations
void setupPins();
void setupRos();
void cmd_vel_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);

// Main
void appMain(void *arg)
{
    setupPins();
    setupRos();
}

void setupPins()
{
    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure 6 PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[6] = {
        {.channel = PWM_SERVO_LEFT_Front,
         .duty = 0,
         .gpio_num = SERVO_LEFT_Front,
         .speed_mode = PWM_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},

        {.channel = PWM_SERVO_RIGHT_Front,
         .duty = 0,
         .gpio_num = SERVO_RIGHT_Front,
         .speed_mode = PWM_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},

        {.channel = PWM_SERVO_LEFT_Middle,
         .duty = 0,
         .gpio_num = SERVO_LEFT_Middle,
         .speed_mode = PWM_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},

        {.channel = PWM_SERVO_RIGHT_Middle,
         .duty = 0,
         .gpio_num = SERVO_RIGHT_Middle,
         .speed_mode = PWM_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},

        {.channel = PWM_SERVO_LEFT_Back,
         .duty = 0,
         .gpio_num = SERVO_LEFT_Back,
         .speed_mode = PWM_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},

        {.channel = PWM_SERVO_RIGHT_Back,
         .duty = 0,
         .gpio_num = SERVO_RIGHT_Back,
         .speed_mode = PWM_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
    };

    for (int i = 0; i < 6; i++)
    {
        ledc_channel_config(&ledc_channel[i]);
    }
}

void setupRos()
{
    // Micro ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "N10_Servo_Controller", "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/n10/servo_cmd_vel"));

    // create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(FRAME_TIME),
        timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME * 1000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

// We don't really need the callback, because msg is set anyway
void cmd_vel_callback(const void *msgin)
{
    //    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;
    //    printf("Message received: %f %f\n", msg->linear.x, msg->angular.z);
}

// Each frame, check msg data and set PWM channels accordingly
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

    gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));

    if (timer == NULL)
    {
        return;
    }

    // Use linear.x for forward value and angular.z for rotation
    float linearX = constrain(msg.linear.x, -1.0f, 1.0f);
    float linearY = constrain(msg.linear.y, -1.0f, 1.0f);
    float angularZ = constrain(msg.angular.z, -1.0f, 1.0f);
    uint32_t PWM_Duty_SLF, PWM_Duty_SRF, PWM_Duty_SLM, PWM_Duty_SRM, PWM_Duty_SLB, PWM_Duty_SRB;

        //------------------------------------------
        //              logic
        //-----------------------------------------
        // angular movement
        const float pi = 3.1415926535897932385f;
        float sr = Rwidth / 2.0f;
        float br = sqrtf(Rwheeldistance * Rwheeldistance + (Rwidth / 2.0f) * (Rwidth / 2.0f));
        float beta = atanf(sr / Rwheeldistance);
        // angulars
        // 0 = left front; 1 = right front; 2 = left middle; 3 = right middle; 4 = left back; 5 = right back;
        float betaServos[6];
        betaServos[0] = beta;
        betaServos[1] = 2.0f * pi - beta;
        betaServos[2] = 0.5f * pi;
        betaServos[3] = 2.0f * pi - betaServos[2];
        betaServos[4] =  pi - beta;
        betaServos[5] =  pi + beta;
        // velocity
        float velocityMotors[6];
        velocityMotors[0] = fabsf(angularZ);
        velocityMotors[1] = velocityMotors[0];
        velocityMotors[2] = (sr / br) * velocityMotors[1];
        velocityMotors[3] = velocityMotors[2];
        velocityMotors[4] = velocityMotors[1];
        velocityMotors[5] = velocityMotors[0];


        // End velocity
        float endVelocity[6];

        float x[6];

        float y[6];

        if (angularZ != 0.0f)
        {
            for (int i = 0; i < 6; i++)
            {
                x[i] = sinf(betaServos[i]) * velocityMotors[i] * ((angularZ > 0) ? 1.0f : -1.0f) + linearX;
                y[i] = cosf(betaServos[i]) * velocityMotors[i] * ((angularZ > 0) ? 1.0f : -1.0f) + linearY;
                endVelocity[i] = sqrtf(y[i] * y[i] + x[i] * x[i]);
            }
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                x[i] = linearX;
                y[i] = linearY;
                endVelocity[i] = sqrtf(y[i] * y[i] + x[i] * x[i]);
            }
        }

        // final movement
        float alpha[6];

        for (int i = 0; i < 6; i++)
        {
            if (x[i] > 0.0f && y[i] > 0.0f)
            {
                alpha[i] = atanf(fabsf(x[i]) / fabsf(y[i]));
            }
            else if (x[i] < 0.0f && y[i] > 0.0f)
            {
                alpha[i] = 2*pi - atanf(fabsf(x[i]) / fabsf(y[i]));
            }
            else if (x[i] < 0.0f && y[i] < 0.0f)
            {
                alpha[i] = pi + atanf(fabsf(x[i]) / fabsf(y[i]));
            }
            else if (x[i] > 0.0f && y[i] < 0.0f)
            {
                alpha[i] =  pi - atanf(fabsf(x[i]) / fabsf(y[i]));
            }
            else if(x[i] > 0.0f && y[i] == 0.0f){
                alpha[i] = 0.5 * pi;
            }
            else if(x[i] < 0.0f && y[i] == 0.0f){
                alpha[i] = 1.5*pi;
            }
            else if(x[i] == 0.0f && y[i] > 0.0f){
                alpha[i] = 0;
            }
            else if(x[i] == 0.0f && y[i] < 0.0f){
                alpha[i] = pi;
            }else{
                alpha[i]= 0.5 * pi;
            }
            // converting for Servos and convert Wheel direction
            if (alpha[i] > pi)
            {
                alpha[i] -= pi;
                endVelocity[i] *= -1.0f;
            }
        }

        PWM_Duty_SLF =(uint32_t) fmap(alpha[0], 0.0f,pi, PWM_Servo_min, PWM_Servo_max);
        PWM_Duty_SRF =(uint32_t) fmap(alpha[1], 0.0f,pi, PWM_Servo_min, PWM_Servo_max);
        PWM_Duty_SLM =(uint32_t) fmap(alpha[2], 0.0f,pi, PWM_Servo_min, PWM_Servo_max);
        PWM_Duty_SRM =(uint32_t) fmap(alpha[3], 0.0f,pi, PWM_Servo_min, PWM_Servo_max);
        PWM_Duty_SLB =(uint32_t) fmap(alpha[4], 0.0f,pi, PWM_Servo_min, PWM_Servo_max);
        PWM_Duty_SRB =(uint32_t) fmap(alpha[5], 0.0f,pi, PWM_Servo_min, PWM_Servo_max);


    //----------------L--------------------------

    // Each servo has a channel for movement
    ledc_set_duty(PWM_MODE, PWM_SERVO_LEFT_Front, PWM_Servo_max);//12
    ledc_set_duty(PWM_MODE, PWM_SERVO_RIGHT_Front, PWM_Servo_min);//13
    ledc_set_duty(PWM_MODE, PWM_SERVO_LEFT_Middle, PWM_Duty_SLM);
    ledc_set_duty(PWM_MODE, PWM_SERVO_RIGHT_Middle, PWM_Duty_SRM);
    ledc_set_duty(PWM_MODE, PWM_SERVO_LEFT_Back, PWM_Duty_SLB);
    ledc_set_duty(PWM_MODE, PWM_SERVO_RIGHT_Back, PWM_Duty_SRB);

    ledc_update_duty(PWM_MODE, PWM_SERVO_LEFT_Front);
    ledc_update_duty(PWM_MODE, PWM_SERVO_RIGHT_Front);
    ledc_update_duty(PWM_MODE, PWM_SERVO_LEFT_Middle);
    ledc_update_duty(PWM_MODE, PWM_SERVO_RIGHT_Middle);
    ledc_update_duty(PWM_MODE, PWM_SERVO_LEFT_Back);
    ledc_update_duty(PWM_MODE, PWM_SERVO_RIGHT_Back);
}

// Helper functions
// -------------------------------------------------------------------

float fmap(float val, float in_min, float in_max, float out_min, float out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}