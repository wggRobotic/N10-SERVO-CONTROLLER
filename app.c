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
#define Rwidth 10 // example values
#define Rlength 20
#define Rwheeldistance (Rlength / 2)

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_SERVO_LEFT_Front LEDC_CHANNEL_2
#define PWM_SERVO_RIGHT_Front LEDC_CHANNEL_3
#define PWM_SERVO_LEFT_Middle LEDC_CHANNEL_4
#define PWM_SERVO_RIGHT_Middle LEDC_CHANNEL_5
#define PWM_SERVO_LEFT_Back LEDC_CHANNEL_6
#define PWM_SERVO_RIGHT_Back LEDC_CHANNEL_7

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

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
    float linearX = constrain(msg.linear.x, -1, 1);
    float linearY = constrain(msg.linear.y, -1, 1);
    float angularY = constrain(msg.angular.z, -1, 1);

    uint32_t PWM_Duty_SLF, PWM_Duty_SRF, PWM_Duty_SLM, PWM_Duty_SRM, PWM_Duty_SLB, PWM_Duty_SRB;

    //------------------------------------------
    //              logic
    //-----------------------------------------
    // angular movement
   const float pi = 3.1415926535897932385;
    float sr = Rwidth / 2;
    float br = sqrtf(Rwheeldistance * Rwheeldistance + (Rwidth/2) * (Rwidth/2));
    float beta = atanf(sr/ Rwheeldistance);
    // angulars
    // 0 = left front; 1 = right front; 2 = left middle; 3 = right middle; 4 = left back; 5 = right back;
    float betaServos[6];
    betaServos[0] = beta;
    betaServos[1] = 2 * pi - beta;
    betaServos[2] = 0.5 * pi;
    betaServos[3] = 2 * pi - betaServos[2];
    betaServos[4] = 2*pi - betaServos[0];
    betaServos[5] = 2*pi - betaServos[1];
    // velocity
    float velocityMotors[6];
    velocityMotors[0] = fabsf(msg.angular.z);
    velocityMotors[1] = velocityMotors[0];
    velocityMotors[2] = (sr / br) * velocityMotors[1];
    velocityMotors[3] = velocityMotors[2];
    velocityMotors[4] = velocityMotors[1];
    velocityMotors[5] = velocityMotors[0];

    // Morph gamma and beta
    // Delta is gamma and betas morphed
    float dServos[6];
    // End velocity
    float endVelocity[6];

    float x[6];

    float y[6];

    if (msg.angular.z != 0)
    {
        for (int i = 0; i < 6; i++)
        {
            x[i] = cosf(betaServos[i]) * velocityMotors[i]  * ((msg.angular.z > 0) ? 1.0f : -1.0f) + msg.linear.x ;
            y[i] = sinf(betaServos[i]) * velocityMotors[i]  * ((msg.angular.z > 0) ? 1.0f : -1.0f) + msg.linear.y ;
            endVelocity[i] = sqrtf(y[i] * y[i] + x[i] * x[i]);
        }
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            x[i] = msg.linear.x;
            y[i] = msg.linear.y;
            endVelocity[i] = sqrtf(y[i] * y[i] + x[i] * x[i]);
        }
    }

    // final movement
    float alpha[6];

    for(int i =0;i<6;i++){
            if(x[i]>0 && y[i]>0){
                alpha[i]= atanf(fabsf(y[i])/fabsf(x[i]));
            }else if(x[i]<0 && y[i]>0){
                alpha[i]=pi - atanf(fabsf(y[i])/fabsf(x[i]));
            }else if(x[i]<0 && y[i]<0){
                alpha[i]=pi + atanf(fabsf(y[i])/fabsf(x[i]));
            }else if(x[i]>0 && y[i]<0){
                alpha[i]= 2*pi - atanf(fabsf(y[i])/fabsf(x[i]));
            }

            if(alpha[i]> pi){
                alpha[i] -=pi;
                endVelocity[i] *= -1;
            }
        }


    PWM_Duty_SLF = fmap(alpha[0], 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SRF = fmap(alpha[1], 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SLM = fmap(alpha[2], 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SRM = fmap(alpha[3], 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SLB = fmap(alpha[4], 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SRB = fmap(alpha[5], 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));

    //------------------------------------------

    // Each servo has a channel for movement
    ledc_set_duty(PWM_MODE, PWM_SERVO_LEFT_Front, PWM_Duty_SLF);
    ledc_set_duty(PWM_MODE, PWM_SERVO_RIGHT_Front, PWM_Duty_SRF);
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