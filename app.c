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
#define Rwheeldistance Rlength / 2

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
    float br = sqrtf(Rwheeldistance * Rwheeldistance + Rwidth * Rwidth);
    float beta = atanf(Rwheeldistance / (Rwidth / 2));
    // angulars
    // B = Beta l/r= left/right f/m/r = front/middles/back 
    float Blf = beta;
    float Brf = 2 * pi - beta;
    float Blm = 0.5 * pi;
    float Brm = 2 * pi - Blm;
    float Blb = Brf;
    float Brb = Blf;
    // velocity
    float Vlf = fabsf(msg.angular.z);
    float Vrf = Vlf;
    float Vlm = (br / sr) * Vlf;
    float Vrm = Vlm;
    float Vlb = Vrf;
    float Vrb = Vlf;

    // linear movement
    float gamma;
    float c;
    if (msg.linear.x != 0 && msg.linear.y != 0)
    {
        gamma = atanf(msg.linear.y / msg.linear.x);
        c = sqrtf(msg.linear.y * msg.linear.y + msg.linear.x * msg.linear.x);
    }
    else if (msg.linear.x != 0)
    {
        if (msg.linear.x > 0)
            gamma = 0.5 * pi;
        else
            gamma = 1.5 * pi;
        c = fabsf(msg.linear.x);
    }
    else if (msg.linear.y != 0)
    {
        if (msg.linear.y > 0)
            gamma = 0;
        else
            gamma = pi;
        c = fabsf(msg.linear.y);
    }

    // Morph gamma and beta
    // Delta is gamma and betas morphed
    float Dlf;
    float Drf;
    float Dlm;
    float Drm;
    float Dlb;
    float Drb;
    // End velocity
    float Evlf;
    float Evrf;
    float Evlm;
    float Evrm;
    float Evlb;
    float Evrb;
    
    float xlf;    
    float xrf;    
    float xlm;
    float xrm;    
    float xlb;
    float xrb;

    float ylf;    
    float yrf;    
    float ylm;
    float yrm;    
    float ylb;
    float yrb;

    if (msg.linear.z != 0)
    {
        xlf = cosf(Blf)*Vlf + msg.linear.x; 
        ylf = sinf(Blf)*Vlf + msg.linear.y;
        Evlf=sqrtf(ylf*ylf + xlf*xlf);



        xrf = cosf(Brf)*Vrf + msg.linear.x;
        yrf = sinf(Brf)*Vrf + msg.linear.y;
        Evrf=sqrtf(yrf*yrf + xrf*xrf);


        xlm = cosf(Blm)*Vlm + msg.linear.x;
        ylm = sinf(Blm)*Vlm + msg.linear.y;
        Evlm=sqrtf(ylm*ylm + xlm*xlm);


        xrm = cosf(Brm)*Vrm + msg.linear.x;
        yrm = sinf(Brm)*Vrm + msg.linear.y;
        Evrm=sqrtf(yrm*yrm + xrm*xrm);


        xlb = cosf(Blb)*Vlb + msg.linear.x;
        ylb = sinf(Blb)*Vlb + msg.linear.y;
        Evlb=sqrtf(ylb*ylb + xlb*xlb);


        xrb = cosf(Brf)*Vrf + msg.linear.x;
        yrb = sinf(Brf)*Vrf + msg.linear.y;
        Evrb=sqrtf(yrb*yrb + xrb*xrb);



    }
    else if (c != 0)
    {
        Dlf = gamma;
        Drf = gamma;
        Dlm = gamma;
        Drm = gamma;
        Dlb = gamma;
        Drb = gamma;

        Evlf = c;
        Evrf = c;
        Evlm = c;
        Evrm = c;
        Evlb = c;
        Evrb = c;
    }

    // final movement

    float Alf;
    float Arf;
    float Alm;
    float Arm;
    float Alb;
    float Arb;

    PWM_Duty_SLF = fmap(Alf, 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SRF = fmap(Arf, 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SLM = fmap(Alm, 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SRM = fmap(Arm, 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SLB = fmap(Alb, 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));
    PWM_Duty_SRB = fmap(Arb, 0, 2 * pi, 0, powf(2, PWM_RESOLUTION));

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