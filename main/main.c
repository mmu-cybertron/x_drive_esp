
/**********************************************************************************************
 * Core 1 is reserved for bluetooth functionality
 * Bluetooth > Bluedroid Options > Core 1
 * Bluetooth > Bluetooth Controller > Core 1
 * Core 0 is running the app_main and used to setup peripherals.
 * In core 0, there is a pulse counting peripheral, with some interrupt footprint, however performance
 * does not seem to be an issue for now.
 * Configure component config > ESP System Settings  > Main Task core affinity to CPU0
 * PS5 Controller Callback function: ~1200ms between each run, 800Hz
 * PID Update function: 1ms between each run, 1kHz
 * MPU6050 Update function: 500us between each run, 2kHz
***********************************************************************************************/
#include "x_drive_esp.h"
#include "ps5.h"
#include "mpu6050.h"
#include <math.h>
// #include "esp_cpu.h"

#define PS5_RX_ROTATION_RATE 1000000.0
#define MPU6050_SAMPLING_US 500 // Sample at 500us, 2kHz

#define MAXIMUM_SPEED_PPU 308.0 // Maximum speed of robot obtained after testing, in pulse per us
#define MAXIMUM_MAG_PS5 180.0 // Result of sqrt(x*x + y*y) when x and y are at maximum value of 127 or 128

const float ppu_per_ps5 = MAXIMUM_SPEED_PPU / MAXIMUM_MAG_PS5;

#define MPU_6050_KP 0.1
#define MPU_6050_KI 0.01
#define MPU_6050_KD 0

int8_t lx;
int8_t ly;
int8_t rx;
int8_t ry;
float target_angle_Z = 0, angleZ = 0;
time_t time_ps5 = {0}; // Used to integrate rx to get target angle
time_t time_mpu6050 = {0}; // For MPU6050 PID loop

mpu6050_pid_t mpu6050_pid_params = {
    .kp = MPU_6050_KP,
    .ki = MPU_6050_KI,
    .integral_limit_max = 1000,
    .integral_limit_min = -1000,
    .tolerance = 0.1
}; // PID parameters for MPU6050

void print_debug_1(void*arg){
    pid_controller_t *pid_params = (pid_controller_t *)arg;
    // int i = 2;
    while (1)
    {
        // ESP_LOGI(TAG, "angleZ: %f, target_angle_Z: %f", angleZ, target_angle_Z);
        ESP_LOGI(TAG, "Delta Time: %lld, Target Angle: %f AngleZ: %f", time_mpu6050.delta_time, target_angle_Z, angleZ);
        // ESP_LOGI(TAG, "lx: %d, ly: %d, rx: %d, ry: %d target angle: %f\n", lx, ly, rx, ry, target_angle);
    }
    
}

void app_main(void)
{
    // Declare the struct array, which contains the pwm and pcnt handler
    pid_controller_t pid_params[4] = {0};
    
    initialize_peripherals(pid_params); 


    // If MPU6050 failed to initialize, the function will block further operations
    mpu6050_init();
    // Timer with 0.5ms periodic task for MPU6050 PID update
    esp_timer_handle_t update_PID_mpu6050 = NULL;
    esp_timer_create_args_t update_PID_mpu6050_config ={
        .arg = (void*)&mpu6050_pid_params,
        .callback = &update_PID_mpu6060
    };
    esp_timer_create(&update_PID_mpu6050_config, &update_PID_mpu6050);
    esp_timer_start_periodic(update_PID_mpu6050, MPU6050_SAMPLING_US);


    // Initialize PS5 controller
    // uint8_t new_mac[8] = {0x24,0xA6,0xFA,0x3B,0x93,0x0A};
    // ps5SetBluetoothMacAddress(new_mac);
    ps5Init();
    while (!ps5IsConnected()) {
        ESP_LOGI("PS5", "Waiting for PS5 controller to connect...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI("PS5", "PS5 controller connected!");
    ps5Enable();
    ps5SetEventObjectCallback((void*)pid_params, cb);


    // Setup timer with a 1ms periodic task to update PID motor speed
    esp_timer_handle_t timer_update_PID_motor = NULL;
    esp_timer_create_args_t timer_update_PID_motor_config ={
        .arg = (void*)pid_params,
        .callback = &update_PID_output
    };
    esp_timer_create(&timer_update_PID_motor_config, &timer_update_PID_motor);

    // Initialize initial values of the time & encoder pulses
    time_encoder.prev_time = esp_timer_get_time();
    for (int i = 0; i < 4; i++){
        pcnt_unit_get_count(pid_params[0].encoder, &encoder_pulses[0].prev_count);
    }
    // Start the timer
    esp_timer_start_periodic(timer_update_PID_motor, TIMER_PERIOD); //1ms as sampling time
    
    // Use core 1 to print debug messages
    xTaskCreatePinnedToCore(&print_debug_1, "print_debug", 2048, (void*)pid_params, 1, NULL, 1);               

    // while(1){
        // mcpwm_comparator_set_compare_value(pid_params[0].motor, 1000);
        // mcpwm_comparator_set_compare_value(pid_params[1].motor, 1000);
        // mcpwm_comparator_set_compare_value(pid_params[2].motor, 1000);
        // mcpwm_comparator_set_compare_value(pid_params[3].motor, 1000);
        // gpio_set_level(MOTOR_1_DIR, 0);
        // gpio_set_level(MOTOR_2_DIR, 0);
        // gpio_set_level(MOTOR_3_DIR, 0);
        // gpio_set_level(MOTOR_4_DIR, 0);
        // vTaskDelay(pdMS_TO_TICKS(2000));
        // gpio_set_level(MOTOR_1_DIR, 1);
        // gpio_set_level(MOTOR_2_DIR, 1);
        // gpio_set_level(MOTOR_3_DIR, 1);
        // gpio_set_level(MOTOR_4_DIR, 1);
        // vTaskDelay(pdMS_TO_TICKS(2000));

        // ESP_LOGI(TAG, "Current Speed: Positive");
        // pid_params[2].target_speed = 0.1;   
        // vTaskDelay(pdMS_TO_TICKS(2000));
        // ESP_LOGI(TAG, "Current Speed: Zero");
        // pid_params[2].target_speed = 0; 
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // ESP_LOGI(TAG, "Current Speed: Negative");   
        // pid_params[2].target_speed = -0.1; 
        // vTaskDelay(pdMS_TO_TICKS(2000));
    // }

}   
// PS5 callback function
// Monitoring delta time shows that this function is running at 800Hz
// Running in core 1
void cb ( void* arg, ps5_t ps5, ps5_event_t event) {
    time_ps5.curr_time = esp_timer_get_time();
    time_ps5.delta_time = time_ps5.curr_time - time_ps5.prev_time;
    time_ps5.prev_time = time_ps5.curr_time;

    pid_controller_t *pid_params = (pid_controller_t *)arg;
    lx = ps5.analog.stick.lx;
    ly = ps5.analog.stick.ly;
    rx = ps5.analog.stick.rx;
    ry = ps5.analog.stick.ry;

    // Remove deadzone of the controller
    if (lx <= 7 && lx >= -7) lx = 0;
    if (ly <= 7 && ly >= -7) ly = 0;
    if (rx <= 7 && rx >= -7) rx = 0;
    if (ry <= 7 && ry >= -7) ry = 0;

    // Integrate the right stick to get the target angle
    target_angle_Z += (float)rx * (float)time_ps5.delta_time / PS5_RX_ROTATION_RATE;
    
    float theta =  atan2f((float)ly, (float)-lx);
    float mag = sqrtf((float)(lx*lx + ly*ly));

    float M1 = mag * cosf(theta - 3.14159/4);
    float M2 = mag * -sinf(theta - 3.14159/4);
    float M3 = mag * -cosf(theta - 3.14159/4);
    float M4 = mag * sinf(theta - 3.14159/4);
    
    M1 = M1 * ppu_per_ps5;
    M2 = M2 * ppu_per_ps5;
    M3 = M3 * ppu_per_ps5;
    M4 = M4 * ppu_per_ps5;

    pid_params[0].target_speed = M1;
    pid_params[1].target_speed = M2;
    pid_params[2].target_speed = M3;
    pid_params[3].target_speed = M4;


    // This function will run on core 1
    // int x = esp_cpu_get_core_id();
    // ESP_LOGI(TAG, "Core ID: %d", x);    

}

void update_PID_mpu6060(void *arg){
    mpu6050_pid_t *mpu6050_pid_params = (mpu6050_pid_t *)arg;
    time_mpu6050.curr_time = esp_timer_get_time();
    time_mpu6050.delta_time = time_mpu6050.curr_time - time_mpu6050.prev_time;
    time_mpu6050.prev_time = time_mpu6050.curr_time;

    mpu6050_updateZ(&angleZ, time_mpu6050.delta_time);

    mpu6050_pid_params->err = target_angle_Z - angleZ;

    // Calculate P term
    mpu6050_pid_params->pTerm = mpu6050_pid_params->kp * mpu6050_pid_params->err;

    // Calculate I term
    mpu6050_pid_params->integral += mpu6050_pid_params->err;
    mpu6050_pid_params->iTerm = mpu6050_pid_params->ki * mpu6050_pid_params->integral;
    if (mpu6050_pid_params->iTerm > mpu6050_pid_params->integral_limit_max)
    {
        mpu6050_pid_params->iTerm = mpu6050_pid_params->integral_limit_max;
    }
    else if (mpu6050_pid_params->iTerm < mpu6050_pid_params->integral_limit_min)
    {
        mpu6050_pid_params->iTerm = mpu6050_pid_params->integral_limit_min;
    }

    // Derivative not implemented
    
    // Calculate output and if it is within tolerance, set output to 0
    if (mpu6050_pid_params->err < fabs(mpu6050_pid_params->tolerance)){
        mpu6050_pid_params->output = 0;
    } else {
        mpu6050_pid_params->output = mpu6050_pid_params->pTerm + mpu6050_pid_params->iTerm;
    }

    // ESP_LOGI(TAG, "AngleZ: %f, Delta Time: %lld", angleZ, time_mpu6050.delta_time);

}


// Periodic timer callback to update PID output
void update_PID_output(void *arg){
    pid_controller_t *pid_params = (pid_controller_t *)arg;  // Cast void* to pid_controller_t*

    pid_params[0].target_speed += mpu6050_pid_params.output;
    pid_params[1].target_speed += mpu6050_pid_params.output;
    pid_params[2].target_speed += mpu6050_pid_params.output;
    pid_params[3].target_speed += mpu6050_pid_params.output;

    float largest_speed = fabs(pid_params[0].curr_speed);
    for (int i = 1; i < 4; i++) {
        if (fabs(pid_params[i].curr_speed) > largest_speed) {
            largest_speed = fabs(pid_params[i].curr_speed);
        }
    }
    // Normalize the speed to the maximum speed
    if (largest_speed > MAXIMUM_SPEED_PPU) {
        for (int i = 0; i < 4; i++) {
            pid_params[i].target_speed = pid_params[i].target_speed / largest_speed * MAXIMUM_SPEED_PPU;
        }
    }

    time_encoder.curr_time = esp_timer_get_time();

    time_encoder.delta_time = time_encoder.curr_time - time_encoder.prev_time;
    time_encoder.prev_time = time_encoder.curr_time;

    int i = 0;
    // Get current encoder count and store into the struct array
    for (i = 0; i < 4; i++) {
        pcnt_unit_get_count(pid_params[i].encoder, &encoder_pulses[i].curr_count);
    }

    for (i = 0; i < 4; i++)
    {
        encoder_pulses[i].delta_count = (encoder_pulses[i].curr_count - encoder_pulses[i].prev_count);
        encoder_pulses[i].prev_count = encoder_pulses[i].curr_count;

        // Calculate instantaneous velocity
        pid_params[i].curr_speed = (float)encoder_pulses[i].delta_count / (float)time_encoder.delta_time;
        pid_params[i].err = pid_params[i].target_speed - pid_params[i].curr_speed;

        pid_params[i].pTerm = pid_params[i].kp * pid_params[i].err;

        // Calculate Integral term
        pid_params[i].integral += pid_params[i].err;
        pid_params[i].iTerm = pid_params[i].ki * pid_params[i].integral;
        if (pid_params[i].iTerm > pid_params[i].integral_limit_max)
        {
            pid_params[i].iTerm = pid_params[i].integral_limit_max;
        }
        else if (pid_params[i].iTerm < pid_params[i].integral_limit_min)
        {
            pid_params[i].iTerm = pid_params[i].integral_limit_min;
        }

        // Calculate Derivative term
        pid_params[i].dTerm = pid_params[i].kd * (pid_params[i].err - pid_params[i].prev_err);
        pid_params[i].prev_err = pid_params[i].err;

        pid_params[i].output = (pid_params[i].pTerm + pid_params[i].iTerm + pid_params[i].dTerm);

        // Limit the ouput to a range within valid comparator value
        if (pid_params[i].output > MCPWM_COMPRATOR_MAX) {
            pid_params[i].output = MCPWM_COMPRATOR_MAX;
        } else if (pid_params[i].output < -MCPWM_COMPRATOR_MAX) {
            pid_params[i].output = -MCPWM_COMPRATOR_MAX;
        }
    
    }

    for(i=0; i<4; i++){
        // Set direction of motor
        if (pid_params[i].output>0) gpio_set_level(motor_dir_array[i], 0);
        else gpio_set_level(motor_dir_array[i], 1);

        // Update new PWM duty cycle
        uint32_t comp_val = fabs(pid_params[i].output);
        mcpwm_comparator_set_compare_value(pid_params[i].motor, comp_val);
    }

    // Print output, error and current speed
    // ESP_LOGI(TAG, "Output: %d, Error: %d, Current Speed: %d", output, err, curr_speed);
    // Print print delta count, delta time
    // ESP_LOGI(TAG, "Delta Count: %d, Delta Time: %lld", delta_count, delta_time);
}