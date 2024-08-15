
/**********************************************************************************************
 * Core 1 is reserved for bluetooth functionality
 * Bluetooth > Bluedroid Options > Core 1
 * Bluetooth > Bluetooth Controller > Core 1
 * Core 0 is running the app_main and used to setup peripherals.
 * In core 0, there is a pulse counting peripheral, with some interrupt footprint, however performance
 * does not seem to be an issue for now.
 * Configure component config > ESP System Settings  > Main Task core affinity to CPU0
***********************************************************************************************/
#include "x_drive_esp.h"
#include "ps5.h"
#include <math.h>

void app_main(void)
{
    // Declare the struct array, which contains the pwm and pcnt handler
    pid_controller_t pid_params[4] = {0};
    initialize_peripherals(pid_params);     

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


    // Setup timer with a 1ms periodic task to update PID output
    esp_timer_handle_t timer_update_PID_output = NULL;
    esp_timer_create_args_t timer_update_PID_output_congfig ={
        .arg = (void*)pid_params,
        .callback = &update_PID_output
    };
    esp_timer_create(&timer_update_PID_output_congfig, &timer_update_PID_output);

    // Initialize initial values of the time & encoder pulses
    prev_time = esp_timer_get_time();
    for (int i = 0; i < 4; i++){
        pcnt_unit_get_count(pid_params[0].encoder, &encoder_pulses[0].prev_count);
    }
    // Start the timer
    esp_timer_start_periodic(timer_update_PID_output, 1000); //1ms as sampling time
    
    // Use core 1 to print debug messages
    xTaskCreatePinnedToCore(&print_debug, "print_debug", 2048, (void*)pid_params, 1, NULL, 1);               

    while(1){
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

    }

}   

void cb ( void* arg, ps5_t ps5, ps5_event_t event) {
    pid_controller_t *pid_params = (pid_controller_t *)arg;
    int8_t lx = ps5.analog.stick.lx;
    int8_t ly = ps5.analog.stick.ly;
    int8_t rx = ps5.analog.stick.rx;

    // Remove deadzone of the controller
    if (lx < 3 && lx > -3) lx = 0;
    if (ly < 3 && ly > -3) ly = 0;
    if (rx < 3 && rx > -3) rx = 0;
    
    float theta =  atan2f((float)ly, (float)-lx);
    float mag = sqrtf((float)(lx*lx + ly*ly));

    float M1 = 1.5 * mag * cosf(theta - 3.14159/4) - rx/3;
    float M2 = 1.5 * mag * -sinf(theta - 3.14159/4) - rx/3;
    float M3 = 1.5 * mag * -cosf(theta - 3.14159/4) - rx/3;
    float M4 = 1.5 * mag * sinf(theta - 3.14159/4) - rx/3;

    M1 = M1/308.0 * 0.50;     
    M2 = M2/308.0 * 0.50;     
    M3 = M3/308.0 * 0.50;     
    M4 = M4/308.0 * 0.50;     

    pid_params[0].target_speed = M1; 
    pid_params[1].target_speed = M2; 
    pid_params[2].target_speed = M3; 
    pid_params[3].target_speed = M4; 

}

// Periodic timer callback to update PID output
void update_PID_output(void *arg){
    pid_controller_t *pid_params = (pid_controller_t *)arg;  // Cast void* to pid_controller_t*

    curr_time = esp_timer_get_time();

    delta_time = curr_time - prev_time;
    prev_time = curr_time;

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
        pid_params[i].curr_speed = (float)encoder_pulses[i].delta_count / (float)delta_time;
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