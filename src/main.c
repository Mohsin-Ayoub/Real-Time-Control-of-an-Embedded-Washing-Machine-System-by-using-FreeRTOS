#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"

/* ===================== HEARTBEAT SYSTEM ===================== */
#define TASK_MOTOR_BIT   (1 << 0)
#define TASK_HEATING_BIT (1 << 1)
volatile uint8_t task_heartbeat = 0; // Supervisor checks this
volatile bool system_running = true; // Global Emergency Flag
int fault_count = 0; // Tracks restart attempts
/* ===================== TEMPERATURE PID ===================== */
#define HEATING_PIN        0
#define TEMP_ADC_CHANNEL   ADC_CHANNEL_3
#define TEMP_SETPOINT_C    20.0f
#define TEMP_HARD_CUTOFF   25.0f
#define SERIES_RESISTOR    4700.0f
#define THERMISTOR_NOMINAL 10000.0f
#define B_COEFFICIENT      3950.0f
#define TEMP_NOMINAL       25.0f
#define TEMP_WINDOW_MS     10000

static const char *TAG_TEMP = "HeatingPID";
static const char *TAG_SAFE = "SAFETY_SYSTEM";

/* --- Emergency Shutdown Helper --- */
void trigger_emergency_stop(const char* reason) {
    if (system_running) {
        system_running = false;
        gpio_set_level(HEATING_PIN, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            
        ESP_LOGE(TAG_SAFE, "!!! EMERGENCY STOP: %s !!!", reason);
        ESP_LOGW(TAG_SAFE, "System will attempt restart in 20 seconds...");
    }
}
adc_oneshot_unit_handle_t adc1_handle;
float temp_integral = 0;
float temp_kp = 18.0f, temp_ki = 0.08f;

float adc_raw_to_voltage(int raw) { return (raw / 4095.0f) * 3.3f; }

float voltage_to_temperature(float voltage) {
    if (voltage <= 0.01f || voltage >= 3.25f) return -273.15f;
    float Rt = SERIES_RESISTOR * (voltage / (3.3f - voltage));
    float T = 1.0f / ((1.0f / (TEMP_NOMINAL + 273.15f)) + (1.0f / B_COEFFICIENT) * log(Rt / THERMISTOR_NOMINAL));
    return T - 293.15f;
}

void relay_init(void) {
    gpio_config_t io = { .pin_bit_mask = 1ULL << HEATING_PIN, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(HEATING_PIN, 0);
}

void adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE };
    adc_oneshot_new_unit(&init_cfg, &adc1_handle);
    adc_oneshot_chan_cfg_t cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_11 };
    adc_oneshot_config_channel(adc1_handle, TEMP_ADC_CHANNEL, &cfg);
}

void heating_task(void *arg) {
    
    int64_t window_start = esp_timer_get_time() / 1000;
    float last_temp = 0;
    int check_timer = 0;
float last_check_temp = 0;
    int64_t last_check_time = 0;
    int seconds_at_full_power = 0;
    while (1) {
        if (!system_running) { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
        task_heartbeat |= TASK_HEATING_BIT;
        
        int raw;
        adc_oneshot_read(adc1_handle, TEMP_ADC_CHANNEL, &raw);
        float temp = voltage_to_temperature(adc_raw_to_voltage(raw));

        
        // 1. FASTER DISCONNECT CHECK (10 seconds)
        if (temp < (TEMP_SETPOINT_C - 2)) {
            seconds_at_full_power++;
            if (seconds_at_full_power >= 60) { // Check every 10s
                if (temp - last_temp -2 < 0.2f) {
                    //trigger_emergency_stop("HEATER_HARDWARE_FAILURE 2");
                }
                last_temp = temp;
                seconds_at_full_power = 0;
            }
        } else {
            seconds_at_full_power = 0;
        }
        // 3. Heater Warning: If Power is 100% for 30s but temp doesn't rise
        check_timer++;
        if (check_timer >= 30) { // roughly 30 seconds
            if (temp - last_temp < 0.2f && temp < TEMP_SETPOINT_C) {
                ESP_LOGW(TAG_TEMP, "WARNING: Heater active but temperature not rising!");
            }
            last_temp = temp;
            check_timer = 0;
        }

        if (temp > TEMP_HARD_CUTOFF) {
            gpio_set_level(HEATING_PIN, 0);
            ESP_LOGE(TAG_TEMP, "OVER TEMP! Heater OFF | Temp: %.2f C", temp);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        float error = TEMP_SETPOINT_C - temp;
        temp_integral += error;
        float output = temp_kp* error + temp_ki * temp_integral; //kp & Ki
        if (output > 100) output = 100;
        if (output < 0) output = 0;

        int on_time = (int)(TEMP_WINDOW_MS * (output / 100.0f));
        int64_t now = esp_timer_get_time() / 1000;
        if (now - window_start > TEMP_WINDOW_MS) window_start = now;

        
        if (system_running) {
            gpio_set_level(HEATING_PIN, ((now - window_start) < on_time) ? 1 : 0);
        ESP_LOGI(TAG_TEMP, "Temp: %.2f C | Power: %.1f%%", temp, output);
        } else {
            gpio_set_level(HEATING_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!system_running) {
            gpio_set_level(HEATING_PIN, 0); // Safety Force
            continue; 
        }
    }
}

/* ===================== MOTOR PID ===================== */
#define IN1_PIN 18
#define IN2_PIN 4
#define MOTOR_SPEED_PIN 2
#define PWM_FREQ 200
#define PWM_MAX 800
#define TARGET_SPEED_HZ 60.0f
#define RAMP_STEP_HZ 3.0f
#define SAMPLE_INTERVAL_MS 500

static const char *TAG_MOTOR = "MotorPID";
typedef enum {STATE_RAMP_UP, STATE_STEADY, STATE_RAMP_DOWN, STATE_STOP} motor_state_t;
motor_state_t motor_state = STATE_RAMP_UP;
volatile int pulse_count = 0;
bool direction_right = true;
float speed_setpoint = 0;
int  pwm_duty = 0;
int state_timer = 0;

float motor_kp = 12.0f, motor_ki = 1.2f, motor_kd = 0.1f;
float motor_integral = 0, motor_last_error = 0;
static void IRAM_ATTR speed_isr(void *arg) { pulse_count++; }

void motor_pwm_init(void) {
    ledc_timer_config_t timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0, .duty_resolution = LEDC_TIMER_10_BIT, .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch1 = {.gpio_num = IN1_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0};
    ledc_channel_config_t ch2 = {.gpio_num = IN2_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0};
    ledc_channel_config(&ch1);
    ledc_channel_config(&ch2);
}

void speed_sensor_init(void) {
    gpio_config_t io_conf = {.intr_type = GPIO_INTR_POSEDGE, .mode = GPIO_MODE_INPUT, .pin_bit_mask = 1ULL << MOTOR_SPEED_PIN, .pull_up_en = 1};
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MOTOR_SPEED_PIN, speed_isr, NULL);
}

void motor_apply(int duty, bool right) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, right ? duty : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right ? 0 : duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motor_task(void *arg) {
    float dt = SAMPLE_INTERVAL_MS / 1000.0f;
     float last_speed = 0;
    int stall_counter = 0;
    const int STALL_THRESHOLD_SAMPLES = 6; // At 500ms intervals, this is 3 seconds
    while (1) {
        if (!system_running) { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
        task_heartbeat |= TASK_MOTOR_BIT;
        pulse_count = 0;
        //vTaskDelay(pdMS_TO_TICKS(500));
        vTaskDelay(SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
        if (!system_running) {
            motor_apply(0, direction_right); // Safety Force
            continue; 
        }
        float speed_hz = pulse_count / 0.5f;

        // --- IMPROVED STALL DETECTION ---
        // 1. Only check if we are actually trying to move (Setpoint > 5)
        // 2. Only check if PWM is high (Power is being applied)
        if (speed_setpoint > 5.0f && pwm_duty > 400) {
            // If speed is very low AND not increasing
            if (speed_hz < (speed_setpoint * 0.2f) && (speed_hz <= last_speed + 0.5f)) {
                stall_counter++;
            } else {
                stall_counter = 0; // Reset if we see progress
            }
        } else {
            stall_counter = 0;
        }

        // If stalled for more than 3 continuous seconds
        if (stall_counter > STALL_THRESHOLD_SAMPLES) {
            trigger_emergency_stop("MOTOR_STALL_OR_SENSOR_LOST");
            stall_counter = 0;
            continue;
        }
        last_speed = speed_hz;
        // if (pwm_duty !=0 && speed_hz < 0.1f && motor_state != STATE_STOP) {
        //     ESP_LOGE(TAG_MOTOR, "HARDWARE ERROR: Motor jammed or Speed Sensor disconnected!");
        //     trigger_emergency_stop("Motor_HARDWARE_FAILURE");
            
        // }
        

        switch (motor_state) {
            case STATE_RAMP_UP:
                speed_setpoint += RAMP_STEP_HZ;
                if (speed_hz >= TARGET_SPEED_HZ) { speed_setpoint = TARGET_SPEED_HZ; motor_state = STATE_STEADY; state_timer = 0; }
                break;

                
            case STATE_STEADY:
                state_timer += SAMPLE_INTERVAL_MS;
                if (state_timer >= 20000) motor_state = STATE_RAMP_DOWN;
                break;
            case STATE_RAMP_DOWN:
                speed_setpoint -= RAMP_STEP_HZ;
                if (speed_setpoint <= 0) { speed_setpoint = 0; motor_state = STATE_STOP; state_timer = 0; }
                break;
            case STATE_STOP:
                motor_apply(0, direction_right);
                state_timer += SAMPLE_INTERVAL_MS;
                if (state_timer >= 1000) { direction_right = !direction_right; motor_state = STATE_RAMP_UP; }
                continue;
        }

        //PID MATH speed 
        float error = speed_setpoint - speed_hz;
        motor_integral += error * dt; // Integral
        float derivative = (error - motor_last_error) / dt; // Derivative
        motor_last_error = error;

        float pid_output = (motor_kp * error) + (motor_ki * motor_integral) + (motor_kd * derivative);
        
        pwm_duty += (int)pid_output; // Update duty cycle based on PID result
        
        if (pwm_duty > PWM_MAX) pwm_duty = PWM_MAX;
        if (pwm_duty < 0) pwm_duty = 0;
        
        // // --- PID MATH ---
        // float error = speed_setpoint - speed_hz;
        
        // // Allow the integral to grow more (increase the limit)
        // motor_integral += error * 0.5f;
        // if (motor_integral > 150) motor_integral = 150; 
        // if (motor_integral < -150) motor_integral = -150;

        // float derivative = (error - motor_last_error) / 0.5f;
        // motor_last_error = error;

        // float pid_output = (motor_kp * error) + (motor_ki * motor_integral) + (motor_kd * derivative);
        
        // pwm_duty = (int)pid_output; // Or pwm_duty += pid_output depending on your tuning
        
        // // Final Bounds
        // if (pwm_duty > PWM_MAX) pwm_duty = PWM_MAX;
        // if (pwm_duty < 0) pwm_duty = 0;
        // Final Safety Check before applying power
        if (system_running) {
            motor_apply(pwm_duty, direction_right);
            ESP_LOGI(TAG_MOTOR, "Speed: %.2f Hz | PWM: %d", speed_hz, pwm_duty);
        } else {
            motor_apply(0, direction_right);
        }
    }
}

/* ===================== 2. SUPERVISOR TASK ===================== */
/* ===================== SUPERVISOR (WITH RECOVERY) ===================== */
void supervisor_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check health every 5s

        if (system_running) {
            // Check if tasks are alive
            if (!((task_heartbeat & TASK_MOTOR_BIT) && (task_heartbeat & TASK_HEATING_BIT))) {
                trigger_emergency_stop("SOFTWARE_TASK_HANG");
            } else {
                ESP_LOGI(TAG_SAFE, "System Health: OK");
                task_heartbeat = 0; // Clear for next 5s window
            }
        } 
        else {
            // --- NEW RECOVERY LOGIC ---
            ESP_LOGW(TAG_SAFE, "System is in FAULT state. Cooling down...");
            
            // Wait for 20 seconds (4 cycles of 5 seconds)
            for (int i = 0; i < 4; i++) {
                vTaskDelay(pdMS_TO_TICKS(5000));
                ESP_LOGI(TAG_SAFE, "Restarting in %d seconds...", (4 - i) * 5);
            }

            // RESET EVERYTHING
            ESP_LOGI(TAG_SAFE, "RECOVERING: Attempting system restart...");
            task_heartbeat = 3; // Give tasks a "free pass" for the first 5s
            fault_count++;
            pwm_duty = 0;
            system_running = true; 
        }
    }
}

/* ===================== MAIN ===================== */
void app_main(void) {
    relay_init();
    adc_init();
    motor_pwm_init();
    speed_sensor_init();

    // 1. Priorities: Supervisor(7) > Motor(6) > Heating(5)
    xTaskCreate(supervisor_task, "supervisor", 3072, NULL, 7, NULL);
    xTaskCreate(motor_task,      "motor_task", 4096, NULL, 6, NULL);
    xTaskCreate(heating_task,    "heating_task", 4096, NULL, 5, NULL);
}