#include <stdio.h>
#include <time.h>
#include <math.h>

#include "SRA18.h"
#include "MPU.h"
#include "TUNING.h"

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

int weights[4] = {3,1,-1,-3};

/*
 * Line Following PID Constants
 */
float kP = 3;
float kI =  0.001;
float kD = 2.0;

/*
 * Motor value constraints
 */
float opt = 80;
float lower_pwm_constrain = 60;
float higher_pwm_constrain = 100;
float left_pwm = 0, right_pwm = 0;

/*
 * Line Following PID Variables
 */
float error=0, prev_error, difference, cumulative_error, correction;

uint32_t adc_reading[4];
float sensor_value[4];


static void read_sensors()
{
  for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
        // printf("raw[%d]: %d ",i, adc_reading[i]);
    }
}

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
        // printf("fil[%d]: %f ", i, sensor_value[i]); 
    }
    printf("\n");

}

static void calculate_error()
{
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0, pos = 0;
    
    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 400)
        {
            all_black_flag = 0;
        }

        weighted_sum += (float)(sensor_value[i]) * (weights[i]);
        sum += sensor_value[i];
        
    }
    
    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(error > 0)
            pos = 2.5;
        else
            pos = -2.5;
    }

    error = pos;

}

static void calculate_correction()
{
    error *= 10;
    difference = (error - prev_error);
    cumulative_error += error;
    
    if(cumulative_error > 30)
    {
        cumulative_error = 30;
    }
    
    else if(cumulative_error < -30)
    {
        cumulative_error = -30;
    }

    correction = kP*error + kI*cumulative_error + kD*difference;
    prev_error = error;
}

void turnbot(char d)
{
    if (d == 'L')
    {
        // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        // while (sensor_value[0] < 50 && sensor_value[0] >=0 && sensor_value[1] > 900 && sensor_value[1] <= 1000 && sensor_value[2] > 900 && sensor_value[2] <= 1000 && sensor_value[3] > 900 && sensor_value[3] <= 1000)
        // {
        //     /* code */
        // }
        
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0,70,70);
        vTaskDelay(300/portTICK_PERIOD_MS); 
    }

    else if(d == 'R')
    {
        // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0,70,70);
        vTaskDelay(300/portTICK_PERIOD_MS);    
    }

    else if (d == 'B')
    {
        // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0,70,70);
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
}
void line_follow_task(void *arg)
{

  mcpwm_initialize();
  char turn;
  while(1)
  {
    read_sensors();
    calc_sensor_values();

    if(sensor_value[0] < 50 && sensor_value[0] >=0 && sensor_value[1] > 900 && sensor_value[1] <= 1000 && sensor_value[2] > 900 && sensor_value[2] <= 1000 && sensor_value[3] > 900 && sensor_value[3] <= 1000)
       {
         turn = 'L';
         printf("turn detected: %c  ", turn);
         turnbot(turn);
         for(int i=0;i<4;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }
       else if (sensor_value[0] > 900 && sensor_value[0] <=1000 && sensor_value[1] > 900 && sensor_value[1] <= 1000 && sensor_value[2] > 900 && sensor_value[2] <= 1000 && sensor_value[3] < 50 && sensor_value[3] >=0) 
       {
         turn = 'R';
         printf("turn detected: %c  ", turn);
         turnbot(turn);
         for(int i=0;i<4;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }
       else if (sensor_value[0] > 900 && sensor_value[0] <=1000 && sensor_value[1] > 900 && sensor_value[1] <= 1000 && sensor_value[2] > 900 && sensor_value[2] <= 1000 && sensor_value[3] > 900 && sensor_value[3] <= 1000)
       {
         turn = 'B';
         printf("turn detected: %c  ", turn);
         turnbot(turn);
         for(int i=0;i<4;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }
       else
       {
           calculate_error();
           calculate_correction();
           left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
           right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
         
           for(int i=0;i<4;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
           bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
       }
}

}

void app_main()
{
    xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}