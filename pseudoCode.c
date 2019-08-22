#include <stdio.h>
#include <time.h>
#include <math.h>

#include "SRA18.h"
#include "MPU.h"
#include "TUNING.h"

adc1_channel_t channel[6] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5};
// adc1_channel_t turn_channel[2] = {ADC_CHANNEL_4, ADC_CHANNEL_5};
int weights[4] = {3,1,-1,-3};
int test = 0;
int q,p;
/*
 * Line Following PID Constants
 */
float kP = 2.0;
float kI =  0.0001;
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
uint32_t adc_turn_reading[2];
float sensor_turn_value[2];

static void read_sensors()
{
  for(int i = 0; i < 6; i++)  //dpin 32 = adcpin4 --> front sensor , adc5 33 --> encoder
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
        // printf("raw[%d]: %d ",i, adc_reading[i]);
    }
}

// static void read_turn_sensors()
// {
//     adc_turn_reading[0] = adc1_get_raw(turn_channel[0]);
//     adc_turn_reading[1] = adc1_get_raw(turn_channel[1]);
//     printf("sensor [0]: %d sensor[1]: %d", adc_turn_reading[0], adc_turn_reading[1]);
// }

static void calc_sensor_values()
{
    for(int i = 0; i < 5; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
        // printf("fil[%d]: %f ", i, sensor_value[i]); 
    }
}

// static void calc_turn_sensor_values()
// {
//     for(int i = 0; i < 2; i++)
//     {
//         sensor_turn_value[i] = map(adc_turn_reading[i], 1700, 4000, 0, 1000);
//         sensor_turn_value[i] = constrain(sensor_turn_value[i],0,1000);
//         // printf("fil[%d]: %f ", i, sensor_turn_value[i]); 
//     }
//     printf("\n");
// }

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

    // if(all_black_flag == 1)
    // {
    //     if(error > 0)
    //         pos = 2.5;
    //     else
    //         pos = -2.5;
    // }

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

// void followpath()
// {
//     while (1)
//     {

//         calculate_error();
//         calculate_correction();
        
//         left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
//         right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
         
//         for(int i=0;i<4;i++)
//         {
//             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
//         }    
//         bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);   


//         if(sensor_value[0] < 100 && sensor_value[0] >=0 && sensor_value[1] > 900 && sensor_value[1] <= 1000 && sensor_value[2] > 900 && sensor_value[2] <= 1000 && sensor_value[3] > 900 && sensor_value[3] <= 1000)
//          {
//             break;
//          }
//         else if (sensor_value[0] > 900 && sensor_value[0] <=1000 && sensor_value[1] > 900 && sensor_value[1] <= 1000 && sensor_value[2] > 900 && sensor_value[2] <= 1000 && sensor_value[3] < 100 && sensor_value[3] >=0) 
//         {
//             break;
//         }
//         else if (sensor_value[0] < 100 && sensor_value[0] >= 0 && sensor_value[1] < 100 && sensor_value[1] >= 0 && sensor_value[2] < 100 && sensor_value[2] >= 0 && sensor_value[3] < 100 && sensor_value[3] >= 0)
//         { 
//             break;
//         }      
//     }
// }


void turnbot(char d)
{
    if (d == 'L')
    {
        while (q==1)
        {   
            read_sensors();read_sensors();
            calc_sensor_values();
            calc_sensor_values();
            if((sensor_value[0] > 700) || (sensor_value[1] > 700) || (sensor_value[2] > 700) || (sensor_value[3] > 700))
            {
                q=0;
            }
            
            bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0,70,70);
        // vTaskDelay(200/portTICK_PERIOD_MS);
        } 
    }

    else if(d == 'R')
    {
        while (p==1)
        {   
            read_sensors();read_sensors();
            calc_sensor_values();
            calc_sensor_values();
            if((sensor_value[0] > 700) || (sensor_value[1] > 700) || (sensor_value[2] > 700) || (sensor_value[3] > 700))
            {
                p=0;
            }
            
            bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0,70,70);
        // vTaskDelay(200/portTICK_PERIOD_MS);
        } 
    
    }

    // else if (d == 'B')
    // {
    //     // vTaskDelay(100/portTICK_PERIOD_MS);
    //     bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0,90,90);
    //     vTaskDelay(240/portTICK_PERIOD_MS);
    // }

    // else
    // {
    //     bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    // }
    
}


void line_follow_task(void *arg)
{
  mcpwm_initialize();
  char turn;
  
  while(1)
  {
    read_sensors();
    // read_turn_sensors();
    calc_sensor_values();
    // calc_turn_sensor_values();
    calculate_error();
    calculate_correction();
        
    left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
         
        for(int i=0;i<5;i++)
        {
            printf("sensor value[%d]: %f  ", i, sensor_value[i]);
        }    
        printf("\n");
        bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);  

    // followpath();

       if(sensor_value[0] < 400  && sensor_value[1] > 900  && sensor_value[2] > 900  && sensor_value[3] > 900)
       {
         turn = 'L';
         printf("turn detected:%c  ", turn);
         q=1;
         vTaskDelay(200/portTICK_PERIOD_MS);
         bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
         vTaskDelay(500/portTICK_PERIOD_MS);    
         turnbot(turn);

         for(int i=0;i<5;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }

       else if (sensor_value[0] > 900 && sensor_value[1] > 900 && sensor_value[2] > 900  && sensor_value[3] < 400) 
       {
         turn = 'R';
         printf("turn detected:%c  ", turn);
         p = 1;
         vTaskDelay(200/portTICK_PERIOD_MS);
         bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
         vTaskDelay(500/portTICK_PERIOD_MS);    
         turnbot(turn);

         for(int i=0;i<5;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

       int junction =0; //1) +  2) T  3) L+S  4) R+S
//////////note: raw readings of fwd sensor limits white >500 and black <200

       if(sensor_value[0] < 400  && sensor_value[1] > 900  && sensor_value[2] > 900  && sensor_value[3] > 900 && sensor_value[4] > 500)
       {
           // L+S junc
         junction = 3;
         printf("junc detected: %d  ", junction);
         q=1;
         bot_spot_left();
         vTaskDelay(200/portTICK_PERIOD_MS);
        // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
         //vTaskDelay(500/portTICK_PERIOD_MS);    
        // turnbot(turn);

         for(int i=0;i<5;i++)
         {
             printf("junc value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }

       else if (sensor_value[0] > 900 && sensor_value[1] > 900 && sensor_value[2] > 900  && sensor_value[3] < 400 && sensor_value[4]>500) 
       {
           //R+S
         junction = 4;
         printf("junc detected: %d  ", junction);
         p = 1;
         vTaskDelay(200/portTICK_PERIOD_MS);
         //bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
         //vTaskDelay(500/portTICK_PERIOD_MS);    
         //turnbot(turn);

         for(int i=0;i<5;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }

       else if (sensor_value[0] > 900 && sensor_value[1] > 900 && sensor_value[2] > 900  && sensor_value[3] > 900 && sensor_value[4]<300) 
       {
         junction = 2; //T
         printf("junc detected: %d  ", junction);
         p = 1;
         bot_spot_left();
         vTaskDelay(200/portTICK_PERIOD_MS);
         //bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
         //vTaskDelay(500/portTICK_PERIOD_MS);    
         //turnbot(turn);

         for(int i=0;i<5;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }

       else if (sensor_value[0] > 900 && sensor_value[1] > 900 && sensor_value[2] > 900  && sensor_value[3] >900 && sensor_value[4]>900) 
       {
         junction = 1; //+
         printf("junc detected: %d  ", junction);
         p = 1;
         bot_spot_left();
         vTaskDelay(200/portTICK_PERIOD_MS);
         //bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
         //vTaskDelay(500/portTICK_PERIOD_MS);    
         //turnbot(turn);

         for(int i=0;i<5;i++)
         {
             printf("sensor value[%d]: %f  ", i, sensor_value[i]);
         }
         printf("\n");
       }


       
       
       
    //     else if (sensor_value[0] < 100 && sensor_value[1] < 100 && sensor_value[2] < 100 && sensor_value[3] < 100 && test == 1)
    //    {
    //       turn = 'B';
    //       printf("turn detected: %c  ", turn);
    //       bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    //       bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,70,70);
    //       vTaskDelay(200/portTICK_PERIOD_MS);
	//       bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
	
    //      turnbot(turn);

    //      for(int i=0;i<4;i++)
    //      {
    //          printf("sensor value[%d]: %f  ", i, sensor_value[i]);
    //      }
    //      printf("\n");
    //    }
    //    else
    //    {
    //     calculate_error();
    //     calculate_correction();
        
    //     left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    //     right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
         
    //     for(int i=0;i<4;i++)
    //     {
    //         printf("sensor value[%d]: %f  ", i, sensor_value[i]);
    //     }    
    //     printf("\n");
    //     bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);   

    //    }
       
    }
}

void app_main()
{
    xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}