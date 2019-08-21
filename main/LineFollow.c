#include <stdio.h>
#include <time.h>
#include <math.h>

#include "SRA18.h"
#include "MPU.h"
#include "TUNING.h"

#define t_turn 2
#define plus_turn 3
#define left_t_turn 2
#define right_t_turn 2

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3}; //ADC CHANNEL 5 used for front sensor // ADC CHANNEL 4 used for encoder
int weights[4] = {3,1,-1,-3};

// used for turning code
int q,p,r,s;

// tag for checking turns
bool l_turn_present = false, r_turn_present = false, s_turn_present = false, dead_turn_present = false, on_a_junction = false; 
//

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
float higher_pwm_constrain = 80;
float left_pwm = 0, right_pwm = 0;

/*
 * Line Following PID Variables
 */
float error=0, prev_error, difference, cumulative_error, correction;

uint32_t adc_reading[4];
float sensor_value[4];

uint32_t front_sensor;
float front_sensor_value;

uint32_t encoder_sensor;
float encoder_sensor_value;

// for encoder
bool current = false, former = false;
int counter = 0;

static void read_sensors()
{
  for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
        // printf("raw[%d]: %d ",i, adc_reading[i]);
    }
    front_sensor = adc1_get_raw(ADC_CHANNEL_5);
}

static void read_encoder_sensor()
{
    encoder_sensor = adc1_get_raw(ADC_CHANNEL_4);
}

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
        // printf("fil[%d]: %f ", i, sensor_value[i]); 
    }
    front_sensor_value = map(front_sensor, 1700, 4000, 0, 1000);
    front_sensor_value = constrain(front_sensor_value,0,1000);
}

static void calc_encoder_sensor_values()
{
    encoder_sensor_value = map(encoder_sensor, 1700, 4000, 0, 1000);
    encoder_sensor_value = constrain(encoder_sensor_value, 0, 1000);
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

void follow_path()
{
    s = 1;

    while(s == 1)
    {
        read_sensors();
        calc_sensor_values();
        calculate_error();
        calculate_correction();
        
        left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
        right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
         
        for(int i=0;i<4;i++)
        {
            printf("sensor value[%d]: %f  ", i, sensor_value[i]);
        }

        printf("front: %f", front_sensor_value);    
        printf("\n");
        bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);  

        if (!(sensor_value[0] < 400  && sensor_value[1] > 900  && sensor_value[2] > 900  && sensor_value[3] < 400))
        {
            s = 0;
        }
    }
}

void turnbot(char d)
{
    q = 1;
    p = 1;
    r = 1;

    if (d == 'L')
    {
        while (q==1)
        {   
            read_sensors();
            calc_sensor_values();

            if((sensor_value[0] > 700) || (sensor_value[1] > 700) || (sensor_value[2] > 700) || (sensor_value[3] > 700))
            {
                q=0;
            }
            
            bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0,85,85);
        } 
    }

    else if(d == 'R')
    {
        while (p==1)
        {   
            read_sensors();
            calc_sensor_values();

            if((sensor_value[0] > 700) || (sensor_value[1] > 700) || (sensor_value[2] > 700) || (sensor_value[3] > 700))
            {
                p=0;
            }
            
            bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0,85,85);
        } 
    
    }

    else if (d == 'B')
    { 
        while (r==1)
        {   
            read_sensors();
            calc_sensor_values();

            if((sensor_value[0] > 700) || (sensor_value[1] > 700) || (sensor_value[2] > 700) || (sensor_value[3] > 700))
            {
                r=0;
            }

            bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0,85,85);
        } 
    
    } 
}

void odometry()
{
    read_encoder_sensor();
    calc_encoder_sensor_values();

    if(encoder_sensor_value < 100)
    {
        current = true;
    }
    else
    {
        current = false;
    }

    if(current != former && current == true)
    {
        counter = counter + 1;
    }

    former = current;

    printf("encoder ticks count: %d\n", counter);
}

void save_to_flash()
{
    // function to final solution to the flash memory
}

void solve_maze()
{
    // function to travel the maze with solution in flash memory
}

// function to return to previous node
void return_back(char prev_turn)
{
    if(prev_turn == 'L')
    {
        turnbot('B');
        follow_path();

        vTaskDelay(200/portTICK_PERIOD_MS);
        bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(200/portTICK_PERIOD_MS);
        
        turnbot('L');
    }

    else if(prev_turn == 'R')
    {
        turnbot('B');
        follow_path();

        vTaskDelay(200/portTICK_PERIOD_MS);
        bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(200/portTICK_PERIOD_MS);
        
        turnbot('R');
    }

    else if(prev_turn == 'S')
    {
        turnbot('B');
        follow_path();

        vTaskDelay(200/portTICK_PERIOD_MS);
        bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(200/portTICK_PERIOD_MS);
        
        turnbot('B');
    }
}

/* there are 8 possible cases:
1. left turn
2. right turn
3. T junction
4. left and straight turn
5. right and straight turn
6. Four way junction
7. u turn
8. end of maze
*/

void maze_discovery()
{
 // this function is called once a node is found, it process the given information and then gives the turn to take

 // uses the Variables
 // 1. l_turn_present
 // 2. s_turn_present
 // 3. r_turn_present 
 // 4. dead_turn_present
}

void line_follow_task(void *arg)
{
  mcpwm_initialize();
  char turn;
  
  while(1)
  {
    read_sensors();
    calc_sensor_values();
    calculate_error();
    calculate_correction();
        
    left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
         
    for(int i=0;i<4;i++)
    {
        printf("sensor value[%d]: %f  ", i, sensor_value[i]);
    }
    printf("front: %f", front_sensor_value);    
    printf("\n");
    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);  

    on_a_junction = false; // set it to false, it will become true only if a junction detected

    if(sensor_value[0] < 400  && sensor_value[1] > 900  && sensor_value[2] > 900  && sensor_value[3] > 900) 
    {
        // case 1. left turn
        //    sub case. when there's a straight turn.
        turn = 'L';
        printf("turn detected: %c  ", turn);
        q=1;
        
        on_a_junction = true;

        vTaskDelay(200/portTICK_PERIOD_MS);
        bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(200/portTICK_PERIOD_MS);
        
        if(front_sensor_value < 200)
        {
            // simple left turn
            l_turn_present = true;
            s_turn_present = false;
            r_turn_present = false;
            dead_turn_present = false;

            turnbot(turn);
        }
        else if(front_sensor_value > 500)
        {
            // when a straight is also possible
            l_turn_present = true;
            s_turn_present = true;
            r_turn_present = false;
            dead_turn_present = false;

            printf("left T turn detected\n");
        }
        
        for(int i=0;i<4;i++)
        {
            printf("sensor value[%d]: %f  ", i, sensor_value[i]);
        }

        printf("front: %f", front_sensor_value);
        printf("\n");
    }

    else if (sensor_value[0] > 900 && sensor_value[1] > 900 && sensor_value[2] > 900  && sensor_value[3] < 400) 
    {
        // case 2. right turn
        //    sub case. when there's a straight turn.
        turn = 'R';
        printf("turn detected: %c  ", turn);
        p = 1;
        
        on_a_junction = true;

        vTaskDelay(200/portTICK_PERIOD_MS);
        bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(200/portTICK_PERIOD_MS);    
        
        if(front_sensor_value < 200)
        {
            // simple right turn
            l_turn_present = false;
            s_turn_present = false;
            r_turn_present = true;
            dead_turn_present = false;

            turnbot(turn);
        }
        else if(front_sensor_value > 500)
        {
            // when a straight turn is also possible
            l_turn_present = false;
            s_turn_present = true;
            r_turn_present = true;
            dead_turn_present = false;

            printf("right T turn detected\n");
        }
        
        for(int i=0;i<4;i++)
        {
            printf("sensor value[%d]: %f  ", i, sensor_value[i]);
        }
        printf("front: %f", front_sensor_value);
        printf("\n");
    }
       
    else if (sensor_value[0] < 100 && sensor_value[1] < 100 && sensor_value[2] < 100 && sensor_value[3] < 100) 
    {
        // case 7. u-turn
        turn = 'B';
        printf("turn detected: %c  ", turn);
        r = 1;

        on_a_junction = true;

        vTaskDelay(200/portTICK_PERIOD_MS);
        bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(200/portTICK_PERIOD_MS); 

        l_turn_present = false;
        s_turn_present = false;
        r_turn_present = false;
        dead_turn_present = true;
               
        turnbot(turn);

        for(int i=0;i<4;i++)
        {
            printf("sensor value[%d]: %f  ", i, sensor_value[i]);
        }
        printf("front: %f", front_sensor_value);
        printf("\n");
    }  

    else if (sensor_value[0] > 900 && sensor_value[1] > 900 && sensor_value[2] > 900 && sensor_value[3] > 900) 
    {
        // case 3. T turn
        //    sub case. when there's a straight path ahead.
        //    sub case. when we reach the end point
        //       condition. all sensors white and the extra sensor also white which is black otherwise
        on_a_junction = true;
         
        if(front_sensor_value < 200)
        {
            // T turn 
            l_turn_present = true;
            s_turn_present = false;
            r_turn_present = true;
            dead_turn_present = false;
            printf("t turn detected\n");
        }
        else if(front_sensor_value > 500)
        {
            // + turn
            l_turn_present = true;
            s_turn_present = true;
            r_turn_present = true;
            dead_turn_present = false;
            printf("+ turn detected\n");
        }
    }

    // if on_a_junction = true, call maze_discovery() function here, it will tell the turn to take the give the handle back to the low iq while loop
  }
}

void app_main()
{
    xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}