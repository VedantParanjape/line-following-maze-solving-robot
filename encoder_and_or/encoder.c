#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

//Components
#include "MPU.h"
#include "SRA18.h"
#include "TUNING.h"

adc1_channel_t channel[6] = {ADC_CHANNEL_7,ADC_CHANNEL_6,ADC_CHANNEL_0,ADC_CHANNEL_3,ADC_CHANNEL_4, ADC_CHANNEL_5};
int inputVal = 0;
int count1 =0, count2=0, flag1=0, flag2=0,former1=0, former2=0, current1=0, current2=0; //do not put these declarations in while(1)
int sensor_value[20];
int adc_reading[6];

void drive_task(void *arg)
{

	enable_buttons();	//Enable the buttons connected to GPIO 2 and GPIO 5
	mcpwm_initialize();	//Initialize PWM to control speed of motors

	while(1)
	{


		for(int i=0; i<6; i++)
		{
			adc_reading[i] = adc1_get_raw(channel[i]);
			//printf("RAW %d: %d\t",i,adc_reading[i]);
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
        printf("RAW %d: %d\t",i, sensor_value[i] );
        //vTaskDelay(20/portTICK_PERIOD_MS);
      if(i==5)
      {

        if(adc_reading[i]<100)
        {
        printf("black detected ok...\n"); 
        flag1=1;
        
        }
        else 
        {
          flag1=0;     
          printf("Am on white....\n");

        }
        printf("out of condition");
  
        current1 = flag1;

        //count1=0;
        if( (current1!=former1) && (flag1 == 1) )
        {count1++;}
      
        former1 = current1;
        printf("\n count = %d",count1);
        printf("\n");
        //vTaskDelay(200);
        }
		}		
		printf("\n");
	
		/*
			Make the Bot go forward, at a PWM of 80 if button 1 is pressed
		*/
	//	if(pressed_switch(BUTTON_1))
	//	{
			
		//	printf("%s\n","BOT FORWARD");

			bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 100, 100);
			//bot_forward(MCPWN_UNIT_)	
	//	}

		/*
			Make the Bot go backward, at a PWM of 80 if button 1 is pressed
		*/
		//if(pressed_switch(BUTTON_2))
		//{
			
		//	printf("%s\n","BOT BACKWARD");  

		//	bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 800, 800);	//Make the Bot go backward, at a PWM of 80 if button 2 is pressed
		//}

		/*
			If no buttons are pressed, stop the bot
		*/

	   /* 	else 
		{
			bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
			printf("%s\n","BOT STOPPED");
		}*/
		//vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	
}

void app_main()
{
	/*
		Basic Function for task creation
	*/
    xTaskCreate(&drive_task,"drive_task",4096,NULL,1,NULL);
}
