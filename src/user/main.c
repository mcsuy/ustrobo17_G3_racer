//good luck
#include "main.h"

#define IMAGE_W 80
#define IMAGE_L 60

extern uint8_t image[IMAGE_L][IMAGE_W];
//extern uint16_t colourImage[IMAGE_L][IMAGE_W];

u32 ticks = 0;

int main(){
  //initialization
	ticks_init();
	ticks_reset();
	
	button_init();
	uint8_t buttonPrevious[3] = { 1, 1, 1 };
	
	u8 orientation = 0;
	u16 in_bg_color = BLACK;
	u16 in_text_color = WHITE;
	u16 in_text_color_sp = YELLOW;
	tft_init(orientation, in_bg_color, in_text_color, in_text_color_sp);
	
	ImageType outputType = BlackNWhite; //black and right
	while(!cameraInit(outputType)){}
	
	while(1)
	{
		if(ticks != get_ticks())
		{
			ticks = get_ticks();

			if(ticks%10 ==1)
			{
				cameraCaptureFrame();
			}
			
			if(ticks%10 ==3)
			{
				cameraReceiveFrame();
			}
			
			if(ticks % 20 == 11)
			{
				
			}
		}
		
	}	
}
