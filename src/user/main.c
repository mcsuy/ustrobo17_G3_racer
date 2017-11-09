//good luck
#include "main.h"

#define IMAGE_W 80
#define IMAGE_L 60

extern uint8_t image[IMAGE_L][IMAGE_W];
//extern uint16_t colourImage[IMAGE_L][IMAGE_W];

u32 ticks = 0;

void tft_displayBandW()
{

		tft_write_command(0x2a);		// Column addr set
		tft_write_data(0x00);
		tft_write_data(0x1A); 				// X START
		tft_write_data(0x00);
		tft_write_data(0x19+IMAGE_W); 			// X END
		tft_write_command(0x2b);		// Row addr set
		tft_write_data(0x00);
		tft_write_data(0x19);				// Y START
		tft_write_data(0x00);
		tft_write_data(0x19+IMAGE_L);			// Y END
		tft_write_command(0x2c); 		// write to RAM*/
		for(uint16_t j=0;j<IMAGE_L;j++)
			for(uint16_t i=0;i<IMAGE_W;i++)
			{
				if(image[j][i]){
						tft_write_data(0xFF);
            tft_write_data(0xFF);
				}else{
					  tft_write_data(0x00);
            tft_write_data(0x00);
				}
			}
}
u8 InitialValue;
u8 median[60];
u8 midpoint(){
	for (u8 y = 0; y <= 60; y++)
	{
		for (u8 x = 0; x <= 80; x++)
		{
			if ((image[y][x+1]-image[y][x])>0)
			{
				InitialValue = x+1;
			}
			if ((image[y][x+1]-image[y][x])<0)
			{
				median[y] = (1/2)*(x+1+InitialValue);
			}
		}
	}
	
}

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
	uart_init(UART1,115200);
	ImageType outputType = BlackNWhite; //black and right
	while(!cameraInit(outputType)){}
	uint32_t t0=get_ticks();
	u8 take_flags =0;
	while(1)
	{

			if(get_ticks()%10 ==1)
			{
				if(cameraCaptureFrame()==SUCCESS && take_flags == 0)
				{
					t0=get_ticks();
					take_flags = 1;
				}
				
			}
			
			if(get_ticks()%10 >=2 ||get_ticks()%10 <=5)
			{
				if(cameraReceiveFrame()==SUCCESS && take_flags ==1)
				{
					take_flags = 2;
				}
			}
			
			if(get_ticks()%12 >=8)
			{
				if(take_flags == 2){
					tft_prints(0,9,"%dms per frame",get_ticks() - t0);
					tft_update();
					tft_displayBandW();
					take_flags = 0;
				}
				
			}
	}	
}
