/*	
 *	Camera library - internal 2017/18
 *	Mode 		: ov7725
 *  Author  : Emmett Yim
 *  Contact : yhyim@ust.hk 
 *						66816823
 *	Lib			: camera.h
 */

#ifndef __OV7725_H
#define __OV7725_H

#include "stm32f10x.h"
#include "ticks.h"
#include "ov7725define.h"
#include "tft.h"


//sccb
uint8_t cameraSccbInit(ImageType outputType);
uint8_t sccbWriteByte(uint16_t address,uint8_t data);
uint8_t sccbReadByte(uint8_t* buffer, uint16_t length, uint8_t address);
//ov7725
uint8_t cameraInit(ImageType outputType);
uint8_t cameraReceiveFrame(void);
uint8_t cameraCaptureFrame(void);
//flipping RGB to BGR or BGR to RGB
uint16_t flipRB(uint16_t input);
//changing 8bit grey scale to 16bit grey scale
uint16_t greyScale8bitTo16bit(uint8_t);
//camera testing function with tft display
void cameraTestTftDisplay(void);


// -image colour, resolution setting-
//sccbWriteByte(COM7,);

// -image size setting-
//sccbWriteByte(HSTART,);
//sccbWriteByte(HSIZE,);
//sccbWriteByte(VSTRT,);
//sccbWriteByte(VSIZE,);
//sccbWriteByte(HREF,);
//sccbWriteByte(HOutSize,);
//sccbWriteByte(VOutSize,);
//sccbWriteByte(EXHCH,);

// -digital effect-
//sccbWriteByte(SDE,);

// -YUV setting-
//sccbWriteByte(UFix,);
//sccbWriteByte(VFix,);

// -Brightness-
//sccbWriteByte(BRIGHT,);

// -Contrast-
//sccbWriteByte(CNST,);

#endif //__OV7725_H
