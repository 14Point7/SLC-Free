//----------------------------------------------------------------------------
// C main line latest
//----------------------------------------------------------------------------
#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules

extern BYTE ADC_Counter; //Variable Declared in ADCININT.asm
extern BYTE Ri_Max_x1; //Variable Declared in ADCININT.asm
extern BYTE Ri_Min_x1; //Variable Declared in ADCININT.asm
extern BYTE ip_x1; //Variable Declared in ADCININT.asm
extern BYTE ADC_IF; //Variable Declared in ADCININT.asm
extern BYTE Sleep_Counter; //Variable Declared in SleepTimerINT.asm

//#define Vout_Lookup_Counter_Set 25
#define LCD_Counter_Set 50

#define Lin_Out
#define NB_Out

//#define PID_Tune
#define Ri_Filter_Strength 4
#define ip_Filter_Strength 4

#ifdef PID_Tune
	INT Ri_Mid_Target= 271; //256+14.769 = 2.53v = 2.08v(VGND) + 0.45v, PID Target Voltage of Nermest Cell 
	INT Ia_PID_Kp = 35;
	INT Ia_PID_Ki = 8;
	INT Ia_Output_Bias = 256;
	INT Ri_Delta_Target=143; //PID Target peak to peak Voltage of Nermest Cell
	INT Heater_PID_Kp =-72;
	INT Heater_PID_Ki =-4;
	INT Heater_PID_Output_Bias =128;
#else
	#define Ri_Mid_Target 271 //256+14.769 = 2.53v = 2.08v(VGND) + 0.45v, PID Target Voltage of Nermest Cell
	#define Ia_PID_Kp 25
	#define Ia_PID_Ki 4
	#define Ia_Output_Bias 256
	#define Ri_Delta_Target 143 //PID Target peak to peak Voltage of Nermest Cell
	#define Heater_PID_Kp -72
	#define Heater_PID_Ki -4
	#define Heater_PID_Output_Bias 128
#endif

#ifdef Lin_Out
	
	#define ip_to_Vout_Lookup_Start 135
	#define ip_to_Vout_Lookup_Size 158
	const BYTE ip_to_Vout_Lookup[ip_to_Vout_Lookup_Size] = {0,1,1,2,3,4,4,5,6,7,8,8,9,10,11,12,12,13,14,15,16,17,17,18,19,20,21,22,22,23,24,25,26,27,27,28,29,30,31,32,33,34,34,35,36,37,38,39,40,41,42,43,44,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,76,77,78,79,80,81,82,83,84,86,87,88,89,90,91,92,94,95,96,97,98,100,101,102,103,105,106,107,108,110,111,112,113,115,116,117,119,120,123,126,129,132,135,138,141,144,147,151,154,157,161,164,168,171,175,178,182,186,190,194,197,201,206,210,214,218,222,227,231,236,240,245,250,255}; 
#endif

	#define ip_to_Lambda_Lookup_Start 135
	#define ip_to_Lambda_Lookup_Size 158
	
	const BYTE ip_to_Lambda_Lookup[ip_to_Lambda_Lookup_Size]={68,68,68,69,69,69,69,69,70,70,70,70,70,71,71,71,71,72,72,72,72,72,73,73,73,73,74,74,74,74,74,75,75,75,75,76,76,76,76,76,77,77,77,77,78,78,78,78,79,79,79,79,80,80,80,80,81,81,81,81,82,82,82,82,83,83,83,83,84,84,84,85,85,85,85,86,86,86,86,87,87,87,88,88,88,88,89,89,89,90,90,90,91,91,91,91,92,92,92,93,93,93,94,94,94,95,95,95,96,96,96,97,97,97,98,98,98,99,99,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,136};

	#define Ri_Delta_to_Temperature_C_Start 113
	#define Ri_Delta_to_Temperature_C_Size 75
	#define Ri_Delta_to_Temperature_C_Offset 740
	
  
INT Ri_Mid_Error_Sum=0;
INT Ri_Mid;
INT Ri_Delta_Error_Sum=0;
INT Ri_Delta;
#define Ia_PID_Counter_Set 1
#define Heatup_Counter_Set 100
#define Heater_PID_Counter_Set 25
BYTE Heatup_Heater_Output=100;

INT Ri_Delta_Error;
INT Heater_Output;
INT Heater_Pout;
INT Heater_Iout;

//PID Controller, Input is Ri_Delta (The Peak to peak voltage on the Vs Port of the sensor) output is PWM8_Heater, PWM8_Heater is controlled such that Ri_Delta is maintained at 80 (80 =750C)
//Only a PI controller is used as adding the D term does not effect performance 
//Everything is inflated by a factor of 4 so that floating point is avioded
void Heater_PID(void) // Ned to run this when counter >50
{
	Ri_Delta_Error=Ri_Delta_Target-Ri_Delta;
	//Put limits on Error so PID does not go Fubar, and also so that the worst case multiplication does not overflow
	if (Ri_Delta_Error>56)
	{
		Ri_Delta_Error=56;
	}
	if (Ri_Delta_Error<-56)
	{
		Ri_Delta_Error=-56;
	}
	Heater_Pout=(Heater_PID_Kp*Ri_Delta_Error)/16;
	Ri_Delta_Error_Sum=Ri_Delta_Error_Sum+Ri_Delta_Error;
	//If the sensor is too hot, then disreguard the Integratal contribution and just use the proportional to quickly correct the sensor temperature
	/*
	if((Ri_Delta<60)&&(Ri_Delta_Error_Sum>0))
	{
		Ri_Delta_Error_Sum=0;
	}
	*/
	//Put limits on Error so PID does not go Fubar, and also so that the worst case multiplication does not overflow
	if (Ri_Delta_Error_Sum>1024)
	{
		Ri_Delta_Error_Sum=1024;
	}
	if (Ri_Delta_Error_Sum<-1024)
	{
		Ri_Delta_Error_Sum=-1024;
	}
	Heater_Iout=(Heater_PID_Ki*Ri_Delta_Error_Sum)/16;
	if (Heatup_Heater_Output<255) // if Heatup_Heater_Output is < 255 that means the unit just turned on and to give control to the heatup routine
	{
		Heater_Output=Heatup_Heater_Output;	
	}
	else
	{	
		Heater_Output=Heater_PID_Output_Bias+Heater_Pout+Heater_Iout;			
	}
	//Heater_Output=Heater_PID_Output_Bias+Heater_Pout+Heater_Iout;	
	if (Heater_Output<0)
	{
		Heater_Output=0;
	}
	if (Heater_Output>255)
	{
		Heater_Output=255;
	}


	PWM8_Heater_WritePulseWidth(Heater_Output);
	//PWM8_Heater_WritePulseWidth(200);
}

INT Ri_Mid_Error;
INT Ia_Output;
INT Ia_Pout;
INT Ia_Iout;
//PID Controller, Input is Ri_Mid (The average voltage on the Vs Port of the sensor) output is DAC9_Ia, DAC9_Ia sink/sources current to the Sensor pump cell such that Ri_Mid is is the same voltage as Vref
//Only a PI controller is used as adding the D term does not effect performance 
//Everything is inflated by a factor of 4 so that floating point is avioded
void Ia_PID(void)
{


	Ri_Mid_Error=Ri_Mid_Target-Ri_Mid;
	//Put limits on Error so PID does not go Fubar, and also so that the worst case multiplication does not overflow
	if (Ri_Mid_Error>163)
	{
		Ri_Mid_Error=163;
	}
	if (Ri_Mid_Error<-163)
	{
		Ri_Mid_Error=-163;
	}
	Ia_Pout=(Ia_PID_Kp*Ri_Mid_Error)/16;
	Ri_Mid_Error_Sum=Ri_Mid_Error_Sum+Ri_Mid_Error;
	//Put limits on Error so PID does not go Fubar, and also so that the worst case multiplication does not overflow
	if (Ri_Mid_Error_Sum>1020)
	{
		Ri_Mid_Error_Sum=1020;
	}
	if (Ri_Mid_Error_Sum<-1020)
	{
		Ri_Mid_Error_Sum=-1020;
	}
	Ia_Iout=(Ia_PID_Ki*Ri_Mid_Error_Sum)/16;
	Ia_Output=Ia_Output_Bias+Ia_Pout+Ia_Iout;
	//9 Bit Dac so only 0-511 is allowed, for some reason i put the limit at 510, I foget exactly why.
	if (Ia_Output<0)
	{
		Ia_Output=0;
	}
	if (Ia_Output>510)
	{
		Ia_Output=510;
	}
	DAC9_Ia_WriteStall (Ia_Output);
	//DAC9_Ia_WriteStall (256);
}
INT IIR_Int(INT Vin, INT Vout, BYTE A)
{
	return (Vout + (Vin - Vout)/A);
}

char toChar(BYTE base10)
{
	char ascii;
	switch (base10)
	{
		case 0:
			ascii='0';
			break;
		case 1:
			ascii='1';
			break;
		case 2:
			ascii='2';
			break;
		case 3:
			ascii='3';
			break;
		case 4:
			ascii='4';
			break;
		case 5:
			ascii='5';
			break;
		case 6:
			ascii='6';
			break;
		case 7:
			ascii='7';
			break;
		case 8:
			ascii='8';
			break;
		case 9:
			ascii='9';
	}
	return (ascii);
}


BYTE Ia_PID_Counter=0;

BYTE Heater_PID_Counter=0;
BYTE LCD_Counter=0;
BYTE Heatup_Counter=0;
INT Ri_Min,Ri_Max;
INT ip;

//INT LSU_Temperature_C;
char Str1[] = "Lambda:x.xx xxxC";
char Str2[] = "Heating..."; 

void main(void)
{
	//unsigned long temp_ulong;
	INT temp_int;

	AMUX4_0_InputSelect(AMUX4_0_PORT0_1);        
   	AMUX4_1_InputSelect(AMUX4_1_PORT0_0);
   	INSAMP_Start(INSAMP_LOWPOWER); 
    ADCINC_Start(ADCINC_HIGHPOWER);      
   	DAC9_Ia_Start(DAC9_Ia_HIGHPOWER);
	DAC6_VGND_Start(DAC6_VGND_MEDPOWER);
	DAC6_VGND_WriteStall (31);
    PWM8_Vout_DisableInt();  
    PWM8_Vout_Start();     
    PWM8_Heater_DisableInt();  
    PWM8_Heater_Start();
	PWM8_NB_Out_DisableInt();  
    PWM8_NB_Out_Start(); 
	ADCINC_GetSamples(0);
	SleepTimer_Start();  
   	SleepTimer_SetInterval(SleepTimer_512_HZ);  
   	SleepTimer_EnableInt();   
	M8C_EnableGInt ;  
	LCD_Start();                  // Initialize LCD
	LCD_InitBG(LCD_SOLID_BG);

	LCD_Position(0,0);
	LCD_PrString(Str2);
	//default to lambda 1 during heatup
	PWM8_Vout_WritePulseWidth(119);
	
	for(;;)
    {
		//temp_ulong++;
		if ((ADC_IF&1)==1)
		{
			ADC_IF=ADC_IF&254;
			Ri_Min=IIR_Int(Ri_Min_x1*2,Ri_Min,Ri_Filter_Strength);
			Ri_Delta=Ri_Max-Ri_Min;
			Ri_Mid=(Ri_Max+Ri_Min)/2;
		}
		if ((ADC_IF&2)==2)
		{
			ADC_IF=ADC_IF&253;
			Ri_Max=IIR_Int(Ri_Max_x1*2,Ri_Max,Ri_Filter_Strength);
			Ri_Delta=Ri_Max-Ri_Min;
			Ri_Mid=(Ri_Max+Ri_Min)/2;
		}
		if ((ADC_IF&4)==4)
		{
			ADC_IF=ADC_IF&251;
			ip=IIR_Int(ip_x1*2,ip,ip_Filter_Strength);
		}
		Ia_PID_Counter+=Sleep_Counter;
		Heater_PID_Counter+=Sleep_Counter;
		Heatup_Counter+=Sleep_Counter;
		LCD_Counter+=Sleep_Counter;
		Sleep_Counter=0;
		//1 thus 1.95ms @ 512hz
		if (Ia_PID_Counter>Ia_PID_Counter_Set)
		{
			Ia_PID_Counter=0;
			Ia_PID();
		}
		//25 thus 48.82ms @ 512hz
		if (Heater_PID_Counter>Heater_PID_Counter_Set && Heatup_Heater_Output >= 255)
		{
			Heater_PID_Counter=0;
			Heater_PID();
		
			temp_int=ip-ip_to_Vout_Lookup_Start;
			if (temp_int<0)
			{
				temp_int=0;
			}
			if (temp_int>(ip_to_Vout_Lookup_Size-1))
			{
				temp_int=(ip_to_Vout_Lookup_Size-1);
			}
	
			PWM8_Vout_WritePulseWidth(ip_to_Vout_Lookup[temp_int]);
			
			#ifdef NB_Out
			    temp_int=23;//0.45v
				if (ip<251) // 251 =0.9797787392968
				{
					temp_int=46; //0.9v
					
				}
			    if (ip>259) //259 = 1.02295956968912
				{
					temp_int=0; //0v
				}
					
				PWM8_NB_Out_WritePulseWidth(temp_int);
			#endif
			
		}
		//50 thus 97.66ms @ 512hz
		if (LCD_Counter>LCD_Counter_Set && Heatup_Heater_Output >= 255)
		{
			LCD_Counter=0;
		
			temp_int = ip-ip_to_Lambda_Lookup_Start;
			if (temp_int<0)
			{
				temp_int=0;
			}
			if (temp_int>(ip_to_Lambda_Lookup_Size-1))
			{
				temp_int=(ip_to_Lambda_Lookup_Size-1);
			}
			temp_int=ip_to_Lambda_Lookup[temp_int];
			
			LCD_DrawBG(1,0,16,(temp_int -56));
			
			LCD_Position(0,0);
			Str1[7]=toChar(temp_int/100);
			Str1[9]=toChar((temp_int/10)%10);
			Str1[10]=toChar(temp_int%10);
			
			temp_int=Ri_Delta-Ri_Delta_to_Temperature_C_Start;
			if (temp_int<0)
			{
				temp_int=0;
			}
			if (temp_int>(Ri_Delta_to_Temperature_C_Size-1))
			{
				temp_int=(Ri_Delta_to_Temperature_C_Size-1);
			}
			
			temp_int = 749 + temp_int; 
			Str1[12]=toChar(temp_int/100);
			Str1[13]=toChar((temp_int/10)%10);
			Str1[14]=toChar(temp_int%10);
			LCD_PrString(Str1);
			 
		}
		
		if (Heatup_Heater_Output<255)
		{   
			//100 thus 195.31ms @ 512hz
			if (Heatup_Counter>Heatup_Counter_Set)
			{
				Heatup_Counter=0;
				Heatup_Heater_Output++;
				//LCD_DrawBG(1,0,16,(Heatup_Heater_Output-100));
				
			}
			
		
			if ((Ri_Min>50) && (Ri_Max<475) && (Ri_Delta<Ri_Delta_Target))
			{
				Heatup_Heater_Output=255;
				Ri_Delta_Error_Sum=0;
			}
			
		}
	}
}
