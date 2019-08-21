//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------
#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
extern BYTE ADC_Counter; //Variable Declared in ADCININT.asm
extern BYTE Ri_Max_x1; //Variable Declared in ADCININT.asm
extern BYTE Ri_Min_x1; //Variable Declared in ADCININT.asm
extern BYTE ip_x1; //Variable Declared in ADCININT.asm
extern BYTE ADC_IF; //Variable Declared in ADCININT.asm
extern BYTE Sleep_Counter; //Variable Declared in SleepTimerINT.asm

#define Vout_Lookup_Counter_Set 25
#define LCD_Counter_Set 50

#define Lin_Out
#define NB_Out
#define LCD_Lambda_Graph
#define LCD_Temperature_Graph
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

#ifdef LCD_Lambda_Graph
	#define ip_to_Lambda_Lookup_Start 135
	#define ip_to_Lambda_Lookup_Size 158
	const BYTE ip_to_Graph_Lookup[ip_to_Lambda_Lookup_Size]={8,8,8,9,9,9,9,9,10,10,10,10,10,11,11,11,11,12,12,12,12,12,13,13,13,13,14,14,14,14,14,15,15,15,15,16,16,16,16,16,17,17,17,17,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,21,22,22,22,22,23,23,23,23,24,24,24,25,25,25,25,26,26,26,26,27,27,27,28,28,28,28,29,29,29,30,30,30,31,31,31,31,32,32,32,33,33,33,34,34,34,35,35,35,36,36,36,37,37,37,38,38,38,39,39,39,40,40,41,42,42,43,44,45,46,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,70,71,72,73,75,76};
#endif

#ifdef LCD_Temperature_Graph
	#define Ri_Delta_to_Temperature_C_Start 113
	#define Ri_Delta_to_Temperature_C_Size 75
	const BYTE Ri_Delta_to_Graph[Ri_Delta_to_Temperature_C_Size]={80,78,77,75,74,73,71,70,68,67,65,64,63,61,60,59,58,56,55,54,52,51,50,49,48,46,45,44,43,42,41,40,39,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,18,17,16,15,14,13,12,11,11,10,9,8,7,6,5,5,4,3,2,1,1,0};
#endif



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


BYTE Ia_PID_Counter=0;
BYTE Vout_Lookup_Counter=0;
BYTE Heater_PID_Counter=0;
BYTE LCD_Counter=0;
BYTE Heatup_Counter=0;
INT Ri_Min,Ri_Max;
INT ip,ip_Justified;
BYTE Lambda_x100;
INT LSU_Temperature_C;
char Str1[] = "Lambda=x.xx";
char Str2[] = "Temperature=xxxC"; 
void main(void)
{
	unsigned long temp_ulong;
	INT temp_int,temp_int2;
	BYTE temp_byte;
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

	for(;;)
    {
		temp_ulong++;
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
		Vout_Lookup_Counter+=Sleep_Counter;
		LCD_Counter+=Sleep_Counter;
		Sleep_Counter=0;
		if (Ia_PID_Counter>Ia_PID_Counter_Set)
		{
			Ia_PID_Counter=0;
			Ia_PID();
		}
		if (Heater_PID_Counter>Heater_PID_Counter_Set)
		{
			Heater_PID_Counter=0;
			Heater_PID();
		}
		if (Vout_Lookup_Counter>Vout_Lookup_Counter_Set)
		{
			Vout_Lookup_Counter=0;
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
				temp_byte=23;//0.45v
				if (ip<251) // 251 =0.9797787392968
				{
					temp_byte=46; //0.9v
					
				}
				if (ip>259) //259 = 1.02295956968912
				{
					temp_byte=0; //0v
				}
				PWM8_NB_Out_WritePulseWidth(temp_byte);
			#endif
			
		}
		if (LCD_Counter>LCD_Counter_Set)
		{
			LCD_Counter=0;
			#ifdef LCD_Lambda_Graph
				temp_int=ip-ip_to_Lambda_Lookup_Start;
				if (temp_int<0)
				{
					temp_int=0;
				}
				if (temp_int>(ip_to_Lambda_Lookup_Size-1))
				{
					temp_int=(ip_to_Lambda_Lookup_Size-1);
				}
				Lambda_x100=ip_to_Graph_Lookup[temp_int];
				LCD_DrawBG(0,0,16,Lambda_x100);
			#endif
			
			#ifdef LCD_Temperature_Graph
				temp_int=Ri_Delta-Ri_Delta_to_Temperature_C_Start;
				if (temp_int<0)
				{
					temp_int=0;
				}
				if (temp_int>(Ri_Delta_to_Temperature_C_Size-1))
				{
					temp_int=(Ri_Delta_to_Temperature_C_Size-1);
				}
				LSU_Temperature_C=Ri_Delta_to_Graph[temp_int];
				LCD_DrawBG(1,0,16,LSU_Temperature_C);
			#endif
		}
		if (Heatup_Heater_Output<255)
		{   
			if (Heatup_Counter>Heatup_Counter_Set)
			{
				Heatup_Counter=0;
				Heatup_Heater_Output++;
			}
			if ((Ri_Min>50) && (Ri_Max<475) && (Ri_Delta<Ri_Delta_Target))
			{
				Heatup_Heater_Output=255;
				Ri_Delta_Error_Sum=0;
			}
		}
	}
}
