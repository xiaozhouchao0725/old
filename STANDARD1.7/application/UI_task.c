
#include "UI_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "remote_control.h"
#include "referee.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"
#include "chassis_task.h"
extern int anglesr;
extern ext_game_robot_state_t robot_state;
extern gimbal_behaviour_e gimbal_behaviour;
extern ext_bullet_remaining_t bullet_remaining_t;
extern chassis_move_t chassis_move;
extern int8_t R,QA,ET,turn_flags,sharp_angle,feipo;
extern int8_t CBLOOD;
extern int8_t FCHO;
extern int8_t GR;
Graph_Data Aim[11];
String_Data strSHOOT,strCAP,strPILL,strMODE,strBLOOD,strSUPREME,strDANCANG,strFlying_slopes,strRF_bursts;
fp32 power;
void UI_task(void const *pvParameters)
{
	//剩余发弹量
			int16_t pill;
			char tmp1[30]={0},tmp2[30]={0},tmp3[30]={0},tmp4[30]={0},tmp5[30]={0},tmp6[30]={0},tmp7[30]={0},tmp8[30]={0},tmp9[30]={0};
			memset(&strCAP,0,sizeof(strCAP));
			memset(&strSHOOT,0,sizeof(strSHOOT));
			memset(&strPILL,0,sizeof(strPILL));
			memset(&strMODE,0,sizeof(strMODE));
			memset(&strBLOOD,0,sizeof(strBLOOD));
			memset(&strSUPREME,0,sizeof(strSUPREME));
			memset(&strDANCANG,0,sizeof(strDANCANG));
			memset(&strFlying_slopes,0,sizeof(strFlying_slopes));
			memset(&strRF_bursts,0,sizeof(strRF_bursts));	
			for(int k=0;k<11;k++)
			{
				memset(&Aim[k],0,sizeof(Aim[k]));
			}
//			memset(&Aim[10],0,sizeof(Aim[10]));			//清空图形数据
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,2,940,520,980,520);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Green,3,930,478,990,478);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Yellow,2,920,380,1000,380);
			Line_Draw(&Aim[8],"AL9",UI_Graph_ADD,6,UI_Color_Purplish_red,2,920,330,1000,330);
			//可通过宽度
			Line_Draw(&Aim[4],"AL5",UI_Graph_ADD,6,UI_Color_Green,2,440,0,627,443);
			Line_Draw(&Aim[5],"AL6",UI_Graph_ADD,6,UI_Color_Green,2,1480,0,1293,443);
			Line_Draw(&Aim[6],"AL7",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[7],"AL8",UI_Graph_ADD,7,UI_Color_Purplish_red,10,1700,600,1700,700);
			Line_Draw(&Aim[10],"AL11",UI_Graph_ADD,7,UI_Color_Cyan,0,1700,600,1800,600);
			Circle_Draw(&Aim[9],"CL9",UI_Graph_ADD,5,UI_Color_Orange,5,1700,600,100);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp2),2,860,300,tmp2);
			Char_Draw(&strPILL,"PILL",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp3),2,10,760,tmp3);
			Char_Draw(&strMODE,"MODE",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp4),2,860,250,tmp4);
			Char_Draw(&strBLOOD,"BLOOD",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp5),2,860,200,tmp5);
			Char_Draw(&strSUPREME,"SUPREME",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp6),2,860,150,tmp6);
			Char_Draw(&strDANCANG,"DANCANG",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp7),2,10,710,tmp7);
			Char_Draw(&strFlying_slopes,"Flying_slopes",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp8),2,10,660,tmp8);
			Char_Draw(&strRF_bursts,"RF_bursts",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp9),2,10,610,tmp9);
			Char_ReFresh(strCAP);
			vTaskDelay(25);
			Char_ReFresh(strSHOOT);
			vTaskDelay(25);
			Char_ReFresh(strPILL);
			vTaskDelay(25);
			Char_ReFresh(strMODE);
			vTaskDelay(25);
			Char_ReFresh(strBLOOD);
			vTaskDelay(25);
			Char_ReFresh(strSUPREME);
			vTaskDelay(25);
			Char_ReFresh(strDANCANG);
			vTaskDelay(25);
			Char_ReFresh(strFlying_slopes);
			vTaskDelay(25);
			Char_ReFresh(strRF_bursts);
			vTaskDelay(25);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
			vTaskDelay(25);
			UI_ReFresh(5,Aim[7],Aim[9],Aim[8],Aim[10]);	
				
	while(1)
	{
		pill=bullet_remaining_t.bullet_remaining_num_17mm;
		if(rc_ctrl.key.v&KEY_PRESSED_OFFSET_Z)
		{
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,2,940,520,980,520);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Green,3,930,478,990,478);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Yellow,2,920,380,1000,380);
			Line_Draw(&Aim[8],"AL9",UI_Graph_ADD,6,UI_Color_Purplish_red,2,920,430,1000,430);
			//可通过宽度
			Line_Draw(&Aim[4],"AL5",UI_Graph_ADD,6,UI_Color_Green,2,440,0,627,443);
			Line_Draw(&Aim[5],"AL6",UI_Graph_ADD,6,UI_Color_Green,2,1480,0,1293,443);
			Line_Draw(&Aim[6],"AL7",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[7],"AL8",UI_Graph_ADD,7,UI_Color_Purplish_red,10,1700,600,1700,700);
			Line_Draw(&Aim[10],"AL11",UI_Graph_ADD,7,UI_Color_Cyan,0,1700,600,1800,600);
			Circle_Draw(&Aim[9],"CL9",UI_Graph_ADD,5,UI_Color_Orange,5,1700,600,100);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp2),2,860,300,tmp2);
			Char_Draw(&strPILL,"PILL",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp3),2,10,760,tmp3);
			Char_Draw(&strMODE,"MODE",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp4),2,860,250,tmp4);
			Char_Draw(&strBLOOD,"BLOOD",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp5),2,860,200,tmp5);
			Char_Draw(&strSUPREME,"SUPREME",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp6),2,860,150,tmp6);
			Char_Draw(&strDANCANG,"DANCANG",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp7),2,10,710,tmp7);
			Char_Draw(&strFlying_slopes,"Flying_slopes",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp8),2,10,660,tmp8);
			Char_Draw(&strRF_bursts,"RF_bursts",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp9),2,10,610,tmp9);
			Char_ReFresh(strCAP);
			vTaskDelay(25);
			Char_ReFresh(strSHOOT);
			vTaskDelay(25);
			Char_ReFresh(strPILL);
			vTaskDelay(25);
			Char_ReFresh(strMODE);
			vTaskDelay(25);
			Char_ReFresh(strBLOOD);
			vTaskDelay(25);
			Char_ReFresh(strSUPREME);
			vTaskDelay(25);		
			Char_ReFresh(strDANCANG);
			vTaskDelay(25);
			Char_ReFresh(strFlying_slopes);
			vTaskDelay(25);
			Char_ReFresh(strRF_bursts);
			vTaskDelay(25);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
			vTaskDelay(25);
			UI_ReFresh(5,Aim[7],Aim[9],Aim[8],Aim[10]);	
		}
	
		sprintf(tmp1,"Cap:%.1f(%.2fV)",((get_capA.cap_voltage/1000.0-13.5)/10.5*100),get_capA.cap_voltage/1000.0);
		if(get_capA.cap_voltage > 16000.0f)
				Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
		else
				Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Orange,20,strlen(tmp1),2,860,100,tmp1);
		Char_ReFresh(strCAP);
		vTaskDelay(25);
		
		//摩擦轮
		if(R)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp2,"(R)SHOOT: ON");
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_Change,9,UI_Color_Orange,30,strlen(tmp2),2,860,300,tmp2);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp2,"(R)SHOOT:OFF");
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_Change,9,UI_Color_Green,30,strlen(tmp2),2,860,300,tmp2);
		}
		Char_ReFresh(strSHOOT);
		vTaskDelay(25);
		//换血
		if(CBLOOD)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp5,"(C)CBLOOD: ON");
			Char_Draw(&strBLOOD,"BLOOD",UI_Graph_Change,9,UI_Color_Orange,20,strlen(tmp5),2,860,200,tmp5);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp5,"(C)CBLOOD:OFF");
			Char_Draw(&strBLOOD,"BLOOD",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp5),2,860,200,tmp5);
		}
		Char_ReFresh(strBLOOD);
		vTaskDelay(25);
		//超电
		if(FCHO)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp6,"(F)SUP_CAP: ON");
			Char_Draw(&strSUPREME,"SUPREME",UI_Graph_Change,9,UI_Color_Orange,20,strlen(tmp6),2,860,150,tmp6);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp6,"(F)SUP_CAP:OFF");
			Char_Draw(&strSUPREME,"SUPREME",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp6),2,860,150,tmp6);
		}
		Char_ReFresh(strSUPREME);
		vTaskDelay(25);
		//弹仓
		if(GR)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp7,"(G)DANCANG:OFF");
			Char_Draw(&strDANCANG,"DANCANG",UI_Graph_Change,9,UI_Color_Orange,20,strlen(tmp7),2,10,710,tmp7);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp7,"(G)DANCANG: ON");
			Char_Draw(&strDANCANG,"DANCANG",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp7),2,10,710,tmp7);
		}
		Char_ReFresh(strDANCANG);
		vTaskDelay(25);
		//爆发射频
		if(ET)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp9,"(E)RF_bursts: ON");
			Char_Draw(&strRF_bursts,"RF_bursts",UI_Graph_Change,9,UI_Color_Orange,20,strlen(tmp9),2,10,610,tmp9);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp9,"(E)RF_bursts:OFF");
			Char_Draw(&strRF_bursts,"RF_bursts",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp9),2,10,610,tmp9);
		}
		Char_ReFresh(strRF_bursts);
		vTaskDelay(25);
		//飞坡
		if(feipo)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp8,"(Ctrl)Flying_slopes: ON");
			Char_Draw(&strFlying_slopes,"Flying_slopes",UI_Graph_Change,9,UI_Color_Orange,20,strlen(tmp8),2,10,660,tmp8);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp8,"(Ctrl)Flying_slopes:OFF");
			Char_Draw(&strFlying_slopes,"Flying_slopes",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp8),2,10,660,tmp8);
		}
		Char_ReFresh(strFlying_slopes);
		vTaskDelay(25);
		//底盘模式
		if(chassis_move.chassis_mode == CHASSIS_VECTOR_BPIN)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp4,"MODE:CHASSIS_SPIN");
			Char_Draw(&strMODE,"MODE",UI_Graph_Change,9,UI_Color_Orange,30,strlen(tmp4),2,860,250,tmp4);		}
		else if(chassis_move.chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp4,"MODE:CHAS_FLL_GIM");
			Char_Draw(&strMODE,"MODE",UI_Graph_Change,9,UI_Color_Green,30,strlen(tmp4),2,860,250,tmp4);
		}
		else if(chassis_move.chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp4,"MODE:CHAS_NOF_GIM");
			Char_Draw(&strMODE,"MODE",UI_Graph_Change,9,UI_Color_Green,30,strlen(tmp4),2,860,250,tmp4);
		}
//		//清空模式
//		if (last_chassis_behaviour != chassis_move.chassis_behaviour)
//		{
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
//			sprintf(tmp4,"                  ");
//			Char_Draw(&strMODE,"SPIN",UI_Graph_Change,9,UI_Color_Green,30,strlen(tmp4),2,860,250,tmp4);
//		}
//		last_chassis_behaviour = chassis_move.chassis_behaviour;
		Char_ReFresh(strMODE);
		vTaskDelay(25);
		//弹量
		sprintf(tmp3,"PILL:%d",pill);
		Char_Draw(&strPILL,"PILL",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp3),2,10,760,tmp3);
		Char_ReFresh(strPILL);
		vTaskDelay(25);
//		//车宽UI
//		Line_Draw(&Aim[4],"AL5",UI_Graph_Change,6,UI_Color_Green,2,440-anglesr,0,627-anglesr,443);
//		Line_Draw(&Aim[5],"AL6",UI_Graph_Change,6,UI_Color_Green,2,1480+anglesr,0,1293+anglesr,443);
		//就近对位UI
		if(turn_flags){	
		Line_Draw(&Aim[7],"AL8",UI_Graph_Change,7,UI_Color_Purplish_red,10,1700,600,1700,500);
		}
		else
		Line_Draw(&Aim[7],"AL8",UI_Graph_Change,7,UI_Color_Green,10,1700,600,1700,700);	
		vTaskDelay(25);
		//尖角UI
		if(sharp_angle){	
		Line_Draw(&Aim[10],"AL11",UI_Graph_Change,7,UI_Color_Cyan,10,1700,600,1800,600);
		}
		else
		Line_Draw(&Aim[10],"AL11",UI_Graph_Change,7,UI_Color_Cyan,0,1700,600,1800,600);
		vTaskDelay(25);
		
		UI_ReFresh(5,Aim[10],Aim[4],Aim[5],Aim[7]);
		vTaskDelay(20);
	}
}






