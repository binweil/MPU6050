#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "VirtualDMP.h"

//ALIENTEK Ì½Ë÷ÕßSTM32F407¿ª·¢°å ÊµÑé32
//MPU6050ÁùÖá´«¸ÐÆ÷ ÊµÑé -¿âº¯Êý°æ±¾
//¼¼ÊõÖ§³Ö£ºwww.openedv.com
//ÌÔ±¦µêÆÌ£ºhttp://eboard.taobao.com  
//¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾  
//×÷Õß£ºÕýµãÔ­×Ó @ALIENTEK

//´®¿Ú1·¢ËÍ1¸ö×Ö·û 
//c:Òª·¢ËÍµÄ×Ö·û
void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET)
    USART_SendData(USART1,c);   

} 
//´«ËÍÊý¾Ý¸øÄäÃûËÄÖáÉÏÎ»»úÈí¼þ(V2.6°æ±¾)
//fun:¹¦ÄÜ×Ö. 0XA0~0XAF
//data:Êý¾Ý»º´æÇø,×î¶à28×Ö½Ú!!
//len:dataÇøÓÐÐ§Êý¾Ý¸öÊý
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//×î¶à28×Ö½ÚÊý¾Ý 
	send_buf[len+3]=0;	//Ð£ÑéÊýÖÃÁã
	send_buf[0]=0X88;	//Ö¡Í·
	send_buf[1]=fun;	//¹¦ÄÜ×Ö
	send_buf[2]=len;	//Êý¾Ý³¤¶È
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//¸´ÖÆÊý¾Ý
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//¼ÆËãÐ£ÑéºÍ	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//·¢ËÍÊý¾Ýµ½´®¿Ú1 
}
//·¢ËÍ¼ÓËÙ¶È´«¸ÐÆ÷Êý¾ÝºÍÍÓÂÝÒÇÊý¾Ý
//aacx,aacy,aacz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄ¼ÓËÙ¶ÈÖµ
//gyrox,gyroy,gyroz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄÍÓÂÝÒÇÖµ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//×Ô¶¨ÒåÖ¡,0XA1
}	
//Í¨¹ý´®¿Ú1ÉÏ±¨½áËãºóµÄ×ËÌ¬Êý¾Ý¸øµçÄÔ
//aacx,aacy,aacz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄ¼ÓËÙ¶ÈÖµ
//gyrox,gyroy,gyroz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄÍÓÂÝÒÇÖµ
//roll:ºá¹ö½Ç.µ¥Î»0.01¶È¡£ -18000 -> 18000 ¶ÔÓ¦ -180.00  ->  180.00¶È
//pitch:¸©Ñö½Ç.µ¥Î» 0.01¶È¡£-9000 - 9000 ¶ÔÓ¦ -90.00 -> 90.00 ¶È
//yaw:º½Ïò½Ç.µ¥Î»Îª0.1¶È 0 -> 3600  ¶ÔÓ¦ 0 -> 360.0¶È
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//Çå0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//·É¿ØÏÔÊ¾Ö¡,0XAF
} 
  
int main(void)
{ 
	u8 t=0,report=1;			//Ä¬ÈÏ¿ªÆôÉÏ±¨
	u8 key;
	float pitch,roll,yaw; 		//Å·À­½Ç
	short aacx,aacy,aacz;		//¼ÓËÙ¶È´«¸ÐÆ÷Ô­Ê¼Êý¾Ý
	short gyrox,gyroy,gyroz;	//ÍÓÂÝÒÇÔ­Ê¼Êý¾Ý
	short temp;					//ÎÂ¶È
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ÉèÖÃÏµÍ³ÖÐ¶ÏÓÅÏÈ¼¶·Ö×é2
	delay_init(168);  //³õÊ¼»¯ÑÓÊ±º¯Êý
	uart_init(115200);		//³õÊ¼»¯´®¿Ú²¨ÌØÂÊÎª500000
	LED_Init();					//³õÊ¼»¯LED 
	KEY_Init();					//³õÊ¼»¯°´¼ü
 	LCD_Init();					//LCD³õÊ¼»¯
	MPU_Init();					//³õÊ¼»¯MPU6050
 	POINT_COLOR=RED;//ÉèÖÃ×ÖÌåÎªºìÉ« 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"MPU6050 TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/5/9");
	
	/************Ö²Èë¼¸¸övariable***************/
	float Pitch,Roll,Yaw; 		//Å·À­½
	delay_ms(100);
	/***********************************************/
	
	while(mpu_dmp_init())
	{
		LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
		delay_ms(200);
		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(200);
	}
	LCD_ShowString(30,130,200,16,16,"MPU6050 OK");
	LCD_ShowString(30,150,200,16,16,"KEY0:UPLOAD ON/OFF");
	POINT_COLOR=BLUE;//ÉèÖÃ×ÖÌåÎªÀ¶É« 
 	LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");	 
 	LCD_ShowString(30,200,200,16,16," Temp:    . C");	
 	LCD_ShowString(30,220,200,16,16,"Pitch:    . C");	
 	LCD_ShowString(30,240,200,16,16," Roll:    . C");	 
 	LCD_ShowString(30,260,200,16,16," Yaw :    . C");	 
 	while(1)
	{
		/**************************************************/
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//Accx original data (aacx,y,z)
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//angular velocity original data(gyrox,y,z)
		aacx=Kalman_Filter_X(aacx,gyrox);
		aacy=Kalman_Filter_Y(aacy,gyroy);
		//aacz=Kalman_Filter_Z(aacz,gyroz);

		
		Roll = GetRoll(aacx,aacy,aacz);
		Pitch = GetPitch(aacx,aacy,aacz);
		Yaw = GetYaw(gyroz);

		//printf("Roll is %0.1f\t",Roll);
		//printf("Pitch is %0.1f\t",Pitch);
		//printf("Yaw is%0.1f\t",Yaw);
		/**************************************************/
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			report=!report;
			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
		}
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//µÃµ½ÎÂ¶ÈÖµ
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//µÃµ½¼ÓËÙ¶È´«¸ÐÆ÷Êý¾Ý
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//µÃµ½ÍÓÂÝÒÇÊý¾Ý
			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//ÓÃ×Ô¶¨ÒåÖ¡·¢ËÍ¼ÓËÙ¶ÈºÍÍÓÂÝÒÇÔ­Ê¼Êý¾Ý
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			if((t%10)==0)
			{ 
				if(temp<0)
				{
					LCD_ShowChar(30+48,200,'-',16,0);		//ÏÔÊ¾¸ººÅ
					temp=-temp;		//×ªÎªÕýÊý
				}else LCD_ShowChar(30+48,200,' ',16,0);		//È¥µô¸ººÅ 
				LCD_ShowNum(30+48+8,200,temp/100,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
				LCD_ShowNum(30+48+40,200,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö 
				temp=pitch*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,220,'-',16,0);		//ÏÔÊ¾¸ººÅ
					temp=-temp;		//×ªÎªÕýÊý
				}else LCD_ShowChar(30+48,220,' ',16,0);		//È¥µô¸ººÅ 
				LCD_ShowNum(30+48+8,220,temp/10,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
				LCD_ShowNum(30+48+40,220,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö 
				temp=roll*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,240,'-',16,0);		//ÏÔÊ¾¸ººÅ
					temp=-temp;		//×ªÎªÕýÊý
				}else LCD_ShowChar(30+48,240,' ',16,0);		//È¥µô¸ººÅ 
				LCD_ShowNum(30+48+8,240,temp/10,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
				LCD_ShowNum(30+48+40,240,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö 
				temp=yaw*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,260,'-',16,0);		//ÏÔÊ¾¸ººÅ
					temp=-temp;		//×ªÎªÕýÊý
				}else LCD_ShowChar(30+48,260,' ',16,0);		//È¥µô¸ººÅ 
				LCD_ShowNum(30+48+8,260,temp/10,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
				LCD_ShowNum(30+48+40,260,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö
				printf("%0.1f\t",Yaw);
				printf("%0.1d\r\n",temp);
				t=0;
				LED0=!LED0;//LEDÉÁË¸
			}
			
		}
		t++; 
	} 	
}
