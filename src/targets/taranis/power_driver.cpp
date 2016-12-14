 
 
#include "../../opentx.h"
 
 
/******************************************************************************/
//! added by apple
//! if any changes are done to the PWR PIN or pwrOn() function
//! then the same changes must be done in _bootStart()
void powerInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//! 电源输出控制引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin   = POWER_GPIO_PIN_ENABLE;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_ResetBits(POWER_GPIO_REG_ENABLE, POWER_GPIO_PIN_ENABLE);
}


void powerOn(void)
{
	GPIO_SetBits(POWER_GPIO_REG_ENABLE, POWER_GPIO_PIN_ENABLE);  //! power ctrl pin
	GPIO_SetBits(LED_GPIO_REG_POWER, LED_GPIO_PIN_POWER);        //! power state led
}


void powerOff(void)
{
	backLightEnable(0, 0);
    GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
    GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
    GPIO_ResetBits(LED_GPIO_REG_H, LED_GPIO_PIN_H);	
	GPIO_ResetBits(LED_GPIO_REG_POWER, LED_GPIO_PIN_POWER);
	GPIO_ResetBits(POWER_GPIO_REG_ENABLE, POWER_GPIO_PIN_ENABLE);
    __disable_irq(); //! disable interrupts
    while(1) wdt_feed();
}



/************************************************
func   : 检测电源按键是否按下
return : 1：按键按下   0：按键没有按下
************************************************/
uint8_t powerCheck(void)
{
	uint8_t state;
	if (~KEYS_GPIO_REG_BUTTON_POWER & KEYS_GPIO_PIN_BUTTON_POWER)
    {
		state = 1;
	}		
    else      
    {
		state = 0;
	}		
	return state;
}

/************************************************
func   : 判断系统是否上电运行并执行相应操作
************************************************/
void powerStartup(void)
{
	uint32_t i;
	uint8_t flag;
	eepromReadBlock(&flag, E_ADDR_UPDATE, 1);
	
	if(flag == 0x11) //! means has just update firmware, so jump across powerCheck and do powerOn()
	{
		flag = 0;
		eepromWriteBlock(&flag, E_ADDR_UPDATE, 1);
		powerOn();
		for(i=0; i<40000; i++) delay_01us(1000); //! must do delay
		return;
	}
	
	if(powerCheck())
	{
		for(i=0; i<40000; i++) delay_01us(1000); //! must do delay
		if(powerCheck())
		{
			powerOn();
		}
		else
		{
			powerOff();
		}					
	}
    else
	{
		for(i=0; i<40000; i++) delay_01us(1000); //! must do delay
		powerOff();
	}		
}


/************************************************
func   : 判断系统是否关机并执行相应操作
************************************************/
void powerShutdown(void)
{
	static uint8_t  cnt = 0;
	static gtime_t  currentTime = 0;
	if(powerCheck())
	{
		if(currentTime != g_rtcTime) 
		{
			currentTime = g_rtcTime; cnt++;	
		}
				
		if(cnt > 3)
		{
			powerOff();			
		}
	}
    else
	{
		cnt = 0;
	}						
}


/************************************************
func   : Get the middle value of three values

 * @param a  First value
 * @param b  Second value
 * @param c  Third value
 * @return   The middle value
************************************************/
uint8_t getMiddleValue(uint8_t a, uint8_t b, uint8_t c) 
{
	uint8_t middle;

	if((a <= b)&&(a <= c))
	{
		middle = (b <= c) ? b : c;
	} 
	else if((b <= a)&&(b <= c)) 
	{
		middle = (a <= c) ? a : c;
	} 
	else 
	{
		middle = (a <= b) ? a : b;
	}
	return middle;
}



/*******************************************************
 * @ get the RC battery voltage 0% -- 100%
 * @ 18650:  0%:6.6V     100%: 8.2V   bat = anaIn(6)*0.5 - 400
 * @ Li-ion: 0%:7.4V     100%: 8.2V   bat = anaIn(6)-900
 
 * @ return: uint8_t battery value
*******************************************************/
void getBatVoltage(uint8_t *battery)
{
	static uint16_t cnt = 0;
	static uint32_t sum = 0;
	static uint16_t sub = 5;
	
#if defined(BAT18650)
    int16_t bat = anaIn(6)*0.67 - 567;
#else	
    int16_t bat = anaIn(6)-900;
#endif

    if(bat > 99)      bat = 99;	
	else if(bat <= 0) bat = 0;	
	
	static uint16_t time = 0;
    if(time++ > 380)
	{
		time = 380; sub = 500;                                //! calculate per 500 times or 5 seconds
	    if(bat-(*battery) >= 3)       bat = (*battery) + 3;   //! to reduce noise
        else if(bat-(*battery) <= -4) bat = (*battery) - 3; 		
	}
	sum += bat;
	cnt++;
	
    static uint8_t c0 = 0;
    static uint8_t c1 = 99;
    static uint8_t c2 = 1;	
			
	if(cnt==sub)
	{
	   c0 = sum / sub; 
	   *battery = (getMiddleValue(c2, c1, c0)+(*battery))/2;
       c2 = c1;
	   c1 = c0;	   
       cnt = 0; 
	   sum = 0;		
	}	
}



/************************************************
func   : low power warning  run per 100ms
 * @param bat      under this value then the system will give a audible and visual alarm
 * @param keyMute  if this key pushed down, then the warn will disable 
 * @(x, y)         x: center
************************************************/
void powerLowWarn(uint16_t x, uint16_t y, uint8_t bat, uint32_t keyMute)
{
  static uint16_t t = 0;
  uint8_t LOWPOWER[] = " LOW POWER! ";
  uint8_t NONELOGO[] = "            ";   
  if(g_eeGeneral.vBattery <= bat)
  {
	t++;
	
	if(t < 200)                                           //! 0 -- 199: delay 20s to wait the bat stable
	{
	  return; 
	} 
	else if(t < 300)                                      //! 200 -- 300: blinking
	{
      if(t%2==0)
      {
	    lcd_ShowString(x-72, y, RED, 24, LOWPOWER);	
        beepActive(1);		
      }
      else
      {
	    lcd_ShowString(x-72, y, BACKCOLOR, 24, NONELOGO);
	    beepActive(0);
      }				
	}
	else if(t == 300)
	{
	  t = 200; 
	  lcd_ShowString(x-72, y, RED, 24, LOWPOWER);
	  beepActive(1);	  
	}  
    else 
	{
	  return;
	}		

	if(g_eeGeneral.key == keyMute)                        //! disable the warn
	{
	  t = 0;
	  lcd_ShowString(x-72, y, RED, 24, LOWPOWER);
	  beepActive(0);
	}	
  }	
  else
  {
	t = 0;
  }
}



/************************************************
func   : low power shutdown
       : run per 1s
************************************************/
void powerLowShutdown(uint8_t bat)
{
   static uint8_t t = 0;
   const uint16_t x = 240;
   const uint16_t y = 3;
   if(g_eeGeneral.vBattery < bat) //! if battery little than bat, then shutdown the system !
   {
	 if(t++ >50) 
	 {
		 powerOff(); 
	 }
     else
	 {
	     uint8_t SHUTDOWN[] = "SHUTDOWNING!";
         lcd_ShowString(x-72, y, RED, 24, SHUTDOWN);			 
	 }		 
   }
   else
   {
	 t = 0;
   }
}
 
 
/************************************************
func   : system reboot
************************************************/
void systemReboot(void)
{
    __disable_fault_irq();   
    NVIC_SystemReset();
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 