1 实现串口发送
2 实现模拟量检测
3 实现按键采集
4 实现LED电灯
5 完成蓝牙驱动改写，还需验证
6 完成mavlink数据解析 22/2
7 蓝牙可以工作，可以进行mavlink解析  23/2
8 可驱动显示屏，画点，线，面，字符，字符串
9 可显示16位bmp格式图片，可画圆
10 eeprom 可工作
11 flash可工作
12 haptic可工作
13 电源检测开机可工作
14 可进入bootloader
15 实现短按显示电量，长按开机， 长按关机

16：14/03/2016
 1：修改getAnalogValue函数，解决有一路模拟量不显示的问题

18/03：
    1：实现小电池电量的显示
    2：实现时间显示
    3：实现摇杆显示
    4：实现processBar显示
    5：删除了simu文件夹（18/3）
    6：删除了其他平台的文件夹（之前）

19/03：
    1：完美的显示了mini电池显示充电状态的图标程序

    2：修改了mavlink中解析的函数，临时修改了 rssi解析部分程序

22/03
    1：修改了_mav_finalize_message_chan_send()函数的定义，将内部三次调用串口发送函数改为一句memcpy函数和一次发送函数，实现了mavlink数据的发送
    2：在mavlink.h文件夹中自定义了数据类型
    3: 在mavlink.c文件中增加了几路数据格式的解析函数，并可解析

23/03：
    1：修改了串口的驱动

25/03：
    1：增加了lac_showfloat函数，可以显示一位小数点的浮点数
    2：将commntask中调用系统时间函数删除后，该任务不再崩溃
    3：将gui里的displayTest函数中，float类型的变量由之前的uint16_t改成float后，menustask耗时仅为1

26/03：
    1 添加了mavlink中common和ardupilotmega文件夹中的mavlink消息
    2 可以在硬件仿真下用遥控器控制飞控（解锁  发送控制指令）

29/03：
    1 实现了通过remrssi变量（0--255）显示无线信号强度
    2 实现了通过system_status变量（==4时表示系统激活）显示系统解锁并启动
    3 实现了通过type变量显示当前的机型
    4 实现了仿真和非仿真下的解锁和上锁，并可以直接控制飞控（已连接舵机进行了测试）
       

30/03
    1 修改了心跳的发送频率为1HZ
    2 修改了遥控器指令的发送模式（由之前定时发送改成某通道值变化大于25时发送，节省了带宽和数据碰撞）
    3 透传功能打开并测试没有问题
    4 无线信号显示在p900下一直显示无信号

02/04
    1 实现了px4飞行模式的切换和显示 
    2 在mavlink.h文件中添加了px4飞行模式的宏定义和枚举等
    3 添加了displayCustomMode函数

09/04
    1 可控制PX4和apm飞控，可解锁，上锁
    2 更新了ARMED 和 DISARMED logo
    3 增加了displayPilotFlightmode函数，删除了displaycustommode函数

14/04
    1 实现了按键采集
    2 添加了飞行模式切换menu
    3 添加了attitudemenu


23/04
    1 修改了PX4模式切换时自锁的问题，加了|MAV_MODE_FLAG_SAFETY_ARMED语句
    2 修改了菜单显示界面
    3 PX4和APM遥控通道总是不同，这个问题需商定



28/04 
    1 修改了PX4发送控制指令的函数：将manualcontrol改为rcchanneloverride
    2 修改了模式切换的方式，PX4第5通道的值由手动和自动按键修改值并切换模式 APM为第8通道
    3 摇杆显示为发送值，接受值数据更新太慢不实用

12/05
    1 将mavlink库更新到2015版
    2 解锁指令删除，飞控直接按RC值进行解锁上锁
    3 增大fifo至512字节
    4 添加usart1 和usart4，其可根据实际连接确认启用哪个串口
    5 修改了电池显示的函数


21/05
    1 spiInit() 函数内初始化了所有片选引脚 这是必须的
    2 boot.cpp中注销了lcd相关函数，


24/05
    1 已实现bootloader更新和跳转
    2 修复了串口不能发送第一个字符的bug
    3 删除了boot.c中不用的函数


25/05
    1 修复了进入bootloader按键的程序，
        GPIOC->MODER = (GPIOC->MODER & 0XFFFFCFFF);    //! moder: input & pull-up
  
        GPIOC->PUPDR = (GPIOC->PUPDR | 0X00001000);    //! PC6//HOME
    2 删除pwr.h 和pwr.c 文件

26/05
    1  增加了从地面站接收数据并组包，在sd卡生成bin文件的程序

27/05
    1  实现从地main站接收文件并生成test.bin文件，还需地面站发送reboot指令进行重启
    2  实现两种更新方法：
          按下home键开机
          地面站发送reboot指令后，eeprom的第一个地址写0x55，bootloader检测该值，为0x55则更新，然后清零

28/05 
    1  将ftp.cpp文件中 myFat 由静态变量改为普通变量
    2  解决没有sd卡启动时程序卡顿的bug
    3  增加了sysreboot函数，可以重启到bootloader
    4  增加了更新后可以不断电直接跳转APP运行的程序

30/05
    1 moved the powerOn()'s location, to make sure when update finished, the cell can display ok
    2 modified the boot.c to display firmware update process
    3 delete the audio task and some other useless functions
    4 modified the cell calculation and display function: 0% means 7.4v and 100% means 8.2v

31/05
    1 modified backLightEnable driver from bit control to pwm control           
	2 added setBackLight() so we can adjust the brightness of the lcd 
	3 solved the cell display bug: modified GetBatVoltage()'s algorithm
	4 solved the short press leading to start up bug
    5 solved the after update finished the cell display flash across bug : added three delayms() in powerStartup()
		
02/06
    1 added dir of SmartConsole and can update from SmartConsole/firmware/opentx.bin
	2 added read pic info from sdcard, displayStartMenu()can read pic info both from extra flash and sdcard
	3 modified the backLightEnable(,)
	
07/06
    1 added middle filter in GetBatVoltage(void) 	
	
28/06
    1 added the writing pic from sd card to flash function in gui.cpp/displayStartmenu function	
	
29/06
    1 added #include "../ardupilotmega/ardupilotmega.h" in ../common/common.h path, for for invoke mavlink_msg_gimbal_control_send()
	
02/07
    1 added void eeWriteJoyScale(void) in eeprom_common.cpp file	
	2 added void eeReadJoyScale(void) in eeprom_common.cpp file
	3 added void eeWriteSingleJoyScale(uint8_t index, uint16_t value) in eeprom_common.cpp file
	4 added uint16_t eeReadSingleJoyScale(uint8_t index) in eeprom_common.cpp file
	5 added two (ltrm & rtrm) inputs in autopilot_joystick_command_send() function
	6 added enum JoyScaleIndex{} in opentx.h 
	
05/07
    1 modified disarm and arm functions, added judge of ele and ail	
	
11/07
    1 delete the heartbeat message when update firmware	
	2 modified the void menuNavigation(void) to make the start up display well 
	
12/07
    1 added leds reset functions in powerOff(void)

14/07
    1 delete flight mode control channel 8, it is only controlled by channel 5 now ! 
	2 manual control message send at 7 Hz
	3 channel 8 is used by gimbal control mode 
	4 the OPENTX_TL1 is added for change gimbal control mode
	5 added smartcontrol version info in ftp.cpp 
	
15/07
    1 add delayms(200) in boot.cpp (line:249) to make sure the capacitor discharge complete and RC will not startup automaticly
    2 add delayms(50) in boot.cpp(line:200) to make update firmware a little slowly!

23/07
    1 modify the fifo size from 512 to 1024
	2 do not send heartbeat message to UAV anymore
	3 only when make sure it is control mode that the rc will send autopilot_joystick_command_send() message in opentx.cpp
	
27/07
    1 add watchdog init in board_taranis.cpp	
	2 modify the rc work mode: a: normal mode  b: update mode  c: configure p900 mode
	
28/07
    1 put the FONT_24 from extended flssh into stm32 flash, and the System halt is solved!
	2 add FLIGHT_MODE_END in mavlink.h
    3 add digicam control in opentx.cpp	
	
	
30/07
    1 add digicam control in opentx.cpp (g_mavlink_msg_camera_trigger_send();)	
	
03/08 
    1 add usart1UsbPlugged() function

05/08
    1 annotated functions in opentx.cpp from line:1627 to line:1632
    2 	
	
12/08
    1 add the judgement of the file_transfer_protocol message is for smartcontrol or uav
    2 when no concect to uav, smartcontrol will not send control message; or when uavUpdate, smartcontrol will not send control message too !

13/08
    1 apply MIXER_STACK_SIZE from 500 to 1000
    2 apply COMMN_STACK_SIZE from 500 to 1000
	3 add self gimbal control message:g_mavlink_msg_command_long_send in mavlink.cpp
	
16/08
    1 将字库改成宏定义的方式，通过定义宏定义后编译即可	
	
	
21/08
    1 修改了遥控器电池采样减去900为负时的bug，将uint16_t bat 改为 int16_t bat	
	
23/08
    1 将电池电压计算函数由void类型改成通过地址返回电压值

24/08
    1 将电池电压计算函数中添加了reduce noise 处理	
	
26/08
    1 添加了检测树莓派和图传是否启动正常的消息处理handle_message_debug()

27/08
    1 添加了遥控器启动时所有灯闪烁的程序
    2 添加了系统定时器	

29/08
    1 添加了 10ms 100ms 1000ms 三个系统定时器
    2 将原10ms定时中断中的函数移到系统10ms定时器中
    3 添加了低电压声光报警及静音等功能
	
31/08
    1 删除了1000ms定时器
	2 添加了其他的飞行模式
	3 重写了displayFlightMode 函数
	4 修改了comn任务中关于在线更新的部分程序
	5 更改了识别raspi和usb的程序
	6 添加了图传启动和信号强度的程序
	7 将姿态显示等函数转移到10ms系统定时器中
	8 将电池电压采样计算函数转移到硬件定时器中以使遥控器上电就开始执行
	
01/09
    1 gps定位后卫星数显示为绿色	
	2 添加了航向旋转方向的箭头指示
	3 解锁前飞行模式为灰白色显示，解锁后飞行模式为绿色显示，表示可以进行控制操作
	4 增加view_information(uint16_t id)，凡显示文字信息的函数均在此调用
	
02/09
    1 view_information()函数中，当遥控器在进行更新时不再更新显示内容，而只显示"updating"	
	2 ftp.cpp函数中根据电池电压值判断可不可以进行更新（20%以上可更新），若可更新则待地面站重发一次后进行更新；若不可更新则显示rejected
	3 在powerLowShutdown(uint8_t bat)函数中关机之前显示“shutdowning！”字符
	4 将if(mavData.mavStatus.health++ == 30) mavlinkReset(); 改为 if(mavData.mavStatus.health++ == 30+1) mavlinkReset();后解决了数据连接伊始数据会反复复位的问题
	
03/09
    1 根据mavData.mavStatus.health的值来计算信号强度：
	  0：连接正常 
	  30：连接失败
      1--29：信号质量，将1--29换算成0--100的信号强度，当UavPort的低频message(heartbeat sys_status)到来时为0；当高频message(attitude..)到来时自减；该值越低表示信号强度越好	  
	
05/09
    1 增加view_calibration(uint8_t state)的摇杆校准函数
	
07/09
    1 增加了8个子菜单的显示	
	
24/09
    1 修改了串口透传的方式
28/09
    1 修复了航向两侧出现瑕疵点的问题
    2 修复了不能通过usb设置P900的问题，透传必须单字节直接发送出去，不可以接收完完整消息再发送，因为设置P900时不是发送的mavlink消息	
	
22/10
    1 添加了PX4_FLIGHT_MODE_RATTITUDE宏定义	

25/10
    1 添加了调焦功能，用到了camera_trigger消息，为防止影响飞控拍照功能，只建议将带有控制调焦功能的遥控器固件给李申龙	
	
08/12
    1 将飞行模式的值改为： 1000(default)  1400   1700   2000 在设置相应值得飞行模式时不能给1000设置任何模式，即遥控器上电时若不按下三个键，则模式完全是由飞控决定的，这样解决了失控保护时遥控器断电再重新上电会修改飞行模式的问题。
	
    2 添加了mavlink2.0的库，默认使用2.0，若要使用1.0，需#define MAVLINK_1
