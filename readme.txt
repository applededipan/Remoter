1 ʵ�ִ��ڷ���
2 ʵ��ģ�������
3 ʵ�ְ����ɼ�
4 ʵ��LED���
5 �������������д��������֤
6 ���mavlink���ݽ��� 22/2
7 �������Թ��������Խ���mavlink����  23/2
8 ��������ʾ�������㣬�ߣ��棬�ַ����ַ���
9 ����ʾ16λbmp��ʽͼƬ���ɻ�Բ
10 eeprom �ɹ���
11 flash�ɹ���
12 haptic�ɹ���
13 ��Դ��⿪���ɹ���
14 �ɽ���bootloader
15 ʵ�ֶ̰���ʾ���������������� �����ػ�

16��14/03/2016
 1���޸�getAnalogValue�����������һ·ģ��������ʾ������

18/03��
    1��ʵ��С��ص�������ʾ
    2��ʵ��ʱ����ʾ
    3��ʵ��ҡ����ʾ
    4��ʵ��processBar��ʾ
    5��ɾ����simu�ļ��У�18/3��
    6��ɾ��������ƽ̨���ļ��У�֮ǰ��

19/03��
    1����������ʾ��mini�����ʾ���״̬��ͼ�����

    2���޸���mavlink�н����ĺ�������ʱ�޸��� rssi�������ֳ���

22/03
    1���޸���_mav_finalize_message_chan_send()�����Ķ��壬���ڲ����ε��ô��ڷ��ͺ�����Ϊһ��memcpy������һ�η��ͺ�����ʵ����mavlink���ݵķ���
    2����mavlink.h�ļ������Զ�������������
    3: ��mavlink.c�ļ��������˼�·���ݸ�ʽ�Ľ������������ɽ���

23/03��
    1���޸��˴��ڵ�����

25/03��
    1��������lac_showfloat������������ʾһλС����ĸ�����
    2����commntask�е���ϵͳʱ�亯��ɾ���󣬸������ٱ���
    3����gui���displayTest�����У�float���͵ı�����֮ǰ��uint16_t�ĳ�float��menustask��ʱ��Ϊ1

26/03��
    1 �����mavlink��common��ardupilotmega�ļ����е�mavlink��Ϣ
    2 ������Ӳ����������ң�������Ʒɿأ�����  ���Ϳ���ָ�

29/03��
    1 ʵ����ͨ��remrssi������0--255����ʾ�����ź�ǿ��
    2 ʵ����ͨ��system_status������==4ʱ��ʾϵͳ�����ʾϵͳ����������
    3 ʵ����ͨ��type������ʾ��ǰ�Ļ���
    4 ʵ���˷���ͷǷ����µĽ�����������������ֱ�ӿ��Ʒɿأ������Ӷ�������˲��ԣ�
       

30/03
    1 �޸��������ķ���Ƶ��Ϊ1HZ
    2 �޸���ң����ָ��ķ���ģʽ����֮ǰ��ʱ���͸ĳ�ĳͨ��ֵ�仯����25ʱ���ͣ���ʡ�˴����������ײ��
    3 ͸�����ܴ򿪲�����û������
    4 �����ź���ʾ��p900��һֱ��ʾ���ź�

02/04
    1 ʵ����px4����ģʽ���л�����ʾ 
    2 ��mavlink.h�ļ��������px4����ģʽ�ĺ궨���ö�ٵ�
    3 �����displayCustomMode����

09/04
    1 �ɿ���PX4��apm�ɿأ��ɽ���������
    2 ������ARMED �� DISARMED logo
    3 ������displayPilotFlightmode������ɾ����displaycustommode����

14/04
    1 ʵ���˰����ɼ�
    2 ����˷���ģʽ�л�menu
    3 �����attitudemenu


23/04
    1 �޸���PX4ģʽ�л�ʱ���������⣬����|MAV_MODE_FLAG_SAFETY_ARMED���
    2 �޸��˲˵���ʾ����
    3 PX4��APMң��ͨ�����ǲ�ͬ������������̶�



28/04 
    1 �޸���PX4���Ϳ���ָ��ĺ�������manualcontrol��Ϊrcchanneloverride
    2 �޸���ģʽ�л��ķ�ʽ��PX4��5ͨ����ֵ���ֶ����Զ������޸�ֵ���л�ģʽ APMΪ��8ͨ��
    3 ҡ����ʾΪ����ֵ������ֵ���ݸ���̫����ʵ��

12/05
    1 ��mavlink����µ�2015��
    2 ����ָ��ɾ�����ɿ�ֱ�Ӱ�RCֵ���н�������
    3 ����fifo��512�ֽ�
    4 ���usart1 ��usart4����ɸ���ʵ������ȷ�������ĸ�����
    5 �޸��˵����ʾ�ĺ���


21/05
    1 spiInit() �����ڳ�ʼ��������Ƭѡ���� ���Ǳ����
    2 boot.cpp��ע����lcd��غ�����


24/05
    1 ��ʵ��bootloader���º���ת
    2 �޸��˴��ڲ��ܷ��͵�һ���ַ���bug
    3 ɾ����boot.c�в��õĺ���


25/05
    1 �޸��˽���bootloader�����ĳ���
        GPIOC->MODER = (GPIOC->MODER & 0XFFFFCFFF);    //! moder: input & pull-up
  
        GPIOC->PUPDR = (GPIOC->PUPDR | 0X00001000);    //! PC6//HOME
    2 ɾ��pwr.h ��pwr.c �ļ�

26/05
    1  �����˴ӵ���վ�������ݲ��������sd������bin�ļ��ĳ���

27/05
    1  ʵ�ִӵ�mainվ�����ļ�������test.bin�ļ����������վ����rebootָ���������
    2  ʵ�����ָ��·�����
          ����home������
          ����վ����rebootָ���eeprom�ĵ�һ����ַд0x55��bootloader����ֵ��Ϊ0x55����£�Ȼ������

28/05 
    1  ��ftp.cpp�ļ��� myFat �ɾ�̬������Ϊ��ͨ����
    2  ���û��sd������ʱ���򿨶ٵ�bug
    3  ������sysreboot����������������bootloader
    4  �����˸��º���Բ��ϵ�ֱ����תAPP���еĳ���

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
    1 ���ֿ�ĳɺ궨��ķ�ʽ��ͨ������궨�����뼴��	
	
	
21/08
    1 �޸���ң������ز�����ȥ900Ϊ��ʱ��bug����uint16_t bat ��Ϊ int16_t bat	
	
23/08
    1 ����ص�ѹ���㺯����void���͸ĳ�ͨ����ַ���ص�ѹֵ

24/08
    1 ����ص�ѹ���㺯���������reduce noise ����	
	
26/08
    1 ����˼����ݮ�ɺ�ͼ���Ƿ�������������Ϣ����handle_message_debug()

27/08
    1 �����ң��������ʱ���е���˸�ĳ���
    2 �����ϵͳ��ʱ��	

29/08
    1 ����� 10ms 100ms 1000ms ����ϵͳ��ʱ��
    2 ��ԭ10ms��ʱ�ж��еĺ����Ƶ�ϵͳ10ms��ʱ����
    3 ����˵͵�ѹ���ⱨ���������ȹ���
	
31/08
    1 ɾ����1000ms��ʱ��
	2 ����������ķ���ģʽ
	3 ��д��displayFlightMode ����
	4 �޸���comn�����й������߸��µĲ��ֳ���
	5 ������ʶ��raspi��usb�ĳ���
	6 �����ͼ���������ź�ǿ�ȵĳ���
	7 ����̬��ʾ�Ⱥ���ת�Ƶ�10msϵͳ��ʱ����
	8 ����ص�ѹ�������㺯��ת�Ƶ�Ӳ����ʱ������ʹң�����ϵ�Ϳ�ʼִ��
	
01/09
    1 gps��λ����������ʾΪ��ɫ	
	2 ����˺�����ת����ļ�ͷָʾ
	3 ����ǰ����ģʽΪ�Ұ�ɫ��ʾ�����������ģʽΪ��ɫ��ʾ����ʾ���Խ��п��Ʋ���
	4 ����view_information(uint16_t id)������ʾ������Ϣ�ĺ������ڴ˵���
	
02/09
    1 view_information()�����У���ң�����ڽ��и���ʱ���ٸ�����ʾ���ݣ���ֻ��ʾ"updating"	
	2 ftp.cpp�����и��ݵ�ص�ѹֵ�жϿɲ����Խ��и��£�20%���Ͽɸ��£������ɸ����������վ�ط�һ�κ���и��£������ɸ�������ʾrejected
	3 ��powerLowShutdown(uint8_t bat)�����йػ�֮ǰ��ʾ��shutdowning�����ַ�
	4 ��if(mavData.mavStatus.health++ == 30) mavlinkReset(); ��Ϊ if(mavData.mavStatus.health++ == 30+1) mavlinkReset();����������������ʼ���ݻᷴ����λ������
	
03/09
    1 ����mavData.mavStatus.health��ֵ�������ź�ǿ�ȣ�
	  0���������� 
	  30������ʧ��
      1--29���ź���������1--29�����0--100���ź�ǿ�ȣ���UavPort�ĵ�Ƶmessage(heartbeat sys_status)����ʱΪ0������Ƶmessage(attitude..)����ʱ�Լ�����ֵԽ�ͱ�ʾ�ź�ǿ��Խ��	  
	
05/09
    1 ����view_calibration(uint8_t state)��ҡ��У׼����
	
07/09
    1 ������8���Ӳ˵�����ʾ	
	
24/09
    1 �޸��˴���͸���ķ�ʽ
28/09
    1 �޸��˺����������覴õ������
    2 �޸��˲���ͨ��usb����P900�����⣬͸�����뵥�ֽ�ֱ�ӷ��ͳ�ȥ�������Խ�����������Ϣ�ٷ��ͣ���Ϊ����P900ʱ���Ƿ��͵�mavlink��Ϣ	
	
22/10
    1 �����PX4_FLIGHT_MODE_RATTITUDE�궨��	

25/10
    1 ����˵������ܣ��õ���camera_trigger��Ϣ��Ϊ��ֹӰ��ɿ����չ��ܣ�ֻ���齫���п��Ƶ������ܵ�ң�����̼���������	
	
08/12
    1 ������ģʽ��ֵ��Ϊ�� 1000(default)  1400   1700   2000 ��������Ӧֵ�÷���ģʽʱ���ܸ�1000�����κ�ģʽ����ң�����ϵ�ʱ������������������ģʽ��ȫ���ɷɿؾ����ģ����������ʧ�ر���ʱң�����ϵ��������ϵ���޸ķ���ģʽ�����⡣
	
    2 �����mavlink2.0�Ŀ⣬Ĭ��ʹ��2.0����Ҫʹ��1.0����#define MAVLINK_1
