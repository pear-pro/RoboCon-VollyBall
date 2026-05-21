(一)注意事项
1.keil项目路径MDK-ARM->DJ_A_wheel.uvprojx
2.芯片stm32F427IIH
3.不要随便动主分支,可以自己加一个分支.在push前先抓取远程提交把远程仓库里的更改先同步到自己的文件里后再提交push
4.最好在vscode里写代码,在keil里调试
5.如果遇到报错可以打开.ioc文件重新生成代码
(二)必备插件链接链接
1.hal库项目文件管理插件
https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension
2.cmake需要下
https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools
3.vscode中文插件
https://marketplace.visualstudio.com/items?itemName=MS-CEINTL.vscode-language-pack-zh-hans
(三)控制逻辑
(1)已实现
1.在tim3中断回调函数里计算pid和发送can报文
2.遥控器串口在uart1,接收中断里调用了一个自己写的接收回调函数将uart1对应的dma数据接收并传到几个全局变量里
(2)未实现,中期考核前必须实现的功能
1.将遥控器的拨杆改成功能按键,至少4个
2.3508电机串级pid控角度的多圈转动
3.发球准备,发球,击球
4.舵机对球的限位
(四)资源分配
1.tim3中断作速度环的更新处理
2.tim14中断作角度环的更新处理
3.uart1作为遥控器接收接口
4.uart6作为串口打印接口
5.i2c1后续挂载陀螺仪
6.can1控制底盘电机
7.can2控制机械臂等,有一个陀螺仪也用can
(五)DT7遥控器各通道对应
1.右摇杆,左右对应ch[0].上下对应ch[1]
2.左摇杆,左右对应ch[2].上下对应ch[3],最下有限位可做为击球
3.左上角滚轮对应ch[4]
4.右上拨杆对应s[0],上中下值分别为1,3,2
5.左上拨杆对应s[1],上中下值分别为1,3,2

2026.4.2 pid计算放回定时中断里了，定时中断的优先级都给成0，can的中断优先级是1，5号3508改为了速度环，具体的对应关系还要修改，然后4310的参数需要修改

2026.5.16 自动调参说明
终端输入命令格式 :
角度
python scripts/tune_c620_up_angle.py COM3 --select up --mode position --target 10 --duration 1.0 --settle-deg 1.0 --zero --max-trials 3 --char-delay 0.003 --verbose
速度
python scripts/tune_c620_up_angle.py COM3 --select pitch_speed --mode speed --target 1000 --duration 1.0 --settle-deg 50 --max-trials 3 --char-delay 0.003 --verbose

**串口需要自己对应自己电脑的串口，目前演示的是角度环
**如果想在vofa上看波形，命令如下python scripts/tune_c620_up_angle.py COM3 --select up --mode position --target 10 --duration 1.0 --settle-deg 1.0 --zero --max-trials 3 --char-delay 0.003 --vofa-port COM11 --verbose

***这里串口11是虚拟串口，VOFA上打开串口12（虚拟），通过下载Virtual Serial Port Driver设置虚拟串口，链接:[text](https://pan.baidu.com/s/1fQu9QWQWkSCthAjdE5lrQA?pwd=6666)
提取码6666
***在ops.c static ops_control_t ops_targets[OPS_TARGET_MAX]中创建自己想要自动调参的对象
--select up / --select pitch_speed：选择固件里的调参对象
--mode position|speed：选择双环控角度或单速度环
--target：position 下是输出轴角度，speed 下是 rpm
--ratio：减速比
***重点参数按优先级看：
score
总评分，越小越好。脚本就是按这个选 best。
settle_ms
稳定时间，越小越好。
如果是 1000000000.0，说明在测试时间内没有稳定到阈值内。
steady
末段平均误差，越小越好。
位置模式单位约等于输出轴角度。比如 steady=0.3 表示最后平均差约 0.3 度。
overshoot
超调量，越小越好。
ripple
末段抖动，越小越好。
这个大，说明到位后还在抖。
sat_ratio
输出饱和比例，越小越好。
接近 1.0 说明电流/输出经常打满，PID 太猛或限幅太低，调出来不稳也不安全。
*** 在c620_up_angle_best.json里看这次最好的pid参数 .csv里可以看收敛情况