# ROS_Tracer: 

20250425 update. 

包含所有始祖尤彌爾debug的Tracer and velodyne VLP-16模擬和實做，velodyne simulator不是官方的，可能會有問題

## 1. 下載任何檔案有問題或是缺少任何套件的話可以參考以下原始連結 or ask chatGPT

### 注意：如果以下方式有問題，請參考上述下載網址自行下載，這個kokroiceork75的README是我自己寫的，所以下載可能有錯，但檔案是沒問題的，所以參考原始連結下載後，再把裡面的檔案替換成這個GitHub的檔案

```
https://github.com/agilexrobotics/ugv_gazebo_sim.git
https://github.com/ros-drivers/velodyne.git
https://github.com/lmark1/velodyne_simulator.git
```

## 2. 可以參考以下步驟

### development environment is: Ubuntu 20.04 + ROS noetic 

Download and install ros-control function package, ros-control is the robot control middleware provided by ROS

```
sudo apt-get install ros-noetic-ros-control
```

​ Download and install ros-controllers function package, ros-controllers are the kinematics plug-in of common models provided by ROS

```
sudo apt-get install ros-noetic-ros-controllers
```

​ Download and install gazebo-ros function package, gazebo-ros is the communication interface between gazebo and ROS, and connect the ROS and Gazebo

```
sudo apt-get install ros-noetic-gazebo-ros
```

​ Download and install gazebo-ros-control function package, gazebo-ros-control is the communication standard controller between ROS and Gazebo

```
sudo apt-get install ros-noetic-gazebo-ros-control
```

Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```
sudo apt-get install ros-noetic-joint-state-publisher-gui 
```

​ Download and install teleop-twist-keyboard function package, telop-twist-keyboard is keyboard control function package, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
sudo apt-get install ros-noetic-teleop-twist-keyboard 
```

## 3. 如何使用

### create workspace

### bashrc可以source，以防每次開terminal都要source一次

```
mkdir tracer_ws
cd tracer_ws
mkdir src
cd src
catkin_init_workspace
git clone this url
cd tracer_ws
rosdep install --from-paths src --ignore-src -r -y 
catkin_make
source devel/setup.bash
```

## 4. 各種功能

### a. Real world

bringup the real robot.

```
rosrun tracer_bringup setup_can2usb.bash 
```

bringup Lidar and robot integration

```
roslaunch tracer_bringup tracer_integrate.launch 
```

default navigation

```
roslaunch tracer_navigation tracer_navigation_auto.launch 
```

fuzzy navigation

```
roslaunch tracer_navigation tracer_navigation_fuzzy.launch 
rosrun controller wei_move_along 
rosrun controller wei_odom
rosrun controller wei_sub
```

### b. Simulation

open gazebo. tracer_car, tracer_car_map ... all can launch different simulation world.

```
roslaunch tracer_gazebo_sim tracer_car.launch 
```

default navigation / fuzzy navigation 擇一，fuzzy waypoint 加在fuzzy navigation後可以啟動巡航

default navigation

```
roslaunch tracer_navigation tracer_navigation_auto.launch 
```

fuzzy navigation(單點導航)

```
roslaunch tracer_navigation tracer_navigation_fuzzy.launch 
rosrun controller wei_move_along 
rosrun controller wei_odom
rosrun controller wei_sub
```

fuzzy waypoint(巡航 ＝ 多點導航)

```
rosrun contoller fuzzy_waypoint
```

by Chuan Wei, Chen

⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⢀⢄⢄⠢⡠⡀⢀⠄⡀⡀⠄⠄⠄⠄⠐⠡⠄⠉⠻⣻⣟⣿⣿⣄⠄⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⢠⢣⠣⡎⡪⢂⠊⡜⣔⠰⡐⠠⠄⡾⠄⠈⠠⡁⡂⠄⠔⠸⣻⣿⣿⣯⢂⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⡀⠄⠄⠄⠄⠄⠄⠄⠐⢰⡱⣝⢕⡇⡪⢂⢊⢪⢎⢗⠕⢕⢠⣻⠄⠄⠄⠂⠢⠌⡀⠄⠨⢚⢿⣿⣧⢄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⡐⡈⠌⠄⠄⠄⠄⠄⠄⠄⡧⣟⢼⣕⢝⢬⠨⡪⡚⡺⡸⡌⡆⠜⣾⠄⠄⠄⠁⡐⠠⣐⠨⠄⠁⠹⡹⡻⣷⡕⢄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⢄⠇⠂⠄⠄⠄⠄⠄⠄⠄⢸⣻⣕⢗⠵⣍⣖⣕⡼⡼⣕⢭⢮⡆⠱⣽⡇⠄⠄⠂⠁⠄⢁⠢⡁⠄⠄⠐⠈⠺⢽⣳⣄⠄⠄
⠄⠄⠄⠄⠄⢔⢕⢌⠄⠄⠄⠄⠄⢀⠄⠄⣾⢯⢳⠹⠪⡺⡺⣚⢜⣽⣮⣳⡻⡇⡙⣜⡇⠄⠄⢸⠄⠄⠂⡀⢠⠂⠄⢶⠊⢉⡁⠨⡒⠄⠄
⠄⠄⠄⠄⡨⣪⣿⢰⠈⠄⠄⠄⡀⠄⠄⠄⣽⣵⢿⣸⢵⣫⣳⢅⠕⡗⣝⣼⣺⠇⡘⡲⠇⠄⠄⠨⠄⠐⢀⠐⠐⠡⢰⠁⠄⣴⣾⣷⣮⣇⠄
⠄⠄⠄⠄⡮⣷⣿⠪⠄⠄⠄⠠⠄⠂⠠⠄⡿⡞⡇⡟⣺⣺⢷⣿⣱⢕⢵⢺⢼⡁⠪⣘⡇⠄⠄⢨⠄⠐⠄⠄⢀⠄⢸⠄⠄⣿⣿⣿⣿⣿⡆
⠄⠄⠄⢸⣺⣿⣿⣇⠄⠄⠄⠄⢀⣤⣖⢯⣻⡑⢕⢭⢷⣻⣽⡾⣮⡳⡵⣕⣗⡇⠡⡣⣃⠄⠄⠸⠄⠄⠄⠄⠄⠄⠈⠄⠄⢻⣿⣿⣵⡿⣹
⠄⠄⠄⢸⣿⣿⣟⣯⢄⢤⢲⣺⣻⣻⡺⡕⡔⡊⡎⡮⣿⣿⣽⡿⣿⣻⣼⣼⣺⡇⡀⢎⢨⢐⢄⡀⠄⢁⠠⠄⠄⠐⠄⠣⠄⠸⣿⣿⣯⣷⣿
⠄⠄⠄⢸⣿⣿⣿⢽⠲⡑⢕⢵⢱⢪⡳⣕⢇⢕⡕⣟⣽⣽⣿⣿⣿⣿⣿⣿⣿⢗⢜⢜⢬⡳⣝⢸⣢⢀⠄⠄⠐⢀⠄⡀⠆⠄⠸⣿⣿⣿⣿
⠄⠄⠄⢸⣿⣿⣿⢽⣝⢎⡪⡰⡢⡱⡝⡮⡪⡣⣫⢎⣿⣿⣿⣿⣿⣿⠟⠋⠄⢄⠄⠈⠑⠑⠭⡪⡪⢏⠗⡦⡀⠐⠄⠄⠈⠄⠄⠙⣿⣿⣿
⠄⠄⠄⠘⣿⣿⣿⣿⡲⣝⢮⢪⢊⢎⢪⢺⠪⣝⢮⣯⢯⣟⡯⠷⠋⢀⣠⣶⣾⡿⠿⢀⣴⣖⢅⠪⠘⡌⡎⢍⣻⠠⠅⠄⠄⠈⠢⠄⠄⠙⠿
⠄⠄⠄⠄⣿⣿⣿⣿⣽⢺⢍⢎⢎⢪⡪⡮⣪⣿⣞⡟⠛⠋⢁⣠⣶⣿⡿⠛⠋⢀⣤⢾⢿⣕⢇⠡⢁⢑⠪⡳⡏⠄⠄⠄⠄⠄⠄⢑⠤⢀⢠
⠄⠄⠄⠄⢸⣿⣿⣿⣟⣮⡳⣭⢪⡣⡯⡮⠗⠋⠁⠄⠄⠈⠿⠟⠋⣁⣀⣴⣾⣿⣗⡯⡳⡕⡕⡕⡡⢂⠊⢮⠃⠄⠄⠄⠄⠄⢀⠐⠨⢁⠨
⠄⠄⠄⠄⠈⢿⣿⣿⣿⠷⠯⠽⠐⠁⠁⢀⡀⣤⢖⣽⢿⣦⣶⣾⣿⣿⣿⣿⣿⣿⢎⠇⡪⣸⡪⡮⠊⠄⠌⠎⡄⠄⠄⠄⠄⠄⠄⡂⢁⠉⡀
⠄⠄⠄⠄⠄⠈⠛⠚⠒⠵⣶⣶⣶⣶⢪⢃⢇⠏⡳⡕⣝⢽⡽⣻⣿⣿⣿⣿⡿⣺⠰⡱⢜⢮⡟⠁⠄⠄⠅⠅⢂⠐⠄⠐⢀⠄⠄⠄⠂⡁⠂
⠄⠄⠄⠄⠄⠄⠄⠰⠄⠐⢒⣠⣿⣟⢖⠅⠆⢝⢸⡪⡗⡅⡯⣻⣺⢯⡷⡯⡏⡇⡅⡏⣯⡟⠄⠄⠄⠨⡊⢔⢁⠠⠄⠄⠄⠄⠄⢀⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠹⣿⣿⣿⣿⢿⢕⢇⢣⢸⢐⢇⢯⢪⢪⠢⡣⠣⢱⢑⢑⠰⡸⡸⡇⠁⠄⠄⠠⡱⠨⢘⠄⠂⡀⠂⠄⠄⠄⠄⠈⠂⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⢻⣿⣿⣿⣟⣝⢔⢅⠸⡘⢌⠮⡨⡪⠨⡂⠅⡑⡠⢂⢇⢇⢿⠁⠄⢀⠠⠨⡘⢌⡐⡈⠄⠄⠠⠄⠄⠄⠄⠄⠄⠁
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠹⣿⣿⣿⣯⢢⢊⢌⢂⠢⠑⠔⢌⡂⢎⠔⢔⢌⠎⡎⡮⡃⢀⠐⡐⠨⡐⠌⠄⡑⠄⢂⠐⢀⠄⠄⠈⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠙⣿⣿⣿⣯⠂⡀⠔⢔⠡⡹⠰⡑⡅⡕⡱⠰⡑⡜⣜⡅⡢⡈⡢⡑⡢⠁⠰⠄⠨⢀⠐⠄⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠈⠻⢿⣿⣷⣢⢱⠡⡊⢌⠌⡪⢨⢘⠜⡌⢆⢕⢢⢇⢆⢪⢢⡑⡅⢁⡖⡄⠄⠄⠄⢀⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠛⢿⣿⣵⡝⣜⢐⠕⢌⠢⡑⢌⠌⠆⠅⠑⠑⠑⠝⢜⠌⠠⢯⡚⡜⢕⢄⠄⠁⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠙⢿⣷⡣⣇⠃⠅⠁⠈⡠⡠⡔⠜⠜⣿⣗⡖⡦⣰⢹⢸⢸⢸⡘⠌⠄⠄⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠈⠋⢍⣠⡤⡆⣎⢇⣇⢧⡳⡍⡆⢿⣯⢯⣞⡮⣗⣝⢎⠇⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠁⣿⣿⣎⢦⠣⠳⠑⠓⠑⠃⠩⠉⠈⠈⠉⠄⠁⠉⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠈⡿⡞⠁⠄⠄⢀⠐⢐⠠⠈⡌⠌⠂⡁⠌⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠈⢂⢂⢀⠡⠄⣈⠠⢄⠡⠒⠈⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄
⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠢⠠⠊⠨⠐⠈⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄

======================[Iron Man]======================


cXzvxxxjffftfffjfjflJha#MMMMMWW##*aapZ0COwqwOQJmqqdk*MMWMMkZzfIltz0p#MM######MMMMMM**hbXIfjjjsjsxxxx
zvzxxxvxffcsjlttxj>J$$o&MMM*%%Mho%$$oZbZcZCcJYxUUw$*W$$$$$$$$hx>YJfvJ$$##m*hW#&MWaMdbk$$J\Xtvxcjvvsx
cXUvxvvzxvjsvxtj}1m$$aWMMM&MM#&M$hCzsjv1I1I-\(]l1l}{}>]{U0$$ZYmjItZ$pzdo$WM**h*&*M8kbdp$$t[Xsjszzvvv
vjsxfjslfjfIlttj\U$Wo8MMM&a*##$oZsfctt{<???=?++!:,`·`:|!~*/>JkOlfs~<0$mJaakbbdo%bbbk%$Mq$$1xvccxjvvv
vxfvsxcjsfjstlx[s$$#aMMMW#&hd&#OCst[<)++=ii|~!;,;i;i!:,`,`+=)<fm--Xj+lZ$0qbaakbwh$kkq#Wd&$U\vssxfvvx
vzczXflxftlIjsl]q$d&MW&&@ak$$Wfj})~!;i!||=+~~!|::,;:i!i!|!|||>[{U])lsx}U$pmmwaM$ah$k$&d%b$ollcxxvvsx
cvxtjfjjltsslt[sQ$k$%#kaa$doOtf]=?=)++~|+!i;``,;i;ii;i!;:!||++\>]b}<<wzIxW$wqbaMbbpdqkhhah$J]zxvvcjs
csvfsfjjsjltlf\xM$b#ko#k8qb0}J>?-)=+!|!~;|i,;;i`;!!!;,,,:;!*~?+-)<j>/(vUC1p$kpwhkM*opkkaok$b<sjjjsxs
zczvzxjssstftf(0$o$$pd&WpUa1m(-<()=|+++:!;:i|,,;i`:`i,`,:;++?=|*<={/{/I]Caxwwhkphaok&M&bhk$o]xffjffj
zcXvzxsxfffIIf/c$o#ppd#os$xp[]1>>-)?+*|~|!~!`,;!;;:i:;i;,,;===(*-]=s:z=v~YZQmCo$Z*hp8**M8b$U<ljtftjj
cczxzxjsttfI1f(c$mqopkMU#Cq}/{<(?|==~+!~~i~i~i;i!;ii;:;·```,!,ii~{=>1+x?Z|OOXZzw$aahq&#W$b$Mtvtffffj
cvczjIjslt}1}t)s$hoq*#qCWtC-1]>---=~!!|~!:i!|;:;;ii:::;i:``·,,ii:));1ix $=1JCJJCbh##dbM&$hW&tf]fffff
cvccvjxjjj{lIl<>$$WakWUk0Yf/-)+|,;,!i;ii:iii,,|!·.;:,:,.          , ~+ l $.vcYXCC%q#$&M#@*#axt}ffjff
vJxxjxflll11[]1|Y$M$p*x&Xfl<[(=*+::     ·`,,iii`::;;, ,··;=?--1xj}xm#zc$=$|{fQUwO$k#Mk$#aaaa$#[ffjff
zXYvzvszcslll[I=s$kh*Wz&x0*W$$$$$$$$$kUc[//(?(~|~!;!(>]w&$$$$$*0f]1>z\<$w$s.am>YxX$qWbw$M$$$$Z/fftsj
zXzxvxvxsf1}f{1|}$pbk$j$}Uv(`     .!<YadZ0l<-)\|+=~(>Ivztf[=·       ,  + fb 1$*x$;M$bOZamXCC!s<jltsj
zccsxsxsfjlII1{[m$o#a$IW>fYI~+)lmz[=!  ;(1cx1</)!i|\(-/<<(fZ$$$$$$$$$pYU<Q$ I#:I$,{$kkZYv`  ,C{ffffj
zvvsxjsjxjll1}<IYJ$*wWjtQ<$$$$$$0$$$$$$$l-lJzf\)!·|)->]xUo$$>M$$$$U J$$$a$0,Ya s$`/*$jYi>)· (b{jffjj
zxcjsjsftlll{I<JpC$Uc$J(O(\O$$[; ~U$$@~\$w<z0x<)·``~?<-[{$x[; IcX{|~f1</i·! fY v$ -%$Q.{,+; {C1sssjj
vsslftfflI1}1t(Z$1cZ{o$-][*i=x$bbI-/<+==I}!>X0<?· .!||i`=?[X&px[>]bU-(,·· ;,0- Z$ {$$a x(~: YtllIlsj
cXvvztfftlft{{-Ia$v*]z$x[</+;;!(1[-\\--(, =*fX]):,i+:::`  . |~*<\=:`  ·,,!`)U;:$$ X$$=;t>  fZ1ftjtxs
zzctjxssI1l11>]?{p$$l{&Jt[>/=|!;;i;!|: ·,=(lJqI/i !=+=,:  .   · ·,,i!;:!~+?=?i{$1;vb$·)]· z$jfjssjvs
XvvxztjjjItIl1}{?>Zh$[t0Xt]>?*~~!~||!i|||?*tUws):  ***?+`  ·,,,,·i;;,i;~!|*?=ib$`?Xh$){|)dmIcjtxtfIx
XYzxxxvvxsjjffllt/=j$$<i1js{>)=+=ii;ii!!i~(akk}?i ,*?{[<\!·``,,,`·`,:;=~|i=(|-$f·vkhfJ]}OsxX1tftfftx
YJCzcscxxsjjltlII1]=l$$1\xccl}\(?=~|!`,:` UO0z?|   :`|)?(((+|!!`,,`,i;,i~*((~$$ *ZQsXU(j{cfltIffjjxs
YczJJzxjjjjjflllfl}}*l%$\lczsf[<\~?~|!:: ($[QQ\,   +>>=~1z\\*?||!iii;:;|+?/*1$`·$I{mb l}vl<I1Iffffxs
JUJczXYXYXxvjffIftflj/x$q-jczXtl>/+~i:,:!v$$O$$ka&q$$$$bY,;+?(=|!,,`;,|!?-)*$1 $$Ot$[,>\{-[tlfIffxsv
CJCCUJOzxzXXcYsjfffftsI$MfXUJUYt}>[(/=~+~|*w$$$$$Oj       . `,:~!!~+!+*(--?x$ $$@v$[ f{(\/I1IIIfxvxv
0ZUUYUjczXCzzzxzxsjsjc-k${fxzJxjfl>>*?)==~,   ·   *,     ,.  `i|||~+*+\-<*1$[+$$ZCkY}Il!)]]>[]ffffxv
0q0wQUUvxvvcUzfjxxxvsXlb$<YvzXCzvs}]>[\/<+,``|//:  i+-/?!.`|][)+=*=++)-\?v$t\/$8p$k$Os<=<[-{{lI1sxsv
0UzYzYXJCccvvxcXsxxxczj1M]zsXcY0Xfj}>>\-t$$$$$$$$$$$$$$$$$$$$0Qx>\))(\(-*${(]<$qOZ$M-jU()1{}[11xsxXv
0ZJ0XYJzYXczXcXvXzXXUv}XXtvczXUYZdx1><)+,!]oWbJU}tt<)==>It1(  :+?)/(((c$W:+[]\$0pq$l<fk&!>]{1Illttsx
0CUCQOzXzJUxvzYvYYUJXC$U1xxjvcXYzCphYs<)*; ·lp$wJXxYZqOJ{<~!*~;;?=|?fb$l *}[]\oZbCUIUx$${i1}1ltfsxYv
QCdUCUUQzUXU0UCJCQQOQdXIjcjxvccvzXvXwkJx]>)·  |]lYUzl<)!|:.`,:!;`~lk*}~ i=?-/]oC}]{{tv$d$*=I1llfjsjv
000ZCQ0CCmUXUxXcXXUJCjfcvvxvxjxvcvXY<C$UCxl(~.           ..`·`:(vh0I! ;;;~(<\xXY}\[mfz$x$$=[{{ltfzcv
Z0OZ0OmUYUzXUzcXzXYYcvzsxjtjjfvtfx{\w$$IJUJzI+;·,;|~|+~!; ,:~}0h0}!·:!`i;~//>z1Q0]$C*zaQa$<<fIltttcx
OCQOC0JzZUU0XUUXXzXJxXcjfjjtt1flf/xk$$$ztccQmpv1/*iii|~~)IX#$bx\, ·,:,:,!+*--s>>$t8=?$-O$$1)j1llvvcs
O0OCJCUC0JJQzCYUzcJYJxxcjftsIIlI*Xh$om$C1zczz0qpdpwwCwd8$$#U\!· `;;;;,,;~|(>*j1+wW{==lj#M$(\l1Ilttzj
OZmOCCYJzUYJCYzJJCXJUzvItI1}111(swmmJ$$zlcczcczxYQ0hWbdJj{~;;.·`,,,:`:!!?+<<~{x|{d)f<{[x$@x/IIstff1f
QOC0XCUYCXJYUJXXYXczczsfIf}{1{1~x$*Uw$ZlsvUQOqkUOYzll][+!~|i;:;,:`` ii~=?)</=]l~=<$q}ms$$Qat\l}1Ifvj
0JCOOUUCUzZYXQZXYUCvvzjfjl1t1]{=X$cp$$Ujzf1}</-<-/\/=~|!;;,``·.`.  :!;!++)))([)\f:)$$of0oWow>IllItxj
0OJO0XCUCCYYJz0CUUJJ0zXfjjII{[1*v$q8$b1j1t\?+==|i;!;!:;:i:,·`· ,`:,:``|?/))=*\!z+.  ?MpzY$$p]{js1lff
0000Q0JJJJUJJJCC0JJJUUcxsjflI1}>tC*#Os{[[[-)*~!ii!iii;;;ii::,:`,::;;:;!+*?))?)*=+,::,!\IUphQf1llltlt

