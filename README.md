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


QQQQ0ZmZmqOYuQaooo***oooooooaaooooobYnnnuf[-?][}}[-?(nOQn|JbhZncp*hXjnx/|///t//tttfffjrxYmwwppppdpkddW0~!_)cba*Z]}/||()1coZzzXXXYObQzzcvvvvunnxnnnnnnn
nYJUzj}:      aqmmqmmmmmmZZmZmmmmpwk[l<_+        '  <;zY: I#dw  a░@< !+    . ''         ~YrrrnnncvJUf░░     ,░bd "~;!I: <░Y!_1[}_X*r_}[[-+-+~~<>~~<__1
{I,    i]Y░░░M*hqZbbbbddbdbbdpqkdkb8r[fxf!l~<~__]{;l}:vq1':░░░dt░░░█W░░8aW█░░░░░░░░░░░░o░░░░░BBWdLOCr&░" >l :░w░ i/?}][i|░0[jnrx/Z%Y|nfjfttftf/jfffft(
+;_uk░░wn?"  p░░░░░8*#M#ohba8ddppp*@t?1j|;,i~+<l`   >[d░░B▒hUzXx?!;!>!I!l!!!!!>!i<>~+][1{[(juYJ0W░░░B░░;     ░w░';{_---I|░Y-|rfj)0&z)r/tf//()())1|1)))
r░mt+        ^haoo*W@%░░░░░8dqkbkpbBf?(u-,<,   .,}x#░░J1]}+lIl!><_-_-?])}}]~>":_{_---++~+~<i!ili~~[|jvp░/>   wd░<:(__<-;/░Q?/jft{0$c/j//t|ft/|)))/))))
!    '+jQ██░WY(?_~>I;^^,;Ii])rZqbbhWf[|u<  '+/b░░░hn[<_}{{1|/(()){}{11-!:i-|p░░?!(|)))1)1||/tttf(()]<lij░░░░░░b░J >}/f)>|░n_)fjt(0░U(jfft//|)/((((((((
_i[rrxf[>:^.'^`";i~{JkMJzOLB░U1[w#hBr<_[?[%░░&Y|?~?{tftt/1(|}))[[}}[>:18░░$j_l~░d;>1|(1))))|))(||1?]z$█u};^itbBwqb/^ .!!L░░(/||/{0&v|rfftt()|()|11((((
O░░WqQccczQZLLZh░░░░░░b(-+`:ki`^o%ho[]a░░█Y}+?{fxnjft//||/|1|)))({iIU░░;        Y░0!!-(t)?))?))?_t*░J"        'c░Ujqx>   ~11z#mt{08c(ttt|||t))(|||((((
x(rCh░░░░░░░█8░░}+~  mn:'  `░?';&&b▒░░qt?-}/jrjrjfjf////(/|1/1(|];Q░q       ''`   d░░Q?I)z+[Q()p░&[     '`''.    d░lx░░░!     ;u{0Mc)ftt//|((1|))|((((
c|?:   "I?(qmU$c^ ;  ░q+!: ;░t>:b░░*u[-)jjjftfffjfjtt//|t/t)))t-[░░    '`'''...`    !Y░░░░*h░░░j     ''......''   n░,!<f░░~   ,f)0Mz(ftt//||t()))1((((
n░░░X[~!' (#mQBQ<_ni @k->:.:░v^:░░f-?tjjfrffffffffjtftft/t/tt/]]░t   `'........'^^.    `~uXa)     ,^^^'........'`  ░k+(+;)░░░zux1LWv1ft//(())(/|||||||
+~zwr"<1^ U░qZ░C<i<, q@?-i I░r-@X[}tnxfftrffjtfffff//ttfffffx||░j  "' ......''.      '     o   `,      `'.......'' +░_)|1~"u░░t--Z░n)jttf(/|/t|||/)(((
:+Lm{ ~]' 0hpO80>"`^ O░{!I  ░░Mr1frrfjrj/rftf/ttffrnnx/|}}}[}~Y░  " ..'.....'   v*80   ^, :░ .^   f░░.  .....'.. '  ░>]1)(1!,*░p;z░x{t/|(/|)(/||||)))(
l]km]'!l imztrd█iix} Y░[>` +░X}/xxjfffrjjrxfxrnrt)-~~_1rYQmdbQ░M .`.........' [░~  I░J  ^ "░  ` >░p  ░░  `..... '` ;░+}-~i~}}I|░░JMz1jft/ft/ftt///ftt/
>[*Ji^++ }░cfrbB~.   x░{i {░r?fjrrfjxrrjrjxxr)]?|LW░░░░░hQznznj░   ^`........ m░░░xu░░  :  ░. . ░░:Z░░░- ' ...`'"  ░░b░░░░Q[i?<<░░*r/xffj///|))))1t///
<tazI;_i f#xjnm█-I-! /░? U░/{nxffffjjtjfrxr}1m░░░░z1            ░/    `^``..`  lh░kh?  ^  J░░    r░░░<  '`^^`     ░n      /░░f!iip░X{xrjjjj//ffttt///t
[ron,i[I Um/jrm░?;?_ {░"x░|1rtfrjrfjrxxxr{r░░h}          .'````  )░q      .``^           ░* :░}                :1o-         ;░░ni>b░u|zzvvvvuuxxxxxxxf
_z8)^>+; LZftrO░1iI" }░v░r1xrjffffffrxr}1░░}       '^`^```'   .`    rmx>^             1░w     :X'   `']1_?(nZ$c~:   ```^^""   :░░+;8░t?)[]]?-?-_-----(
[U8->t{ `pOjfxZ░(ll; <░░L|rrj/jrjrxxn){M░X'._}+I'          ````.'`    '>tzbO}[-?)/Jb@h1  u░░░░░░░W{   !iI:"'      ,^``          -░)l░░uXzXzcczcccccccn
)wh!^!",t#Jxfn0█1  ' !░█)frjfrjjfjnn[z░M   li-{cbB░░░k|I        ``^"^       '^^^`.     |░░c   lnvm░░^         ^"^       .l{nQkkt `░/{░*JOQQ0OJJJJJUUJU
tkb_!>i>v&Unfcq&p;', +░xjjrjft/frx/]░░_  ^^          '!x$░░░/;       '`^^`            J░nj<    )Zf}░░. ,`````     :[L░░░░U<:'     -░~C░kW##WW#WMM#&MMQ
YBdj)-[}J8fttxqdd]f+ C░(njjxjjrrr/)░░   "                   ;n░░Qj^   .....`````````` [░░fzX/nCc{}f░░  `'...  ?vh#Ox;              c░n░f/f/|())))))||r
QBQjx{{?wh|t)/mb$]t~ ░░)xnxrrfjn/1░p  "^.,)xnuxj/?_~:`'         .+&░}. '.............  !░░░░Umhq█░░J  .`...' >z!     "l<~~]|/|f(?^ l░n░f[t|)|)1111)1)/
Z&0t[_<)░acLYw░░░{>. ░█(xxxxxnnj1░Z  "^^^l+}{1|uXJLqb*@8B&░░░░Q;     ..'...............   ;{░d)X}`   '.......    Ijvv)}{({+>!I,`.   ░U░v/xjrjrjfffffrf
q@z/I<+tucLCr["   `i1░&fcvnrrxn)k░  ,^"^"'                      .`^`...................^    B|     ``........`^``    `           `  ░X░jtff///|ttt|/(t
k░Un><LZi,     >jXCxU░anvvvvvxr(░) ,"^^,^            .:>?1frcYc_ '......................``, _░  "``..............`^`` _b░░░░░░░m!  <░wW1fxrxrrrjjjrrfj
m░Oui     I(LpmJcYQ*a░░fzvvuucuc░! I",:^.";,]vZh░░░░░░░8OX/[+l,. .'...................... ` "░  ` .................  ;C+      l/|  z░░Y+)1{{}]]]]]]]]{
?    :+jmhwzvYpW░░0nin░UxXzunvtJ░: lI;;"x#o░░zj)+:`             ``'.'`....................^  ░  `.................` (░          `░░&░░!~?-_-++~~~~~-<+
_<vphkwOJQh░░$Ou[II>i~░MjYzzzcfz░< >!lil^ >░-    '`,:;lllIlI;::",`"``'`''.................^  ░` `.................` ;L. ^``````^ l░8░(,]+__+++~~~~~><>
YdYjnQb░░8Lj?ii+]-[{1lL░nxXcccun░_ ~lllI>"I░░i <?~l:;;:",,"""",^"^^^'````'''..............` '░. ^...................  ' .....'^  &░░x.i>ii!II;::::::II
Cd#░░aY(l,:i_}{(/[+-{<<░@|zcuccf░d ;>!!l!! i░░J  .!>l;;::::,,"^,",^`^`'^^`''.'............` i░  ^...................`' ....`^   k░░t Ii!lIIII;,""":,",
ULn],'i!l?{[[?][{]_?]}l]░Q|czczuc░] <ii!!l!  ?░░ml   "i>!l;,,,,,"","`^^`'`````'.''........' u░  `.......................`^`   c░░░} IiII:;,,"^^^^^^^^^
{>;`i1?]l<+-?[[?[[{[[[[ln░U)cuvcfq░, <!iii!~+  [*░░81'    ,!!i!I,:""^``^```'````^'`''....`  ░W  `..................`^^^     ░░░░░> i!I;;:,,""^`^^^^^^^
[1_!i?i-~!++?]][?----{)]lv░Q1ncvXt░░  ~ill!!>+>   -L░░░a|!      `;Ill!l;;,^^`^`'`'`.` '''`  ░+ `............``^^^^'      |░░  ░░, i!:I;,:,,^`^``'''^'`
?-+:^?i-!!<~?--?-~__-l,;~:(░B)rvvnt░░^ i~<!li!!+-l   '_ub░░░Wz[I         .^,;;:;:;::,,^^^" ░░  ``````^^^^^`          (░░░Q  "░░  ilI:::,",",`^`'`''`.'
_-~`'?>_IIi>-+_-]~~i;[X░/<;_░░xtcYct░░< '~_>!i!!i!>~<I    'i{Yq░░░░Wmr}~:                  -`                 .{uo░░░B)    ░░f '>;:,,",^`"`'''.. . '..
+~l`.+>>;;I!>~~~~~i/&kb░{+[><b░Ytfjx)C░d; '<+~<!illlIl!i>!"      "l>[nqM░░░░░&kUcr([_>i"'         .'"!-)za░░░░du_'       h░#  ii;::,:,""^^`''.     ...
~+! ^_<+l:li<~~++~_ ` ?░]<?/-:z░▒8*wY/)*░n^  :>~__-+<<i!IlI!i>il,^         ^I~[(xYZM░░░░░░░░░░░░░░░░8qz/~,             X░8^ ">;::::,,^"^``''..        
<]<.^>>>Il;><~~++i[`^,t░]<?)]{░ptv0h░░░qa░░o?'       `";!!liii!!ii!!<<<il:"`                                '"^^     X░B, '>l;;;,,,,""^^^``'''........
<?~`;~<il;:;~~~+-~]'I;Y░]i?)_)░░Jnj({)/vznY░░░#b0z(?>;'             .'`",:IIli><?+_+~~~>!I;:"",""^^`''.           ?o░X  .lI;;:,,;::,""^^^```''........
<_!"!]_i+!I<<>~~~<? !lf░|+]}{>}o░&M░░░WwYXx(1nzCqa8░░░▒░pQJzn{+!,`                                        `;+/cJ*░░p░& !il;;;;::,,,,^"^^^`'''.........
><>;l~i!<;"~<i+_<+?:'  :_--)[?|ko(txrvXJZwwa&&▒MakZLUUJCmo░M#W8░░░░░░░░░W%#MahdLUnXUQZOZQXr1|nYJqoB░░░░░░░░░#pQCJXL░{  >;II;;::,:,,,"^^^`^````.'''`..'
>+<^l-!!<I^;i<]+<~_]+~/}-?-1]-f░WfXvvvnxfjjfffrrXCmphbdMkOLwbdwJv|}_-)YXYunuuccJ░░░░░%&Mw&░░░W▒░Wcrjt)1)tjcY0q#▒BoUU░x.",:;:;::::,,:^""^^^^^````''''`'
i_>,;_~>~:;i<<~~>~~_~I>?_-]{]-/▒bfzvvvzzzvczzcvvxff/))())|/)(ruLh░░░░bjnp░░░░░░@░)          m░0Y/░░░░░░░░░&dLcj1[){!░░",!l;III;I::,,^"^^^^^^^''```````
>>?li!><>";;<i!-~+[++!<???}{[+f░m|Xuuvcczuzcxvvcczcvzzczzcnvjf(1??](0h}        Q░'C░%░░░░Mo|~░  ;░|1){]_-~_?[{)fn(_░░, iIIl::::,"""""""""`^`"^^```^^^`
+?1<<+++~!>>+>_}[]+_~><][?{)}+j░Yfccnuunncunurxnxurxxnjrrrxxufvnnnf)}(O░░8+     ░u    u░.   +░ :░1-((jnnxxurnucn[1░░^ >I!;l!;II;I:;:,,,""""^^^^^^^^^^^
-]{?_?}?<iI~<-]]]-->i!~-?]?{]~c░vfvnuvnnnnnvcxxfjjrnjrxxnrrxxnxfrrxnxx/[)Z░░j    pk(<;░k^xJQh" J░░░░kn{(xxnuj)?+U░m .~iiii!!!IIII:::,::,,",""^^^^^^^^^
]?/+-[{]?-+~<?}1{[}]]+-?}{{{[iv░rrcznnurxxnjxnunrjffjjrrrxjrrxxnrjrxnrncr]>z░░Q{+:/0rj░hu~i{[|v+"">10░░m/t1]/O░░░i ;<lii!!!!IIlI;I::::,,:,,","^^^^^^^^
{?][?[1}][?-[[{{{{{{1{{[1()1{>z░nfcvxnnxxnrxrfjjrfjfffjrxrxjjrrnxxxxnucj(░░m~!?_)}-]un   -d-          ,pBz░░░m;q░:i]~<!!i!!!llIII;;::,,,,,","^^^^^^^^^
|//(}{|)1(][}}(|)}111{{{))}]}_f░Xjnnnurxrrffj/t//rfrjfj){|truxnnuuvcczfr░J.          v░ |░  ',I;II;,"`  ░#(i   a░  .^i+>i!!lllI;:I;,:,,,"^^^`^^```^^^^
ft///|//t1}[))(/)1|(())(1[?-l _░Y/crxrnnffjjjjjfjjtfr)0%XYY1?]{|f11))1)░1  I!lI;;II!. █░░x ;l,'   ^,!:  ░     "░o/Ct~. li!iI!I;II;:;,,"""","^^^^^^^^^^
rtf)(t//()}{)jfj(/(|([-_{r?]Y**░0tvnrxjrrxtf////|tfjff(f░Lf░░░░Cv░░░░░░░| `I!>>>>iI   ░JI░r    ^".    <o░U|mJd░░░mZ░░░(':>lllII;;::::::,,""^^^^^^^^^^^
vQQftrurxrt/frnrxt/}?)rQz;)░&qzod/nuxxfftfff/()))(|t|tf[$c   i[░░^I:.   M/         iQ░░`  %░░░░░░░░░░░wBd0z-+>,~t░X](J░Y^><li!!!llII;;:,,,""^`````````
)}?{nnjtur((|/vf1{1nCc)l^f░prfxm░ufnxxxuftff|//|/(|/tft)(░▒     C+ `;>]~{░░mc/[]/mmQ/   ,z░W{,    '+O░░!         i░or(j░t,+<<<>iillII;;::,""^^^^^^```^
-><]cf/jvff)/|(tcLX[""<nnr░Z(vxJ░O(nuuuxrfff/tt//fffrxun([░░░coWB░░░░░░░bruh░░░░░░░/+: <░@`  ':l!I'   u░{ ~illIl, -░nv(░x,-><>iii!!llI;;:,",^"^^"^^^^^
][<[vrttf/)-]UwZ)I`ivJz({}h░░Yr(m▒jfunnjrxfjjf/t(fffjftrxr1|YqUnt}}{(((/rczf///|){u*░░░░}  +>>llIli<~: z░; ;,""": I░u1Y░-i[~<<i>!i!ll!II;;;:"""`"^"^""
?]-tJvrxt/rZ#0! ;nZQt(nO#djfd▒░&B░m/vvzvvcvvrcnrrvuvvXunvcuvxjxzYYCCCCLLJUJCLJCJL0YvjfvL░l'~>>><<<<>>-< ░$ !>lI;l (░/x░Yi{{}[--_+++~<>>i!lIllI;;;II;:,
+^ l]}?_[x/i  <COj[)r0Cn{[+>:lii,|░n[))){[{{{{][[[1{}|{{1){/(|/||ftt//ttfffjjjrfxrtjrrj}░░  '         ' }░        ░dt░n I>llII,,"^^'`'..             "
ft/rXUJLLvjnUQQCULZqwOLUXzzcvnxrrxmkLJUYYYYYXXXzXXYYUJJJCJCLLQ000OOO00OOOOOOOOOO0000000Qwaf-]]]][[[[}[}[zO{??-__~faphZf///|||))))11{{{}}[]]??-----___-


