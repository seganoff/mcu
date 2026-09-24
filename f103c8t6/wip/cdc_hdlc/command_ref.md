Baud:       9600
Data:       8 bits
Parity:     None
Stop bits:  1
Encoding:   ASCII
Cable:      Crossed / null-modem RS-232
[Command1][Command2][SPACE][Set ID][SPACE][Data][CR]
SPACE = 0x20
CR    = 0x0D
kh 01 32<CR>
The manuals specify Set ID as 1–10; Set ID 0 broadcasts to every connected set.
Set ID = 01
reply format
[Command2][SPACE][Set ID][SPACE][OK][Data][x]
h 01 OK 32x
TX:
kh 01 32<CR>
RX:
h 01 OK 32x

kh > h, kg > g mb > b
[Command2][SPACE][Set ID][SPACE][NG][Data][x]
h 01 NG FFx
kh 01 FF > h 01 NG FF (error ack)
!!!SetId 00: broadcast !!!
Function	Command	Data
Power	ka	00–01
Screen Mute	kd	00–01
Input Main	xb	00–FF
Input Sub	xc	00–FF
Input Sub2	xd	00–FF
Input Sub3	xe	00–FF
Aspect Ratio Main	xf	00–02
Aspect Ratio Sub	xg	00–01
Aspect Ratio Sub2	xh	00–01
Aspect Ratio Sub3	xi	00–01
PBP/PIP	kn	00–09
PIP Size	kp	00–02
Main/Sub Screen Change	ma	01
Picture Mode	dx	00–15
Brightness	kh	00–64
Contrast	kg	00–64
Sharpness	kk	00–64
Brightness Stabilization	mb	00–01
SUPERRESOLUTION+	mc	00–03
Black Level	md	00–01
HDMI ULTRA HD Deep Color	me	00–01
DFC	mf	00–01
Response Time	mg	00–03
Black Stabilizer	mh	00–64
Uniformity	mi	00–01
Gamma	mj	00–03
Color Temperature	ku	00–04
Red Gain	jw	00–64
Green Gain	jy	00–64
Blue Gain	jz	00–64
Language	fi	00–10
SMART ENERGY SAVING	mk	00–02
Auto Screen Off	mn	00–01
DisplayPort Version	mo	00–02
OSD Lock	km	00–01
Reset	fk	00–01
Volume Mute	ke	00–01
Volume Control	kf	00–64

ka 01 00    Power OFF
ka 01 01    Power ON
brightness
kh 01 00    minimum
kh 01 64    maximum
43ud79
Function	Command	Data
Power	ka	00–01
Screen Mute	kd	00–01
Input Main	xb	00–FF
Input Sub	xc	00–FF
Input Sub2	xd	00–FF
Input Sub3	xe	00–FF
Aspect Ratio Main	xf	00–02
Aspect Ratio Sub	xg	00–01
Aspect Ratio Sub2	xh	00–01
Aspect Ratio Sub3	xi	00–01
PBP/PIP	kn	00–09
PIP Size	kp	00–02
Main/Sub Screen Change	ma	01
Picture Mode	dx	00–14
Brightness	kh	00–64
Contrast	kg	00–64
Sharpness	kk	00–64
Brightness Stabilization	mb	00–01
SUPERRESOLUTION+	mc	00–03
BlackLevel	md	00–01
HDMI ULTRA HD Deep Color	me	00–01
DFC	mf	00–01
Response Time	mg	00–03
Black Stabilizer	mh	00–64
Uniformity	mi	00–01
Gamma	mj	00–09
Color Temperature	ku	00–04
Red Gain	jw	00–64
Green Gain	jy	00–64
Blue Gain	jz	00–64
Language	fi	00–10
SMART ENERGY SAVING	mk	00–02
LED Control Button	ml	00–03
DVI Power Supply	mm	00–01
Auto Screen Off	mn	00–01
DisplayPort 1.2	mo	00–01
OSD Lock	km	00–01
Reset	fk	00–01
Volume Mute	ke	00–01
Volume Control	kf	00–64

unpublished:
Total Power On Time
Backlight Hours
Total Operating Hours
Serial Number
Firmware Version
Panel Hours
Usage Statistics
Temperature
Diagnostic information

