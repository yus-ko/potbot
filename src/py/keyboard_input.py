#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import io
from ctypes import Structure, sizeof, c_long, c_int32, c_uint16
import math
from geometry_msgs.msg import Twist
import glob
import os

# Event types
types = {
    0x01 : 'KEY'
}

# Keys and buttons
keys = {
    0   : 'RESERVED',
    1   : 'ESC',
    2   : '1',
    3   : '2',
    4   : '3',
    5   : '4',
    6   : '5',
    7   : '6',
    8   : '7',
    9   : '8',
    10  : '9',
    11  : '0',
    12  : 'MINUS',
    13  : 'EQUAL',
    14  : 'BACKSPACE',
    15  : 'TAB',
    16  : 'Q',
    17  : 'W',
    18  : 'E',
    19  : 'R',
    20  : 'T',
    21  : 'Y',
    22  : 'U',
    23  : 'I',
    24  : 'O',
    25  : 'P',
    26  : 'LEFTBRACE',
    27  : 'RIGHTBRACE',
    28  : 'ENTER',
    29  : 'LEFTCTRL',
    30  : 'A',
    31  : 'S',
    32  : 'D',
    33  : 'F',
    34  : 'G',
    35  : 'H',
    36  : 'J',
    37  : 'K',
    38  : 'L',
    39  : 'SEMICOLON',
    40  : 'APOSTROPHE',
    41  : 'GRAVE',
    42  : 'LEFTSHIFT',
    43  : 'BACKSLASH',
    44  : 'Z',
    45  : 'X',
    46  : 'C',
    47  : 'V',
    48  : 'B',
    49  : 'N',
    50  : 'M',
    51  : 'COMMA',
    52  : 'DOT',
    53  : 'SLASH',
    54  : 'RIGHTSHIFT',
    55  : 'KPASTERISK',
    56  : 'LEFTALT',
    57  : 'SPACE',
    58  : 'CAPSLOCK',
    59  : 'F1',
    60  : 'F2',
    61  : 'F3',
    62  : 'F4',
    63  : 'F5',
    64  : 'F6',
    65  : 'F7',
    66  : 'F8',
    67  : 'F9',
    68  : 'F10',
    69  : 'NUMLOCK',
    70  : 'SCROLLLOCK',
    71  : 'KP7',
    72  : 'KP8',
    73  : 'KP9',
    74  : 'KPMINUS',
    75  : 'KP4',
    76  : 'KP5',
    77  : 'KP6',
    78  : 'KPPLUS',
    79  : 'KP1',
    80  : 'KP2',
    81  : 'KP3',
    82  : 'KP0',
    83  : 'KPDOT',
        
    85  : 'ZENKAKUHANKAKU',
    86  : '102ND',
    87  : 'F11',
    88  : 'F12',
    89  : 'RO',
    90  : 'KATAKANA',
    91  : 'HIRAGANA',
    92  : 'HENKAN',
    93  : 'KATAKANAHIRAGANA',
    94  : 'MUHENKAN',
    95  : 'KPJPCOMMA',
    96  : 'KPENTER',
    97  : 'RIGHTCTRL',
    98  : 'KPSLASH',
    99  : 'SYSRQ',
    100 : 'RIGHTALT',
    101 : 'LINEFEED',
    102 : 'HOME',
    103 : 'UP',
    104 : 'PAGEUP',
    105 : 'LEFT',
    106 : 'RIGHT',
    107 : 'END',
    108 : 'DOWN',
    109 : 'PAGEDOWN',
    110 : 'INSERT',
    111 : 'DELETE',
    112 : 'MACRO',
    113 : 'MUTE',
    114 : 'VOLUMEDOWN',
    115 : 'VOLUMEUP',
    116 : 'POWER',  # SC System Power Down
    117 : 'KPEQUAL',
    118 : 'KPPLUSMINUS',
    119 : 'PAUSE',
    120 : 'SCALE',  # AL Compiz Scale (Expose)
        
    121 : 'KPCOMMA',
    122 : 'HANGEUL',
    123 : 'HANJA',
    124 : 'YEN',
    125 : 'LEFTMETA',
    126 : 'RIGHTMETA',
    127 : 'COMPOSE',
        
    128 : 'STOP',  # AC Stop
    129 : 'AGAIN',
    130 : 'PROPS',  # AC Properties
    131 : 'UNDO',  # AC Undo
    132 : 'FRONT',
    133 : 'COPY',  # AC Copy
    134 : 'OPEN',  # AC Open
    135 : 'PASTE',  # AC Paste
    136 : 'FIND',  # AC Search
    137 : 'CUT',  # AC Cut
    138 : 'HELP',  # AL Integrated Help Center
    139 : 'MENU',  # Menu (show menu)
    140 : 'CALC',  # AL Calculator
    141 : 'SETUP',
    142 : 'SLEEP',  # SC System Sleep
    143 : 'WAKEUP',  # System Wake Up
    144 : 'FILE',  # AL Local Machine Browser
    145 : 'SENDFILE',
    146 : 'DELETEFILE',
    147 : 'XFER',
    148 : 'PROG1',
    149 : 'PROG2',
    150 : 'WWW',  # AL Internet Browser
    151 : 'MSDOS',
    152 : 'SCREENLOCK',  # AL Terminal Lock/Screensaver
    153 : 'DIRECTION',
    154 : 'CYCLEWINDOWS',
    155 : 'MAIL',
    156 : 'BOOKMARKS',  # AC Bookmarks
    157 : 'COMPUTER',
    158 : 'BACK',  # AC Back
    159 : 'FORWARD',  # AC Forward
    160 : 'CLOSECD',
    161 : 'EJECTCD',
    162 : 'EJECTCLOSECD',
    163 : 'NEXTSONG',
    164 : 'PLAYPAUSE',
    165 : 'PREVIOUSSONG',
    166 : 'STOPCD',
    167 : 'RECORD',
    168 : 'REWIND',
    169 : 'PHONE',  # Media Select Telephone
    170 : 'ISO',
    171 : 'CONFIG',  # AL Consumer Control Configuration
    172 : 'HOMEPAGE',  # AC Home
    173 : 'REFRESH',  # AC Refresh
    174 : 'EXIT',  # AC Exit
    175 : 'MOVE',
    176 : 'EDIT',
    177 : 'SCROLLUP',
    178 : 'SCROLLDOWN',
    179 : 'KPLEFTPAREN',
    180 : 'KPRIGHTPAREN',
    181 : 'NEW',  # AC New
    182 : 'REDO',  # AC Redo/Repeat
        
    183 : 'F13',
    184 : 'F14',
    185 : 'F15',
    186 : 'F16',
    187 : 'F17',
    188 : 'F18',
    189 : 'F19',
    190 : 'F20',
    191 : 'F21',
    192 : 'F22',
    193 : 'F23',
    194 : 'F24',
        
    200 : 'PLAYCD',
    201 : 'PAUSECD',
    202 : 'PROG3',
    203 : 'PROG4',
    204 : 'DASHBOARD',  # AL Dashboard
    205 : 'SUSPEND',
    206 : 'CLOSE',  # AC Close
    207 : 'PLAY',
    208 : 'FASTFORWARD',
    209 : 'BASSBOOST',
    210 : 'PRINT',  # AC Print
    211 : 'HP',
    212 : 'CAMERA',
    213 : 'SOUND',
    214 : 'QUESTION',
    215 : 'EMAIL',
    216 : 'CHAT',
    217 : 'SEARCH',
    218 : 'CONNECT',
    219 : 'FINANCE',  # AL Checkbook/Finance
    220 : 'SPORT',
    221 : 'SHOP',
    222 : 'ALTERASE',
    223 : 'CANCEL',  # AC Cancel
    224 : 'BRIGHTNESSDOWN',
    225 : 'BRIGHTNESSUP',
    226 : 'MEDIA',
        
    227 : 'SWITCHVIDEOMODE',  # Cycle between available video
                              # outputs (Monitor/LCD/TV-out/etc)
    228 : 'KBDILLUMTOGGLE',
    229 : 'KBDILLUMDOWN',
    230 : 'KBDILLUMUP',
        
    231 : 'SEND',  # AC Send
    232 : 'REPLY',  # AC Reply
    233 : 'FORWARDMAIL',  # AC Forward Msg
    234 : 'SAVE',  # AC Save
    235 : 'DOCUMENTS',
        
    236 : 'BATTERY',
        
    237 : 'BLUETOOTH',
    238 : 'WLAN',
    239 : 'UWB',
        
    240 : 'UNKNOWN',
        
    241 : 'VIDEO_NEXT',  # drive next video source
    242 : 'VIDEO_PREV',  # drive previous video source
    243 : 'BRIGHTNESS_CYCLE',  # brightness up, after max is min
    244 : 'BRIGHTNESS_ZERO',  # brightness off, use ambient
    245 : 'DISPLAY_OFF',  # display device to off state
        
    246 : 'WIMAX',
}

# Range 248 - 255 is reserved for special needs of AT keyboard driver
actions = {
    0 : 'UP',
    1 : 'DOWN',
    2 : 'HELD',
}

class TimeVal(Structure):
    _fields_ = (
        ("tv_sec", c_long),
        ("tv_usec", c_long),
    )

class InputEvent(Structure):
    _fields_ = (
        ('time', TimeVal),
        ('type', c_uint16),
        ('code', c_uint16),
        ('value', c_int32),
    )

event_size = sizeof(InputEvent)

def get_input_event_struct(f):
    ie = io.BytesIO(f.read(event_size))
    struct_ie = InputEvent()
    ie.readinto(struct_ie)
    return struct_ie

pressed_keys = set()


def detect_hotkey(hotkeys_dict, key, action):
    global pressed_keys
    
    if action == 'UP':
        pressed_keys.remove(key)
    
    elif action == 'DOWN' or action == 'HELD':
        pressed_keys.add(key)
    else: 
        return None

    
    for message, hotkey in hotkeys_dict.items():
        if hotkey == pressed_keys:
            
            return message 
    return None
	
if __name__ == '__main__':
    
    file_list=glob.glob("/dev/input/*")
    file_event = []
    for f in file_list:
        if "event" in f:
            file_event.append(f)
    sorted(file_event, key=lambda f: os.stat(f).st_mtime, reverse=True)
    msg = "event file: " + file_event[0]
    print(msg)

    dev_input_file = file_event[0]
    
    hotkeys_dict = {'straight':         {'W'},
                    'left':             {'A'},
                    'back':             {'S'},
                    'right':            {'D'},
                    'straight_sprint':  {'W', 'LEFTSHIFT'},
                    'left_sprint':      {'A', 'LEFTSHIFT'},
                    'back_sprint':      {'S', 'LEFTSHIFT'},
                    'right_sprint':     {'D', 'LEFTSHIFT'}}

    print('go straight:     W')
    print('turn left:       A')
    print('go back:         S')
    print('turn right:      D')
    print('sprint:          + LEFTSHIFT')
    
    cmd = Twist()
    vel_straight = 0.1
    vel_back = -0.1
    vel_straight_sprint = 0.4
    vel_back_sprint = -0.4

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size =20)
    rospy.init_node('keyboard_input', anonymous=True)

    with open(dev_input_file, mode='rb', buffering=1) as f:

        while not rospy.is_shutdown():
            
            struct_ie = get_input_event_struct(f)
            
            if struct_ie.type == 0x01: # key event type
                
                key = keys.get(struct_ie.code)
                if key is None:
                    continue 

                action = actions.get(struct_ie.value)
                if action is None:
                    continue 
                
                message = detect_hotkey(hotkeys_dict, key, action)
                
                # if message == 'straight':
                #     cmd.linear.x = 0.2
                # elif message == 'straight_sprint':
                #     cmd.linear.x = 0.4

                # elif message == 'back':
                #     cmd.linear.x = -0.2
                # elif message == 'back_sprint':
                #     cmd.linear.x = -0.4
                
                # elif message is None:
                #     cmd.linear.x = 0

                # if message == 'left':
                #     cmd.angular.z = math.pi/2
                # elif message == 'left_sprint':
                #     cmd.angular.z = 2*math.pi/2

                # elif message == 'right':
                #     cmd.angular.z = -math.pi/2
                # elif message == 'right_sprint':
                #     cmd.angular.z = -2*math.pi/2

                # elif message is None:
                #     cmd.angular.z = 0

                if key == 'W':
                    if action == 'DOWN' or action == 'HELD':
                        cmd.linear.x = vel_straight
                    elif action == 'UP':
                        cmd.linear.x = 0

                elif key == 'S':
                    if action == 'DOWN' or action == 'HELD':
                        cmd.linear.x = vel_back
                    elif action == 'UP':
                        cmd.linear.x = 0

                if key == 'A':
                    if action == 'DOWN' or action == 'HELD':
                        cmd.angular.z = math.pi/2
                    elif action == 'UP':
                        cmd.angular.z = 0

                elif key == 'D':
                    if action == 'DOWN' or action == 'HELD':
                        cmd.angular.z = -math.pi/2
                    elif action == 'UP':
                        cmd.angular.z = 0

                if key == 'LEFTSHIFT':
                    if action == 'DOWN' or action == 'HELD':
                        if cmd.linear.x == vel_straight:
                            cmd.linear.x = vel_straight_sprint
                        elif cmd.linear.x == vel_back:
                            cmd.linear.x = vel_back_sprint

                    elif action == 'UP':
                        if cmd.linear.x == vel_straight_sprint:
                            cmd.linear.x = vel_straight
                        elif cmd.linear.x == vel_back_sprint:
                            cmd.linear.x = vel_back
                            
                pub_cmd.publish(cmd)