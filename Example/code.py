# -*- coding: utf-8 -*-
# SPDX-FileCopyrightText: 2023 Paulus Schulinck
#
# SPDX-License-Identifier: MIT
# ===============================================
# ESP32-S2-GPS_sentences_via_Serial_GPRMA_GPRMC
# for use with a Kaluga-1 dev board
# ===============================================

"""Main script to receive, decode and present FS2020 FSUIPC7 GPSout2 data"""

#from docutils import nodes
#from docutils.parsers.rst import Directive

# pylint: disable=too-few-public-methods

#from _typeshed import NoneType
import os
import sys
#sys.path.insert(0, '/media/paulus/CIRCUITPY/lib/') # when used with Linux Ubuntu 10.04
#sys.path.insert(0, '/media/paulus/CIRCUITPY/lib/adafruit_ili9341/')  # to prevent module ili9341 not found
#sys.path.insert(0, 'D:/lib')  # when used with MS Windows 11

# host_chk()  # Check the OS of a host PC and add search path for CIRCUITPY disk accordingly

import board
import neopixel
import digitalio

import adafruit_gfx
from adafruit_display_text import label
#from adafruit_rgb_display import ili9341
#from adafruit_rgb_display.rgb import color565
from lib.adafruit_ili9341 import ili9341    # has as only font file: 'gfx_standard_font_01'
#from adafruit_bitmap_font import bitmap_font

import displayio
from busdisplay import BusDisplay as display
import fourwire
import terminalio
import json
from array import array

# import pdb; pdb.set_trace()  # REPL answered: no module named 'pdb'

import busio
import time

import adafruit_ntp
import binascii
import struct

# import tft_defs  # tft definitions, originally in this script, however not used in this moment

# +-----------------------------+
# | COM port definition         |
# | (is just informative)       |
# | Port subject to change in   |
# | MS Windows 10 O.S.          |
# | ATTENTION:                  |
# | Set COM port in FSUIPC7     |
# | GPSout1 accoridingly        |
# | See config.json             |
# | and: state.globCOMprt.add() |
# +-----------------------------+

class State:
    def __init__(self, saved_state_json=None):
        self.board_id = None
        self.my_debug = False
        self.globCOMprt = 'COM17'
        self.use_wifi = False
        self.wlan = None
        self.lStart = True
        self.startup = -1
        self.loop_nr = -1
        self.max_loop_nr = 30
        self.max_msg_len = 18
        self.ssid = ""
        self.pw = ""
        self.ip = None
        self.s__ip = None
        self.mac = None
        self.use_TAG = True
        self.tag_le_max = 24
        self.encoding = "utf-8"
        self.dt_str_usa = True
        self.use_dst = False
        self.dst = 0
        self.COUNTRY = None
        self.STATE = None
        self.tm_tmzone = None # was: 'Europe/Lisbon' # abbreviation of timezone name
        #tm_tmzone_dst = "WET0WEST,M3.5.0/1,M10.5.0"
        self.UTC_OFFSET = None
        self.is_12hr = None
        self.dst = 0
        self.gps_variant = None
        self.lGPSgroup = False
        self.y_list = [60, 100, 140, 180] 
        self.text1 = ['LAT', 'LON', 'GS       KTS', 'CRS      DEGS']
        self.lat_lbl = None 
        self.lon_lbl = None 
        self.gs_lbl = None 
        self.crs_lbl = None
        self.HIGH = 1
        self.LOW = 0
        self.led_state = self.HIGH  # idem. When HIGH the LED is OFF
        self.led_colors_dict = {
            'green' : 0,
            'red' : 1,
            'blue' : 2,
            'white' : 3,
            'off' : 4 }
        self.bril = 50  # Brilliance LED. Was: 20
        self.bril_dim = 5  # Dimmed brilliance LED (e.g. for White color)
        #                GREEN          RED           BLUE                  WHITE                   OFF
        self.color_c_arr = [(0, self.bril, 0), (self.bril, 0, 0), (0, 0, self.bril), (self.bril_dim, self.bril_dim, self.bril_dim), (0, 0, 0)]
        self.use_neopixel = True
        self.neopixel_brightness = 0.5
        self.num_pixels = 1  # The number of NeoPixels
        self.neopixel_dict = {
            "BLK": (0, 0, 0),
            "RED": (200, 0, 0),
            "GRN": (0, 200, 0),
            "BLU": (0, 0, 200)}
        self.neopixel_rev_dict = {
            (0, 0, 0)   : "BLK",
            (200, 0, 0) : "RED",
            (0, 200, 0) : "GRN",
            (0, 0, 200) : "BLU"}
        # Values of GPRMC_Dict and GPRMA_Dict except the value for 'descr' that will be done by 'csv_read()'
        self.GPRMC_Dict = {  # Empty strings default to ...
            '': {},
            'GPRMC' : {
            'ID': '$GPRMC',
            'type': 'RMC',
            'fields': 12,
            'descr': ''},
        }
        self.GPRMA_Dict = {  # Empty strings default to ...
            '': {},
            'GPRMA' : {
            'ID': '$GPRMA',
            'type': 'RMA',
            'fields': 12,
            'descr': ''},
        }
        self.my_gprmx = {}
        self.tuples = {}
        self.GPSvariantsDict = {}  # Create an empty Dictionary. It will be initiated in setup() and filled by 'setup_GPS_main_dict()'
        self.GlobalGPSvariantDict = {}
        self.GlobalGPSvariantID = ''
        self.lGlobalGPSvariantDictIsSet = False
        self.o_data = b''
        self.lOnREPL = True
        self.lMsgOK = False
        self.nr_of_msgs = 0
        self.msg_nr = 0
        self.previousMillis = 0

state = State()

state.board_id = board.board_id

if state.use_wifi:
    import wifi
    import ipaddress
    import socketpool

    if not state.my_debug:
        if wifi is not None:
            print(f"wifi= {type(wifi)}")
            
    pool = socketpool.SocketPool(wifi.radio)
    ntp = adafruit_ntp.NTP(pool, tz_offset=0)

# +--------------------------+
# | Disable USB Mass Storage |
# +--------------------------+
#import storage 
#storage.disable_usb_drive()

board_id = board.board_id

# Define ESP32 SPI pins to which the display is attached
# tft_sck = board.IO19   <-- this one is connected to the USB-B breakout board

tft_cs = board.LCD_CS  # SPI chip select pin
tft_sck = board.LCD_CLK  # SPI clock pin
tft_miso = board.LCD_MISO  # SPI MISO
tft_mosi = board.LCD_MOSI  # SPI MOSI
tft_dc= board.LCD_D_C  # Display command/data pin
tft_rst = board.LCD_RST # Display reset

tft_bl_ctr = board.LCD_BL_CTR
backlight = digitalio.DigitalInOut(board.LCD_BL_CTR)
backlight.switch_to_output()
backlight.value = True

# Set this to 1 if you want to use touch sceen functions
tft_USE_TOUCH = False  # was: tft_TOUCH_TYPE_NONE

# Release any resources currently in use for the displays
displayio.release_displays()

# Use Hardware SPItitle_grp
spi = busio.SPI(clock=tft_sck, MOSI=tft_mosi, MISO=tft_miso)

# RESET = board.IO??

tft_width = 320
tft_height = 240
# y_list = [60, 100, 140, 180]  # Moved to State Class. This list contains the y-values for frame texts (see pr_frame() and pr_fs_data())
# y_list = [30, 50, 70, 90]

display_bus = fourwire.FourWire(spi, command=tft_dc, chip_select=tft_cs)

display = ili9341.ILI9341(display_bus, width=tft_width, height=tft_height) # , baudrate=16000000, phase=0, rotation=0)

title_grp = displayio.Group()
display.root_group = title_grp
fs_grp = displayio.Group()
frame_grp = displayio.Group()

lbl_scale = 2
state.lat_lbl = None
state.lon_lbl = None
state.gs_lbl = None
state.crs_lbl = None

# Moved to State Class. text1 = ['LAT', 'LON', 'GS       KTS', 'CRS      DEGS']

""" The Bitmap and Palette classes work together to generate colored pixels """

# Create a bitmap 320 pixels wide, 240 pixels high, with each pixel having 3 possible values
bitmap = displayio.Bitmap(320, 240, 3)

# --- STATIC (not to be changed!) -------

tft_name1 = "MSFS 2020"
tft_name2 = "GPS data RX"

"""
for an ESP32-S2-Kaluga-1 the builtin LED pin is 45.
"""
BUILTIN_LED = board.NEOPIXEL
pixel_pin = board.NEOPIXEL

"""
The order of the pixel colors - RGB or GRB.
Some NeoPixels have red and green reversed!
For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
"""

ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin,
    state.num_pixels,
    brightness=state.neopixel_brightness,
    auto_write=False,
    pixel_order=ORDER
)

# +-------------------------+
# | Definition for the UART |
# +-------------------------+

"""
Note @paulsk: I experienced that one needs to use
the 'tx =', 'rx =', 'baudrate =' and 'timeout ='
when these parameters are used.
"""
# See: https://github.com/adafruit/circuitpython/blob/main/ports/espressif/boards/espressif_kaluga_1.3/pins.c
my_UART_TX_pin = 'board.IO43' # was: 'board.IO16' board.TX = GPIO43
my_UART_RX_pin = 'board.IO44' # was: 'board.IO17' board.RX = GPIO44
uart = busio.UART(tx = board.TX, rx = board.RX, baudrate = 4800, timeout = 2.00)

# +-----------------------------+
# | Global variables definition |
# +-----------------------------+

# ---------------------+------------------------------------------------------------------------------------+
third_part = True   #  | <<< --------THIS DETERMINIS IF THE 3RD PART (Show message on LCD) WILL BE EXECUTED |
# ---------------------+------------------------------------------------------------------------------------+

# +---------------+--------------------------------------------------------------------------------------+
max_outer_lp_cnt = 32  # | <<<-----------------OUTER LOOP COUNT ----(32 x the outer loop)                |
max_inner_lp_cnt =  1  # | was 13 # | <<<------INNER LOOP COUNT ----( 1 GPS sentence messsage to catch)  |
# +---------------+--------------------------------------------------------------------------------------+

# See: https://stackoverflow.com/questions/51970303/dictionary-conversion-form-python-string-with-trailing-l

if sys.version_info > (3,):
    long = int  # See the notation: '1000L' below at line 634

lstop = 0

msg_elements = []
NoPrintMsg = "will not be shown because of status \"False\"\nof one of the flags: \"first_part\", \"second_part\" or \"third_part\"\n"

# Copied from: I:\PaulskPt\ESP32-S3-Box_MSFS2020_GPSout_GPRMC_and_GPGGA\Examples\version_3\code.py
# Buffers
rx_buffer_len = 160
rx_buffer = bytearray(rx_buffer_len * b'\x00')

# +-----------------------------+
# |GPRMC_Dict Dictionary        |
# | and associated definitions  |
# +-----------------------------+
csv_name = ''

""" Class RMA (live example from MSFS2020 on 2021-020-02): ' $GPRMA,A,3609.0915,N,00520.4463,W,0.0,0.0,0.0,266.6,2.1,W*70' """

# +---------------------------+
# | End of global Definitions |
# +---------------------------+

# +--------------------+
# | Start of functions |
# +--------------------+
def tag_adj(state, t):
    
    if state.use_TAG:
        le = 0
        spc = 0
        ret = t

        if isinstance(t, str):
            le = len(t)
        if le >0:
            spc = state.tag_le_max - le
            #print(f"spc= {spc}")
            ret = ""+t+"{0:>{1:d}s}".format("",spc)
            #print(f"s=\'{s}\'")
        return ret
    return ""

"""function to save the config dict to the JSON file"""
def save_config(state):
    TAG = tag_adj(state, "save_config():"+" "*12)
    ret = 0
    try:
        with open("config.json", "w") as f:
            json.dump(config, f)
        ret = 1
    except OSError as e:
        print(TAG+f"Error: {e}")
        return ret
    return ret

config = None

# load the config file from flash
with open("config.json") as f:
    config = json.load(f)
if not state.my_debug:
    print(f"global(): config: {config}")


def read_fm_config(state):
    TAG = tag_adj(state, "read_fm_config(): ")
    key_lst = list(config.keys())
    if state.my_debug:
        print(TAG+f"global, key_lst: {key_lst}")
        print(TAG+"setting state class variables:")
    for k,v in config.items():
        if isinstance(v, int):
            s_v = str(v)
        elif isinstance(v, str):
            s_v = v
        elif isinstance(v, bool):
            if v:
                s_v = "True"
            else:
                s_v = "False"
        if state.my_debug:
            print("\tk: \'{:10s}\', v: \'{:s}\'".format(k, s_v))
        if k in key_lst:
            if k == "globCOMprt":
                state.globCOMprt = v
            if k == "GPS_variant":
                state.gps_variant = v
                state.GlobalGPSvariantID = v
            if k == "COUNTRY":
                if v == "PRT":
                    config["STATE"] == ""
                    config["UTC_OFFSET"] == 1
                elif v == "USA":
                    config["STATE"] = "NY"
                    config["UTC_OFFSET"] = -4
                state.COUNTRY = v
                state.STATE = config["STATE"]
            if k == "dt_str_usa":
                state.dt_str_usa = v
            if k == "is_12hr":
                state.is_12hr = v
            if k == "Use_dst":
                state.use_dst = v
            if k == "UTC_OFFSET":
                state.UTC_OFFSET = v
            if k == "tmzone":
                state.tm_tmzone = v
    if state.my_debug:
        print(TAG+f"for check:\n\tstate.COUNTRY: \'{state.COUNTRY}\', state.STATE: \'{state.STATE}\', \
            state.UTC_OFFSET: {state.UTC_OFFSET}, state.tm_tmzone: \'{state.tm_tmzone}\'")

save_config(state)


def setup_lbls(state):
    global fs_grp
    TAG = tag_adj(state, "setup_lbls(): ")
    if fs_grp is None:
        fs_grp = displayio.Group()

    lat_txt_grp = None
    lon_txt_grp = None
    gs_txt_grp = None
    crs_txt_grp = None

    clr = 0xFFFF00
    fs_scale = 2

    state.lat_lbl = label.Label(
        font=terminalio.FONT,
        scale=fs_scale,
        text="",
        color=clr)
    
    state.lon_lbl = label.Label(
        font=terminalio.FONT,
        scale=fs_scale,
        text="",
        color=clr)

    state.gs_lbl = label.Label(
        font=terminalio.FONT,
        scale=fs_scale,
        text="",
        color=clr)
    
    state.crs_lbl = label.Label(
        font=terminalio.FONT,
        scale=fs_scale,
        text="",
        color=clr)
    
    grp_x = 10
    lat_txt_grp = displayio.Group(scale=1, x=grp_x, y=state.y_list[0])
    lat_txt_grp.append(state.lat_lbl)
    fs_grp.append(lat_txt_grp)

    lon_txt_grp = displayio.Group(scale=1, x=grp_x, y=state.y_list[1])
    lon_txt_grp.append(state.lon_lbl)
    fs_grp.append(lon_txt_grp)

    gs_txt_grp = displayio.Group(scale=1, x=grp_x, y=state.y_list[2])
    gs_txt_grp.append(state.gs_lbl)
    fs_grp.append(gs_txt_grp)

    crs_txt_grp = displayio.Group(scale=1, x=grp_x, y=state.y_list[3])
    crs_txt_grp.append(state.crs_lbl)
    fs_grp.append(crs_txt_grp)


"""
 * @brief In this version of CircuitPython one can only check if there is a WiFi connection
 * by checking if an IP address exists.
 * In the function do_connect() the global variable s_ip is set.
 *
 * @param sate class object
 *
 * @return boolean. True if exists an ip address. False if not.
"""
def wifi_is_connected(state):
    if state.ip is not None:
        my_ip = state.ip
    else:
        my_ip = wifi.radio.ipv4_address

    if my_ip is None:
        return False
    else:
        my_s__ip = str(my_ip)
        state.s__ip = my_s__ip
        return True if my_s__ip is not None and len(my_s__ip) > 0 and my_s__ip != '0.0.0.0' else False

"""
 * @brief function that establish WiFi connection
 * Function tries to establish a WiFi connection with the given Access Point
 * If a WiFi connection has been established, function will:
 * sets the global variables: 'ip' and 's_ip' ( the latter used by function wifi_is_connected() )
 *
 * @param None
 *
 * @return None
"""
def do_connect(state):
    TAG = tag_adj(state, "do_connect(): ")

    # Get env variables from file settings.toml
    state.ssid = os.getenv("CIRCUITPY_WIFI_SSID")
    state.pw = os.getenv("CIRCUITPY_WIFI_PASSWORD")
    
    try:
        wifi.radio.connect(ssid=state.ssid, password=state.pw)
    except ConnectionError as e:
        print(TAG+f"WiFi connection Error: \'{e}\'")
    except Exception as e:
        print(TAG+f"Error: {dir(e)}")

    state.ip = wifi.radio.ipv4_address

    if state.ip:
        state.s__ip = str(state.ip)

def button_loop():
    global buttonA, buttonB, backlight, tft
    while True:
        if buttonA.value and buttonB.value:
            backlight.value = False  # turn off backlight
        else:
            backlight.value = True  # turn on backlight
        if buttonB.value and not buttonA.value:  # just button A pressed
            display.fill(display.color565(255, 0, 0))  # red
        if buttonA.value and not buttonB.value:  # just button B pressed
            display.fill(display.color565(0, 0, 255))  # blue
        if not buttonA.value and not buttonB.value:  # none pressed
            display.fill(display.color565(0, 255, 0))  # green

"""
    @brief
    Setup in fact performs the main task of this CircuitPython script.
    It consists of an outer loop and an inner loop.
    Print the received GPS data to the display:

    param:  None

    :returns: execution was successful or not:
    :rtype: boolean
"""
def loop(state):
    global max_outer_lp_cnt, max_inner_lp_cnt, lstop

    TAG = tag_adj(state, "loop(): ")

    outer_lp_cnt = 0  # Set the initial value of the outer_lp_cnt
    msg_lp_cnt = 0  # Set the initial value of the inner (msg_lp_cnt) loop
    msg_data = None

    # currentMillis = millis() # (unsigned long)

    # a = LOW  # (int) When LOW, the LED will be ON.

    """
    Collect Serial Input NON-BLocking.
    Continue receiving characters until '\n' (linefeed) is received.
    """

    outer_lp_cnt += 1  # increase the outer loop counter, to start it with index 1

    while True:  # Entry of the outer loop
        if state.startup == -1:
            if state.use_wifi:
                if wifi_is_connected(state):
                    print(TAG+f"WiFi connected to: {state.ssid}. IP: {state.s__ip}")
            pr_intro()  # Print the intro

        if state.lOnREPL:
            print(TAG+"Outer loop count value: {}".format(outer_lp_cnt), end='\n')

        # display.clear()  # clean the LCD
        if state.lOnREPL:
            print("\nStart of loop {}".format(outer_lp_cnt), end='\n')
            print("........................\n")

        if state.led_state == state.HIGH:  # Switch off the builtin LED
            led_toggle(state)

        if state.startup == -1:

            # The next lines are to prepare the LCD for the displaying of the GPR.. type of messages
            # We moved these lines to here so that they are only written once (shortens the write LCD time) (pr_third_part_GPRMA())

            pr_frame(state)

            # state.startup = 0  # switch off flag. Showing this text only once. We do this later in setup()

        lStop = False
        state.msg_nr = 0   # Reset the message number count
        wait_cnt = 0
        msg_lp_cnt = 0  # Reset msg_lp_cnt (inner loop count). We will do this loop 'max_inner_lp_cnt' times to catch a whole GPS sentences cycle
        while True:  # Entry of the inner loop
            currentMillis = time.monotonic()  # float (according to the CircuitPython Documentation)
            msg_data = None  # Clear

            # +---------------------------------------+
            # | Here we are receiving the serial data |
            # +---------------------------------------+

            msg_data = ck_uart(state)  # Check the uart line and if available, collect the uart data

            #print("OK, we are in the GPS data collection part...")

            if state.my_debug:
                print(TAG+f"Message data is: {msg_data}", end='\n')
            if msg_data:
                # print(TAG+f"type of rcvd data is: {msg_data}")  # Result: class <str>
                state.msg_nr += 1  # Increase the message index pointer
                state.nr_of_msgs += 1  # Increase the number of messages counter
                
                for _ in range(6):
                    led_toggle(state)  # Switch external led opposite state
                    time.sleep(0.2)
                if state.led_state == state.HIGH:
                    led_toggle(state)  # Switch the LEDs off
            else:
                wait_cnt += 1  # Not receiving data yet...increase wait_cnt
                if wait_cnt % 100 == 0 and state.my_debug:
                    print(TAG+"UART not receiving data (yet)", end='\n')
            # End if msg_data:
            msg_lp_cnt += 1
            if msg_lp_cnt >= max_inner_lp_cnt:
                break  # Leave the GPS senctence messages collection loop

        # End of the inner loop

        # time.sleep(0.1)  # <--------------- DELAY ---------------
        if state.my_debug:
            print(TAG+f"state.nr_of_msgs: {state.nr_of_msgs}")
        if state.nr_of_msgs > 0:
            if third_part: # <<<---------- HERE STARTS THE THIRD PART ----------<<<
                # +----------------------------------------+
                # | PRINT RECEIVED GPS DATA TO THE DISPLAY |
                # +----------------------------------------+
                pr_fs_data(state)
            else:
                print("\n\nThe third part ", end='\n')
                print(NoPrintMsg, end='\n')  # will not be shown here because of the status \"0\" of one of the flags (see global definitions)

        # End if (nr_of_msgs > 0)

        if state.lOnREPL:
            print("\nEnd of loop {}".format(outer_lp_cnt), end='\n')

        #t = 24 * " "
        #print("{}".format(t), end="\n")

        outer_lp_cnt += 1  # Increase the outer loop count

        if outer_lp_cnt < max_outer_lp_cnt:  # Are we done?
            if state.my_debug:
                print(TAG+"We go around, doing another loop")
            if state.startup == -1:
                state.startup = 0  # switch off flag. Showing this text only once.
        else:
            lstop = True  # force end of sketch execution

        # ------------------------------------------------------

        if lStop:
            print(TAG+"We go to stop", end='\n')
            break  # Yes, we're done!

    # End of the outer loop

    # Do cleanup

    # TODO

    # time.sleep(0.5)  # <--------------- DELAY ---------------

    return lStop

# End of setUp()

"""
    @ brief
    This functions receives a raw_key. It search for an '@' character. If found, it will be sliced off.
    The return value defaults to '':

    :param  k:  (which stands for 'key'):
    :type k: str:

    :returns: k(sliced k):
    :rtype: str
"""
def ck_key(state, k):
    TAG = tag_adj(state, 'ck_key(): ')
    n1 = n2 = 0
    t = type(k)
    retval = ''
    if state.my_debug:
        print(TAG+"type of parameter k is: \'{}\'".format(t), end='\n')
    if t is str:
        if state.my_debug:
            print(TAG+"parameter k value is: \'{}\'".format(k), end='\n')
        n1 = k.find("$")
        n2 = k.find("@")
        if n1 >= 0:
            k = k[n1+1:]  # Strip '$'
        elif n2 >= 0:
            k = k[n2+1:]  # Strip '@'
        elif k.find('02z') >= 0:
            k = 'AV400'
        elif k.find('21') >= 0:
            k = 'TEXT'
        elif k.find('RPY') >= 0:
            k = 'RPY'
        retval = k
    if state.my_debug:
        print(TAG+"return value is: \'{}\'".format(retval), end='\n')
    return retval

"""
    @brief
    This function creates a global dictionary with the name 'GPRMC_Dict'

    :param:  fname
    :type: str

    :returns: None
"""
def setup_GPS_main_dict(state):
    # Each entry of this dictionary contains the ID of the type
    # of the GPS sentence, the name of the type,
    # how many fields the type of GPS sentence has and a description.
    # The description here is empty. The descriptions are in the file GPSvariants.csv
    # and can be imported later.
    
    if state.gps_variant == "GPRMA":
        state.GPSvariantsDict = state.GPRMA_Dict
    elif state.gps_variant == "GPRMC":
        state.GPSvariantsDict = state.GPRMC_Dict

    if state.my_debug:
        print(f"setup_GPS_main_dict(): contents of state.GPSvariantDict:\n{state.GPSvariantsDict}", end='\n')

"""
    @brief
    This functions returns the value of GlobalGPSvariantDict['ID']
    After a first GPS variant message is received, this value will be set accordingly.
    So, this function shows the current GPS sentence variant being received.

    :param:  key     e.g. 'type' or 'fields'
    :type: key str

    :returns: a GPSvariantType (eventually an empty string)
    :rtype: str
"""
def get_GPSvariantValue(state, k):
    global GlobalGPSvariantDict
    TAG = tag_adj(state, 'get_GPSvariantValue(): ')
    retval = ''
    # When initiated the GlobalGPSvariantDict contains: {'ID': '$GPRMC', 'type': 'RMC', 'fields': 12, 'descr': ''}
    # When set (by ck_variant() at startup of this script) it should containt some ID like: '$GPRMA' (type 'str')
    if state.my_debug:
        print(TAG+f"state.gps_variant: {state.gps_variant}")
    """
    if state.gps_variant == "GPRMA":
        d = state.GPRMA_Dict
    elif state.gps_variant == "GPRMC":
        d = state.GPRMC_Dict
    """
    d = state.GPSvariantsDict  # set in setup_GPS_main_dict()
    if state.my_debug:
        print(TAG+f"value of parameter k is: \'{k}\'", end='\n')
        print(TAG+f"d: {d}")
    
    if k is not None and len(k) > 0:
        if k in d[state.gps_variant]:  # was: if k in state.GlobalGPSvariantDict.keys():
            retval = d[state.gps_variant][k]  # was: retval = state.GlobalGPSvariantDict[k]
            if state.my_debug:
                print(TAG+f"key \'{k}\' found in state.GPSvariantsDict[\'{state.gps_variant}\']: {d[state.gps_variant].keys()}. Value is: \'{retval}\'", end='\n')
        else:
            if state.my_debug:
                tmp = d[state.gps_variant]
                print(TAG+f"key \'{k}\' not found in state.GPSvariantsDict[{state.gps_variant}]: {tmp}", end='\n')
    if state.my_debug:
        print(TAG+f"return value is: \'{retval}\'", end='\n')
    return retval

"""
    @ brief
    This function checks if the ID of the message is the same as set globally
    Then it checks if the number of fields are as specified.

    param  bdata: string of bytes:
    type bdata: bytes:

    :returns: message has been positively validated or not:
    :rtype: boolean
"""
def ck_msg_validity(state, data):
    global msg_elements, lMsgOK
    TAG = tag_adj(state, 'ck_msg_validity(): ')
    retval = False
    k = ''
    if state.my_debug:
        print(TAG+f"param data: {data}")
    sGPSvarType = get_GPSvariantValue(state, 'type')
    nr_of_fields =  get_GPSvariantValue(state, 'fields')
    if state.my_debug:
        print(TAG+"sGPSvarType is: \'{}\', nr_of_fields is: \'{}\'".format(sGPSvarType, nr_of_fields), end='\n')
    if not sGPSvarType == '' and not nr_of_fields == 0:
        if isinstance(data, bytearray):
            sdata = data.decode(state.encoding)
        if isinstance(data, str):
            sdata = data
           
        sdata = stripCRLF(state, sdata)
        #elements = split_in_elements(bdata.decode(state.encoding), None)
        elements = split_in_elements(sdata, None)
        if state.my_debug:
            print(TAG+f"elements: {elements}")
        le_elements = len(elements)
        if le_elements == nr_of_fields:
            k = elements[0]
            k = ck_key(state, k)  # Check, slice or alter the key
            n1 = k.find(sGPSvarType)  # e.g.:  sGPSvarType = 'RMC' and k = 'GPRMC'
            if n1 >= 0:
                if state.my_debug:
                    print(TAG+"key extracted \'{}\', is found in sGPSvarType \'{}\'. That\'s good!".format(k, sGPSvarType), end='\n')
            else:
                if state.my_debug:
                    print(TAG+"key extracted from received message \'{}\', not found in sGPSvarType: \'{}\'".format(k, sGPSvarType), end='\n')
            # Find checksum
            n2 = elements[le_elements-1].find("*")
            if n2 >= 0:
                if state.my_debug:
                    print(TAG+f"\'*\' found in elements[{le_elements-1}]: \'{elements[le_elements-1]}\'")
            if n1 >= 0 and n2 >= 0:
                retval = True
                msg_elements = elements
                state.lMsgOK = retval
        if state.my_debug:
            print(TAG+"return value: \'{}\'".format(retval), end='\n')
    return retval

"""
    @ brief
    This function attempts to read a line of bytes from the uart.
    It then checks the type of incoming GPS sentence.
    Output to the REPL window will only occur if the flag 'state.lOnREPL' is True and when the data received is new
    or has changed since a previous reception.

    .. warning:: when the type of the uart object is NoneType, this function will raise a TypeError

    :param: None

    :returns: a msg as string of bytes
    :rtype: bytes

    Example:: result = b'$GPRMC,131756.99,a,3600.8292,n,00528.7885,W,133.3,215.6,100221,2.1,w*57\r\n'
"""
def ck_uart(state):  # returns an int
    TAG = tag_adj(state, 'ck_uart(): ')

    #srch_str = b'\x02z'  # Messages cyclus start bytes: bSTX followed by a 'z'
    
    # print(TAG+f"encoding: \'{state.encoding}\', state.GlobalGPSvariantID: \'{state.GlobalGPSvariantID}\'")

    # New method: we use the ID of the variant of the GPS message received (set in ck_variant())
    #bsrch_str = bytes(state.GlobalGPSvariantID, state.encoding)  
    bsrch_str = state.GlobalGPSvariantID
    if state.my_debug:
        print(TAG+f"bsrch_str: \'{bsrch_str}\', type(bsrch_str): {type(bsrch_str)}")
    le = 0

    #GPS_variantDict = {}

    u_data = None
    no_data_cnt = 0
    state.lMsgOK = False 
    lNoData = False
    GPS_Dict = {}

    if uart:
        while True:
            #u_data = uart.readline()  # The data received is in 'bytes'
            rx_buffer = bytearray(rx_buffer_len * b'\x00')
            nr_bytes = uart.readinto(rx_buffer)
            if rx_buffer:
                s_rx_buffer = rx_buffer.decode(state.encoding)
                le = len(s_rx_buffer)
                if state.my_debug:
                    print(TAG+f"state.o_data: {state.o_data}, rx_buffer: {rx_buffer}, type(rx_buffer): {type(rx_buffer)}, state.encoding: {state.encoding}")
                #print(TAG+f"rx_buffer.decode(state.encoding): {rx_buffer.decode(state.encoding)}")
                #return u_data
                if not state.o_data == rx_buffer:
                    state.o_data = rx_buffer
                    if state.lOnREPL:  # Print only if there is new  or changed data
                        if state.my_debug:
                            print(TAG+f"New or changed data rcvd, length: {le}, s_rx_buffer: \'{s_rx_buffer}\'", end ='\n')
                            # print(f"The value of state.lGlobalGPSvariantDictIsSet is: {state.lGlobalGPSvariantDictIsSet}", end='\n')
                
                n = s_rx_buffer.find("$"+state.gps_variant)
                if state.my_debug:
                    print(TAG+f"n: {n}")
                if n >= 0:
                    bsrch_str = s_rx_buffer[n:n+6] #bytes(state.GlobalGPSvariantID,state.encoding)  
                    # New method: we use the ID of the variant of the GPS message received (set in ck_variant())
                    if state.my_debug:
                        print(TAG+f"the value of bsrch_str is: {bsrch_str}", end='\n')
                        print(TAG+f"msg_nr is: {state.msg_nr}", end='\n')
                    # state.lMsgOK = True if '$'+state.gps_variant == bsrch_str else False
                    state.lMsgOK = ck_msg_validity(state, s_rx_buffer)
                    if not state.my_debug:
                        print(TAG+f"state.lMsgOK: {state.lMsgOK}")
                    if state.lMsgOK and not state.lGlobalGPSvariantDictIsSet:
                        # The state.GlobalGPSvariantDict has not yet been set. Try to set it now.
                        GPS_Dict = ck_variant(state, s_rx_buffer)  # Try to find the GPS variant of the message received
                        if state.my_debug:
                            print(TAG+f"GPS_Dict value is: {GPS_Dict}", end='\n')
                    
                    if state.lMsgOK:
                        if lNoData:
                            lNoData = False
                            pr_frame(state)  # re-build up the LCD frame
                        break  # We have data. It is not state.msg_nr 0 (base 0) and we've already received and identified the message ID
                    else:
                        continue  # Message not valid. Go around to get another message
                    #
                # End if le > 0:
            else:
                no_data_cnt += 1
                if no_data_cnt >= 10 and no_data_cnt % 10 == 0:
                    if not lNoData:
                        t = 10*'\r\n'
                        print(t)
                    lNoData = True

                    # Data became NoneType after I disconnected the serial data line !

                    print(TAG+"No data reception! Connection broken? FS2020 not running?", end='\n')
                    pr_to_tft(state, "No data reception!")
                    #print("No data reception!")

                    # raise ConnectionError

                if no_data_cnt == 100:
                    no_data_cnt = 0

                # End if u_data:
            # End if u_data...else...
        # End while loop

    else:
        print(TAG+"uart object is None", end='\n')
        raise TypeError
    if state.my_debug:
        print(TAG+"return value is: {}".format(s_rx_buffer), end='\n')
    return s_rx_buffer  # if no data return None

"""
    @ brief
    This function splits the received string parameter 's' into a list of strings.
    Optional 2nd parameter: a plit delimiter character. This parameter defaults to a comma character
    If parameter 's' is None or type is str and length is 0, this function will return an empty list.
    :param  s: str
    :type s: str

    :param s_schr: string containing a split delimiter character e.g.: ","
    :type s_schr: str:

    return: list of strings, e.g.: ['$GPRMC', '110517.00', 'A', '3609.0915', 'N', '00520.4464', 'W', '0.0', '259.4', '010221', '2.1', 'W*51\r\n']: rtype: list
"""
def split_in_elements(s, s_chr):
    retval = []
    delim = ','  # Default split delimiter character to comma
    lDefault = False

    if s is not None:  # Catch NoneType
        if isinstance(s, str): # Catch non str type
            if len(s) > 0:  # Catch empty str
                if not s_chr is None:  # Catch NoneType
                    if  isinstance(s_chr, str):  # Catch non str type
                        if len(s_chr) == 0:  # Catch zero length
                            lDefault = True
                    else:
                        lDefault = True
                else:
                    lDefault = True
                if lDefault:
                    s_chr = delim

                retval = s.split(s_chr)
    return retval

"""
    @ brief
    This function attempt to determine the variant type of GPS sentence of the message received.
    If the variant found, this function fills the GlobGPSvariantDict accordingly.
    If variant not found, the RMC variation will be set as default.

    :param  bData:  a string of bytes
    :type bData: bytes:

    return: GPS variant dict or None
    :rtype: dict
"""
def ck_variant(state, data):
    TAG = tag_adj(state, 'ck_variant(): ')
    sData = ''
    k = raw_key = tmp_ID = ''

    # tmp_type = ''

    s = 14*" "
    sGPSvarType = ''
    d = {}
    tmp_dict = {}
    retval = None  # Default return value
    elements = []
    lDoSetVariant = False
    l_key_found = False

    if isinstance(state.GlobalGPSvariantDict, dict):
        if len(state.GlobalGPSvariantDict) == 0:
            lDoSetVariant = True   # The dictionary is empty. Set the flag to fill it.
    else:
        lDoSetVariant = True  # The dictionary does not exist. Create it.

    if data is None:
        print(TAG+"received message is empty")
    else:
        # state.max_msg_len = 30
        le = len(data)
        if state.my_debug:
            print(TAG+f"length data: {le}")
        #if le > state.max_msg_len:
        #    #bData = bData[:state.max_msg_len]
        #    print(TAG+f"received message cut to: {bData}")
        #else:
        if state.my_debug:
            print(TAG+f"received message: {data}")
        
        try:
            if isinstance(data, str):
                sData = data
            elif isinstance(data, bytearray):
                sData = data.decode(state.encoding,'strict')  # Convert the bytes type data to string type
        except UnicodeError:
            # print(TAG+f"error at decoding bData[{_}] = {bData[_]}")
            #print(TAG+f"Error: {e}")
            raise
            
        if state.my_debug:
            print(TAG+f"Data (decoded) received in this function: {sData}", end='\n')
          
        n = len(sData)
        pos = sData.find("$"+state.gps_variant) # Find the ID for the type of GPS variant
        if state.my_debug:
            print(TAG+f"pos: {pos}")
        if pos >= 0:
            sData2 = sData[pos:]
            pos2 = sData2.find("\r\n")
            if state.my_debug:
                print(TAG+f"pos2: {pos2}")
            if pos2 >= 0:
                k = sData2[:pos2]
            else:
                return ""
        le = len(k)
        if le > 0:
            print(TAG+state.gps_variant+f" msg extracted: \'{k}\'")
        else:
            print(TAG+state.gps_variant+f" msg not found in: \'{sData}\'")
            return retval  # return empty dict

        elements = split_in_elements(k, None)  # 2nd param None to force to use default delimiter
        if state.my_debug:
            print(TAG+f"Data split in elements is: {elements}", end='\n')
        n = len(elements)
        if n == 0:
            if state.my_debug:
                print(TAG+"A split of the received data into fields reveiled nothing.", end='\n')
                return retval
        else:
            for _ in range(n):
                e = elements[_]
                pos = e.find("$"+state.gps_variant) # Find the ID for the type of GPS variant
                if pos >= 0:
                    k = elements[pos]
                    break
            if pos >= 0:
                raw_key = k  # was:  k = elements[0]
            else:
                raw_key = ""
                return retval

            sGPSvarType = get_GPSvariantValue(state, 'type')
            if len(sGPSvarType) == 0:  # the function get_GPSvariantValue could not find the 'type' key, maybe because the globalGPSvariantDict was not (yet) set.
                lDoSetVariant = True  # Set the flag to fill the globalGPSvariantDict
                if state.my_debug:
                    print(TAG+f"Value sGPSvarType is: \'{sGPSvarType}\'", end='\n')
            else:
                if state.my_debug:
                    print(TAG+f"value of the raw_key is: \'{raw_key}\', value sGPSvarType is: {sGPSvarType}", end='\n')

            #if not raw_key.find(sGPSvarType) > 0:  # check if sGPSvarType exists inside variable raw_key, e.g.: "$GPRMC".find("PRMC") > 0 ?
            #    # The extracted key is probably incomplete, so exit this function
            #    return None

            state.lGPSgroup = True  # if sGPSvarType in [ 'RMA', 'RMC', 'VTG', 'GGA', 'GLL', 'GSA', 'GSV' ] else False

            if state.my_debug:
                print(TAG+f"First element of received message: {k}", end='\n')

            k = ck_key(state, k)
            if state.my_debug:
                print(TAG+f"ck_key(state, k) returned: value of key k: \'{k}\'", end='\n')
            if len(k) == 0:
                return {}  # return empty dict
        """
        if state.gps_variant == "GPRMA":
            d = state.GPRMA_Dict
        elif state.gps_variant == "GPRMC":
            d = state.GPRMC_Dict
        else:
            d = {}
        """ 
        d = state.GPSvariantsDict

        if state.my_debug:
            print(TAG+f"msgType: {state.gps_variant}")
            print(TAG+state.gps_variant+f"_Dict: {d}")
        l_key_found = True if k in d else False

        if lDoSetVariant:
            # The state.GlobalGPSvariantDict has not yet been set. Try to set it now.

            if state.my_debug:
                print(TAG+f"Variant ID to look for in dictionary: \'{k}\'", end='\n')

            if l_key_found:
                if state.my_debug:
                    print(TAG+f"Variant \'{k}\' found in dictionary.", end='\n')
                state.GlobalGPSvariantDict = d.get(k)  # Set the global GPS variant dictionary
                state.GlobalGPSvariantID = raw_key
                state.lGlobalGPSvariantDictIsSet = True
                if state.my_debug:
                    print(TAG+"The global GPS variant dictionary was empty but now updated.", end='\n')

                    #print(TAG+"The value of the \'type\' field of received GPS variant is: {}".format(var_dict_type), end='\n')
        else:
            # The state.GlobalGPSvariantDict has been set.
            # Check if the type of the received data is the same as that present in
            # the state.GlobalGPSvariantDict when set containts e.g.: {'ID': '$GPRMA', 'type': 'RMA', 'fields': 12, 'descr': ''}
            if l_key_found:
                tmp_dict = d.get(k)  # Set the global GPS variant dictionary
                if state.my_debug:
                    print(TAG+f"tmp_dict: {tmp_dict}, k: {k}")
                    time.sleep(10)
                tmp_ID = tmp_dict['ID']

                # tmp_type = tmp_dict['type']

                if not raw_key == tmp_ID:
                    if state.my_debug:
                        print(TAG+f"Difference in GPS variant types:\n   The variant type received: {raw_key}.\n   The variant type set for: {tmp_ID}.", end='\n')
                        print("The global variant dictionary will be updated.")
                    state.GlobalGPSvariantDict = tmp_dict  # Set the Global GPS variant dictionary to the current variant being received
                    state.GlobalGPSvariantID = raw_key
                else:
                    if state.my_debug:
                        print(TAG+"the ID of the received message GPS variant", end ='\n')
                        print(f"{s}is equal to the one set in the global GPS variant dictionary", end='\n')
            else:
                print(TAG+f"key extracted from received message: \'{k}\' not found in {state.GPRMC}_Dict", end='\n')

    retval = state.GlobalGPSvariantDict

    return retval

"""
'FoamyGuy_CSV`_.
.. _FoamyGuy_CSV: https://github.com/FoamyGuy/CircuitPython_CSV_TileMap_Game/blob/master/code.py

    csv_read(csv_name) -> Dict(tmpDict)
    @ brief
    This functions opens and reads a csv file. Name chosen by parameter.
    Defaults to 'gprmc2.csv'

    Basic csv file: GPSvariants.csv which holds a dict structure for
    all 12 different GPS sentences variants.

    :param csv_name: (file name of a .csv type of file to be openend for reading)
    :type csv_name: str

    :returns: dictionary with keys and values of created from csv file
    :rtype: dict
"""
def csv_read(state, f_name):
    global csv_name
    TAG = tag_adj(state, 'csv_read(): ')
    TAG2 = ''
    org_my_debug = state.my_debug # remember
    
    state.my_debug = False
        
    tmpDict = {}  # Create an empty dictionary
    f = None
    line = ''
    t = []  # create an empty list to hold a list of tuples
    nl = nt = 0

    if f_name is None or len(f_name) == 0 or not f_name == 'gprma2.csv': # was: 'gprmc2.csv':
        f_name = 'gprma2.csv' # was: 'gprmc2.csv'  # default filename

    csv_name = f_name # Copy the filename to the global var

    # Open and read a list of tuples from the csv file

    try:
        f = open(csv_name, 'r')
        line = f.read()  # NOTE: Tuples is a string!!!
    except Exception as e:
        if e.args[0] == 2:  # Directory/File not found
            print(TAG+" Error: file: \"{}\" not found!".format(csv_name), end='\n')
        else:
            print(TAG+"Error: {}".format(e), end='\n')
        state.my_debug = org_my_debug
        return None
    finally:
        if not f is None:
            f.close()
    if state.my_debug:
        print(TAG+f"type(line): {type(line)}, line: {line}")
    if isinstance(line, str):
        nl = len(line)

        if nl > 0:
            if state.my_debug:
                print(TAG+f" this list of tuples: {line} is imported from file: \'{f_name}\'", end='\n\n')

            if csv_name == 'GPSvariants.csv':
                return {'ID': '$GPRMC', 'type': 'RMC', 'fields': 12, 'descr': ''}
            else:
                """ Handle GPRMC sentence variant """
                tmpDict = conv_it(state, line)  # create a dictionary containing the indexes and the values of the GPRMx message,
                # e.g.:  {0: 'id', 1: 'stat', 2: 'lat', 3: 'lat_ns', 4: 'lon', 5: 'lon_ew', 6: 'dummy', 7: 'dummy2', 8: 'gs', 9: 'crs', 10: 'var', 12: 'cksum'}
                TAG2 = '.GPRMxVariant: '
                k = v = ''
                t2 = ''
                nt = 0
                if isinstance(tmpDict, dict):
                    nt = len(t)  # Get the number of tuples in list t
                    if nt > 0:
                        if state.my_debug:
                            print(TAG+TAG2+f"length of list of tmDict is: {nt}", end='\n')
                            print(f"nr of keys in tmpDict is: {nt}", end='\n')
                            print("type of tmpDict is: {type(tmpDict)}\nnew contents of tmpDict is: {tmpDict}", end='\n\n')
                    else:
                        if state.my_debug:
                            print(TAG+f"length of tmpDict (returned from function conv_it(state, line) is: {nt}.\nCannot fill the appropriate GPS dictionary", end='\n')
        else:
            if state.my_debug:
                print(TAG+f"List of tuples extracted from file: {f_name} is empty.\nCannot fill the appropriate GPS dictionary.", end='\n')
    else:
        if state.my_debug:
            print(TAG+f"expected a type list but received a: {type(line)}")
    state.my_debug = org_my_debug

    return tmpDict
    
"""
    @ brief
    This functions receives a string
    bing a list of tuple. The function tries to find the , delimiters.
    When a tuple delimiter is found the function adds the index to
    the list occ_lst.

    :param  s
    :type s: str

    :returns: occ_lst  list of numeric indexes to delimiters found
    :rtype: list
"""    
def cnt_occurrances(state, s):
    TAG = tag_adj(state, "cnt_occurrances(): ")
    occ_lst = []
    last_occ = 0
    if isinstance(s, str):
        occ_cnt = 0
        start = 0
        end = 0
        occ = s.count("),(")
        if state.my_debug:
            print(TAG+f"nr of occurrances of \'),(\' in param s: {occ}")
        if occ > 0:
            try:
                if state.my_debug:
                    print(TAG+f"s: {s}")
                for _ in range(occ):
                    
                    n1 = s.find(")", start, len(s)-1)
                    if n1 >= 0:
                        if s[n1+1] == "," and s[n1+2] == "(":  # We found an "),(" occurrance
                            last_occ = n1+1
                            start = last_occ
                            occ_cnt += 1
                            occ_lst.append(last_occ)
                            #s = s[n1+2:]  # slice off the part checked
                    else:
                        break  # No more closing bracket
            except ValueError as e:
                print(TAG+f"Error: {e}")
            if state.my_debug:
                print(TAG+f"occurrance found and checked: {occ_cnt}")    
    return occ_lst

"""
    @ brief
    This functions receives a string
    from function conv_it() from which it will extract: key (k) and value (v)

    :param  state and s
    :type string

    :returns: ret 
    :rtype: list
"""
def conv_it_sub(state, s2):
    TAG = tag_adj(state, "conv_it_sub(): ")
    ret = []
    n = s2.find(",")
    if n >= 0:
        v = s2[1:n]    # the value
        k = s2[n+1:-1]  # the key
        if state.my_debug:
            print(TAG+f"k: \'{k}\', v: \'{v}\'")
        if isinstance(k, str) and len(k) > 0:
            nk = int(float(k))
        elif isinstance(k, int):
            nk = k
        ret.append(nk)
        ret.append(v)
        if state.my_debug:
            print(TAG+f"nk: {nk}")
    return ret
    
"""
    @ brief
    This functions receives a string
    from which it will create a list of tuples.

    :param  s
    :type s: str

    :returns: t  list of tuples t
    :rtype: list
"""
def conv_it(state, s):  # -> t   (list of tuples)
    TAG = tag_adj(state, 'conv_it(): ')
    le = len(s)
    tmp_dict = {}
    nk = 0
    v = None
    k = None
    start = 0
    end = 0
    # print("contents s2 is: {}".format(s2), end='\n\n')
    # print("type(tuples) is: {}, contents t is: {}".format(type(tuples), tuples), end='\n')
    lst1 = lst2 = []
    
    # see: https://stackoverflow.com/questions/7558908/unpacking-a-list-tuple-of-pairs-into-two-lists-tuples
    if isinstance(s, str):
        if state.my_debug:
            print(TAG+f"s: {s}")
            
        occ_lst = cnt_occurrances(state, s)
        old_idx = 0
        if state.my_debug:
            print(TAG+f"s: {s}")
            print(TAG+f"occ_lst: {occ_lst}")
        if isinstance(occ_lst, list):
            le = len(occ_lst)
            if le > 0:
                for _ in range(le):
                    idx = occ_lst[_]
                    if state.my_debug:
                        print(TAG+f"idx: {idx}")
                    if _ == 0:
                        start = _+1
                        end = idx
                    elif _ < le:
                        start = old_idx+1
                        end = idx
                    elif _ == le:
                        start = idx+1
                        end = len(s) -4
                        
                    s2 = s[start:end]
                    if state.my_debug:
                        print(TAG+f"_: {_}")
                        print(TAG+f"start: {start}, end: {end}")
                        print(TAG+f"substr to search: \'{s2}\'")
                    kv_lst = conv_it_sub(state, s2)
                    if isinstance(kv_lst, list) and len(kv_lst) > 0:
                        nk = kv_lst[0]
                        v = kv_lst[1]
                        if nk not in tmp_dict:
                            tmp_dict[nk] = v
                    old_idx = idx
                
                # now do the last part:
                idx = occ_lst[le-1]  # _ = 12
                start = idx+1
                end = len(s)-4  # -2 for ',)' and -2 for (hidden) '\r\n'
                s2 = s[start:end]
                if state.my_debug:
                    print(TAG+f"_: {_}")
                    print(TAG+f"start: {start}, end: {end}")
                    print(TAG+f"substr to search: \'{s2}\'")
                kv_lst = conv_it_sub(state, s2)
                if isinstance(kv_lst, list) and len(kv_lst) > 0:
                    nk = kv_lst[0]
                    v = kv_lst[1]
                    if nk not in tmp_dict:
                        tmp_dict[nk] = v
                
    if state.my_debug:
        print(TAG+f"return value: tmp_dict: {tmp_dict}")
    return tmp_dict

"""
    @ brief
    This function prints a text to the tft display in a graphical form
    :param  t: str
    :type t: str

    return: None
"""
def pr_to_tft(state, t):
    global label, splash,  main_group
    i = j = 0
    TAG = tag_adj(state, 'pr_to_tft(): ')

    if type(t) is not None:
        if state.my_debug:
            print(TAG+" received text: \'{}\' to print.".format(t), end='\n')

        le = len(t)
        txt_lbl_area = []

        # Make the display context
        if splash_grp is None:
            splash_grp = displayio.Group() # was: (max_size=10)

        # Draw a label
        txt_grp = []
        
        splash_grp.scale=2
        splash_grp.x=0
        splash_grp.y=0

        #for i in range(le):
        j = i*20 if i > 0 else 20
        if j > tft_height:
            j = tft_height-20  # tft_height = 240

        txt_grp.append(displayio.Group(scale=2, x=10, y=j))  # was: (max_size=5, ...)
        txt_lbl_area.append(label.Label(terminalio.FONT, text=t, color=0xFFFF00))
        txt_grp[i].append(txt_lbl_area[i])  # Subgroup for text scaling
        splash.append(txt_grp[i])
        main_group.append(splash_grp)
        display.root_group = splash_grp

        # time.sleep(0.5) # <----- DELAY -----

"""
    @brief
    This function writes a standard frame of text to the LCD for the RMC type of GPS sentences
    Wrting this frame first instead of every time data has to be written to the LCD, speeds up the process.
    It also makes viewing more calm.
    After text 'No data...' has been written to the LCD, this function will be called again to re-write this frame.

    :param:  None

    :returns: None
"""
def pr_frame(state):
    
    le = len(state.text1)

    txt_lbl = []

    # Make the display context
    #if splash is None:
    grp_x = 10
    
    frame_grp.scale=1
    frame_grp.x=0
    frame_grp.y=0

    # Draw a label
    txt_grp = []

    for i in range(le):
        txt_grp.append(displayio.Group(scale=2, x=grp_x, y=state.y_list[i]))  # was: (max_size=5, ...)
        txt_lbl.append(label.Label(terminalio.FONT, text=state.text1[i], color=0xFFFF00))
        txt_grp[i].append(txt_lbl[i])  # Subgroup for text scaling
        frame_grp.append(txt_grp[i])
    display.root_group = frame_grp
    
"""
    @ brief
    This functions prints text on the tft display

    :param:  None

    :returns: None
"""
def pr_title_scrn():
    
    title_grp.scale=3
    title_grp.x=17
    title_grp.y=120
    
    title_lbl = label.Label(terminalio.FONT, text="MSFS2020 data rx", color=0xFFFF00)
    title_grp.append(title_lbl)  # Subgroup for text scaling
    display.root_group = title_grp

"""
    @ brief
    This functions prints several lines of intro texts to the REPL window

    :param:  None

    :returns: None
"""
def pr_intro():
    global my_UART_TX_pin, my_UART_RX_pin

    print('', end='\n')

    #print("{}".format(111*" "), end='\n')

    print(f"MSFS2020 and FSUIPC7 (GPSout2) GPS data sentences sent from desktop PC Paul1 via {state.globCOMprt}", end='\n')
    print(f"to an Espressif ESP32-S2 Kaluga-1 microcontroller board, UART TXd ({my_UART_TX_pin}) - RXd ({my_UART_RX_pin}) data PINs.", end='\n')
    print("Starting to read the GPS data sentences:", end='\n\n')

"""
    @ brief
    This functions prints several lines of explanatory texts
    in respect to the AV400 GPS sentences contents to the REPL window

    :param:  None

    :returns: None

"""
def pr_heading(state):
    global nr_of_msgs

    # print("\nAV400 GPS sentence sequence{} follow below:".format('s' if nr_of_msgs > 1 else ''), end='\n') # using a ternary construction
    if state.my_debug:
        print("Sentence explanation:")
        print("STX...(= Start of Text code)")
        print("z... (followed by a 5-digit number CR LF")
        print("A... Latitude (e.g. N 41 1431 CR LF")
        print("B... Longitude (e.g. W 008 4049 CR LF")
        print("C... Track (mag) in degrees CR LF")
        print("D... ? CR LF")
        print("E... ? CR LF")
        print("GR.. ? CR LF Groundspeed(KTS)?")
        print("...evt some other lines like 'IN100C', '000W' or 'KL0000'")
        print("ETX..(= End of Text code -- Not alway seen in these sentences")
        print()
        

"""
    @brief
    This function prints the received RMA/RMC type of GPS variant sentences
    to the display. A frame is already shown on the display.
    This speeds up the presenting of the data. It makes viewing also more calm.
    Data will only be presented when new or updated. (The decision for this is
    made in the function ck_uart() ).
    This function is called from loop().
    If there is no data received, ck_uart() will write
    'No data reception! ...' to the display. When new data is received after a 'no reception'
    period, the fixed text frame will be written to the LCD first.

    :param:  None

    :returns: None
"""
def pr_fs_data(state):
    global main_group, fs_grp, msg_elements
    TAG = tag_adj(state, "pr_fs_data(): ")

    # +---------------------------------+
    # | GPS variantDict related defines |
    # +---------------------------------+
    # the GlobalGPSvariantDict when set containts e.g.: {'ID': '$GPRMC', 'type': 'RMC', 'fields': 12, 'descr': ''}

    sGPSvarType = get_GPSvariantValue(state, 'type')

    if state.my_debug:
        print(TAG+f"sGPSvarType is: \'{sGPSvarType}\'. It\'s type is: {type(sGPSvarType)}", end='\n')

    state.lGPSgroup = IsGPSmsg(state)
    nr_of_fields =  get_GPSvariantValue(state, 'fields')

    if state.my_debug:
        print(TAG+f"nr_of_fields is: \'{nr_of_fields}\'. It\'s type is: {type(nr_of_fields)}", end='\n')

    lGPSvarList = list(state.GlobalGPSvariantDict)  # Convert dict to a list

    # For GPRMC type of GPS message:
    # pos_utc = 1
    # pos_stat = 2
    # pos_date = 9
    # var = 10
    # var_dir = 11
    # mod_ind = 12
    # chksum = 13
    
    if state.my_debug:
        print(TAG+f"state.gps_variant: {state.gps_variant}")

    if state.gps_variant == "GPRMC":
        lat = 3
        lat_ns = 4
        lon = 5
        lon_ew = 6
        gs = 7
        crs = 8  # track_true
        sMemLon = sMemLat = sMemGs = sMemTt = ''
        nGs = nCrs = 0
    if state.gps_variant == "GPRMA":
        lat = 2
        lat_ns = 3
        lon = 4
        lon_ew = 5
        gs = 8
        crs = 9  # course over ground
        sMemLon = sMemLat = sMemGs = sMemTt = ''
        nGs = nCrs = 0

    #qth_long = qth_lat = False  # GPS sentence aircraft position (Lat, Long) flags

    if state.my_debug:
        print(TAG+f"lGPSvarType is: {sGPSvarType}", end='\n')
        print(TAG+f"lGPSvarList is: {lGPSvarList}", end='\n')
        print(TAG+f"Current GPS variant being received: {sGPSvarType}, belongs to GPRMAgroup?: {state.lGPSgroup}", end='\n')

        print("\n"+TAG+f"Nr of {sGPSvarType} GPS messages received: {nr_of_msgs}.\nOnly some of them will be shown on display.", end='\n')

    le_elements = len(msg_elements)
    if state.lMsgOK and le_elements > 0:
        if state.my_debug:
            # print(TAG+f"type(le_elements) is: {type(le_elements)}, type(nr_of_fields) is: {type(nr_of_fields)}", end='\n')
            print(TAG+f"msg_elements: {msg_elements}")

        # In GPRMA the element nrs are:
        #           2      4      8              9
        # text1 = ['LAT', 'LON', 'GS       KTS', 'CRS      DEGS']    (= global variable)
        le = len(state.text1)
        
        """
        lat_lbl.text = ""
        lon_lbl.text = ""
        gs_lbl.text = ""
        crs_lbl.text = ""
        """
        
        degree_sign = '.'  # u'\xb0'

        # Handle LAT
        if msg_elements[lat] != sMemLat:
            sMemLat = msg_elements[lat]
            s0 = " {}{}{}\'{}\" {}".format(
                msg_elements[lat][:2], \
                degree_sign, \
                msg_elements[lat][2:4], \
                msg_elements[lat][5:], \
                msg_elements[lat_ns])
            if state.my_debug:
                print(TAG+f"state.text1[{0}]: {state.text1[0]}, s0: {s0}")
            state.lat_lbl.text = state.text1[0]+" "+s0
            if not state.my_debug:
                print(TAG+f"state.lat_lbl.text: {state.lat_lbl.text}")

        # Handle LON
        if msg_elements[lon] != sMemLon:
            sMemLon = msg_elements[lon]
            s1 = "{}{}{}\'{}\" {}".format(
                msg_elements[lon][:3], \
                degree_sign, \
                msg_elements[lon][3:5], \
                msg_elements[lon][6:], \
                msg_elements[lon_ew])
            state.lon_lbl.text = state.text1[1]+" "+s1
            if state.my_debug:
                print(TAG+f"state.text1[{1}]: {state.text1[1]}, s1: {s1}")
            if not state.my_debug:
               print(TAG+f"state.lon_lbl.text: {state.lon_lbl.text}")

        # Handle GS
        if msg_elements[gs] != sMemGs:
            sMemGs = msg_elements[gs]
            nGs = int(float(msg_elements[gs]))
            s2 = "{:3d}".format(nGs)
            state.gs_lbl.text = state.text1[2][:2]+"  "+s2+" "+state.text1[2][-3:]
            if state.my_debug:
                print(TAG+f"state.text1[{2}]: {state.text1[2]}, s2: {s2}")
            if not state.my_debug:
                print(TAG+f"state.gs_lbl.text: {state.gs_lbl.text}")

        # Handle CRS
        if msg_elements[crs] != sMemTt:
            sMemTt = msg_elements[crs]
            nCrs = int(float(msg_elements[crs]))
            s3 = "{:3d}".format(nCrs)
            state.crs_lbl.text = state.text1[3][:3]+" "+s3+" "+state.text1[3][-4:]
            if state.my_debug:
                print(TAG+f"state.text1[{3}]: {state.text1[3]}, s3: {s3}")
            if not state.my_debug:
                print(TAG+f"state.crs_lbl.text: {state.crs_lbl.text}")
        print()
        
        if state.my_debug:
            print(TAG+f"fs_grp: {fs_grp}, type(fs_grp): {type(fs_grp)}")
        display.root_group = fs_grp

        time.sleep(3) # was: (0.5)

        #display.clear()
        #
    else:
        print(TAG+"Nr of msg_elements to print is {le_elements} so we exit this function.", end='\n')

"""
    @brief
    This functions toggles the builtin neopixel LED and and external connected 3-color LED

    :param:  None

    :returns: None
"""
def led_toggle(state):
    # uses state Class variable state.led_state

    if state.led_state == state.HIGH:
        state.led_state = state.LOW  # a=!a
    elif state.led_state == state.LOW:
        state.led_state = state.HIGH  # a=!a

    if state.led_state == state.LOW:
        pixels[0] = state.color_c_arr[state.led_colors_dict['off']]  # Set the color of the builtin LED to black
        pixels.show()
    else:
        pixels[0] = state.color_c_arr[state.led_colors_dict['green']]  # Set the color of the builtin LED to GREEN
        pixels.show()

"""
    @brief
    This function calls the function get_GPSvariantValue() with parameter 'type'.
    It sets the local variable 'sGPSvarType' to the return value of the called function.
    Then this function tests if the value of sGPSvarType exists in a list of types
    of the group GPRMA... GPS variant of sentences and returns the result as a boolean.

    :param:  None

    :returns:  True if sGPSvarType occurs in the given list of GPS sentence variation types
"""
def IsGPSmsg(state):
    TAG = tag_adj(state, "IsGPSmsg(): ")
    sGPSvarType = get_GPSvariantValue(state, 'type')
    ret = True if sGPSvarType in [ 'RMA', 'RMC', 'VTG', 'GGA', 'GLL', 'GSA', 'GSV' ] else False
    if state.my_debug:
        print(TAG+f"sGPSvarType: {sGPSvarType}")
        print(TAG+f"return: {ret}")
    return ret

"""
    @brief
    This function searches for the occurrance of:
    - a CRLF pair or chars.
    If found, it strips them off

    :param  msg:  (to be stripped)
    :type msg: str

    :returns: msg (stripped, if msg was not NoneType at entry)
    :rtype: str
"""
def stripCRLF(state, msg):

    s = ''
    eol_fnd = False
    retval = msg  # Default return: the value as it was received
    if isinstance(msg, str):
        if not msg is None:
            n = msg.find("$"+state.gps_variant)  # find e.g. '$GPRMA'
            if n >= 0:
                msg2 = msg[n:]
                n2 = msg2.find('\n')
                if n2 >= 0:
                    eol_fnd = True
                    s = msg2[:n2]  # slice-off the CR and LF characters
                    retval = s

    if state.my_debug:
        t = "True" if eol_fnd else "False"
        print(f"stripCRLF: LF found: {t}. Return value is: \'{s}\'", end='\n')

    return retval  # Return the message, stripped of unprintable characters

def show_terminal():
    display.root_group = None  # This will show the terminal 

# Start of setup

"""
    @brief
    This function is the main setup for this CircuitPython script

    :param: None

    :returns: void
"""
def setup(state):
    # global uart
    TAG = tag_adj(state, "setup(): ")
    
    read_fm_config(state)
    time.sleep(10)  # Wait to have view of mu-editor REPL
    
    setup_lbls(state)  # create the fs_grp and its text labels
    
    if state.use_wifi:
        wifi.AuthMode.WPA2   # set only once
    
        do_connect(state)
        
        if wifi_is_connected(state):
            print(TAG+f"WiFi connected to: {state.ssid}. IP: {state.s__ip}")
            
        time.sleep(10)

    state.o_data = b''  # this holds the previous rcvd msg. To compare if there is a change

    state.GlobalGPSvariantDict = {}
    state.GlobalGPSvariantID = ''
    state.lGlobalGPSvariantDictIsSet = False  # An important flag (!)

    # Contents of gprmc2.csv:
    # ('id',0),('utc',1),('stat',2),('lat',3),('lat_ns',4),('lon',5),('lon_ew',6), \
    # ('gs',7),('trktrue',8),('ddmmyy',9),('var',10),('var_ew_and_mode_ind_and_chksum',11)

    # state.my_gprmx: {"'dummy2'": ' 7)', "'cksum'": '12)', "'lat_ns'": '3)', "'lat'": '2)', "'var'": '10)', "'gs'": '8)', 
    # "'stat'": '1)', "'id'": '0)', "'dummy'": '6)', "'var_ew'": '11)', "'crs'": '9)'}
    state.my_gprmx  = csv_read(state,'gprma2.csv')
    state.GlobalGPSvariantDict = state.my_gprmx  # Copy the 
    if state.my_debug:
        print(TAG+f"state.my_gprmx: {state.my_gprmx}")

    setup_GPS_main_dict(state)
    # just a dummy dict to get output from function read_csv() (see below)

    dummyDict = csv_read(state,'GPSvariants.csv')  # Add the descriptions to the state.GPRMC_Dict, read from GPSvariants.csv

    # Now that work is done, cleanup dummyDict

    dummyDict = {}

    # +--------------------+
    # | Display setup      |
    # + -------------------+
    # palette = displayio.Palette(3)  # Not used in this moment

    bitmap[23,42] = 2  # This is how you would set the pixel at (x,y) = (23,42) to a value of 2

    # ====================================================================
    # === Pins MUST be initialized before SPI interface initialization ===
    # ====================================================================

    # ------------------------------------------------------------------------------+
    # | Perform display initialization sequence                                     |
    # | Sets orientation to landscape; clears the screen                            |
    # | * All pins must be configured                                               |
    # | * SPI interface must already be setup                                       |
    # | * tft_disp_type', 'COLOR_BITS', '_width', '_height' variables must be set   |
    # ------------------------------------------------------------------------------+

    print(tft_name1)
    print(tft_name2)

    time.sleep(5)  # <--------------- DELAY ---------------

    if uart:
        uart.reset_input_buffer()  # Clear the rx buffer

    time.sleep(1)  # <--------------- DELAY ---------------

    # uart.begin(4800) # Pins TX (Pin 17) and RX (Pin 16)
    # via a USB-to-TTL converter cable connected to desktop PC Paul1
    # Currently COM17. See global var: globCOMprt.
    # (COMport depends on MS Windows 10 system. Can change!)

    time.sleep(1)
    print()
    print("CircuitPython MSFS2020 GPS sentences data reception decoder by @paulsk (Discord.com Adafruit/CiruitPython). ", end='\n')

    #print("{}".format(dts))
    #print("\nNumber of loops in this run: {}".format(max_outer_lp_cnt))
    #print(dts)

"""
    @brief
    This is the start function

    :param: None

    :returns: None
"""
def main():
    TAG = tag_adj(state, "main(): ")
    loop_nr = 1
    max_loop_nr = 10
    wait = 2
    lStart = True

    time.sleep(5)  # wait for mu-editor is set for REPL output
    setup(state)  # Perform setup

    pr_title_scrn()
    time.sleep(wait)
    # pr_frame()
    # time.sleep(wait)
    while True:
        if lStart:
            print(TAG+"displayio test in Circuitpython V9.0.0-alpha.2")
            print(TAG+f"board: \'{state.board_id}\'")
            lStart = False
        lStop = loop(state)
        if lStop:
            print(TAG+f"loop() returned stop. Loop_nr: {loop_nr-1}. Exiting...", end='\n')
            break  # Go into the perpetual loop
        loop_nr += 1
        if loop_nr > max_loop_nr:
            loop_nr = 1

    # Perpetual loop
    loop_nr = 1
    show_terminal()
    time.sleep(wait)
    """
    while True:
        loop_nr += 1
        if loop_nr == 1000:
            loop_nr = 1
    """

# +------------------------------+
# | Call to the main() function  |
# +------------------------------+
if __name__ == '__main__':
    main()


# +----- END-OF-THIS-CP-SKETCH -----+
