# Satellite Tracking Software - This program obtains the latest TLE date (Needed to calculate satellite trajectories)
#                               and then predicts rise,set times and path data for a satellite.  It then awaits user
#                               instructions via a command line interface, before sending data to a satellite tracking
#                               Apparatus, allowing it to accurately track a satellite pass
# Written by Tony Carrick with contributions form Jakob Otten
# Latest revision/compile on 25/10/2017

#######################################################################################################################
#   IMPORT LIBRARIES
#######################################################################################################################
from math import degrees
import ephem
from datetime import datetime
import urllib2
import time
import serial
import sys

######################################################################################################################
#   CONSTANTS - Following Constants used throughout Program
######################################################################################################################

BROADCAST_ADDRESS = 0x000000000000FFFF
STU_ADDRESS = 0x000000000000FFFF
# Default xbee, USB address for the raspberry pi
XBEE_PATH = '/dev/ttyUSB0'


#####################################################################################################################
#   FUNCTIONS
#####################################################################################################################
#       "Display_Data" --> This function formats the calculated pass data of the satellite and prints it
#        formatted to the terminal window
def display_data(obj, NextPass):
    #
    RiseTime, RiseAzimuth, MaxAltitudeTime, Elevation, SetTime, SetAzimuth = NextPass #Setup array of data in "NextPass"
    if obj != 0:
        print(obj.name)
    print(round((RiseTime - CurrentTime) *3600*24), "Seconds until the next pass.")   #Calculate time to next pass
    VisibleTime = round((SetTime - RiseTime) *3600*24)
    print("Visible for:   ", VisibleTime, "seconds")    
    RiseTime = ephem.localtime(RiseTime).strftime("%x %X") #Format calculated dates/times into human readable strings
    TransitTime = ephem.localtime(MaxAltitudeTime).strftime("%x %X")
    SetTime = ephem.localtime(SetTime).strftime("%x %X")
    print("Rise Time:   ", RiseTime, "Azimuth:   %5.1f" %  degrees(RiseAzimuth))
    print("Transit Time:", TransitTime, "Elevation: %5.1f" % degrees(Elevation))
    print("Set Time:    ", SetTime, "Azimuth:   %5.1f" % degrees(SetAzimuth))
    print
    return RiseTime, VisibleTime #Return times for use in other areas of the program


def create_API_packet(frame_type, frame_id, address64, address16, options, data):
#       "Create_API_Packet" --> packages data into ZigBee API format, one byte at a time    
    packet = [0x7E]                     # StartFrame
    packet_length = len(data) + 0xE     # size of an empty packet is 0xE
    # breaks up the length into two, two byte pairs
    packet.append(packet_length >> 8)
    packet.append(packet_length & 0xFF)
    # frame type and frame ID
    packet.append(frame_type)
    packet.append(frame_id)
    # apply bitshifting and a mask to break 64bit address into individual two byte pairs
    for i in range(7, -1, -1):
        packet.append((address64 >> (i * 8)) & 0xFF)
    # 16 bit address conversion
    packet.append(address16 >> 8)
    packet.append(address16 & 0xFF)
    # broadcast radius and options
    packet.append(0x00)
    packet.append(options)
    # converts the characters one at a time into ASCII values then appends them.
    for character in data:
        packet.append(int(character.encode('hex'), 16))
    packet.append(0xFF - (sum(packet[3:]) & 0xFF))  # finally a checksum attached on the end.
    

    return packet

#==============================END OF FUNCTION DECLARATION=============================================#

########################################################################################################
#   GET DATA NEEDED FOR SAT CALCS - Retrieves TLE data from the internet for use in calculations
########################################################################################################
DataRequest = urllib2.Request("http://www.celestrak.com/NORAD/elements/noaa.txt")  #Get TLE data
RequestResponse = urllib2.urlopen(DataRequest)
Data = RequestResponse.read().splitlines()  #Parse the TLE contents into variable called data

# ensures that the xbee is initialised before continuing 
print "\nLooking for xBee..."
connected = False
if len(sys.argv) > 1:
    userInput = sys.argv[1]
while not connected:
    try:
        ser = serial.Serial(userInput,9600)
        print "xBee Found!\n"
        connected = True
    except:
        print "Looks like no xBee exists on that port.\n"
        userInput = raw_input("Please enter a new USB port address: ")


Objects = [] #Setup blank variable to store the satellites in
############################################################################################################################
#   EXTRACT DATA
############################################################################################################################
# Add all the TLE data into the objects list (reading TLE file line by line and attaching to it relevant satellite
for idx in range(0,len(Data),3):
    Objects.append(ephem.readtle(Data[idx],Data[idx+1],Data[idx+2]))

############################################################################################################################
#   SET HOME POSITION - Need to set ground position for predictions to be accurate
############################################################################################################################
Home = ephem.Observer()
Home.lon = 146.758323
Home.lat = -19.331480
Home.elevation = 21
Home.date = datetime.utcnow() 


SateliteList = []   #Setup blank Satellite List
###########################################################################################################################
#   POPULATE SAT LIST WITH LISTS CONTAINING NEXT PASS DATA - Self Explanatory
###########################################################################################################################
#Go through list of "Objects", calculating each next pass and details, before putting this data into "Satellites"
for obj in Objects:
    Home.date = datetime.utcnow()                           #Set current time before calculating next pass
    NextPass = Home.next_pass(obj)                          #Setup NextPass object, containing Pass Data
    SateliteList.append((NextPass[0],obj,NextPass))         #Add NextPassData to the List

SateliteList.sort()      #Order from closest pass to furtherest pass
for t, obj, NextPass in SateliteList:
    CurrentTime = ephem.now()
    RiseTime = display_data(obj, NextPass)

#######################################################################################################################
#   USER INTERFACE - Mostly string handling for terminal, and handling user input
#######################################################################################################################

print("Type the name of the satelite you would like to track EXACTLY as appears above or enter 'MANUALMODE'")

while True:
    UserSelection = raw_input("Enter Satellite Name to Track: ")        #Get user input about satellite to track
    # MANUAL MODE - This allows user to enter a path for the tracking unit to take manually
    if UserSelection == 'manual mode':
        print "Manual Mode enabled"
        Azimuth1 = int(raw_input("Enter the rise Azimuth in degrees: "))#Get data from user about the path they want tracking to take
        Elevation = int(raw_input("Enter Elevation in degrees: "))
        Azimuth2 = int(raw_input("Enter set Azimuth in degrees: "))
        Time = int(raw_input("Enter time to complete pass in seconds: "))
        data = '%i,%i,%i,%i' % (Azimuth1, Elevation, Azimuth2, Time)
        packet = create_API_packet(0x10, 01, BROADCAST_ADDRESS, 0xFFFE, 0x00, data)#Create Packet based on above data
        ser.write(bytearray(packet)) #Send Packet to tracking device
        print "Data sent"
        time.sleep(15)  #Give Tracker time to move to user entered position
        Command = raw_input("Enter 'GO' to commence tracking: ").lower()
        packet = create_API_packet(0x10, 01, BROADCAST_ADDRESS, 0xFFFE, 0x00, Command)
        ser.write(bytearray(packet))
        time.sleep(Time) #Timeout command prompt while the satellite tracking unit is moving
    #REFRESH - get latest next pass data/update user interface
    elif UserSelection == 'refresh':
        del(SateliteList)   #clear all stored pass data
        SateliteList = []
        for obj in Objects:
            NextPass = Home.next_pass(obj)
            SateliteList.append((NextPass[0],obj,NextPass)) #Add NextPassData to the List
        SateliteList.sort(reverse=True)         #Order from closest to pass to furtherest.
        for t, obj, NextPass in SateliteList:
            CurrentTime = ephem.now()
            RiseTime = display_data(obj,NextPass)
    #EXIT - Allow program to exit gracefully upon exit
    elif UserSelection == 'exit':
        break
    for t, obj, NextPass in SateliteList: #check to make sure user entered satellites exists in collected TLE File
        if UserSelection in (obj.name.lower()):
            print("Tracking Satellite", UserSelection)
            CurrentTime = ephem.now()
            RiseTime, RiseAzimuth, MaxAltitudeTime, Elevation, SetTime, SetAzimuth = NextPass
            RiseTime, TimeForMove = display_data(0,NextPass)

            try:
                #Extract the data needed for the satellite tracker to track accurately and send to the apparatus
                data = '%i,%i,%i,%i' % (round(degrees(RiseAzimuth)), round(degrees(Elevation)), round(degrees(SetAzimuth)), round(TimeForMove))
                packet = create_API_packet(0x10, 01, BROADCAST_ADDRESS, 0xFFFE, 0x00, data)
                ser.write(bytearray(packet))
                print data + " \nsent succussfully\n"
                print("Waiting for Satellite to appear. Will commence tracking on rise")
            except: #Error handling in case the Xbee is not connected
                print data + '\nCould not be sent, no xBee connected \n'
                CurrentTime = ephem.localtime(ephem.now()).strftime("%x %X")

            try:
                while RiseTime > CurrentTime: #while waiting for the satellite to rise, keep checking time, before sending the go command
                    CurrentTime = ephem.localtime(ephem.now()).strftime("%x %X")
                    time.sleep(1)
                    Flag = 1
            except KeyboardInterrupt:  #Allow user to CANCEL satellite tracking
                print "\nTracking aborted"
                packet = create_API_packet(0x10, 01, BROADCAST_ADDRESS, 0XFFEE, 0X00, 'CANCEL')
                ser.write(bytearray(packet))
                Flag = 0
                break

            if Flag == 1: #Upon Satellite rise, send command to apparatus, triggering tracking to begin
                print "Tracking Commenced"
                #Send GO command to Tracker
                packet = create_API_packet(0x10, 01, BROADCAST_ADDRESS, 0XFFEE, 0X00, 'GO')
                ser.write(bytearray(packet))
                Flag = 0
    if UserSelection not in obj.name.lower(): #Error Handling in case satellite user enters does not exist
        print("Enter Valid satelite to track or enter 'Manual Mode")
            

