#!/usr/bin/env python                                                          
# rasp_serial.py                                                              
#Andre Torres Dez2015                                                                                                                                         
import serial
from time import gmtime, strftime
import threading
import sys
import re
import MySQLdb


ficheiro=open("log2812.txt",'a')                                #Log file logDDMM.txt

ficheiro.write("\n---------------ARDUINO LOG------------\n")    #Write preamble to file
ficheiro.write(strftime("%a, %d %b %Y %X", gmtime()))           #with starting time
ficheiro.write("\n")                                    
ficheiro.close                                                  #close file

print("\n------------------- ARDUINO LOG ----------------\n")   #print the same in terminal
print(strftime("%a, %d %b %Y %X", gmtime()))
print("\n")

serial_port=serial.Serial('/dev/ttyACM0',9600)                  #open serial connection

class KeyEventThread(threading.Thread):                         #define separate thread to handle user inputs whilst scaning for arduino autputs in main thread
    def run(self):
        while True:
            c=raw_input('')                                     #get user input
            if c=='exit':                                       #exit command to close this thread
                ficheiro.close();                               #close file    
                sys.exit()                                      #kill thread
            else:
                serial_port.write(c)                            #send user input to arduino otherwise


keythread= KeyEventThread()                                     #initialise the thread
keythread.start()                                               #start the thread

while True:                                                     #begin infite loop
    if keythread.is_alive()==True:                              #test keyboard interrupt
        sline = serial_port.readline()                          #get line string
        ficheiro=open("log2812.txt",'a')                        #open file in append mode
        print sline                                             #prints the line to terminal
        ficheiro.write(sline)                                   #prints the line to file
        ficheiro.close()                                        #close file
        vector=[long (s) for s in re.findall('\\d+', sline)]    #split string and get numerals
        if len(vector)==6:                                      #Case: regular arduino output with 6 numerals
            cnx=MySQLdb.connect(user="root",db='rega_db')       #connect to db
            cursor=cnx.cursor()                                 #cursor to manipulate db
                                                                #write values to db
            cursor.execute('''INSERT INTO rega_table (time , temp_1, temp_2,temp_3,temp_4, cond_1, cond_2, cond_3, cond_4, light) VALUES (%s, %s,%s,%s,%s,%s,%s,%s,%s,%s)''',(strftime("%Y-%m-%d %H:%M:%S", gmtime()),vector[2],vector[4],0,0,vector[3],vector[5],0,0,vector[1]))
            cnx.commit()                                        #commit data to db
            cnx.close()                                         #close db
    else:                                                       #case: keyboard interrupt
        sys.exit()                                              #quit program
