import serial
import serial.tools.list_ports
import threading
import time
import pygame

class ReadThread(threading.Thread):
  def __init__(self, ser):
    threading.Thread.__init__(self, name="readthread")
    self.ser = ser

  def run(self):
    while(True):
      self.readSerial()
      time.sleep(.0001)

  def readSerial(self):
    read = self.ser.readline()
    if read:
      print(read.decode('utf-8', "ignore"))

class JoyThread(threading.Thread):
  def __init__(self, ser):
    threading.Thread.__init__(self, name="joythread")
    self.ser = ser
    pygame.init()
    self.stick = pygame.joystick.Joystick(0)
    self.stick.init()
    self.axes = [ 0.0 ] * self.stick.get_numaxes()
    self.buttons = [False] * self.stick.get_numbuttons()

    self.importantButtons = [4,5,6,7,10,11,12,13,14,15]
    self.importantAxes = [0, 1, 2, 3]

  def run(self):
    while True:
      #print(round(self.axes[15] * 100));
      for e in pygame.event.get():
        if e.type == pygame.QUIT:
          break
        elif e.type == pygame.JOYAXISMOTION:
          e = e.dict
          if e['axis'] in self.importantAxes and e['value'] != self.axes[e['axis']]:
            #self.ser.write(self.writeStr(0,10,axis=True))
            if e['axis'] == 3:
              e['value'] = e['value'] * -1
            self.ser.write(self.writeStr(e['axis'], round(e['value']*-100), axis=True))
            print(e['value'] * -100)
            #time.sleep(.01)
          self.axes[e['axis']] = e['value']
        elif e.type in [pygame.JOYBUTTONUP, pygame.JOYBUTTONDOWN]:
          e = e.dict
          self.buttons[e['button']] ^= True
          if e['button'] in self.importantButtons:
            towrite = self.writeStr(e['button'], self.buttons[e['button']], axis=False)
            self.ser.write(towrite)
            print(towrite)
      time.sleep(.0001)

  def writeStr(self,num, value, axis):
    if axis:
      prefix = "A" #A means axis
    else:
      prefix = "B" #B means button
      value = int(value)
    return bytes(prefix + str(num) + "%" + str(value), encoding='ascii')



def main():
  
  ports = list(serial.tools.list_ports.comports())

  arduino_port =''

  for p in ports:
    if p[1] == 'Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC ':
      arduino_port = p[0]
      print("Serial port found: ", arduino_port)
  if arduino_port == '':
    print("FTDI not found")
    exit()

  baud = 57600

  ser = serial.Serial(arduino_port, baud, timeout=10)
  tRead = ReadThread(ser)
  #tRead.daemon = True;
  tRead.start()
  tJoy = JoyThread(ser)
  #tJoy.daemon = True;
  tJoy.start()
  # while True:
  #   time.sleep(.0001)
    # string = input()
    # print("input: ", string)
    # lock.acquire()
    # ser.write(bytes(string, encoding='utf-8'))
    # print("input sent")
    # lock.release()
if __name__ == '__main__':
  main()