import smbus2
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

lcd_i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(lcd_i2c, 16, 2)
bus = smbus2.SMBus(1)

ARDUINO_ADDRESS = 0x04

def sendQuad(quadNum):
    quadNum = int(quadNum)
    if quadNum == 1 or 2 or 3 or 4: # marker is in the top left corner
        try:
            bus.write_byte_data(ARDUINO_ADDRESS, 0, quadNum)
        except:
            print("I2C connection failed, please check connection")
        lcd.color = [0, 100, 0]
        lcd.message = "Current Quad:\n" + str(quadNum)

    else:
        print("Quadrant number: " + str(quadNum) + " was not recognized\n")

while True:
    userInput = input("Enter a number: ")
    sendQuad(int(userInput))
