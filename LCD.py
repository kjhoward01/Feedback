#certifi==2024.8.30
"""charset-normalizer==3.4.0
idna==3.10
requests==2.32.3
RPLCD==1.3.1
urllib3==2.2.3
"""
from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
import serial
import time

# Configuration Constants
LCD_ADDRESS = 0x27    # I2C address of LCD
LCD_COLS = 16         # LCD columns
LCD_ROWS = 2          # LCD rows
LED_PIN = 18          # GPIO pin for LED
VOLTAGE_THRESHOLD = 25 # Voltage threshold for warning
SERIAL_PORT = '/dev/ttyACM0'  # Arduino serial port
BAUD_RATE = 9600      # Serial baud rate

class BatteryMonitor:
    def __init__(self):
        # Initialize LCD
        self.lcd = CharLCD('PCF8574', LCD_ADDRESS, cols=LCD_COLS, rows=LCD_ROWS)
        
        # Initialize GPIO for LED
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_PIN, GPIO.OUT)
        
        # Initialize Serial connection to Arduino
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for serial initialization
        
        # LED state
        self.led_state = False

    def clear_display(self):
        self.lcd.clear()
    
    def display_message(self, message, line=0):
        self.lcd.cursor_pos = (line, 0)
        self.lcd.write_string(message)

    def toggle_led(self):
        self.led_state = not self.led_state
        GPIO.output(LED_PIN, self.led_state)

    def read_voltage(self):
        try:
            line = self.ser.readline().decode('utf-8').rstrip()
            return float(line) if line else None
        except (ValueError, serial.SerialException):
            return None

    def run(self):
        try:
            while True:
                voltage = self.read_voltage()
                
                if voltage is not None:
                    self.clear_display()
                    
                    # Display voltage reading
                    self.display_message(f"Voltage: {voltage}V")
                    
                    # Check if voltage is below threshold
                    if voltage < VOLTAGE_THRESHOLD:
                        # Display warning
                        self.display_message("Battery Low!", line=1)
                        
                        # Toggle LED
                        self.toggle_led()
                    else:
                        # Turn off LED if voltage is normal
                        GPIO.output(LED_PIN, False)
                        self.led_state = False
                        
                        # Display normal status
                        self.display_message("Status: Normal", line=1)
                    
                    print(f"Current Voltage: {voltage}V")  # Console output
                
                time.sleep(0.5)  # Adjust flash rate here
                
        except KeyboardInterrupt:
            self.cleanup()
        except Exception as e:
            print(f"Error: {e}")
            self.cleanup()

    def cleanup(self):
        # Display shutdown message
        self.clear_display()
        self.display_message("Shutting down...")
        time.sleep(2)
        
        # Cleanup
        self.clear_display()
        self.lcd.close(clear=True)
        self.ser.close()
        GPIO.cleanup()

if __name__ == "__LCD__":
    monitor = BatteryMonitor()
    monitor.run()
