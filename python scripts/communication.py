import serial
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class SerialCommunication:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        # Set to True when the OS/driver reports the device is gone (e.g. USB unplugged).
        # In that case we stop trying to reconnect automatically and let the GUI handle it.
        self.disconnected = False
        self.disconnected_reason = None
        self.connect()

    def connect(self):
        """Attempts to establish a serial connection."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Wait for Arduino reset
            self.disconnected = False
            self.disconnected_reason = None
            logging.info(f"Connected to {self.port} at {self.baudrate}")
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Failed to connect to {self.port}: {e}")
            self.ser = None

    def _mark_disconnected(self, error):
        """Marks the connection as disconnected and closes the serial port."""
        if self.disconnected:
            return
        self.disconnected = True
        self.disconnected_reason = str(error)
        try:
            self.close()
        except Exception:
            pass
        self.ser = None

    def _ensure_connection(self):
        """Checks if the connection is open.

        Note: If the device has been unplugged (disconnected=True), we do NOT auto-reconnect.
        The GUI should prompt the user to reconnect explicitly.
        """
        if self.disconnected:
            return False
        return self.ser is not None and self.ser.is_open

    def wait_for_ready(self, token, timeout=30):
        """Waits for a specific handshake token from the Arduino."""
        if not self._ensure_connection(): return False
        
        logging.info(f"Waiting for handshake: {token}...")
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line == token:
                        return True
            except (serial.SerialException, OSError, Exception):
                self._mark_disconnected("Handshake failed (device disconnected)")
            time.sleep(0.1)
        return False

    def send_command(self, command):
        """Sends a command string safely."""
        if not self._ensure_connection(): return False
        try:
            # Debug: show every outgoing command (target pressures: "<10.0,0.0,...>")
            #print(f"[TX] {command}", flush=True)
            #logging.info(f"TX: {command}")
            self.ser.write(command.encode('utf-8'))
            return True
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Write error: {e}")
            self._mark_disconnected(e)
            return False

    def read_response(self):
        """Reads a line from serial if available, handling errors."""
        if not self._ensure_connection(): return None
        try:
            if self.ser.in_waiting > 0:
                return self.ser.readline().decode('utf-8', errors='ignore').strip()
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Read error: {e}")
            self._mark_disconnected(e)
        return None

    def read_latest_response(self):
        """
        Reads ALL waiting lines in the buffer and returns only the most recent one.
        Use this for plotting to eliminate lag.
        """
        if not self._ensure_connection(): return None
        last_line = None
        try:
            # Check if there is data in the buffer
            if self.ser.in_waiting > 0:
                # Loop to drain the buffer (read everything available)
                # We limit to 100 iterations to prevent hanging if Arduino sends faster than we can read
                for _ in range(100): 
                    if self.ser.in_waiting == 0:
                        break
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        last_line = line
            return last_line
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Read error: {e}")
            self._mark_disconnected(e)
        return None

    def close(self):
        if self.ser and getattr(self.ser, 'is_open', False):
            self.ser.close()
            logging.info("Serial connection closed.")