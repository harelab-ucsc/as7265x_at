#!/usr/bin/env python3
"""
File:
    as7265x_at_node.py

Description:
    AS7265x continuous streaming ROS 2 node:
    *   Option A: Hardware-driven burst mode (ATBURST=255)
            Publishes incoming spectral data automatically.

    The AS7265x is really three sensors modules combined into one package. Data
    are stored in a set of 18 numbered registers whose contents are subject to
    change depending on BANK mode settings on the AS72651. Each module streams
    with 6 channels (18 channels total) with the following specs:

    AS72651:
        T   ->  730nm
        U   ->  760nm
        S   ->  680nm
        R   ->  610nm
        V   ->  810nm
        W   ->  860nm
    AS72652:
        G   ->  560nm
        H   ->  585nm
        I   ->  645nm
        J   ->  705nm
        K   ->  900nm
        L   ->  940nm
    AS72653:
        A   ->  410nm
        B   ->  435nm
        C   ->  460nm
        D   ->  485nm
        E   ->  510nm
        F   ->  535nm

Author:
    Slugraculture
    Morgan Masters
    nubby

Date:
    6 Apr 2026

Version:
    0.1.1
"""
import rclpy
import re
import serial
import threading
import time

from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String, Header

from as7265x_at_msgs.msg import AS7265xRaw, AS7265xCal


# TODO: Make port selection intelligent.
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.1


class AS7265xStreamNode(Node):
    def __init__(self):
        super().__init__('as7265x_stream')

        # Device configurations.
        self.declare_parameter('serial_port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUD)
        self.declare_parameter('integration_time', 20)  # ~2.8ms increments
        self.declare_parameter('gain', 1)               # 0–3
        self.declare_parameter('interval', 1)           # 1–255
        self.declare_parameter('calibrated', True)      # True -> ATCDATA mode

        # Serial connection settings.
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)

        # ROS publishers.
        self.pub_raw = self.create_publisher(
                AS7265xRaw, 
                'as7265x/raw_values',
                10
            )
        self.pub_cal = self.create_publisher(
                AS7265xCal, 
                'as7265x/calibrated_values',
                10
            )
        self.pub_temp = self.create_publisher(
                Temperature,
                'as7265x/temperature',
                10
            )
        self.pub_debug = self.create_publisher(String, 'as7265x/at_raw', 10)

        # Serial link.
        try:
            self.ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
            self.get_logger().info(f"Opened {port} @ {baud}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial: {e}")
            raise SystemExit

        # Stop flag for clean exit.
        self.stop_evt = threading.Event()

        # Configure device.
        self.configure_device()

        # Start background reader thread.
        self.reader_thread = threading.Thread(
                target=self.read_loop,
                daemon=True
            )
        self.reader_thread.start()

    def configure_device(self):
        """
        Send device configs to device and ensure proper responses.
        Configurations include:
            - Integration time  (ATINTTIME)
            - Gain              (ATGAIN)
            - Sampling interval (ATINTRVL)
            - Burst mode        (ATBURST)

        Todo:
            * Calibrate device using saved data.
        """
        resp = []

        # Integration time.
        it = int(self.get_parameter('integration_time').value)
        self.send(f"ATINTTIME={it}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

        # Gain.
        g = int(self.get_parameter('gain').value)
        self.send(f"ATGAIN={g}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

        # Sampling interval multiplier.
        iv = int(self.get_parameter('interval').value)
        self.send(f"ATINTRVL={iv}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

        # Enable continuous burst mode.
        #   mode = 0 -> Raw data returned as [].
        #   mode = 1 -> Cal values returned as [float32, float32, float32][6].
        mode = 1 if self.get_parameter('calibrated').value else 0
        self.send(f"ATBURST=255,{mode}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

        # self.get_logger().info(resp)
        # Confirm that all responses are 'OK'; otherwise, kill node.
        if all('OK\n' in r for r in resp):
            self.get_logger().info(
                    "AS7265x is now streaming continuously (burst mode 255)."
                )
        else:
            self.get_logger().info(
                    "AS7265x configuration failure. Killing node."
                )
            self.destroy_node()

    def send(self, cmd: str):
        """
        Send AT command.

        Args:
            cmd (str)   AT command string to be UTF-8 encoded and transmitted.
        """
        try:
            self.ser.write((cmd + "\r\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Write error: {e}")

    def read_loop(self):
        """
        Background serial reader that does the following:
        - Handles partial reads
        - Handles None returns from serial.read()
        - Safely buffers until newline
        - Supports CR, LF, CRLF, \r\r\n
        - Never crashes on invalid UTF-8
        
        It does so by appending to a buffer until a newline is reached, then
        passes that line to the `handle_line()` method for processing and
        reporting.
        """
        buf = bytearray()

        # Read until a stop event is detected.
        while not self.stop_evt.is_set():
            try:
                data = self.ser.read(256)

                # pyserial CAN return None on USB glitches.
                if data is None:
                    time.sleep(0.01)
                    continue

                # If no data, loop again.
                if len(data) == 0:
                    time.sleep(0.005)
                    continue

                # Append incoming bytes.
                buf.extend(data)

                # Process complete lines.
                while True:
                    nl = buf.find(b'\n')
                    if nl == -1:
                        break   # No complete line yet.

                    # Extract line (strip CR and whitespace).
                    raw = buf[:nl].rstrip(b'\r')
                    del buf[:nl+1]  # Remove line including newline.

                    if not raw:
                        continue

                    # Decode safely.
                    try:
                        line = raw.decode('utf-8', errors='replace').strip()
                    except Exception as e:
                        self.get_logger().warn(f"Decode error: {e}")
                        continue

                    if line:
                        self.handle_line(line)

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

    def handle_line(self, line: str):
        """
        Parse incoming burst lines picked up from the reader loop and sends them
        to their appropriate publishers.

        Args:
            line    (str)
        """
        # Debug publisher for all messages.
        dbg = String()
        dbg.data = line
        self.pub_debug.publish(dbg)

        # Parse comma values: raw (ints) or calibrated (floats).
        if "," in line:
            parts = [p.strip() for p in line.split(",")]

            # --- Temperature format:
            #   [Header header,
            #    double temperature,
            #    double variance]
            if len(parts) == 3 and all(
                    re.match(r'^-?\d+(\.\d+)?$', p) for p in parts
                ):
                temps = [float(x) for x in parts]
                tmsg = Temperature()
                tmsg.temperature = sum(temps) / len(temps)
                tmsg.variance = 0.0
                self.pub_temp.publish(tmsg)
                return

            elif len(parts) >= 18:
                calibrated = self.get_parameter('calibrated').value

                # --- Calibrated format: float[18]
                if calibrated:
                    pub = self.pub_cal
                    m = AS7265xCal()
                    data = [float(p) for p in parts]
                # --- Raw format: int32[18] 
                else:
                    pub = self.pub_raw
                    m = AS7265xRaw()
                    data = [int(p) for p in parts]

                try:
                    header = Header()
                    header.stamp = self.get_clock().now().to_msg()
                    header.frame_id = "as7265x"

                    m.header = header
                    m.values = data
                    pub.publish(m)
                    return
                except Exception as e:
                    self.get_logger().info(e)
                    pass

        elif line == "OK":
            # self.get_logger().info(line)
            return

        else:
            self.get_logger().info(f'Unexpected line recieved: \n    {line}')
            return

    def destroy_node(self):
        """
        1. Deactivate sensor.
        2. Set thread-safe "stop event".
        3. Close serial connection.
        4. Delete this node.
        """
        self.send(f"ATBURST=0")
        self.stop_evt.set()
        try:
            self.ser.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AS7265xStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
