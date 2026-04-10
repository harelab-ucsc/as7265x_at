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

Date:
    8 Apr 2026

Version:
    0.1.2
"""
# ROS2 node imports.
import rclpy
from rclpy.node import Node

# ROS2 messages.
from builtin_interfaces.msg import Time as BuiltinTime
from sensor_msgs.msg import Temperature
from std_msgs.msg import String, Header

# Module-level ROS2 messages.
from as7265x_at_msgs.msg import AS7265xRaw, AS7265xCal

# Standard imports.
import re
import serial
import threading
import time


# Global definitions.
# TODO: Make port selection intelligent.
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.1


class AS7265xStreamNode(Node):
    def __init__(
            self,
            gain: int = 16,
            time_int_ms: int = 166
        ):
        """
        Description:
            Calibration information:
                Each sensor element is pre-calibrated using diffused,
                incandescent light as specified in the AS7265x datasheet. The
                calibration settings are as follows:
                    * GAIN  = 16x       (0b10)
                    * INT_T = 166ms     (0x3B)
                    * VDD   = 3.3V
                    * T_AMB = 25ºC
                - Gain can be adjusted by setting the [5:4] bits of the
                    configuration register (0x04) with the following:
                        * 0b00  = 1x
                        * 0b01  = 3.7x  [default]
                        * 0b10  = 16x   [calibration]
                        * 0b11  = 64x
                - Integration time for all channels can be adjusted by setting
                    the full integration time register (0x05) with a conversion
                    of:
                        <integration_time> = <value+1> * 2.78ms
                    Calibration was done with integration time of 166ms = 0x3B. 
        """
        super().__init__('as7265x_stream')

        # ----------------------------
        # Parameters.
        # ----------------------------
        self.declare_parameter('serial_port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUD)
        self.declare_parameter('integration_time', time_int_ms)
        self.declare_parameter('gain', gain)
        self.declare_parameter('interval', 1)           # 1–255
        self.declare_parameter('calibrated', True)      # True -> ATCDATA mode
        self.declare_parameter('pps_topic', '/pps/time')
        self.pps_topic = self.get_parameter('pps_topic').value

        # ----------------------------
        # Serial connection settings.
        # ----------------------------
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)

        # ----------------------------
        # Publishers.
        # ----------------------------
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

        # ----------------------------
        # PPS subscriber
        # ----------------------------
        self.latest_pps_stamp = None
        self.pps_lock = threading.Lock()
        self.warned_no_pps = False

        self.create_subscription(
            BuiltinTime,
            self.pps_topic,
            self.pps_cb,
            10
        )

        # ----------------------------
        # Serial link.
        # ----------------------------
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

        self.get_logger().info(
            f"AS7265xStreamNode running. Using PPS topic: {self.pps_topic}"
        )

    def pps_cb(self, msg: BuiltinTime):
        """
        PPS callback.

        Args:
            msg (BuiltinTime)
        """
        with self.pps_lock:
            self.latest_pps_stamp = msg

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
        # TODO: Convert human-readable integration time to machine.
        it = int(self.get_parameter('integration_time').value)
        self.send(f"ATINTTIME={it}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

        # Gain.
        # TODO: Convert human-readable gain param to machine.
        g = int(self.get_parameter('gain').value)
        self.send(f"ATGAIN={g}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

        # Sampling interval multiplier.
        iv = int(self.get_parameter('interval').value)
        self.send(f"ATINTRVL={iv}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

        # Enable continuous burst mode.
        #   mode = 0 -> Raw data returned as int32s.
        #   mode = 1 -> Cal values returned as floats.
        mode = 1 if bool(self.get_parameter('calibrated').value) else 0
        self.send(f"ATBURST=255,{mode}")
        resp.append(self.ser.read(256).decode('utf-8', errors='replace'))

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

    def get_best_stamp(self) -> BuiltinTime:
        """
        Stamp helper.

        Returns:
            ts  (BuiltinTime)   PPS if available, else node time.
        """
        # Get the most recent and available PPS timestamp.
        with self.pps_lock:
            pps_stamp = self.latest_pps_stamp

        if pps_stamp is None:
            if not self.warned_no_pps:
                self.get_logger().warn(
                    f"No PPS timestamp received on {self.pps_topic} yet; "
                    f"using node clock time."
                )
                self.warned_no_pps = True
            return self.get_clock().now().to_msg()

        return pps_stamp

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

                    # Decode and process full line.
                    line = raw.decode('utf-8', errors='replace').strip()
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
        # Debug publisher for raw AT line.
        dbg = String()
        dbg.data = line
        self.pub_debug.publish(dbg)

        # Remove 'data' prefix before moving on.
        line = line.strip()
        if line.lower().startswith("data:"):
            line = line.split(":", 1)[1].strip()

        # Parse comma values: raw (ints) or calibrated (floats).
        if "," in line:
            parts = [p.strip() for p in line.split(",")]

            # Temperature format:
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
                # Set message header with timestamp.
                tmsg.header.stamp = self.get_best_stamp()
                tmsg.header.frame_id = "as7265x"
                self.pub_temp.publish(tmsg)
                return

            # TODO: Verify this.
            # Spectral payload  message accepts 12+ values (some firmwares
            #   output 14, others 18).
            elif len(parts) >= 12:
                calibrated = self.get_parameter("calibrated").value

                # --- Calibrated format: float[18]
                if calibrated:
                    pub = self.pub_cal
                    m = AS7265xCal()
                    try:
                        data = [float(p) for p in parts]
                    except ValueError:
                        return
                # --- Raw format: int32[18] 
                else:
                    pub = self.pub_raw
                    m = AS7265xRaw()
                    try:
                        data = [int(float(p)) for p in parts]
                    except ValueError:
                        return

                header = Header()
                header.stamp = self.get_best_stamp()
                header.frame_id = "as7265x"

                m.header = header
                m.values = data
                pub.publish(m)
                return

        if line == "OK":
            return

        # Anything else
        self.get_logger().info(f"Unexpected line received:\n    {line}")

    def destroy_node(self):
        """
        1. Deactivate sensor.
        2. Set thread-safe "stop event".
        3. Close serial connection.
        4. Delete this node.
        """
        try:
            self.send(f"ATBURST=0")
        except Exception:
            pass

        self.stop_evt.set()

        try:
            self.ser.close()
        except:
            pass

        super().destroy_node()


def main(args=None):
    """
    Launch node.

    Args:
        args.gain           (optional; int) Sensor gain; defaults to 16.
        args.time_int_ms    (optional; int) Integration time in ms; defaults
                                            to 166.
    """
    rclpy.init(args=args)

    # Arg parsing.
    try:
        gain = args.gain
    except:
        gain = 16
    try:
        time_int_ms = args.time_int_ms
    except:
        time_int_ms = 166

    # Spooling up node.
    node = AS7265xStreamNode(gain=gain, time_int_ms=time_int_ms)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Idiomatic destruction.
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
