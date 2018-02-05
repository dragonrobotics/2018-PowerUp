import math

def print_response_data(data):
    for frame in range(0, len(data), 8):
        s = ''
        for b in data[frame:frame+8]:
            s += '{:02x} '.format(b)
        print('{:02x}: {}'.format(frame, s))

# Commands that can be sent to RPLIDARs.
class Command:
    Stop = 0x25
    Reset = 0x40
    Scan = 0x20
    ExpressScan = 0x82
    ForceScan = 0x21
    GetInfo = 0x50
    GetHealth = 0x52
    GetSampleRate = 0x59

# data types returned in data response descriptors
class ResponseType:
    ScanData = 0x81
    ExpressScanData = 0x82
    DeviceInfo = 0x04
    HealthInfo = 0x06
    SamplingRate = 0x15

# two-bit send modes returned in data response descriptors
class SendMode:
    SingleResponse = 0x00
    MultipleResponse = 0x01


class RPLidar:
    """
    Controls and reads data from an RPLIDAR laser rangefinder.

    Args:
        serial_type (str): Serial port API to use. Must be one of either
            ``pyserial`` or ``wpilib``.
        port (str): Serial port to use. If using ``pyserial``, this should be
            a serial port device name (i.e. ``/dev/ttyS1`` or ``COM0``).
            If using ``wpilib``, this should be one of
            :obj:`wpilib.SerialPort.Port`.
        read_timeout (float, optional): Maximum number of seconds to spend
            waiting for data from the RPLIDAR.
    """

    __exp_packet_buf = None  # buffered Express Scan packet.

    def __init__(
        self,
        serial_type='pyserial',
        port='/dev/ttyS1',
        read_timeout=0.010  # seconds to wait for I/O
    ):
        self.type = serial_type

        # pyserial.Serial and wpilib.SerialPort have identical signatures
        # for read() and write(). That's all we need.
        if serial_type == 'pyserial':
            import serial
            import time

            self.dev = serial.Serial(port, 115200, timeout=read_timeout)

            self.sleep = time.sleep
            self.reset_input_buffer = self.dev.reset_input_buffer
        elif serial_type == 'wpilib':
            import wpilib

            self.dev = wpilib.SerialPort(115200, port)
            self.dev.setWriteBufferMode(wpilib.SerialPort.WriteBufferMode.kFlushOnAccess)  # noqa: E501
            self.dev.setReadBufferSize(40000)
            self.dev.setTimeout(read_timeout)

            self.sleep = wpilib.Timer.delay
            self.reset_input_buffer = self.dev.reset

    def _bytes_available(self):
        if self.type == 'pyserial':
            return self.dev.in_waiting
        elif self.type == 'wpilib':
            return self.dev.getBytesReceived()

    def _read_response_descriptor(self):
        while True:
            checked_byte = self.dev.read(1)
            #print("Read {}...".format(checked_byte))
            if checked_byte == b'\xA5':
                break

        raw_desc = bytearray(b'\xA5')
        raw_desc.extend(self.dev.read(6))

        if len(raw_desc) < 6:
            raise IOError("RPLIDAR timed out while waiting for command response")  # noqa: E501

        if raw_desc[1] != 0x5A:
            raise IOError('RPLIDAR communication error (response descriptor header not found)')  # noqa: E501

        t = (
            raw_desc[2]
            | (raw_desc[3] << 8)
            | (raw_desc[4] << 16)
            | (raw_desc[5] << 24)
        )

        data_type = raw_desc[6]

        response_length = t & ~(3 << 30)
        send_mode = (t & (3 << 30)) >> 30

        print("Got response descriptor...")
        print("Len: {}, Mode: {}, Data Type: {:x}".format(
            response_length, send_mode, data_type
        ))

        return response_length, send_mode, data_type

    def _wait_response_descriptor(self, dtype):
        while True:
            dlen, mode, resp_dtype = self._read_response_descriptor()
            if dtype == resp_dtype:
                return dlen, mode, resp_dtype

    def stop_scan(self):
        """
        Command the RPLIDAR to stop scanning.
        """
        self.dev.write(bytes([0xA5, 0x25]))
        self.sleep(0.001)

    def reset(self):
        """
        Command the RPLIDAR to reboot itself.
        """

        self.dev.write(bytes([0xA5, 0x40]))
        self.sleep(0.002)

    def start_scan(self):
        """
        Command the RPLIDAR to begin standard scanning.

        To retrieve scan measurements, call :method:`poll_scan_samples`.

        Note:
            The RPLIDAR will only begin to return samples after the sensor's
            motor rotation has stabilized.
        """

        self.dev.write([0xA5, 0x20])
        self._wait_response_descriptor(ResponseType.ScanData)

    def poll_scan_samples(self):
        """
        Read all available scan samples from the RPLIDAR.

        Returns:
            A (possibly empty) list of (angle, distance, scan_start) tuples:

            - Angles are returned in units of degrees.
            - Distances are returned in units of millimeters.
            - ``scan_start`` will be True if the sample and all successive
              samples belong to a new 360-degree scan.
        """

        samples_available = math.floor(self._bytes_available() / 5)
        if samples_available <= 0:
            return []

        data = bytearray(self.dev.read(samples_available * 5))
        samples = []  # list of (angle, distance, scan_start) tuples

        for i in range(samples_available):
            packet = data[i*5 : (i+1)*5]

            # True if incoming packets are part of new scan
            scan_start = bool(packet[0] & 1)
            inv_scan_start = bool(packet[0] & 2)  # inverse of scan_start
            check_bit = bool(packet[1] & 1)  # should always be True

            # data check
            if (scan_start == inv_scan_start) or not check_bit:
                continue  # invalid packet

            # (logical) right shift for low bits, combine with high bits
            # Angles are returned as Q6 fixed-point numbers; divide everything
            # by 64.0 to account for this.
            angle_q6 = float(
                ((packet[1] >> 1) & 0x7F)
                | (packet[2] << 7)
            ) / 64.0

            distance_q2 = float(packet[3] | (packet[4] << 8)) / 4

            samples.append((angle_q6, distance_q2, scan_start))

        return samples

    def start_express_scan(self):
        """
        Command the RPLIDAR to begin express scanning.

        Express scanning uses a higher sampling rate than standard scanning,
        but uses a different and more processing-intensive format
        for transmitting data samples.

        To retrieve scan measurements, call
        :method:`poll_express_scan_samples`.
        """

        # Yes, the payload is 5 zero bytes. This is demanded by the spec.
        self.dev.write([0xA5, 0x82, 0x05, 0, 0, 0, 0, 0, 0x22])
        self._wait_response_descriptor(ResponseType.ExpressScanData)

    def poll_express_scan_samples(self):
        """
        Read all available express scan samples from the RPLIDAR.

        The format used by express scan packets represents angles in
        a compressed format. Unfortunately, the way this compression is
        done requires that packets be processed sequentially; in order to
        process packet *i*, packet *i+1* must first be read.

        Returns:
            A (possibly empty) list of (angle, distance, scan_start) tuples.
            The semantics of these quantities are the same as
            :method:`poll_scan_results`.
        """

        packets_available = math.floor(self._bytes_available() / 84)
        if packets_available <= 0:
            return []

        data = bytearray(self.dev.read(packets_available * 84))
        samples = []

        for i in range(packets_available):
            p_next = data[i*84 : (i+1)*84]  # packet

            sync1 = (p_next[0] >> 4) & 0xF
            sync2 = (p_next[1] >> 4) & 0xF

            # Received checksum
            chksum_rcv = (p_next[0] & 0xF) | ((p_next[1] & 0xF) << 4)

            # Computed checksum; XOR over all packet data bytes
            chksum_cmp = 0
            for i in range(2, 84):
                chksum_cmp ^= p_next[i]

            if sync1 != 0xA or sync2 != 0x5 or chksum_cmp != chksum_rcv:
                # This packet is invalid.
                # Neither this packet, nor the previous packet, can be
                # processed.
                self.__exp_packet_buf = None
                continue

            p = self.__exp_packet_buf
            self.__exp_packet_buf = p_next

            if p is None:
                # Wait for a packet that we can process
                continue

            # Angles are represented as Q6 fixed-point numbers, in degrees
            cur_angle = float(p[2] | ((p[3] & 0x7F) << 8)) / 64.0
            next_angle = float(p_next[2] | ((p_next[3] & 0x7F) << 8)) / 64.0

            angle_diff = next_angle - cur_angle
            if angle_diff < 0:
                angle_diff += 360

            scan_start = bool(p[3] & 0x80)

            # Each express scan packet contains 16 5-byte "cabin"
            # substructures, each of which contains 2 samples.
            for cabin_idx in range(16):
                c = p[4+(cabin_idx*5) : 9+(cabin_idx*5)]  # cabin data

                dist_1 = ((c[0] >> 1) & 0x7F) | (c[1] << 7)
                dist_2 = ((c[2] >> 1) & 0x7F) | (c[3] << 7)

                angle_comp_1 = float(c[4] & 0xF) / 8.0
                angle_comp_2 = float((c[4] >> 4) & 0xF) / 8.0

                # check sign bits
                if c[0] & 1:
                    angle_comp_1 *= -1

                if c[2] & 1:
                    angle_comp_2 *= -1

                angle_1 = cur_angle + (angle_diff / 32 * k) - angle_comp_1
                angle_2 = cur_angle + (angle_diff / 32 * k) - angle_comp_2

                if dist_1 != 0:
                    samples.append((angle_1, float(dist_1), scan_start))

                    # ensure only 1 sample in a scan has scan_start = True
                    scan_start = False

                if dist_2 != 0:
                    samples.append((angle_2, float(dist_2), scan_start))
                    scan_start = False

        return samples

    def force_start_scan(self):
        """
        Command the RPLIDAR to begin normal scanning and immediately return
        measurement samples.

        This method puts the RPLIDAR into the same scanning mode as
        :method:`start_scan`, but further commands the RPLIDAR to immediately
        begin returning measurement samples; in other words, the RPLIDAR will
        not wait for sensor rotation to stabilize before returning data.
        """

        self.dev.write([0xA5, 0x21])
        self._wait_response_descriptor(ResponseType.ScanData)

    def get_device_info(self):
        """
        Retrieve hardware information from the RPLIDAR.

        Returns:
            A tuple containing:

            - The RPLIDAR model ID, as an integer
            - The firmware version number, as a ``float``
            - The hardware version number, as an integer
            - The 128-bit serial number, as ``bytes``.
        """
        print("Sent Device Info command...")
        self.dev.write(bytes([0xA5, 0x50]))
        self._wait_response_descriptor(ResponseType.DeviceInfo)

        data = bytearray(self.dev.read(20))

        print("Device Info:")
        print_response_data(data)

        if len(data) < 20:
            raise IOError("RPLIDAR timed out while waiting for device info")

        model = data[0]
        fw = float(data[1] | (data[2] << 8)) / 256.0
        hw = data[3]
        serial_no = bytes(data[4:])

        return (model, fw, hw, serial_no)

    def get_device_health(self):
        """
        Retrieve the health status of the RPLIDAR.

        Returns:
            A tuple containing:

            - The RPLIDAR's status

                - 0 indicates a healthy device.
                - 1 indicates a warning status: the device works, but may fail
                  soon.
                - 2 indicates an error status.

            - An error code
        """

        self.dev.write(bytes([0xA5, 0x52]))
        self._wait_response_descriptor(ResponseType.HealthInfo)

        data = bytearray(self.dev.read(3))

        print("Device Health Info:")
        print_response_data(data)

        if len(data) < 3:
            raise IOError("RPLIDAR timed out while waiting for health info")

        error_code = data[1] | (data[2] << 8)

        return data[0], error_code

    def get_sample_period(self):
        """
        Get the time taken by the RPLIDAR between measurements.

        Returns:
            A tuple containing:

            - The time between standard sample measurements, in microseconds.
            - The time between express sample measurements, in microseconds.
        """

        self.dev.write([0xA5, 0x59])
        self._wait_response_descriptor(ResponseType.SampleRate)

        data = bytearray(self.dev.read(4))

        if len(data) < 20:
            raise IOError("RPLIDAR timed out while waiting for sampling period")  # noqa: E501

        t_standard = data[0] | (data[1] << 8)
        t_express = data[2] | (data[3] << 8)

        return t_standard, t_express

    def acc_board_set_pwm(self, pwm_level=660):
        """
        Set the PWM output via the Accessory Board.

        Args:
            pwm_level (integer): An integer from 0-1023 setting the RPLIDAR's
                motor power.
        """

        payload1 = pwm_level & 0xFF
        payload2 = (pwm_level >> 8) & 0xFF

        cmd_string = [0xA5, 0xF0, 2, payload1, payload2]
        checksum = 0
        for b in cmd_string:
            checksum ^= b
        cmd_string.append(checksum)

        print_response_data(cmd_string)

        self.dev.write(bytes(cmd_string))
        self.sleep(0.020)
