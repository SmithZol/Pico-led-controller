import time
import json
import machine
from machine import Pin, PWM, UART
import ubinascii
import secrets
import gc


# =========================
# USER CONFIG
# =========================
DEVICE_ID = getattr(secrets, "DEVICE_ID", "livingroom")
BASE_TOPIC = "ledstrip/{}".format(DEVICE_ID)
TOPIC_SET = "{}/set".format(BASE_TOPIC)
TOPIC_STATE = "{}/state".format(BASE_TOPIC)
TOPIC_AVAIL = "{}/availability".format(BASE_TOPIC)
TOPIC_TEL = "{}/telemetry".format(BASE_TOPIC)
TELEMETRY_PERIOD_MS = 5000


# PWM pins
PIN_B = 20
PIN_G = 21
PIN_R = 22

PWM_FREQ = 3000
DEFAULT_TRANSITION_MS = 300
TICK_MS = 5
GAMMA = 2.0
INVERT_OUTPUT = False

# ESP8285 AT on UART0
ESP_UART_ID = 0
ESP_TX_PIN = 0
ESP_RX_PIN = 1
ESP_BAUD = 115200

# MQTT keepalive seconds
MQTT_KEEPALIVE_S = 30

# Debug logging
DEBUG_AT = True
DEBUG_MQTT = True
DEBUG_CMD = True


# =========================
# Helpers
# =========================
DUTY_MAX = 65535


def log_at(*args):
    if DEBUG_AT:
        print("[AT]", *args)


def log_mqtt(*args):
    if DEBUG_MQTT:
        print("[MQTT]", *args)


def log_cmd(*args):
    if DEBUG_CMD:
        print("[CMD]", *args)


def u8_to_u16(x_u8: int) -> int:
    # Map 0..255 to 0..65535 with full coverage
    return x_u8 * 257


def u16_to_u8(x_u16: int) -> int:
    return (x_u16 + 128) // 257


def clamp_int(x, lo, hi, default):
    try:
        v = int(x)
    except Exception:
        return default

    if v < lo:
        return lo
    if v > hi:
        return hi

    return v


def clamp_bool(x, default):
    return x if isinstance(x, bool) else default


def gamma_u16_to_duty(x_u16: int) -> int:
    # x_u16: 0..65535 linear
    # output: 0..65535 gamma-corrected duty
    x = x_u16 / 65535.0
    y = pow(x, GAMMA)
    duty = int(y * 65535 + 0.5)

    if duty < 0:
        duty = 0
    if duty > 65535:
        duty = 65535

    return duty


def scale_u16(a, b):
    # Scale two 16-bit values with proper 65535 full-scale behavior.
    # 65535 * 65535 -> 65535
    return (a * b + 32767) // 65535


def apply_pwm(pwm_r, pwm_g, pwm_b, r16, g16, b16, br16, power):
    if (not power) or br16 <= 0:
        r_d = 0
        g_d = 0
        b_d = 0
    else:
        # Brightness scale in linear 16-bit space.
        r_lin = scale_u16(r16, br16)
        g_lin = scale_u16(g16, br16)
        b_lin = scale_u16(b16, br16)

        r_d = gamma_u16_to_duty(r_lin)
        g_d = gamma_u16_to_duty(g_lin)
        b_d = gamma_u16_to_duty(b_lin)

    if INVERT_OUTPUT:
        r_d = 65535 - r_d
        g_d = 65535 - g_d
        b_d = 65535 - b_d

    pwm_r.duty_u16(r_d)
    pwm_g.duty_u16(g_d)
    pwm_b.duty_u16(b_d)


def unique_client_id():
    uid = ubinascii.hexlify(machine.unique_id()).decode()
    return "pico-{}-{}".format(DEVICE_ID, uid)


# =========================
# Fade/Retarget State
# =========================
# 255, 160,  60 - warm
state = {
    "power": True,
    "brightness": 240,
    "color": {
        "r": 255,
        "g": 160,
        "b": 60,
    },
}

trans = {
    "active": False,
    "t0_ms": 0,
    "dur_ms": 0,
    # 16-bit 0..65535 for smoother fades
    "start": {
        "r": 0,
        "g": 0,
        "b": 0,
        "br": 0,
    },
    "target": {
        "r": 0,
        "g": 0,
        "b": 0,
        "br": 0,
    },
    "current": {
        "r": 0,
        "g": 0,
        "b": 0,
        "br": 0,
    },
}


def start_transition(new_power, new_br, new_rgb, transition_ms, pwm_r, pwm_g, pwm_b):
    now = time.ticks_ms()

    # Interrupt and retarget: start from current output.
    trans["start"] = dict(trans["current"])
    trans["target"] = {
        "r": u8_to_u16(new_rgb["r"]),
        "g": u8_to_u16(new_rgb["g"]),
        "b": u8_to_u16(new_rgb["b"]),
        "br": u8_to_u16(new_br),
    }

    trans["t0_ms"] = now
    trans["dur_ms"] = max(0, int(transition_ms))
    trans["active"] = trans["dur_ms"] > 0

    state["power"] = bool(new_power)
    state["brightness"] = int(new_br)
    state["color"] = dict(new_rgb)

    if not trans["active"]:
        trans["current"] = dict(trans["target"])
        c = trans["current"]
        apply_pwm(
            pwm_r,
            pwm_g,
            pwm_b,
            c["r"],
            c["g"],
            c["b"],
            c["br"],
            state["power"],
        )


def tick_transition(pwm_r, pwm_g, pwm_b):
    if not trans["active"]:
        c = trans["current"]
        apply_pwm(
            pwm_r,
            pwm_g,
            pwm_b,
            c["r"],
            c["g"],
            c["b"],
            c["br"],
            state["power"],
        )
        return

    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, trans["t0_ms"])
    dur = trans["dur_ms"]

    if elapsed >= dur:
        trans["current"] = dict(trans["target"])
        trans["active"] = False
    else:
        t = elapsed / dur
        s = trans["start"]
        tg = trans["target"]
        trans["current"] = {
            "r": int(s["r"] + (tg["r"] - s["r"]) * t),
            "g": int(s["g"] + (tg["g"] - s["g"]) * t),
            "b": int(s["b"] + (tg["b"] - s["b"]) * t),
            "br": int(s["br"] + (tg["br"] - s["br"]) * t),
        }

    c = trans["current"]
    apply_pwm(
        pwm_r,
        pwm_g,
        pwm_b,
        c["r"],
        c["g"],
        c["b"],
        c["br"],
        state["power"],
    )


def merge_command(cmd):
    power = state["power"]
    br = state["brightness"]
    rgb = dict(state["color"])

    if "power" in cmd:
        power = clamp_bool(cmd["power"], power)

    if "brightness" in cmd:
        br = clamp_int(cmd["brightness"], 0, 255, br)

    if "color" in cmd and isinstance(cmd["color"], dict):
        c = cmd["color"]

        if "r" in c:
            rgb["r"] = clamp_int(c["r"], 0, 255, rgb["r"])
        if "g" in c:
            rgb["g"] = clamp_int(c["g"], 0, 255, rgb["g"])
        if "b" in c:
            rgb["b"] = clamp_int(c["b"], 0, 255, rgb["b"])

    if "r" in cmd:
        rgb["r"] = clamp_int(cmd["r"], 0, 255, rgb["r"])
    if "g" in cmd:
        rgb["g"] = clamp_int(cmd["g"], 0, 255, rgb["g"])
    if "b" in cmd:
        rgb["b"] = clamp_int(cmd["b"], 0, 255, rgb["b"])

    transition_ms = DEFAULT_TRANSITION_MS
    if "transition_ms" in cmd:
        transition_ms = clamp_int(cmd["transition_ms"], 0, 60000, DEFAULT_TRANSITION_MS)

    return power, br, rgb, transition_ms


def state_payload():
    return json.dumps({
        "power": state["power"],
        "brightness": state["brightness"],
        "color": state["color"],
    })


# =========================
# ESP8266 / ESP8285 AT Driver
# Active +IPD receive mode only
# =========================
class EspAT:
    def __init__(self, uart_id=0, tx=0, rx=1, baud=115200):
        self.u = UART(
            uart_id,
            baudrate=baud,
            tx=Pin(tx),
            rx=Pin(rx),
            timeout=30,
        )
        self._rx = b""
        self.closed = False

    def _read_uart(self):
        data = self.u.read()
        if data:
            self._rx += data

            if DEBUG_AT:
                # Avoid printing huge binary MQTT data constantly.
                if len(data) <= 96:
                    log_at("RX:", data)
                else:
                    log_at("RX:", len(data), "bytes")

            if b"CLOSED" in self._rx or b"CONNECT FAIL" in self._rx:
                self.closed = True

    def _take_line(self):
        i = self._rx.find(b"\r\n")
        if i < 0:
            return None

        line = self._rx[:i]
        self._rx = self._rx[i + 2:]
        return line

    def _discard_until_interesting(self):
        """
        Keep the receive buffer from growing forever with AT noise.

        We preserve data starting from +IPD if available.
        Otherwise we drop old command/notification noise when it is safe.
        """
        ipd = self._rx.find(b"+IPD,")
        if ipd > 0:
            dropped = self._rx[:ipd]
            self._rx = self._rx[ipd:]
            if DEBUG_AT and dropped.strip():
                log_at("DROPPED BEFORE IPD:", dropped)
            return

        # If no +IPD exists and the buffer is large, keep only the tail.
        if ipd < 0 and len(self._rx) > 512:
            if DEBUG_AT:
                log_at("DROPPING OLD RX NOISE:", len(self._rx), "bytes")
            self._rx = self._rx[-128:]

    def cmd(self, s, timeout_ms=2000):
        # Clear old non-IPD noise before command.
        self._discard_until_interesting()

        log_at("CMD:", s)
        self.u.write(s.encode() + b"\r\n")

        t0 = time.ticks_ms()
        lines = []

        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            self._read_uart()

            line = self._take_line()
            if line is None:
                time.sleep_ms(10)
                continue

            if line:
                lines.append(line)
                log_at("LINE:", line)

            if line in (b"OK", b"ERROR", b"FAIL"):
                break

        return lines

    def expect(self, needle, timeout_ms=2000):
        t0 = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            self._read_uart()

            idx = self._rx.find(needle)
            if idx >= 0:
                self._rx = self._rx[idx + len(needle):]
                return True

            time.sleep_ms(10)

        return False

    def wifi_join(self, ssid, pw):
        self.cmd("AT", 1000)
        self.cmd("AT", 1000)
        self.cmd("ATE0", 1000)
        self.cmd("AT+CWMODE=1", 1000)

        lines = self.cmd('AT+CWJAP="{}","{}"'.format(ssid, pw), 20000)
        log_at("CWJAP RESULT:", lines)

        self.cmd("AT+CIPMUX=0", 1000)
        self.cmd("AT+CIPDINFO=0", 1000)
        self.cmd("AT+CIPMODE=0", 1000)

        # Active receive mode: incoming data is delivered as unsolicited +IPD.
        # Do NOT use AT+CIPRECVDATA in this mode.
        self.cmd("AT+CIPRECVMODE=0", 1000)

    def tcp_connect(self, host, port):
        self.closed = False

        self.cmd("AT+CIPCLOSE", 1000)

        lines = self.cmd(
            'AT+CIPSTART="TCP","{}",{}'.format(host, port),
            8000,
        )
        log_at("CIPSTART:", lines)

        st = self.cmd("AT+CIPSTATUS", 2000)
        log_at("CIPSTATUS:", st)

        if any(x in (b"ERROR", b"FAIL") for x in lines):
            raise RuntimeError("CIPSTART failed")

    def tcp_send(self, payload_bytes):
        if self.closed:
            log_at("tcp_send refused: connection is marked closed")
            return False

        self.u.write(("AT+CIPSEND={}\r\n".format(len(payload_bytes))).encode())

        if not self.expect(b">", 3000):
            log_at("CIPSEND prompt not received")
            return False

        self.u.write(payload_bytes)

        t0 = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), t0) < 6000:
            self._read_uart()

            idx = self._rx.find(b"SEND OK")
            if idx >= 0:
                self._rx = self._rx[idx + len(b"SEND OK"):]
                return True

            if b"ERROR" in self._rx or b"FAIL" in self._rx:
                log_at("SEND failed:", self._rx)
                return False

            if b"CLOSED" in self._rx:
                self.closed = True
                log_at("SEND failed: CLOSED")
                return False

            time.sleep_ms(10)

        log_at("SEND timed out")
        return False

    def poll_ipd(self):
        """
        Poll one complete +IPD frame from the AT receive buffer.

        Returns:
            bytes payload if a complete frame is available
            None if no complete frame is available
        """
        self._read_uart()

        if b"CLOSED" in self._rx:
            self.closed = True

        start = self._rx.find(b"+IPD,")
        if start < 0:
            self._discard_until_interesting()
            return None

        if start > 0:
            dropped = self._rx[:start]
            self._rx = self._rx[start:]
            if DEBUG_AT and dropped.strip():
                log_at("DROPPED BEFORE IPD:", dropped)

        colon = self._rx.find(b":")
        if colon < 0:
            return None

        try:
            data_len = int(self._rx[5:colon])
        except Exception as e:
            log_at("Bad +IPD length, resync:", e, self._rx[:colon + 1])
            self._rx = self._rx[colon + 1:]
            return None

        after = colon + 1

        if len(self._rx) < after + data_len:
            return None

        payload = self._rx[after:after + data_len]
        self._rx = self._rx[after + data_len:]

        if DEBUG_AT:
            log_at("+IPD payload:", len(payload), "bytes")

        return payload


# =========================
# Minimal MQTT 3.1.1 QoS 0
# =========================
def _enc_varlen(n):
    out = bytearray()

    while True:
        digit = n % 128
        n //= 128

        if n > 0:
            digit |= 0x80

        out.append(digit)

        if n == 0:
            break

    return bytes(out)


def _enc_str(s):
    b = s.encode()
    return bytes([len(b) >> 8, len(b) & 0xFF]) + b


def mqtt_connect_pkt(client_id, user, password, keepalive_s):
    user = user or ""
    password = password or ""

    proto = _enc_str("MQTT") + bytes([4])

    # Clean Session
    flags = 0x02

    if user:
        flags |= 0x80
        if password:
            flags |= 0x40
    else:
        password = ""

    vh = proto + bytes([flags]) + bytes([keepalive_s >> 8, keepalive_s & 0xFF])

    pl = _enc_str(client_id)

    if user:
        pl += _enc_str(user)
        if password:
            pl += _enc_str(password)

    remaining = _enc_varlen(len(vh) + len(pl))

    return bytes([0x10]) + remaining + vh + pl


def mqtt_subscribe_pkt(packet_id, topic, qos=0):
    vh = bytes([packet_id >> 8, packet_id & 0xFF])
    pl = _enc_str(topic) + bytes([qos])
    remaining = _enc_varlen(len(vh) + len(pl))

    return bytes([0x82]) + remaining + vh + pl


def mqtt_publish_pkt(topic, payload_bytes, retain=False):
    # QoS 0 only
    hdr = 0x30 | (0x01 if retain else 0x00)
    vh = _enc_str(topic)
    pl = payload_bytes
    remaining = _enc_varlen(len(vh) + len(pl))

    return bytes([hdr]) + remaining + vh + pl


def mqtt_pingreq_pkt():
    return b"\xC0\x00"


def mqtt_parse_packets(buf):
    """
    Parse complete MQTT packets from buf.

    Returns:
        packets, remaining_buf

    packets:
        list of (ptype, flags, payload_bytes)
    """
    packets = []
    i = 0
    n = len(buf)

    while True:
        if i + 2 > n:
            break

        byte1 = buf[i]
        ptype = byte1 >> 4
        flags = byte1 & 0x0F

        # Remaining length varint
        mul = 1
        rem = 0
        j = i + 1

        while True:
            if j >= n:
                return packets, buf[i:]

            digit = buf[j]
            rem += (digit & 0x7F) * mul
            mul *= 128
            j += 1

            if (digit & 0x80) == 0:
                break

            if mul > 128 * 128 * 128 * 128:
                log_mqtt("Malformed remaining length; dropping buffer")
                return packets, b""

        header_len = j - i
        total = header_len + rem

        if i + total > n:
            break

        payload = buf[j:j + rem]
        packets.append((ptype, flags, payload))

        i += total

        if i >= n:
            break

    return packets, buf[i:]


def mqtt_decode_publish(payload, flags):
    """
    payload: variable header + payload
    flags: low nibble of fixed header

    Returns:
        topic, msg_bytes
        or None, None if unsupported/invalid
    """
    if len(payload) < 2:
        return None, None

    tlen = (payload[0] << 8) | payload[1]

    if len(payload) < 2 + tlen:
        return None, None

    try:
        topic = payload[2:2 + tlen].decode()
    except Exception:
        return None, None

    pos = 2 + tlen

    qos = (flags >> 1) & 0x03

    if qos != 0:
        log_mqtt("Unsupported incoming PUBLISH QoS:", qos)
        return None, None

    msg = payload[pos:]

    return topic, msg


def telemetry_payload(last_cmd_ms):
    c = trans["current"]
    t = trans["target"]

    return json.dumps({
        "uptime_ms": time.ticks_ms(),
        "heap_free": gc.mem_free(),
        "fade_active": bool(trans["active"]),
        "current": {
            "r": u16_to_u8(c["r"]),
            "g": u16_to_u8(c["g"]),
            "b": u16_to_u8(c["b"]),
            "brightness": u16_to_u8(c["br"]),
            "power": bool(state["power"]),
        },
        "target": {
            "r": u16_to_u8(t["r"]),
            "g": u16_to_u8(t["g"]),
            "b": u16_to_u8(t["b"]),
            "brightness": u16_to_u8(t["br"]),
        },
        "ms_since_cmd": time.ticks_diff(time.ticks_ms(), last_cmd_ms),
    })


def handle_mqtt_packet(esp, ptype, flags, payload, pwm_r, pwm_g, pwm_b, last_cmd_ms):
    """
    Handle packets in the main loop.

    Returns updated last_cmd_ms.
    """
    if DEBUG_MQTT:
        log_mqtt("packet type:", ptype, "flags:", flags, "payload_len:", len(payload))

    if ptype == 3:
        topic, msg = mqtt_decode_publish(payload, flags)

        if topic is None:
            log_mqtt("invalid/unsupported PUBLISH")
            return last_cmd_ms

        if DEBUG_MQTT:
            log_mqtt("PUBLISH topic:", topic, "payload:", msg)

        if topic == TOPIC_SET and msg is not None:
            try:
                cmd = json.loads(msg)

                if not isinstance(cmd, dict):
                    log_cmd("ignored non-dict JSON:", cmd)
                    return last_cmd_ms

                last_cmd_ms = time.ticks_ms()

                power, br, rgb, transition_ms = merge_command(cmd)

                log_cmd(
                    "apply",
                    "power:", power,
                    "brightness:", br,
                    "rgb:", rgb,
                    "transition_ms:", transition_ms,
                )

                start_transition(
                    power,
                    br,
                    rgb,
                    transition_ms,
                    pwm_r,
                    pwm_g,
                    pwm_b,
                )

                ok = esp.tcp_send(
                    mqtt_publish_pkt(
                        TOPIC_STATE,
                        state_payload().encode(),
                        retain=True,
                    )
                )

                if not ok:
                    log_mqtt("failed to publish state")

            except Exception as e:
                log_cmd("command error:", repr(e), "raw:", msg)

    elif ptype == 13:
        # PINGRESP
        log_mqtt("PINGRESP")

    elif ptype == 9:
        # SUBACK can also arrive later, although normally handled at startup.
        log_mqtt("SUBACK:", ubinascii.hexlify(payload))

    elif ptype == 2:
        # CONNACK should normally only happen during startup.
        log_mqtt("CONNACK:", ubinascii.hexlify(payload))

    else:
        log_mqtt("unhandled packet type:", ptype)

    return last_cmd_ms


def wait_for_mqtt_packet(esp, wanted_type, timeout_ms, rxbuf):
    """
    Wait for a specific MQTT packet type.

    Returns:
        packet_tuple, rxbuf

    packet_tuple:
        (ptype, flags, payload) or None
    """
    t0 = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        ipd = esp.poll_ipd()

        if ipd:
            rxbuf += ipd
            packets, rxbuf = mqtt_parse_packets(rxbuf)

            for ptype, flags, payload in packets:
                log_mqtt(
                    "startup packet type:",
                    ptype,
                    "flags:",
                    flags,
                    "payload:",
                    ubinascii.hexlify(payload),
                )

                if ptype == wanted_type:
                    return (ptype, flags, payload), rxbuf

        if esp.closed:
            raise RuntimeError("TCP connection closed while waiting for MQTT packet")

        time.sleep_ms(20)

    return None, rxbuf


def validate_connack(payload):
    if len(payload) < 2:
        raise RuntimeError("Invalid CONNACK length")

    session_present = payload[0]
    return_code = payload[1]

    log_mqtt("CONNACK session_present:", session_present, "return_code:", return_code)

    if return_code != 0:
        raise RuntimeError("MQTT rejected connection, return code {}".format(return_code))


def validate_suback(payload, packet_id):
    if len(payload) < 3:
        raise RuntimeError("Invalid SUBACK length")

    sub_pid = (payload[0] << 8) | payload[1]
    result = payload[2]

    log_mqtt("SUBACK packet_id:", sub_pid, "result:", result)

    if sub_pid != packet_id:
        raise RuntimeError(
            "SUBACK packet id mismatch: expected {}, got {}".format(packet_id, sub_pid)
        )

    if result == 0x80:
        raise RuntimeError("MQTT subscription rejected")

    if result > 2:
        raise RuntimeError("Unexpected SUBACK result {}".format(result))


# =========================
# Main
# =========================
def main():
    # PWM init
    pwm_r = PWM(Pin(PIN_R))
    pwm_r.freq(PWM_FREQ)

    pwm_g = PWM(Pin(PIN_G))
    pwm_g.freq(PWM_FREQ)

    pwm_b = PWM(Pin(PIN_B))
    pwm_b.freq(PWM_FREQ)

    # Init current and target outputs.
    initial_output = {
        "r": u8_to_u16(state["color"]["r"]),
        "g": u8_to_u16(state["color"]["g"]),
        "b": u8_to_u16(state["color"]["b"]),
        "br": u8_to_u16(state["brightness"]),
    }

    trans["current"] = dict(initial_output)
    trans["target"] = dict(initial_output)
    trans["start"] = dict(initial_output)

    c = trans["current"]
    apply_pwm(
        pwm_r,
        pwm_g,
        pwm_b,
        c["r"],
        c["g"],
        c["b"],
        c["br"],
        state["power"],
    )

    # ESP AT init + Wi-Fi + TCP
    esp = EspAT(ESP_UART_ID, ESP_TX_PIN, ESP_RX_PIN, ESP_BAUD)

    last_cmd_ms = time.ticks_ms()
    last_tel = time.ticks_ms()

    esp.wifi_join(secrets.WIFI_SSID, secrets.WIFI_PASSWORD)
    esp.tcp_connect(secrets.MQTT_HOST, getattr(secrets, "MQTT_PORT", 1883))

    # MQTT connect
    client_id = unique_client_id()
    user = getattr(secrets, "MQTT_USER", "")
    pw = getattr(secrets, "MQTT_PASS", "")

    if not user:
        raise RuntimeError("MQTT_USER is empty; check secrets.py variable name")

    log_mqtt("client_id:", client_id)
    log_mqtt("connecting to broker")

    if not esp.tcp_send(mqtt_connect_pkt(client_id, user, pw, MQTT_KEEPALIVE_S)):
        raise RuntimeError("MQTT CONNECT send failed")

    mqtt_rx = b""

    connack, mqtt_rx = wait_for_mqtt_packet(
        esp,
        wanted_type=2,
        timeout_ms=5000,
        rxbuf=mqtt_rx,
    )

    if connack is None:
        raise RuntimeError("MQTT CONNACK not received")

    validate_connack(connack[2])

    # Subscribe to set topic and wait for SUBACK.
    pid = 1

    log_mqtt("subscribing to:", TOPIC_SET)

    if not esp.tcp_send(mqtt_subscribe_pkt(pid, TOPIC_SET, qos=0)):
        raise RuntimeError("MQTT SUBSCRIBE send failed")

    suback, mqtt_rx = wait_for_mqtt_packet(
        esp,
        wanted_type=9,
        timeout_ms=5000,
        rxbuf=mqtt_rx,
    )

    if suback is None:
        raise RuntimeError("MQTT SUBACK not received")

    validate_suback(suback[2], pid)

    # Publish availability + initial state retained.
    if not esp.tcp_send(mqtt_publish_pkt(TOPIC_AVAIL, b"online", retain=True)):
        log_mqtt("failed to publish availability")

    if not esp.tcp_send(mqtt_publish_pkt(TOPIC_STATE, state_payload().encode(), retain=True)):
        log_mqtt("failed to publish initial state")

    log_mqtt("startup complete")

    # Main loop
    last_tick = time.ticks_ms()
    last_ping = time.ticks_ms()

    while True:
        if esp.closed:
            raise RuntimeError("TCP/MQTT connection closed")

        # Active receive mode only:
        # incoming TCP data arrives as unsolicited +IPD frames.
        ipd = esp.poll_ipd()

        if ipd:
            mqtt_rx += ipd
            packets, mqtt_rx = mqtt_parse_packets(mqtt_rx)

            for ptype, flags, payload in packets:
                last_cmd_ms = handle_mqtt_packet(
                    esp,
                    ptype,
                    flags,
                    payload,
                    pwm_r,
                    pwm_g,
                    pwm_b,
                    last_cmd_ms,
                )

        # Fade tick
        now = time.ticks_ms()

        if time.ticks_diff(now, last_tick) >= TICK_MS:
            last_tick = now
            tick_transition(pwm_r, pwm_g, pwm_b)

        # Telemetry heartbeat
        if time.ticks_diff(now, last_tel) >= TELEMETRY_PERIOD_MS:
            last_tel = now

            ok = esp.tcp_send(
                mqtt_publish_pkt(
                    TOPIC_TEL,
                    telemetry_payload(last_cmd_ms).encode(),
                    retain=False,
                )
            )

            if not ok:
                log_mqtt("failed to publish telemetry")

        # Keepalive ping
        if time.ticks_diff(now, last_ping) >= (MQTT_KEEPALIVE_S * 1000) // 2:
            last_ping = now

            ok = esp.tcp_send(mqtt_pingreq_pkt())

            if not ok:
                log_mqtt("failed to send PINGREQ")

        time.sleep_ms(2)


# Run
main()