import socket
import threading
import json
import time


# =========================
# CONFIGURACION PRINCIPAL
# =========================

ENABLE_UDP_DISCOVERY = True

UDP_DISCOVERY_PORT = 7010
TCP_SERVICE_PORT = 7001

BODY_PART = "Pecho"
FIRE_INTERVAL_MS = 5000

TCP_HOST = "0.0.0.0"
UDP_HOST = "0.0.0.0"

SEND_TCP_ACK = True


# =========================
# UTILIDADES
# =========================

def get_lan_ip():
    """
    Intenta obtener la IP local real del computador dentro de la red.
    No necesita enviar datos reales a internet; solo fuerza al sistema a elegir interfaz.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        try:
            ip = socket.gethostbyname(socket.gethostname())
        except Exception:
            ip = "127.0.0.1"
    finally:
        s.close()

    return ip


def compact_json(data):
    return json.dumps(data, separators=(",", ":"), ensure_ascii=False)


# =========================
# UDP DISCOVERY MASTER
# =========================

def udp_discovery_server(master_ip):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_HOST, UDP_DISCOVERY_PORT))

    print(f"[UDP] Escuchando discovery en {UDP_HOST}:{UDP_DISCOVERY_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            text = data.decode("utf-8", errors="replace").strip()

            print(f"\n[UDP] Recibido desde {addr[0]}:{addr[1]}")
            print(f"[UDP] RAW: {text}")

            try:
                msg = json.loads(text)
            except json.JSONDecodeError as e:
                print(f"[UDP] JSON invalido: {e}")
                continue

            msg_type = msg.get("type", "")

            if msg_type != "NODE_HELLO":
                print(f"[UDP] Mensaje ignorado. type={msg_type}")
                continue

            udp_listen = int(msg.get("udp_listen", addr[1]))

            reply = {
                "type": "MASTER_ACK",
                "ip": "192.168.1.12",
                "tcp": 7001,
                "nodeName": "Pecho",
                "fireInterval": 5000
                }

            reply_text = compact_json(reply)
            sock.sendto(reply_text.encode("utf-8"), (addr[0], udp_listen))

            print(f"[UDP] MASTER_ACK enviado hacia {addr[0]}:{udp_listen}")
            print(f"[UDP] Reply: {reply_text}")

        except Exception as e:
            print(f"[UDP] Error: {e}")
            time.sleep(0.5)


# =========================
# TCP SERVICE MASTER
# =========================

def handle_tcp_client(conn, addr):
    print(f"\n[TCP] Cliente conectado desde {addr[0]}:{addr[1]}")

    buffer = ""

    try:
        with conn:
            conn.settimeout(0.5)

            while True:
                try:
                    data = conn.recv(1024)

                    if not data:
                        print(f"[TCP] Cliente desconectado: {addr[0]}:{addr[1]}")
                        break

                    chunk = data.decode("utf-8", errors="replace")
                    buffer += chunk

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if not line:
                            continue

                        print(f"\n[TCP] RX desde {addr[0]}:{addr[1]}")
                        print(f"[TCP] RAW: {line}")

                        try:
                            msg = json.loads(line)
                            print(f"[TCP] JSON: {json.dumps(msg, indent=2, ensure_ascii=False)}")

                            zona_hit = msg.get("zonaHit")
                            ok = msg.get("OK")

                            if zona_hit is not None:
                                print(f"[TCP] zonaHit = {zona_hit}")

                            if ok is not None:
                                print(f"[TCP] OK = {ok}")

                            if zona_hit == BODY_PART and ok is True:
                                print("[TCP] >>> EVENTO VALIDO RECIBIDO <<<")

                            if SEND_TCP_ACK:
                                ack = {
                                    "type": "TCP_ACK",
                                    "ok": True,
                                    "rxZonaHit": zona_hit
                                }

                                ack_text = compact_json(ack) + "\n"
                                conn.sendall(ack_text.encode("utf-8"))

                        except json.JSONDecodeError as e:
                            print(f"[TCP] JSON invalido: {e}")

                except socket.timeout:
                    continue

    except ConnectionResetError:
        print(f"[TCP] Conexion reiniciada por cliente: {addr[0]}:{addr[1]}")
    except Exception as e:
        print(f"[TCP] Error con cliente {addr[0]}:{addr[1]}: {e}")


def tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_SERVICE_PORT))
    server.listen(5)

    print(f"[TCP] Servicio escuchando en {TCP_HOST}:{TCP_SERVICE_PORT}")

    while True:
        conn, addr = server.accept()

        thread = threading.Thread(
            target=handle_tcp_client,
            args=(conn, addr),
            daemon=True
        )

        thread.start()


# =========================
# MAIN
# =========================

def main():
    master_ip = get_lan_ip()

    print("====================================")
    print(" TEST MASTER UDP + TCP PARA ESP32 C3")
    print("====================================")
    print(f"[INFO] IP local detectada: {master_ip}")
    print(f"[INFO] UDP discovery port: {UDP_DISCOVERY_PORT}")
    print(f"[INFO] TCP service port: {TCP_SERVICE_PORT}")
    print(f"[INFO] bodyPart enviado: {BODY_PART}")
    print(f"[INFO] fireInterval enviado: {FIRE_INTERVAL_MS}")
    print("====================================\n")

    if ENABLE_UDP_DISCOVERY:
        udp_thread = threading.Thread(
            target=udp_discovery_server,
            args=(master_ip,),
            daemon=True
        )
        udp_thread.start()

    tcp_server()


if __name__ == "__main__":
    main()