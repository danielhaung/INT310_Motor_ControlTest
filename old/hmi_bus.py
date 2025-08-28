# hmi_bus.py
import json
import socket
import threading
import time
from typing import Dict, Any

# ===== 可調整：是否輸出除錯訊息 =====
DEBUG = True

def _log(msg: str):
    if DEBUG:
        print(f"[HMI-BUS] {msg}")

# ===== 連線管理 =====
_clients: set[socket.socket] = set()
_clients_lock = threading.Lock()

def _client_handler(conn: socket.socket, addr):
    _log(f"client connected: {addr}")
    conn.settimeout(5)
    with _clients_lock:
        _clients.add(conn)
    try:
        # 僅維持連線；不主動收，讓 server 保持活著
        while True:
            time.sleep(60)
    except Exception:
        pass
    finally:
        with _clients_lock:
            _clients.discard(conn)
        try:
            conn.close()
        except:
            pass
        _log(f"client disconnected: {addr}")

def _server_loop(host: str, port: int):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 重複啟動時可立即重綁
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.listen(8)
    _log(f"state server listening on {host}:{port}")
    while True:
        conn, addr = s.accept()
        th = threading.Thread(target=_client_handler, args=(conn, addr), daemon=True)
        th.start()

def init_state_bus(host: str = "127.0.0.1", port: int = 8765) -> threading.Thread:
    """
    啟動狀態廣播伺服器（non-blocking）。
    - 同機測試：host="127.0.0.1"
    - 跨機器：host="0.0.0.0" 並讓客戶端連伺服器IP
    """
    th = threading.Thread(target=_server_loop, args=(host, port), daemon=True)
    th.start()
    return th

def get_client_count() -> int:
    """回傳當前已連線的客戶端數量。"""
    with _clients_lock:
        return len(_clients)

def broadcast_state(obj: Dict[str, Any]) -> int:
    """
    將狀態以 NDJSON（每行一筆 JSON）廣播給所有已連線的 HMI 客戶端。
    回傳：本次成功送達的客戶端數。
    """
    # 確保結尾換行，便於客戶端逐行解析
    data = (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")

    dead = []
    sent = 0
    with _clients_lock:
        for c in list(_clients):
            try:
                c.sendall(data)
                sent += 1
            except Exception:
                dead.append(c)
        # 移除掛掉的連線
        for c in dead:
            _clients.discard(c)
            try:
                c.close()
            except:
                pass

    if DEBUG:
        _log(f"broadcast to {sent} client(s) | keys={list(obj.keys())}")
    return sent

# 方便單獨測試：python hmi_bus.py
if __name__ == "__main__":
    init_state_bus("127.0.0.1", 8765)
    i = 0
    while True:
        payload = {"hello": "world", "i": i, "ts": time.time()}
        n = broadcast_state(payload)
        _log(f"tick={i}, delivered={n}")
        i += 1
        time.sleep(1.0)
