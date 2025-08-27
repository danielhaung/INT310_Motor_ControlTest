# main.py
# 主程式入口，負責整合各模組並執行馬達控制流程

from comm import send_command

def main():
    # 這裡直接發送指令，例如 "START"
    send_command("START")

if __name__ == "__main__":
    main()