# main.py
# 主程式入口，負責整合各模組並執行馬達控制流程

from comm import send_modbus_command

def main():
    send_modbus_command()

if __name__ == "__main__":
    main()