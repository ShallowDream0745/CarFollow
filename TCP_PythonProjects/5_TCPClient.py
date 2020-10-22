import socket
import threading
import time
is_running=False
def clientSending():
    global is_running
    while True:
        try:
            msg=input('')
            if not is_running:
                break
            if not msg:
                continue
            client.send(msg.encode('utf-8'))
            if msg=='exit':
                is_running=False
                client.close()
                return
        except Exception as e:
            is_running=False
            print('clientSending Error\n',e)
            return
        finally:
            time.sleep(0.5)
    return

def clientListening():
    global is_running
    try:
        while is_running:
            msg=client.recv(1024)#阻塞进程并等待接收信息
            if not is_running:
                break
            if msg.decode('utf-8')=='exit':
                print('Server Disconnected!')
                is_running=False
                client.close()
                return
            time.sleep(0.5)
            print('Server:',msg.decode('utf-8'))
    except Exception as e:
        is_running=False
        print('clientListening Error\n',e)
        return
    finally:
        time.sleep(0.5)

if __name__ == "__main__":
    # hostname=socket.gethostname()
    # ip=socket.gethostbyname(hostname)
    client=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    client.connect(('106.39.42.226',31415))
    is_running=True
    print('聊天开始，输入exit可随时退出：')
    th_send=threading.Thread(target=clientSending)
    th_listen=threading.Thread(target=clientListening)

    th_send.setDaemon(True)
    th_listen.setDaemon(True)

    th_send.start()
    th_listen.start()

    while is_running:
        time.sleep(1)
    print('ClientThread exit.')