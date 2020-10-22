import socket,sys
import time
import threading
# HOST='183.172.188.190'
PORT=31415
MAX_CLIENTS=2
class MyServer(threading.Thread):
    def __init__(self,ip,max_clients):
        threading.Thread.__init__(self)
        self.ip_address=ip
        self.max_clients=max_clients
        self.is_running=False
        self.is_waiting=True
    def run(self):
        self.server=socket.socket(socket.AF_INET,socket.SOCK_STREAM)#新建socket
        self.server.bind(self.ip_address)#绑定socket
        self.server.listen(2)#设置最大接入客户端数量
        # th_exit=threading.Thread(target=self.exitListening)
        # th_exit.start()

        print('Waiting:')
        self.client,client_ip=self.server.accept()#阻塞进程并等待客户端连接
        self.is_running=True
        print('Connected client address:',client_ip)

        th_listen=threading.Thread(target=self.serverListening)
        th_send=threading.Thread(target=self.serverSending)

        th_listen.setDaemon(True)
        th_send.setDaemon(True)

        th_listen.start()
        th_send.start()

        while self.is_running:
            time.sleep(1)

        print('Server Exit.')

    def serverListening(self):
        while True:
            try:
                msg=self.client.recv(1024)#阻塞进程并等待接收信息
                if not self.is_running:
                    break
                if msg.decode('utf-8')=='exit':
                    print('Client Disconnected!')
                    self.is_running=False
                    self.client.close()
                    exit(0)
                    return
                time.sleep(0.5)
                print('Client:',msg.decode('utf-8'))
                
            except Exception as e:
                self.is_running=False
                print('serverListening Error\n',e)
                return
            finally:
                time.sleep(0.5)

    def serverSending(self):
        while True:
            try:
                msg_send=input('')
                if not self.is_running:
                    break
                if not msg_send:
                    continue
                if msg_send=='exit':
                    self.is_running=False
                    self.client.close()
                    exit(0)
                    return
                self.client.send(msg_send.encode('utf-8'))
            except Exception as e:
                self.is_running=False
                print('serverSending Error\n',e)
                return
            finally:
                time.sleep(0.5)
        return
    
    def exitListening(self):
        while input() == '0':
            self.is_waiting=False


if __name__ == "__main__":
    hostname=socket.gethostname()
    #获取本机IP
    ip=socket.gethostbyname(hostname)
    myserver=MyServer((ip,PORT),MAX_CLIENTS)
    myserver.start()
    myserver.join()
    print('ServerThread exit.')