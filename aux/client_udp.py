import socket

class ClientUDP:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))

    def send(self, msg):
        self.sock.sendto(msg.encode(), (self.ip, self.port))

    def recv(self):
        return self.sock.recv(10240).decode()

    def close(self):
        self.sock.close()

if __name__ == '__main__':
    # connect to 127.0.0.1
    client = ClientUDP('127.0.0.1', 9810) 
    # client = ClientUDP('172.26.56.49', 9810)
    while True:
        print('msg: ', end='')
        msg = client.recv()
        _init, data, ground_truth, _end = msg.strip().split('\r\n')
        # data = list(map(float, data.split(' ')))
        # data = data.split(' ')
        gr = {}
        gr['x'],gr['y'], gr['a'] = ground_truth.strip().split(' ')

        print(f'Ground truth: x={gr["x"]}, y={gr["y"]}, a={gr["a"]}. Firsts: {data[:5]}')
        # print(f'data: {data}')
        

