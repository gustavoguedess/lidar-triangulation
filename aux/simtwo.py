from aux.client_udp import ClientUDP
import json

class SimTwo:
    def __init__(self):
        self.client = ClientUDP('127.0.0.1', 9810)

    def convert_msg(self, msg):
        _init, data, ground_truth, _end = msg.strip().split('\r\n')
        data = list(map(float, data.split(' ')))
        x, y, a = ground_truth.strip().split(' ')
        data = {
            'ground_truth': {
                'x': float(x),
                'y': float(y),
                'theta': float(a)
            },
            'raw_lidar': data
        }
        return data

    def get_lidar_data(self):
        msg = self.client.recv()
        data = self.convert_msg(msg)
        return data['lidar']

    def get_data(self):
        # print('Getting data')
        msg = self.client.recv()
        data = self.convert_msg(msg)
        return data

    def gen_data(self):
        data = []
        with open('lidar.txt', 'r') as f:
            for line in f.readlines():
                data.append(json.loads(line))

        for line in data:
            line = self.convert_msg(line)
            yield line

    def __del__(self):
        self.client.close()

if __name__ == '__main__':
    sim = SimTwo()
    while True:
        data = sim.get_data()
        print(data) 
    del sim