import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import datetime
import time
from math import pi as pi
import os
import re

class ScriptA(Node):

    def __init__(self):
        super().__init__('scriptA', allow_undeclared_parameters=True,)
        self.waiting_grids = {}
        self.charging_grids = {}
        self.system_out_grids = {}
        self.dumping_grids = {}
        self.height = 0
        self.width = 0
     
    def printVars(self):
        print('waiting_grids: ',self.waiting_grids)
        print('charging_grids: ',self.charging_grids)
        print('system_out_grids: ',self.system_out_grids)
        print('dumping_grids: ',self.dumping_grids)
        print('height: ',self.height)
        print('width: ',self.width)
     
    def getParameter(self, configName, attribute, isPrefix=False):
        '''
        return dictionary containing key value pairs of the same prefix
        returns value of the asked parameter
        '''
        if isPrefix:
            if attribute==None:
                res = self.get_parameters_by_prefix(configName)
            else:
                res = self.get_parameters_by_prefix(configName+'.'+attribute)
            if res==None:
                raise Exception('The prefix '+str(configName+'.'+attribute)+' does not exist ')
        else:
            res = self.get_parameter(configName+'.'+attribute).value
            if res==None:
                raise Exception('The attribute '+str(attribute)+' does not exist for config '+ str(configName))
        return res

    def detect_angle(self, direction):
        try:
            # 1 indicates up
            if 1 in eval(direction):
                return -pi/2
            # 2 indicates down
            if 2 in eval(direction):
                return pi/2
            # 3 indicates right
            if 3 in eval(direction):
                return pi
            # 4 indicates left grid is neighbour
            if 4 in eval(direction):
                return 0

        except NameError or SyntaxError:
            print("Directions are not in specified format : '(1,2,3,4)' \
                for given coord : " )
        return 0
    
    def get_queue_entries(self, queue_no):
        entries = self.getParameter(configName='queue_data', attribute='queue_'+str(queue_no)+'.entries')
        print(entries)
        pos_entries = {}
        for entry in entries:
            x = self.getParameter(configName='queue_data', attribute='queue_'+str(queue_no)+'.'+str(entry)+'.X')
            y = self.getParameter(configName='queue_data', attribute='queue_'+str(queue_no)+'.'+str(entry)+'.Y')
            theta = self.getParameter(configName='queue_data', attribute='queue_'+str(queue_no)+'.'+str(entry)+'.theta')
            pos_entries[entry] = (x, y, theta)
        return pos_entries

    def get_queue_entry(self, queue_no):
        queue_entry = self.getParameter(configName='queue_data', attribute='queue_'+str(queue_no)+'.grid_1', isPrefix=True)
        x = queue_entry["X"].value
        y = queue_entry["Y"].value
        theta = queue_entry["theta"].value
        return (x, y, theta)
    
    def initiatize(self):
        self.height = self.get_parameter("map_synthesis.height").value
        print(rclpy.Parameter("map_synthesis"))
        self.width = self.get_parameter("map_synthesis.width").value
        try:
            for j in range(1, self.height+1):
                    for i in range(1, self.width+1):
                        nav_id = str(self.getParameter('xy_id', str((i,j))))
                        # nav_id = self.xy_id[str((i, j))]
                        if self.getParameter('grid_map_nav',nav_id+'.obstacle') == 0:
                            if self.getParameter('grid_map_nav',nav_id+'.type') == "waiting":
                                theta = self.detect_angle(
                                    self.getParameter('grid_map_nav',nav_id+'.direction'))
                                self.waiting_grids.update({(i, j): {"status": "empty",
                                                                    "angle": theta}})
                            if self.getParameter('grid_map_nav',nav_id+'.type') == "charging":
                                theta = self.detect_angle(
                                    self.getParameter('grid_map_nav',nav_id+'.direction'))
                                self.charging_grids.update({(i, j): {"status": "empty",
                                                                    "angle": theta}})
                            if self.getParameter('grid_map_nav',nav_id+'.type') == "system_out":
                                theta = self.detect_angle(
                                    self.getParameter('grid_map_nav',nav_id+'.direction'))
                                self.system_out_grids.update({(i, j): {"status": "empty",
                                                                    "angle": theta}})
                            if self.getParameter('grid_map_nav',nav_id+'.type') == "dumping":
                                self.dumping_grids[(i, j)] = 0
        except Exception as e:
            print(str(e))
                            
def runBashScript(node,loadFilePath):
    print("ros2 param load /"+str(node)+" "+str(loadFilePath))
    bashScript = "ros2 param load /"+str(node)+" "+str(loadFilePath)
    res = os.system(bashScript)
    if res==256:
        return -1
    print("Parameters loaded")

def main(args=None):
    rclpy.init(args=args)
    script_A = ScriptA()
    thread = threading.Thread(target=rclpy.spin, args=(script_A, ), daemon=True)
    thread.start()
    print(not script_A.get_parameter_or("map_synthesis.width", None).value)
    runBashScript('scriptA', 'scriptA/resource/test.yaml')
    runBashScript('scriptA', 'scriptA/resource/grid_map_nav.yaml')
    runBashScript('scriptA', 'scriptA/resource/xy_id.yaml')
    runBashScript('scriptA', 'scriptA/resource/queue_data.yaml')
    print('Trying to get parameters with prefix map_synthesis')
    '''
    Testing getParameter with isPrefix true
    '''
    params = script_A.getParameter(configName='grid_map_nav',attribute='500100500300',isPrefix=True)
    print(params, type(params))
    for p in params.keys():
        print(p,' : ',params[p].value)
    '''
    Testing the initialize function
    '''
    script_A.printVars()
    script_A.initiatize()
    script_A.printVars()
    '''
    Testing queue_data
    '''
    print(script_A.get_queue_entry(2))
    print(script_A.get_queue_entries(2))
    '''
    queue_robot_map problem
    '''
    queue_robot_map = {}
    queue_data = script_A.getParameter(configName='queue_data', attribute=None, isPrefix=True)
    print(len(queue_data.keys()))
    for key in queue_data.keys():
        if 'entries' in key:
            queue_robot_map[str(key).replace('.entries','')] = 0
    print(queue_robot_map)
    '''
    length problem
    '''
    script_A.width = script_A.get_parameter("map_synthesis.width").value
    script_A.height = script_A.get_parameter("map_synthesis.height").value
    grid_map_nav = script_A.getParameter(configName='grid_map_nav', attribute=None, isPrefix=True)
    xy_id = script_A.getParameter(configName='xy_id', attribute=None, isPrefix=True)   
    total_grids = script_A.width*script_A.height
    # print(grid_map_nav.keys())
    while True:
        # print(total_grids)
        grid_map_nav_length = len(re.findall(r'\w*pos', str(grid_map_nav.keys())))
        print(grid_map_nav_length)
        if((len(xy_id) < total_grids) or (grid_map_nav_length < total_grids)):
            print("Loading map incomplete ... Retrying.....")
            time.sleep(1)
        else:
            break
    print("loaded")
    '''
    get_parameter_or trail
    '''
    print(not script_A.get_parameter_or("map_synthesis.width", None).value)
        
    time.sleep(20)
    rclpy.shutdown()

