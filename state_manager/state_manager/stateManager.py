import rclpy
from robot_msgs.msg import RobotState, PathComplete, StopCommand, LoadParcel, UnloadParcel, RobotReset
from . import robot 
import time
from array import array
from math import pi as pi
from std_msgs.msg import String
import os
import re
from rclpy.parameter import Parameter

def getParameter(node, configName, attribute=None, isPrefix=False):
        '''
        Function to get parameters of the node
        configName: The name of the configuration needed
        attribute: Attribute name of the config if needed(eg. attribute=width and configName=map_synthesis)
        isPrefix: 
            -True: If a dictionary which has configName.attribute as prefix is needed
            -False: If the value of configName.attribute is needed
        '''
        if isPrefix:
            if attribute==None:
                res = node.get_parameters_by_prefix(configName)
                if res==None:
                    raise Exception('The config'+ str(configName)+'does not exist')
            else:
                res = node.get_parameters_by_prefix(configName+'.'+attribute)
                if res==None:
                    raise Exception('The prefix '+str(configName+'.'+attribute)+' does not exist ')
        else:
            if attribute==None:
                res = node.get_parameter(configName).value
                if res==None:
                    raise Exception('The config'+ str(configName)+'does not exist')
            else:
                res = node.get_parameter(configName+'.'+attribute).value
                if res==None:
                    raise Exception('The attribute '+str(attribute)+' does not exist for config '+ str(configName))
        return res

class StateManager():

    def __init__(self,taskManagerObject):
        '''
        The taskManagerObject is the ECS rclpy Node whose parameters will be accessed by stateManager 
        '''   
        self.tm = taskManagerObject
        #setLogger()
        print('Running..')
        self.robots = {}
        self.active_sys_out_grid_count = 0
        # initialize the variables, read params
        self.initialize_parameters()
        self.qos = rclpy.qos.QoSProfile(depth=200, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        self.tm.subscriptionRobotState = self.tm.create_subscription(
            RobotState,
            'robot/robot_state',
            self.robot_state_callback,
            self.qos
        )
        self.tm.subscriptionPathComplete = self.tm.create_subscription(
            PathComplete,
            '/path_complete',
            self.path_complete_callback,
            10
        )
        #self.tm.create_timer(timer_period_sec=0.02, callback=self.update_callback)
        #self.lock = threading.Lock()
        self.logStatement("info", 'Parameters initialized')
   
    def logStatement(self,level,statement):
        if level == "debug":
            self.tm.logger.debug(statement,extra=self.tm.Logger.log_extras())
            #print("Debug: "+str(statement))
        elif level == "info":
            self.tm.logger.info(statement,extra=self.tm.Logger.log_extras())
            #print("Info: "+str(statement))
        elif level == "warn":
            self.tm.logger.warn(statement,extra=self.tm.Logger.log_extras())
            #print("Warn: "+str(statement))
        elif level == "error":
            self.tm.logger.error(statement,extra=self.tm.Logger.log_extras())
            #print("Error: "+str(statement))
        elif level == "fatal":
            self.tm.logger.fatal(statement,extra=self.tm.Logger.log_extras())
            #print("Fatal: "+str(statement))

    def initialize_parameters(self):
        """
            Initializing queue and robot data here
        """
        self.robot_location = {}
        # What qualifies as being in a certain grid
        # less than half grid distance
        self.eps = 0.5
        self.waiting_grids = {}

        self.charging_grids = {}
        self.dumping_grids = {}
        self.autodocking_grids={}
        self.charging_grids_list = {}

        # Smart/Non Smart Commission Decommission
        self.system_out_grids = {}
        self.system_out_queue_grids = {}
        self.system_in_grids = {}

        # This is where queue information is stored
        # currently it only stores the number of robots in a given
        # queue
        # key: queue_i, value: n (number of robots)
        self.queue_robot_map = {}

        # robot state, grid, status is stored here
        # key: robot_i, value=Robot()

        self.robot_data_received = {}
        # temprorily stored robot data
        self.robot_data = RobotState()
        self.robots_data = {}
        self.robot_mesg_recvd_timestamp = {}
  
        self.mesg_timeout = 10.0

        self.xy_id={}

        self.width = getParameter(node=self.tm, configName='map_synthesis', attribute='width')
        self.height = getParameter(node=self.tm, configName='map_synthesis', attribute='height')
           
        total_grids = self.width*self.height
        while True:
            time.sleep(5)
            try:
                # print('keys: '+str(len(grid_map_nav.keys())/4))
                grid_map_nav = getParameter(node=self.tm, configName='grid_map_nav', attribute=None, isPrefix=True)
                xy_id = getParameter(node=self.tm, configName='xy_id', attribute=None, isPrefix=True)
                grid_map_nav_length = len(re.findall(r'\w*pos', str(grid_map_nav.keys())))
                print("total grids:",total_grids)
                print("grid_map_nav",grid_map_nav_length)
                #print("xy_id:",xy_id)
                if((len(xy_id) < total_grids) or (grid_map_nav_length < total_grids)):
                    print("Loading map incomplete ... Retrying.....")
                    time.sleep(1)
                else:
                    break
            except KeyError:
                self.logStatement("warn", "gridmap not found in parameter server")
                time.sleep(1)

        self.grid_map_nav = self.getParameterDictionary("grid_map_nav")
        
        queue_data = getParameter(node=self.tm, configName='queue_data', attribute=None, isPrefix=True)
        for key in queue_data.keys():
            if 'entries' in key:
                self.queue_robot_map[str(key).replace('.entries','')] = 0
        
        # # for i in range(0, self.robots_number):
        # for i in range(0, (self.robots_number+self.extra_robots)):    
        #     key = "robot_" + str(i)
        #     self.robots[key] = robot.Robot(key)
        #     if (i<self.robots_number):
        #         self.robot_data_received[key] = False
                
        # print(self.robot_data_received)

        self.width = getParameter(node=self.tm, configName='map_synthesis', attribute='width')
        self.height = getParameter(node=self.tm, configName='map_synthesis', attribute='height')
        try:
            for j in range(1, self.height+1):
                for i in range(1, self.width+1):
                    co_ordinate = str((i,j))
                    nav_id = getParameter(node=self.tm, configName='xy_id',attribute= str((i,j)))
                    # nav_id = self.xy_id[str((i, j))]
                    if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.obstacle') == 0:
                        if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type') == "waiting":
                            theta = self.detect_angle(
                                getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.direction'))
                            self.waiting_grids.update({(i, j): {"status": "empty",
                                                                "angle": theta}})


                        if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type') == "dumping":
                            self.dumping_grids[(i, j)] = 0



                        # Autodocking / Charging

                        if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type') == "autodocking":
                            theta = self.detect_angle( getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.direction'))
                            self.autodocking_grids.update({(i, j): {"status": "empty","angle": theta,"is_enabled": True}})

                        if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type') == "charging":
                            theta = self.detect_angle( getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.direction'))
                            self.charging_grids_list.update({(i, j): {"status": "empty", "angle": theta,"is_enabled": True}})

                        # Smart Commission Decommission

                        try :
                            if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type') == "system_out":
                                theta = self.detect_angle(getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.direction'))
                                # sys_out = [key for key,value in self.tm.service_grids["service_grids"]["system_out"].items() if (value["X"]==i and value["Y"]==j)]
                                sys_out = None
                                for key,value in self.tm.service_grids["service_grids"]["system_out"].items() :
                                    if value["X"] == i and value["Y"] == j:
                                        sys_out = key
                                        break
                                self.system_out_grids.update({(i, j): {"status": "empty", "angle": theta , "is_enabled": False, "safe_to_decommission": False, 
                                                                    "robot_expected": False, "ack": False, "robot_count": 0, "queue_size": 0}})
                                if sys_out and self.tm.system_in_out_grids:
                                    self.system_out_grids[(i, j)]["priority"] = self.tm.system_in_out_grids["system_in_out_grids"]["system_out"][sys_out]["priority"] if self.tm.system_in_out_grids["system_in_out_grids"]["system_out"][sys_out]["priority"] else 100000
                                    self.system_out_grids[(i, j)]["id"] = key
                                # {(5,4):{"status": "empty", "angle": theta}, "is_enabled": True, "safe_to_decomission": True}
                        except Exception as error :
                            print ("Error while fetching system out grids in state manager : ", error)


                        if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type') == "system_out_queue":
                            theta = self.detect_angle(getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.direction'))
                            self.system_out_queue_grids.update({(i, j): {"status": "empty", "angle": theta}})
                            

                        if getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type') == "system_in":
                            theta = self.detect_angle(getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.direction'))
                            self.system_in_grids.update({(i, j): {"is_enabled": False, "safe_to_commission": False, "robot_expected": False, "ack": False}})
                            # {(5,4):{"status": "empty", "angle": theta}, "is_enabled": True, "safe_to_decomission": True}


                            
                    self.xy_id[co_ordinate] = nav_id
            print("autodocking")
            print(self.autodocking_grids)
            print("-------------------x-----------------")
            if len(self.autodocking_grids) >= 1:
                print("Grouping Autodocking Grids")
                self.group_docking_grids()

            try :
                if self.tm.system_in_out_grids :
                    print ("System-out grids BEFORE SORTING ", self.system_out_grids)
                    sorted_system_out_grids = sorted(self.system_out_grids.items(), key=lambda x: x[1]['priority'])
                    self.system_out_grids = dict(sorted_system_out_grids)
                    print ("System-out grids AFTER SORTING ", self.system_out_grids)
            except Exception as error :
                print("Error while sorting system out grids ", error)

            if len(self.system_out_grids) >=1:
                print("Grouping System Out Grids and System Out Queue Grids")
                self.group_system_out_grids()

            print("System Out Grids Grouping: ", self.system_out_grids, " System Out Queue Grids: ", self.system_out_queue_grids)
            print("Autodocking Grids")
            print(self.autodocking_grids)
            
            print("Charging Grids")
            print(self.charging_grids_list)  

            print ("SM parameters are initialized")

        except Exception as e:
            self.logStatement("error", str(e))

    def is_initialized(self):

        ###############################################################################
        # if len(self.robot_data_received.keys()) < self.robots_number:
        #     out = False
        # else:
        #     out = True
        #     for key in self.robot_data_received:
        #         out = out and self.robot_data_received[key]
        # return out

        return True

    def in_bounds(self, ids):
        """
            Method to check if given coordinates
            is within defined bounds
        """
        try:
            (x, y) = ids
            return 0 <= x < self.width and 0 <= y < self.height
        except:
            raise Exception('The argument '+str(ids)+' is not of proper datatype. ids is a tuple with two integer entries.')
    
    def decommission_robot_sm(self,robot_id):
        try:
            if robot_id in self.robots_data:
                robot_data = self.robots_data[robot_id]
                robot = self.robots[robot_id]
                if robot.get_commission_state() == "inactive":
                    return
                elif (robot.get_commission_state() == "to_decommission_almost"):
                    print(" $$$$$$$$$$$$$$$$$$$$$$$ decommission process started SM $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                    print("Robot: ",robot_data.robot_id)
                    print("Hardware ID: ",robot_data.hardware_id)
                    print("Previous state: ")
                    print("x: ",robot_data.x,"; y: ",robot_data.y,"; theta: ",robot_data.theta)
                    try:
                        nav_id = getParameter(node=self.tm, configName='xy_id',attribute= str((x,y)))
                        #print("grid: ",getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type'))
                        grid_type = getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type')
                        if (grid_type == "queue"):
                            queue_no = self.robots[robot_data.robot_id].queue_no
                            if self.queue_robot_map["queue_"+str(queue_no)] > 0:
                                self.queue_robot_map["queue_"+str(queue_no)] -= 1
                            else:
                                pass
                        # elif (grid_type == "charging"):
                        #     self.charging_grids[(robot_data.x,robot_data.y)]["status"] = "empty"
                        else:
                            pass
                    except:
                        pass
                    robot.set_commission_state("inactive")
                    # --------------
                    self.robots_data[robot_id].x = 1.0
                    self.robots_data[robot_id].y = 1.0
                    self.robots_data[robot_id].theta = 0.0
                    robot.set_UID(-1)
                    # grid_type = "none"
                    # self.robots[robot_data.robot_id].grid_type = grid_type
                    # self.robots[robot_data.robot_id].prev_grid_type = grid_type
                    print("Updated state: ")
                    print("x: ",self.robots_data[robot_id].x,"; y: ",self.robots_data[robot_id].y,"; theta: ",self.robots_data[robot_id].theta)
                    #nav_id = getParameter(node=self.tm, configName='xy_id',attribute= str((int(self.robots_data[robot_id].x), int(self.robots_data[robot_id].y))))
                    #print("grid: ",getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type'))
                    # --------------
                    print(" $$$$--------------$$$$ decommission process ended SM $$$$--------------$$$$")
                    return
        except Exception as e:
            self.logStatement("error", "[decommission_robot_sm] error in state manager while decommission robot_id:"+robot_id+" error:"+str(e))
            return

    def update_robot_data(self,robot_id):
        """
            Get the robot, update the position (in ints, theta)
            In future receive calls from robot path end and update
            state
            Algo:
                1. Get robot state, decide what data to write using id
                2. update position if within threshold
                3. check if it is in bounds
                4. Get the grid type of current (x, y)
                5. If change from not queue to queue,
                    get the queue number and update robot count
                6. similarly from queue to non queue deduct the count
            # TODO: update a flag after all robots have been read at least
                    once
        """
        # t1 = time.time()
        # print('update robot callback: ',(self.is_initialized()))
        # print (self.robots_data.keys())
        try:
            # print('update robot callback')
            
            if robot_id in self.robots_data:
                robot_data = self.robots_data[robot_id]
                robot = self.robots[robot_id]
                robot.robot_mode = robot_data.robot_mode
                if robot_data.robot_mode_status != '' and robot.robot_mode == "charging":
                    try: 
                        self.robot_mode_status = robot_data.robot_mode_status
                        (robot.operation_state,robot.docking_pin_status,robot.battery_state,robot.mode_state) = robot_data.robot_mode_status.split(',')
                    except Exception as e:
                        print("exception duing spliting robot_mode state",e)

                prev_grid_type = self.robots[robot_data.robot_id].grid_type

                # capture a set of data
                x = int(round(robot_data.x))
                y = int(round(robot_data.y))
                theta = robot_data.theta
                uid = robot_data.uid
                control_status = robot_data.control_status
                error_status = robot_data.error_status
                parcel_status = robot_data.parcel_status
                charge = robot_data.charge
                hardware_id = robot_data.hardware_id
                hardware_model = robot_data.hardware_model
                robot_mode = robot_data.robot_mode
                v = robot_data.v
                w = robot_data.w
                control_status = robot_data.control_status
                robot_mode_status = robot_data.robot_mode_status
                timestamp = robot_data.timestamp
                robot_task_id = robot_data.robot_task_id
                robot_task_status = robot_data.robot_task_status
                configuration_version = robot_data.configuration_version
                software_version = robot_data.software_version
                cb_firmware_version = robot_data.cb_firmware_version
                ods_firmware_version = robot_data.ods_firmware_version
                load_request_uid = robot_data.load_request_uid
                unload_request_uid = robot_data.unload_request_uid
                reset_error_request_uid = robot_data.reset_error_request_uid
                stop_command_uid = robot_data.stop_command_uid
                log_uuid = robot_data.log_uuid
                # voltage = robot_data.voltage

                robot.set_hardwareId(hardware_id)
                robot.set_hardwareModel(hardware_model)
                robot.set_robotMode(robot_mode)
                robot.set_v(v)
                robot.set_w(w)
                robot.set_controlStatus(control_status)
                robot.set_robotModeStatus(robot_mode_status)
                robot.set_timestamp(timestamp)
                robot.set_robotTaskId(robot_task_id)
                robot.set_robotTaskStatus(robot_task_status)
                robot.set_configurationVersion(configuration_version)
                robot.set_softwareVersion(software_version)
                robot.set_cbFirmwareVersion(cb_firmware_version)
                robot.set_odsFirmwareVersion(ods_firmware_version)
                # robot.setVoltage(voltage)
            
                #if robot.get_commission_state() == "inactive":
                #    robot.set_pose(x, y, theta)
                #    robot.set_status(control_status)
                #    robot.set_error_status(error_status)
                #    robot.set_load_unload_status(parcel_status)
                #    robot.set_charging_status(charge)
                #    robot.set_UID(uid)
                    
                # safety check to avoid unwanted coordinates
                
                if str((x, y)) not in self.xy_id:
                    robot.set_pose(x, y, theta)
                    robot.set_status(control_status)
                    robot.set_error_status(error_status)
                    robot.set_load_unload_status(parcel_status)
                    robot.set_charging_status(charge)
                    robot.set_UID(uid)

                    robot.set_loadRequestUID(load_request_uid)
                    robot.set_unloadRequestUID(unload_request_uid)
                    robot.set_stopCommandUID(stop_command_uid)
                    robot.set_resetErrorRequestUID(reset_error_request_uid)
                    robot.setLogUUID(log_uuid)

                    self.robots[robot_data.robot_id].prev_grid_type = "na"
                    self.logStatement("error", " coordinates are not within map boundary in SM")
                    return

                # force reset UID
                if (robot_data.uid <= 10):
                    robot.set_UID(-1)

                # skip older robot mesg otherwise update
                if ((robot_data.uid < robot.get_UID()) and robot_data.uid != 0):
                    self.logStatement("error", "SM001: older robot-state mesg. Ignoring")
                    print(self.robots_data[robot_id])
                    return
                else:
                    robot.set_pose(x, y, theta)
                    robot.set_status(control_status)

                    robot.set_error_status(error_status)

                    robot.set_load_unload_status(parcel_status)
                    robot.set_charging_status(charge)
                    robot.set_UID(uid)

                    robot.set_loadRequestUID(load_request_uid)
                    robot.set_unloadRequestUID(unload_request_uid)
                    robot.set_stopCommandUID(stop_command_uid)
                    robot.set_resetErrorRequestUID(reset_error_request_uid)
                    robot.setLogUUID(log_uuid)

                    nav_id = getParameter(node=self.tm, configName='xy_id',attribute= str((x, y)))
                    if (not self.in_bounds((x, y))):
                        if (getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.obstacle') == 1):
                            self.logStatement("error", "robot in obstacle grid")
                            return
                        return

                    grid_type = getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.type')
                    queue_no = self.robots[robot_data.robot_id].queue_no
                    if (grid_type == "queue"):
                        self.robots[robot_data.robot_id].queue_no = int(
                            getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.queue_no'))
                    elif grid_type == "feeding":
                        self.robots[robot_data.robot_id].queue_no = int(
                            getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.queue_no'))
                    else:
                        self.robots[robot_data.robot_id].queue_no = -1
                    
                    self.robots[robot_data.robot_id].grid_type = grid_type

                    if (grid_type == "queue" and prev_grid_type != "queue"):
                        queue_no = getParameter(self.tm, 'grid_map_nav',str(nav_id)+'.queue_no')
                        self.queue_robot_map["queue_"+str(queue_no)] += 1
                    elif (grid_type != "queue" and prev_grid_type == "queue"):
                        self.queue_robot_map["queue_"+str(queue_no)] -= 1
                    
                    self.robots[robot_data.robot_id].prev_grid_type = prev_grid_type

                # print("Done update_robot_data: ",self.robots)
                # print(self.robots_data)
                    # t2 = time.time()
        except RuntimeError or NameError or TypeError or SyntaxError as e:
            self.logStatement("error", " ************************************  StateManager Exception 001 *************************************** ")
            self.logStatement("error","error: "+str(e)+", robot_id: "+str(robot_id)+", hardware id: ",str(self.robots_data[robot_id].hardware_id))
            print(self.robots_data)
#grouping charging grids and autodocking grids
    def group_docking_grids(self):
        dockinglen= len(self.autodocking_grids)
        counter =0
        group={}
        for x,y in self.charging_grids_list:
           # print("charging grids",x,y)
            group={}
            for x1,y1 in self.autodocking_grids:
                #print("auto docking",x1,y1)
                if ((abs(x1-1)==x) or (abs(x1+1)==x)) and (y1 == y ):
                    #rospy.logerr("grouping x matched")
                    #print({"docking":{(x1,y1):self.autodocking_grids[x1,y1]}})
                    group.update({(x1,y1):self.autodocking_grids[x1,y1]})
                    counter +=1

                elif ((abs(y1-1)==y) or (abs(y1+1)==y)) and (x1 == x ):
                    #rospy.logerr("grouping y matched")
                    group.update({(x1,y1):self.autodocking_grids[x1,y1]})
                    counter +=1
            if len(group) >0:
                self.charging_grids.update({(x,y):{"docking":group,"status":"empty","angle":self.charging_grids_list[x,y]["angle"],"is_enabled": True}})
        print(self.charging_grids)
        if counter != dockinglen:
            print("some docking grids are not associated with charging grids")


    def group_system_out_grids(self):
        sysoutlen= len(self.system_out_grids)
        counter =0
        group={}
        for x,y in self.system_out_queue_grids:
           # print("System Out Queue Grids",x,y)
            group={}
            for x1,y1 in self.system_out_grids:
                #print("System Out Grids",x1,y1)
                if ((abs(x1-1)==x) or (abs(x1+1)==x)) and (y1 == y ):
                    group.update({(x1,y1):self.system_out_grids[x1,y1]})
                    counter +=1

                elif ((abs(y1-1)==y) or (abs(y1+1)==y)) and (x1 == x ):
                    #rospy.logerr("grouping y matched")
                    group.update({(x1,y1):self.system_out_grids[x1,y1]})
                    counter +=1
            if len(group) >0:
                self.system_out_queue_grids.update({(x,y):{"system_out_smart":group,"status":"empty","angle":self.system_out_queue_grids[x,y]["angle"]}}) 
        
        #{(8,3): {"system_out_smart":{(8,4):{"status": "empty", "angle": theta}}, "is_enabled": True, "safe_to_decomission": True}, 1:2}
        # Henceforth Any time Status update comes, Please update both system_out_queue grouping and system_out

        print(self.system_out_queue_grids)
        if counter != sysoutlen:
            print("some System Out grids are not associated with System_out_queue Grids")

    def get_robots(self, queue_no):
        """
            Function to return the number of robots in a
            given queue
        """
        try:
            return self.queue_robot_map[queue_no]
        except Exception as e:
            raise Exception('Either queue_no not in proper format or the entry does not exist. '+str(e))

    def get_queue_entries(self, queue_no):
        '''
        Get all queue entries for all the grids present in .entries field of queue_data for the given queue_no
        '''
        try:
            entries = getParameter(node=self.tm, configName='queue_data', attribute='queue_'+str(queue_no)+'.entries')
            print(entries)
            pos_entries = {}
            for entry in entries:
                x = getParameter(node=self.tm, configName='queue_data', attribute='queue_'+str(queue_no)+'.'+str(entry)+'.X')
                y = getParameter(node=self.tm, configName='queue_data', attribute='queue_'+str(queue_no)+'.'+str(entry)+'.Y')
                theta = getParameter(node=self.tm, configName='queue_data', attribute='queue_'+str(queue_no)+'.'+str(entry)+'.theta')
                pos_entries[entry] = (x, y, theta)
            return pos_entries
        except Exception as e:
            raise Exception('Either queue_no not in proper format or the parameter does not exit. '+str(e))
            
    def get_queue_entry(self, queue_no):
        '''
        Get queue entry for grid_1 present in queue_data for the given queue_no
        '''
        try:
            queue_entry = getParameter(node=self.tm, configName='queue_data', attribute='queue_'+str(queue_no)+'.grid_1', isPrefix=True)
            x = queue_entry["X"].value
            y = queue_entry["Y"].value
            theta = queue_entry["theta"].value
            return (x, y, theta)
        except Exception as e:
            raise Exception('Either queue_no not in proper format or the parameter does not exit. '+str(e))
            
    def get_feeder_pose(self, queue_no):
        """
            Return (x, y) pos of the feeder
        """
        try:
            feeder = getParameter(node=self.tm,configName='queue_data', attribute="queue_"+str(queue_no)+".feeding", isPrefix=True)
            x = feeder["X"].value
            y = feeder["Y"].value
            theta = feeder["theta"].value
            return (x, y, theta)
        except Exception as e:
            raise Exception('Either queue_no not in proper format or the parameter does not exit. '+str(e))

    def path_complete_callback(self, msg):
        '''
        Callback of the /path_complete subscriber
        '''
        robot_id = msg.robot_id
        try:
            now = int((self.tm.get_clock().now().nanoseconds)*(10**-6))
            #self.logStatement("info", "Current time %s", str(now))
            #self.logStatement("info", "robot_id: %s completed path. time %s", robot_id, str(now))
            #self.logStatement("info", "Message received: ",str(msg))
            self.robots[robot_id].path_complete_status = True
            self.robots[robot_id].puid = msg.uid
        except Exception as e:
            time = str( int((self.tm.get_clock().now().nanoseconds)*(10**-6)))
            statement= "["+(time) +"]"+"[StateManager]error in path_complete_callback for robot_id:"+str(robot_id)+ ", exception: " + str(e)
            self.logStatement("error", statement)

    def robot_state_callback(self, robot_data):
        '''
        Callback of the robot_state subscriber.
        NOTE:  time.time can be replaced by rosclpy
        '''

        try:
            robot_id = robot_data.robot_id
            if robot_id not in self.robots.keys():
                self.robots[robot_id] = robot.Robot(robot_id)
                # if (len(self.robots.keys()) < self.robots_number):
                self.robot_data_received[robot_id] = False
            self.robots_data[robot_id] = robot_data
            self.robot_mesg_recvd_timestamp[robot_id] =  int((self.tm.get_clock().now().nanoseconds)*(10**-6))
            self.robot_data_received[robot_id] = True
            self.update_robot_data(robot_id)
        except Exception as e:
            time = str( int((self.tm.get_clock().now().nanoseconds)*(10**-6)))
            statement= "["+time +"]"+"[StateManager]error in robot_state_callback for robot_id:"+str(robot_id)+ ", exception: " + str(e)
            self.logStatement("error", statement)

    def detect_angle(self, direction):
        try:
            if type(direction) != String: 
                direction = str(direction)
                self.logStatement("debug", 'Data type of direction changed to string')
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

        except Exception as e:
            self.logStatement("fatal", 
                "Directions are not in specified format : '(1,2,3,4)'. Execption: "+str(e))
                # for given coord : " + str((x, y)))
            raise Exception("Directions are not in specified format : '(1,2,3,4)'")
            # return 0

    def getParameterDictionary(self, configName, isDict=True):
        '''
        Function to get parameters of the node as a dictionary
        '''
        if not isDict:
            result = self.tm.get_parameter(configName).value
            if result==None:
                raise Exception('The config '+ str(configName)+' does not exist')
            return result
        res = self.tm.get_parameters_by_prefix(configName)
        if res=={}:
            result = self.tm.get_parameter(configName).value
            if result==None:
                raise Exception('The config '+ str(configName)+' does not exist')
            return result
        if res==None:
            raise Exception('The config '+ str(configName)+' does not exist')
        responseDict = {}
        current = responseDict
        
        for key in res.keys():
            attributes = key.split('.')
            current = responseDict
            for attribute in attributes:
                if attribute == attributes[-1]:
                    if (type(res[key].value) is array):
                        current[attribute] = res[key].value.tolist()
                        continue
                    elif (type(res[key].value)==str) and (',' in res[key].value):
                        if ('(' in res[key].value):
                            res_ = res[key].value.strip('()')
                            responseList = res_.split(',')
                            for response in responseList:
                                response = float(response)
                            # print("Returning: ",responseList)
                            current[attribute] = tuple(responseList)
                        elif ('[' in res[key].value):
                            res_ = res[key].value.strip('[]')
                            responseList = res_.split(',')
                            for response in responseList:
                                response = float(response)
                            current[attribute] = responseList
                        continue
                    current[attribute] = res[key].value
                else:
                    try:
                        current = current[attribute]
                    except:
                        current[attribute] = {}
                        current = current[attribute]
        return responseDict
