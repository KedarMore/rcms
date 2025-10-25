import rclpy
from robot_msgs.msg import PathQuery
import time
"""
Class to hold robot state
"""


class Robot:
    # @TODO: make the variables protected
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.hardware_id = "none"
        self.robot_mode="none"
        self.x = -1
        self.y = -1
        self.theta = -1
        self.grid_update_timestamp = time.time()
        self.status = -1
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.path_complete_status = 0
        self.state = "stray"
        self.grid_type = "none"
        self.queue_no = -1
        # to hold any error from robot
        self.error_status = "none"
        self.error_severity = "none"
        self.error_resettable = "none"
        self.path_query = PathQuery()
        self.goal = None
        self.parcel_data = None
        self.charging = 100
        self.in_charging_state = 0
        self.load_unload_status = "empty"
        self.commission_state = "inactive"

        self.parcelId = "none"
        self.binList = []
        self.destBin = "none"
        self.loadApprovalStatus = False
        self.unloadAckStatus = False
        self.unloadApprovalStatus = 0
        self.load_timer = 0
        self.gotoChargingStation = False
        self.parcel_journey = "none"
        self.bin_list_received = False
        self.gotoSysoutGrid = False
        self.feederId = None
        self.errorHandled = 3
        self.networkErrorHandled = False
        self.robotLoadInterruptState = "empty"
        self.unloadCompletion = False
        self.unload_timer = 0

        self.liftHeight = 0
        self.binLevel = 0
        self.dumpAngle = 0

        self.robot_available = False

        self.uid = -1
        self.puid = -1

        self.mesg_counter = 0

        self.prevDestBin="none"
        self.prevParcelId="none"
        self.prevFeederId="none"

        self.eval_dump=False
        self.gotoautodocking = False
        self.fromcharging=False
        self.docking_pin_status = 'none'
        self.battery_state= 'none'
        self.operation_state = 'none'
        self.mode_state = 'none'
        self.robot_mode_status= 'none'
        self.unload_retry = 0 

        self.retry_docking_state = 0
        self.docking_history = {"error_count":{},"invalid_battery_state":{}}
        self.error_history = {}

        self.event_trace_id = "none"
        self.event_retry_count = {"queue_entry":0, "feeding_entry":0, "bot_loaded":0, "load_approval":0, 
                                   "unload_in_queue":0, "robot_at_feeder":0, "feeding_exit":0}   
        self.prev_parcel_journey = "none"
        self.prev_state = "none"
        self.prev_grid_type = "none"
        self.prev_grid_type_feederMatrix = "none"

        self.hardware_model = "none"
        self.v = -1
        self.w = -1
        self.control_status = -1
        self.charge = -1
        self.path_uid = -1
        self.path_error = False
        self.timestamp = 0
        self.robot_task_id = "none"
        self.robot_task_status = -1
        self.configuration_version = "none"
        self.software_version = "none"
        self.cb_firmware_version = "none"
        self.ods_firmware_version = "none"
        
        self.load_request_uid = "none"
        self.unload_request_uid = "none"
        self.reset_error_request_uid = "none"
        self.stop_command_uid = "none"
        self.log_uuid = "none"
        self.voltage = 100
        self.superparent = "none"
        self.parent_query_status = 0
        self.parent_query_response_timestamp = 0

        self.obstacle_distance = -1

        self.leave_feeder = False 

        self.command_queue = []

    def set_commission_state(self, msg):
        self.commission_state = msg

    def get_commission_state(self):
        return self.commission_state

    def get_charging_status(self):
        return self.charging

    def set_charging_status(self, data):
        self.charging = data

    def set_charging_state(self, data):
        self.in_charging_state = data

    def get_charging_state(self):
        return self.in_charging_state

    def get_queue_no(self):
        return self.queue_no

    def get_grid_type(self):
        return self.grid_type
    
    def set_grid_type(self,data):
        self.grid_type = data

    def get_error_status(self):
        return self.error_status

    def set_error_status(self, error_status):
        self.error_status = error_status

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state

    def get_pose(self):
        return (self.x, self.y, self.theta)

    def set_pose(self, x, y, theta):
        if(round(self.x) != round(x) or round(self.y) != round(y)):
            self.grid_update_timestamp = int(time.time())
        self.x = x
        self.y = y
        self.theta = theta

    def set_path_query(self, path_query):
        self.path_query = path_query

    def get_path_query(self):
        return self.path_query

    def get_status(self):
        return self.status

    def set_status(self, status):
        self.status = status

    def get_path_complete_status(self):
        return self.path_complete_status

    def set_path_complete_status(self, status):
        self.path_complete_status = status

    def get_goal(self):
        return self.goal

    def set_ultimate_goal(self, goal):
        self.goal = goal

    def set_parcel_data(self, data):
        self.parcel_data = data

    def get_parcel_data(self):
        return self.parcel_data

    def get_load_unload_status(self):
        return self.load_unload_status

    def set_load_unload_status(self, load_unload_status):
        self.load_unload_status = load_unload_status

    def set_parcelId(self, data):
        self.parcelId = data

    def get_parcelId(self):
        return self.parcelId

    def set_binList(self, data):
        self.binList = data

    def get_binList(self):
        return self.binList

    def set_destBin(self, data):
        self.destBin = data

    def get_destBin(self):
        return self.destBin

    def set_loadApprovalStatus(self, data):
        self.loadApprovalStatus = data

    def get_loadApprovalStatus(self):
        return self.loadApprovalStatus

    def set_unloadAckStatus(self, data):
        self.unloadAckStatus = data

    def get_unloadAckStatus(self):
        return self.unloadAckStatus

    def set_unloadApprovalStatus(self, data):
        self.unloadApprovalStatus = data

    def get_unloadApprovalStatus(self):
        return self.unloadApprovalStatus

    def set_liftHeight(self, data):
        self.liftHeight = data

    def get_liftHeight(self):
        return self.liftHeight

    def set_binLevel(self, data):
        self.binLevel = data

    def get_binLevel(self):
        return self.binLevel

    def set_dumpAngle(self, data):
        self.dumpAngle = data

    def get_dumpAngle(self):
        return self.dumpAngle

    def set_hardwareId(self, data):
        self.hardware_id = data

    def get_hardwareId(self):
        return self.hardware_id

    def set_UID(self, data):
        self.uid = data

    def get_UID(self):
        return self.uid

    def set_pUID(self, data):
        self.puid = data

    def get_pUID(self):
        return self.puid

    def get_hardwareModel(self):
        return self.hardware_model 

    def set_hardwareModel(self, data):
        self.hardware_model = data

    def get_robotId(self):
        return self.robot_id
    
    def set_robotId(self,data):
        self.robot_id = data

    def get_robotMode(self):
        return self.robot_mode

    def set_robotMode(self,data):
        self.robot_mode = data

    def get_controlStatus(self):
        return self.control_status

    def set_controlStatus(self,data):
        self.control_status = data

    def get_charge(self):
        return self.charge

    def set_charge(self,data):
        self.charge = data

    def get_pathUID(self):
        return self.path_uid
    
    def set_pathUID(self,data):
        self.path_uid = data

    def get_pathError(self):
        return self.path_error

    def set_pathError(self,data):
        self.path_error = data

    def get_robotTaskId(self):
        return self.robot_task_id
    
    def set_robotTaskId(self,data):
        self.robot_task_id = data

    def get_robotTaskStatus(self):
        return self.robot_task_status
    
    def set_robotTaskStatus(self,data):
        self.robot_task_status = data

    def get_configurationVersion(self):
        return self.configuration_version
    
    def set_configurationVersion(self,data):
        self.configuration_version = data

    def get_softwareVersion(self):
        return self.software_version
    
    def set_softwareVersion(self,data):
        self.software_version = data

    def get_cbFirmwareVersion(self):
        return self.cb_firmware_version
    
    def set_cbFirmwareVersion(self,data):
        self.cb_firmware_version = data

    def get_odsFirmwareVersion(self):
        return self.ods_firmware_version
    
    def set_odsFirmwareVersion(self,data):
        self.ods_firmware_version = data

    def get_loadRequestUID(self):
        return self.load_request_uid
    
    def set_loadRequestUID(self,data):
        self.load_request_uid = data

    def get_unloadRequestUID(self):
        return self.unload_request_uid
    
    def set_unloadRequestUID(self,data):
        self.unload_request_uid = data

    def get_resetErrorRequestUID(self):
        return self.reset_error_request_uid
    
    def set_resetErrorRequestUID(self,data):
        self.reset_error_request_uid = data

    def get_stopCommandUID(self):
        return self.stop_command_uid
    
    def set_stopCommandUID(self,data):
        self.stop_command_uid = data

    def getLogUUID(self):
        return self.log_uuid
    
    def setLogUUID(self,data):
        self.log_uuid = data
    
    def setVoltage(self,data):
        self.voltage = data
    
    def getVoltage(self):
        return self.voltage

    def get_obstacleDistance(self):
        return self.obstacle_distance
    
    def set_obstacleDistance(self,data):
        self.obstacle_distance = data

    def get_timestamp(self):
        return self.timestamp
    
    def set_timestamp(self,data):
        self.timestamp = data

    def get_robotModeStatus(self):
        return self.robot_mode_status
    
    def set_robotModeStatus(self,data):
        self.robot_mode_status = data

    def get_v(self):
        return self.v
    
    def set_v(self,data):
        self.v = data

    def get_w(self):
        return self.w
    
    def set_w(self,data):
        self.w = data

    def get_linearVelocity(self):
        return self.linear_velocity
    
    def set_linearVelocity(self,data):
        self.linear_velocity = data

    def get_angularVelocity(self):
        return self.angular_velocity
    
    def set_angularVelocity(self,data):
        self.angular_velocity = data

    def get_prevGridType(self):
        return self.prev_grid_type
    
    def set_prevGridType(self,data):
        self.prev_grid_type = data

    def set_commandQueue(self,data):
        self.command_queue = data

    def get_commandQueue(self):
        return self.command_queue

    def update_error_history(self, error_id):
        if error_id in self.error_history.keys():
            self.error_history[error_id] +=1
        else:
            self.error_history[error_id] = 1

    def get_error_history(self,error_id):
        if self.error_history.keys():
            return self.error_history[error_id]
        else:
            return 0
        
    def reset_error_history(self):
        self.error_history = {}
    
    def set_error_severity(self,data):
        self.error_severity = data
    
    def get_error_severity(self):
        return self.error_severity

    def set_resettability(self,data):
        self.error_resettable = data
    
    def get_resettability(self):
        return self.error_resettable
    
    def get_superparent(self):
        return self.superparent
    
    def set_superparent(self,robot_id):
        self.superparent = robot_id
    
    def get_parent_query_status(self):
        return self.parent_query_status
    
    def set_parent_query_status(self,value):
        self.parent_query_status = value

    def get_parent_query_response_timestamp(self):
        return self.parent_query_response_timestamp
    
    def update_parent_query_response_timestamp(self):
        self.parent_query_response_timestamp = time.time()

    def get_leave_feeder(self):
        '''
        bool value to force the robot to leave the feeder if it is obstructing 
        another robot that has to move to the charging grid
        '''
        return self.leave_feeder
    
    def set_leave_feeder(self,value):
        self.leave_feeder = value