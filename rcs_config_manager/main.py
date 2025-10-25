#!/usr/bin/python
import threading
import signal
import threading
import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from . import models
import json
import yaml
from datetime import datetime
import os
from robot_msgs.srv import GetParam
from robot_msgs.msg import SessionEvent
import uvicorn
import requests
import logging
import glob
from std_msgs.msg import String
from rcs_logger.rcs_logger import Logger as log_module
from fastapi.responses import JSONResponse
from tenacity import retry, stop_after_attempt, wait_exponential, Retrying, retry_if_exception_type
import time
from fastapi import Header
from typing import Optional

paramConfigJson = {}
dynamicConfigJson = {}
updatedConfigJson = {}
getConfigJson = {}
serviceRequestRate = 1
configVersion = ""
activeLayout = -1
responseTimeout = 30
requestTime = 0
paramtersInitialized = False
nodes_list = []
relConfigPath = "src/rcs_config_manager/cms_config/config.json"
app = FastAPI()
rcsCmsNode = None
configJson = {}
srcDir='/'
date_format = '%d-%m-%Y'
time_format = '%H:%M:%S'

Logger = log_module('RCS_Config_Manager')
# Init logger object
logger = Logger.init_logger()
logger.info("Logger Configured", extra=Logger.log_extras())

logLevel = {
    'debug':0,
    'info':1,
    'warn':2,
    'error':3 
}

global_config_file = "/config/config.yml"

with open(global_config_file, 'r') as f:
    global_config = yaml.safe_load(f)

IP = global_config["COMMON"]["ENDPOINT"]["CBS"]["BASE"]
PORT = global_config["CBS"]["CMS"]["PORT"]

Base_URL = "http://" + str(IP) + ":" + str(PORT) + "/"

#---------------------------------------------------------------------------------------------------------------

def read_zone_id_from_yaml():
    """
    Reads the ZONE_ID from a YAML file.

    :return: Value of ZONE_ID or None if not found
    """
    try:
        global global_config_file
        with open(global_config_file, 'r') as file:
            data = yaml.safe_load(file)
            return data.get('COMMON', {}).get('ZONE_ID', "-1")
    except FileNotFoundError:
        logger.info("File not found: "+str(global_config_file), extra=Logger.log_extras())
    except yaml.YAMLError as e:
        logger.info("YAML parsing error: "+str(e), extra=Logger.log_extras())
    except Exception as e:
        logger.info("Some issue with zone id: " + str(e), extra=Logger.log_extras())
    return "-1"

def loadConfig(): 
    '''
    Initialises a logger and loads the system config from config.json
    '''
     
    global configJson,srcDir
    srcDir = os.getcwd()
    
    configFilePath = os.path.join(srcDir,relConfigPath)
    configFile = open(configFilePath,'r')
    configJson = json.load(configFile)
    zone_id = read_zone_id_from_yaml()
    if zone_id == "-1":
        logger.info("Zone id is -1", extra=Logger.log_extras())
    configJson["ZONE_ID"] = zone_id
    configFile.close()

class RCS_CMS_Node(Node):
    '''
    A node with a publisher function to publish whenever a config file is loaded to a node 
    '''

    def __init__(self):
        super().__init__('RCS_CMS_Node')
        self.Logger = Logger
        self.logger = logger

        self.logger.info("RCS CMS Node initialized", extra=self.Logger.log_extras())

        self.req_nodes = {}
        self.timer_period = configJson["retryTimer"]
        # self.configTimer = self.create_timer(self.timer_period,self.callbackRetries)

        # PUT
        # self.session_state = "init"
        # self.event_status = "ack"

        # PATCH
        self.session_state = "trial"
        self.event_status = "trial"

        self.create_subscriber_session_even_update() # background subscriber for topic 'session_event_update'

    def spin(self):
        self.rate_ = self.create_rate(50)
        while(rclpy.ok()):
            self.rate_.sleep()
    
    def createService(self):
        self.srv = self.create_service(
            GetParam, 
            'get_param', 
            self.getParameterFromCMS
            )
        logger.info('Service created', extra=self.Logger.log_extras())

    def getParameterFromCMS(self, request, response):
        '''
        'get_param' service callback function
        Will return True if all the parameters are set for the particular node else False
        '''
        try: # change the json structure here
            global paramConfigJson
            self.req_nodes[request.node_name.data] = False
            configs = json.dumps( paramConfigJson.get(request.node_name.data,{}) )
            returner = String()
            returner.data = configs
            response.data = returner
            self.logger.info('Request received returning true for: '+request.node_name.data, extra=self.Logger.log_extras())
            self.logger.info('Sending config: '+str(configs), extra=self.Logger.log_extras())
        except Exception as e:
            self.logger.error("Cannot send the json", extra=self.Logger.log_extras())
            self.logger.error(str(e), extra=self.Logger.log_extras())
            returner = String()
            returner.data = "False"
            response.data = returner
        return response

    # def callbackRetries (self):
    #     print("========================================================================================node queue==================================\n",self.node_queue)
    #     print("in the call back retries",str(datetime.now()))
    #     if len(self.node_queue) == 0:
    #         print("no element in queue")
    #         return 
    #     for queue_element in self.node_queue:
    #         print("in the loop queue element",queue_element)
    #         # with timeout(seconds=3):
    #         try:
    #             date = (datetime.now()).strftime(date_format)
    #             filepath = getRecentFile(configJson['loadParamFiles'],queue_element['filename'],date)
    #             filepath = os.path.join(srcDir,filepath)
    #             result = runBashScript(queue_element["nodename"],filepath,queue_element["filename"],retry=False)
    #             print("result of run bash script in callback",result)

    #             if result == 1:
    #                 self.node_queue.remove(queue_element)
    #                 print("Removed queue element",queue_element,"from node list")
    #         except Exception as e:
    #             print("Could not retry loading params to node",queue_element["nodename"])
    #             logStatement(logLevel['error'],e)

    # def callbackRetries (self):
    #     #print("=========================================================node queue==================================\n",self.node_dict)
    #     # print("in the call back retries",str(datetime.now()))
    #     self.logger.debug("In the call back retries " + str(datetime.now()), extra=self.Logger.log_extras())
    #     active_nodes =self.get_node_names()
    #     if len(self.node_dict) == 0:
    #         # print("no element in dictionary")
    #         self.logger.warning("No element in dictionary", extra=self.Logger.log_extras())
    #         return 
    #     for nodename in self.node_dict.copy().keys():
    #         # print("in the loop dictionary element",nodename)
    #         self.logger.debug("In the loop dictionary element " + str(nodename), extra=self.Logger.log_extras())
    #         # with timeout(seconds=3):
    #         try:
    #             if nodename in active_nodes:
    #                 for fileName in self.node_dict[nodename].copy():
    #                     date = (datetime.now()).strftime(date_format)
    #                     filepath = getRecentFile(configJson['loadParamFiles'],fileName,date)
    #                     filepath = os.path.join(srcDir,filepath)
    #                     result = runBashScript(nodename,filepath,fileName,retry=False)
    #                     # print("result of run bash script in callback",result)
    #                     self.logger.info("Result of run bash script in callback " + str(result), extra=self.Logger.log_extras())

    #                     if result == 1:
    #                         self.node_dict[nodename].discard(fileName)
    #                         # print("Removed set element ",fileName," from dictionary")
    #                         self.logger.info("Removed set element " + str(fileName) + " from dictionary", extra=self.Logger.log_extras())
    #                 if self.node_dict[nodename] == set():
    #                     self.node_dict.pop(nodename)
    #                     # print("Removed dictionary element ", nodename)
    #                     self.logger.info("Removed dictionary element " + str(nodename), extra=self.Logger.log_extras())
    #             else:
    #                 msg= "node is bot active skipping to upload config to nodename:"+nodename
    #                 # print(msg)
    #                 # self.logger.warning(msg, extra=self.Logger.log_extras())
    #                 self.logger.warning(msg, extra=self.Logger.log_extras())
    #                 #logStatement(logLevel['info',],msg)
    #         except Exception as e:
    #             # print("[callbackRetries] Exception while loading param to ",nodename)
    #             # logStatement(logLevel['error'],e)
    #             self.logger.error("[callbackRetries] Exception while loading param to " + nodename + ": " + str(e), extra=self.Logger.log_extras())
        
    # def appendQueue(self,nodename,filename):
    #     if nodename not in self.node_dict.keys():
    #         self.node_dict[nodename] = set()
    #     self.node_dict[nodename].add(filename)
    #     # print(filename," added for key ",nodename)
    #     self.logger.debug(filename + " added for key " + nodename, extra=self.Logger.log_extras())

    def get_configuration(self, node_name):
        global getConfigJson
        client = self.create_client(GetParam, f'{node_name}_get_config_service')
        try:
            finder = client.wait_for_service(timeout_sec=2.0)
            if not finder:
                self.get_logger().info(f'{node_name} is not available. Moving to the next node.')
                return False
            rate = self.create_rate(serviceRequestRate)
        except Exception as e:
            logger.error('Exception occurred in get_configuration'+str(e), extra=self.Logger.log_extras())
        response = False
        while not response:
            rate.sleep()
            try:
                request = GetParam.Request()
                configs = json.dumps( dynamicConfigJson.get(node_name,{}) )
                self.logger.info('Sending get-config request to '+str(node_name), extra=self.Logger.log_extras()) # change

                requester = String()
                requester.data = "GetConfigRequest"
                request.node_name = requester
                self.future = client.call_async(request)
                while not self.future.done():
                    rate.sleep()

                result =  self.future.result().data
                response_json = json.loads(result.data)
            
                getConfigJson.update({node_name : response_json})

                response = True
            except Exception as e:
                self.logger.error('Could not get config . Execption occured: ' + str(e), extra=self.Logger.log_extras())
                return False
        self.logger.info('All parameters are received from '+ node_name, extra=self.Logger.log_extras())
        return True

    def send_configuration(self, node_name, http_method="PUT"):
        global responseTimeout, requestTime, updatedConfigJson
        configs = {}
        client = self.create_client(GetParam, f'{node_name}_set_config_service')
        try:
            finder = client.wait_for_service(timeout_sec=2.0)
            if not finder:
                self.get_logger().info(f'{node_name} is not available. Moving to the next node.')
                return False
            rate = self.create_rate(serviceRequestRate)
        except Exception as e:
            logger.error('Exception occurred in send_configuration'+str(e), extra=self.Logger.log_extras())
        response = False
        while not response:
            rate.sleep()
            try:
                request = GetParam.Request()
                self.logger.info('Updated Config '+json.dumps(updatedConfigJson,indent=4), extra=self.Logger.log_extras()) # change
                # for entry in dynamicConfigJson:
                #     if entry==node_name:
                #         # configs = json.dumps(dynamicConfigJson.get('config',{})[0].get('config',{}).get('dynamic',{}))
                #         logger.info("configs"+str(configs), extra=self.Logger.log_extras())
                #     else:
                #         # configs = json.dumps( dynamicConfigJson.get(node_name,{}))
                #         logger.info("configs"+str(configs), extra=self.Logger.log_extras())
                #     # doubt about nodes
                # PUT vs PATCH
                if http_method == "PUT":
                    # configs = json.dumps(updatedConfigJson["config"])
                    for nodes in updatedConfigJson:
                        if nodes["config_name"]==node_name:
                            configs = json.dumps(nodes["config"])
                elif http_method == "PATCH":
                    configs = json.dumps(updatedConfigJson)
                # configs = json.dumps( dynamicConfigJson.get(node_name,{}))
                self.logger.info('configs '+configs, extra=self.Logger.log_extras())
                requester = String()
                requester.data = configs
                request.node_name = requester
                self.future = client.call_async(request)
                requestTime = time.time()
                while not self.future.done():
                    if time.time() - requestTime > responseTimeout :
                        return False                        
                    rate.sleep()
                if self.future.result().data.data == "True":
                    response = True
                    self.get_logger().info("New config request is sent for "+ node_name)
                else:
                    response =  False
                    self.get_logger().info("New config request is not sent for "+ node_name)
                    return False
            except Exception as e:
                self.logger.error('Could not send request. Execption occured: ' + str(e), extra=self.Logger.log_extras())
                return False
        self.logger.info('All parameters are loaded to the parameter server of '+ node_name, extra=self.Logger.log_extras())
        return True

    def configure_nodes(self, config_req_type, node_list: Optional[list], http_method="PUT"):
        failed_nodes = []
        global paramConfigJson,nodes_list
        logger.info("Configuring nodes with config_req_type: %s, nodes_list: %s",config_req_type, nodes_list, extra=self.Logger.log_extras())
        # First iteration: Send configuration to all nodes
        for entry in nodes_list :
        # for entry in paramConfigJson:
            node_name = entry
            if config_req_type == "set-config" and http_method == "PUT":
                # logger.info("updatedConfigJson"+json.dumps(updatedConfigJson,indent=4), extra=self.Logger.log_extras())
                # logger.info("node_name "+str(node_name), extra=self.Logger.log_extras())
                # if node_name in updatedConfigJson : 
                #     logger.info("I was here inside configure_nodes. (PUT) ", extra=self.Logger.log_extras())
                #     if not self.send_configuration(node_name, http_method):
                #         failed_nodes.append(node_name)
                for nodes in updatedConfigJson:
                    if nodes["config_name"]==entry:
                        if not self.send_configuration(node_name, http_method):
                            failed_nodes.append(node_name)
            elif config_req_type == "set-config" and http_method == "PATCH":
                logger.info("I was here inside configure_nodes. (PATCH) ", extra=self.Logger.log_extras())
                for node in node_list:
                    if node == node_name:
                        if not self.send_configuration(node_name, http_method):
                            failed_nodes.append(node_name)
            elif config_req_type == "get-config" :
                if not self.get_configuration(node_name):
                    failed_nodes.append(node_name)
        logger.info('Attempt 1 -> ' + str(failed_nodes), extra=self.Logger.log_extras())
        # Retry for the failed nodes until all nodes receive the configuration
        max_retries = 3  # You can adjust the number of retries as per your requirement
        for retry in range(max_retries):
            if not failed_nodes:
                # All nodes received the configuration, exit the loop
                break
            logger.info('Attempt 2 -> ' + str(failed_nodes), extra=self.Logger.log_extras())
            # Retry for the failed nodes
            new_failed_nodes = []
            for node_name in failed_nodes:
                if config_req_type == "set-config" :
                    if not self.send_configuration(node_name, http_method):
                        new_failed_nodes.append(node_name)
                elif config_req_type == "get-config" :
                    if not self.get_configuration(node_name):
                        new_failed_nodes.append(node_name)
            logger.info('Attempt 3 -> ' + str(failed_nodes), extra=self.Logger.log_extras())
            failed_nodes = new_failed_nodes

        # different for PATCH?



            if not failed_nodes:
                # All nodes received the configuration after retry, exit the loop
                break

        if failed_nodes:
            self.get_logger().error(f"The list of nodes whose configuration failed to load is: {failed_nodes}")
            return (f"The list of nodes whose configuration failed to load is: {failed_nodes}")
        
        return True

    # subscription for topic 'session_event_update'
    def create_subscriber_session_even_update(self):
        self.subscriber = self.create_subscription(
            SessionEvent,
            'session_event_update',
            self.listener_session_event_update,
            10)
        self.subscriber  # prevent unused variable warning
    
    def listener_session_event_update(self, msg): 
        # self.get_logger().info('Received session event update: "%s"' % msg.data)
        try:
            updates = json.dumps(msg)
            self.session_state = updates.get("session_state", "")
            self.event_status = updates.get("event_status", "")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode session_event_update message: {e}")


@app.get("/cms/v2/rcs/get-rcs-config")
async def handleDynamicGetParameters():
    '''
    handles the get request
    '''
    try:
        logger.info("Received request - get configurations from all modules",extra=Logger.log_extras())
        response = {}
        global getConfigJson, configVersion
        res = rcsCmsNode.configure_nodes("get-config")
        paramConfigArray = nested_json_to_array_of_json(getConfigJson)

        if res == True:
            response = {
                "status": True,
                "data": {
                            "config": paramConfigArray,
                            "config_version": configVersion
                        },
            }
            return JSONResponse(content=response,status_code = 200)
        else:
            response = {
                "status": False,
                "message": res
            }
            logger.info("Could not get the parameters ", extra=Logger.log_extras())

            return JSONResponse(content=response,status_code = 400)
    except Exception as e:
        response = {
                 "status": False,
                 "message": "Internal server error",
                 "error": str(e)
                 }
        return JSONResponse(content=response,status_code = 500)

def nested_json_to_array_of_json (param_json):
    param_array = [] 
    data: dict = {}
    for node_name, node_data in param_json.items():
        for config_name, config_data in node_data.items():
            config_entry = next(
                (entry for entry in param_array if entry["config_name"] == config_name),
                None,
            )

            if config_entry:
                config_entry["nodes"].append(node_name)
            else:
                config_entry = {
                    "config_name": config_name,
                    "config": {
                        "static": config_data.get("static", {}),
                        "dynamic": config_data.get("dynamic", {}),
                        "adaptive": config_data.get("adaptive", {})
                    },
                    "nodes": [node_name],
                }
                param_array.append(config_entry)

    return param_array


@app.get("/health")
async def health_check():
    if (rclpy.ok()):
        response = {"message": "RCS Config Manager is healthy"}
        status_code = 200
    else:
        response = {"message": "ROS2 is not healthy"}
        status_code = 500
    return JSONResponse(status_code=status_code, content=response)
    
@app.put("/RCS/v2/set-rcs-config")
# handles the PUT request
def handleDynamicUpdatePut(reqData: models.Config, zone_id: Optional[str] = Header(default="Zone_1", alias="zone_id")):
    logger.info("In PUT method: session_state = "+str(rcsCmsNode.session_state)+", event_status = "+str(rcsCmsNode.event_status), extra=Logger.log_extras())
    if (rcsCmsNode.session_state == "init" and rcsCmsNode.event_status == "ack") or (rcsCmsNode.session_state == "end" and rcsCmsNode.event_status == "success"):
        try:
            if configJson["ZONE_ID"] != zone_id:
                response = {
                    "status": False,
                    "message": "Zone id mismatch"
                }
                logger.info("Zone id mismatch.", extra=Logger.log_extras())
                return JSONResponse(content=response,status_code = 400)
            logger.info("Received request - set configurations to all modules",extra=Logger.log_extras())
            epoch_timestamp_s = int(reqData.timestamp) / 1000
            current_datetime = datetime.fromtimestamp(epoch_timestamp_s)
            formatted_datetime = current_datetime.strftime('%Y-%m-%d %H:%M:%S')
            logger.info("New configs received at: " + formatted_datetime,extra=Logger.log_extras())

            global dynamicConfigJson, paramConfigJson, updatedConfigJson, configVersion, activeLayout, paramtersInitialized
            configVersion = reqData.dict().get('config_version', '')
            activeLayout = reqData.dict().get('active_layout_id', -1)

            dynamicConfigJson = paramConfigJson

            # updatedConfigJson = array_of_json_to_nested_json(reqData.dict()["config"])
            updatedConfigJson = reqData.config
            # dynamicConfigJson = update_parameter_json(dynamicConfigJson, updatedConfigJson)

            logger.info("DynamicConfigJson: "+json.dumps(dynamicConfigJson, indent=4), extra=Logger.log_extras())
            
            global nodes_list, responseTimeout

            # nodes_list = dynamicConfigJson["RCS_CMS_Node"]["rcs_config_manager"]["rcs_config_manager"]["node_list"]
            # nodes_list = dynamicConfigJson["RCS_CMS_Node"]["node"]
            # responseTimeout = dynamicConfigJson["RCS_CMS_Node"]["rcs_config_manager"]["rcs_config_manager"]["response_time_for_nodes_to_set_configurations"]

            res = rcsCmsNode.configure_nodes("set-config","PUT")

            saveYaml(configJson['cbsDataLog'], reqData.dict())

            if res == True:
                response = {
                    "status": True,
                    "message": "Configuration Updated Successfully"
                }
            else:
                response = {
                    "status": False,
                    "message": "Could not send the parameters."
                }
                return JSONResponse(content=response,status_code = 400)
        except Exception as e:
            response = {
                "status": False,
                "message": "Internal server error",
                "error": str(e)
            }
    else:
        response = {
            "status": False,
            "message": "Session state is not valid to update the config",
            # "error": str(e)
        }
    return JSONResponse(content=response, status_code=200)

@app.patch("/RCS/v2/set-rcs-config")
# handles the PATCH request
def handleDynamicUpdatePatch(reqData: models.Config, zone_id: Optional[str] = Header(default="Zone_1", alias="zone_id")):
    logger.info("In PATCH method: session_state = "+str(rcsCmsNode.session_state)+", event_status = "+str(rcsCmsNode.event_status), extra=Logger.log_extras())
    if not (rcsCmsNode.session_state == "init" and rcsCmsNode.event_status == "ack") and not (rcsCmsNode.session_state == "end" and rcsCmsNode.event_status == "success"): 
        try:
            logger.info("configJson[ZONE_ID]: "+str(configJson["ZONE_ID"])+" zone_id: "+str(zone_id), extra=Logger.log_extras())
            if configJson["ZONE_ID"] != zone_id:
                response = {
                    "status": False,
                    "message": "Zone id mismatch"
                }
                logger.info("Zone id mismatch.", extra=Logger.log_extras())
                return JSONResponse(content=response,status_code = 400)
            logger.info("Received request - set configurations to all modules",extra=Logger.log_extras())
            epoch_timestamp_s = int(reqData.timestamp) / 1000
            current_datetime = datetime.fromtimestamp(epoch_timestamp_s)
            formatted_datetime = current_datetime.strftime('%Y-%m-%d %H:%M:%S')
            logger.info("New configs received at: " + formatted_datetime,extra=Logger.log_extras())
            # logger.info("reqData type: "+type(reqData), extra=Logger.log_extras())
            # logger.info("reqData: "+str(reqData.config[0].get('config').get('dynamic')), extra=Logger.log_extras())
            global dynamicConfigJson, paramConfigJson, updatedConfigJson, configVersion, activeLayout, paramtersInitialized
            configVersion = reqData.dict().get('config_version', '')
            activeLayout = reqData.dict().get('active_layout_id', -1)
            logger.info("I was here 1.", extra=Logger.log_extras())
            dynamicConfigJson = paramConfigJson

            # updatedConfigJson = array_of_json_to_nested_json_single_node(reqData.dict().get("config",{}))
            updatedConfigJson = reqData.config[0].get('config').get('dynamic')
            logger.info("updatedConfigJson: "+json.dumps(updatedConfigJson,indent=4), extra=Logger.log_extras())
            # updatedConfigJson = paramConfigJson
            # dynamicConfigJson = update_parameter_json(dynamicConfigJson, updatedConfigJson)
            logger.info("I was here 2.", extra=Logger.log_extras())
            global nodes_list, responseTimeout

            # nodes_list = dynamicConfigJson["RCS_CMS_Node"]["rcs_config_manager"]["rcs_config_manager"]["node_list"]
            patch_node = [reqData.config[0].get('config_name')]

            res = rcsCmsNode.configure_nodes("set-config",patch_node,"PATCH")

            saveYaml(configJson['cbsDataLog'], reqData.dict())

            if res == True:
                response = {
                    "status": True,
                    "message": "Configuration Updated Successfully"
                }
            else:
                response = {
                    "status": False,
                    "message": "Could not send the parameters."
                }
                return JSONResponse(content=response,status_code = 400)
        except Exception as e:
            response = {
                "status": False,
                "message": "Internal server error",
                "error": str(e)
            }
    else:
        response = {
            "status": False,
            "message": "Session state is not valid to update the config",
            # "error": str(e)
        }
    return JSONResponse(content=response, status_code=200)

def array_of_json_to_nested_json(param_array):
    param_json = {}
    for entry in param_array :
        for node in entry["nodes"]:
            if node not in param_json:
                param_json[node] = {}
            param_json[node][entry["config_name"]] = entry["config"]
    # logger.info("param_json: "+str(param_json), extra=Logger.log_extras())
    return param_json

def array_of_json_to_nested_json_single_node(param_array):
    param_json = {}
    param_json = param_array.get('config',{})[0]

    return param_json

def update_parameter_json(old_json, new_json):

    for entry in new_json:
        if entry in old_json:
            old_json[entry].update(new_json[entry])
        else:
            old_json[entry] = new_json[entry]
    return old_json

# @retry(stop=stop_never, wait=wait_fixed(1), reraise=True)
# @retry(stop=stop_after_attempt(configJson['CBS_URLs']['getFileData']), 
#        wait=wait_exponential(multiplier=configJson['retryAPI']['multiplier'], 
#                              min=configJson['retryAPI']['min'], 
#                              max=configJson['retryAPI']['max']), 
#        reraise=True)

@retry(stop=stop_after_attempt(20),
       wait=wait_exponential(multiplier=2, min=20, max=80),
       reraise=True)
def getAllFilesData():
    '''
    It gets the data for all files at once from cbs and loads it to the nodes filewise
    '''
    try:
        logger.info(f"Retry Timestamp: {(datetime.now()).strftime(time_format)}\n",extra=Logger.log_extras())
        url = Base_URL+str(configJson['CBS_URLs']['getFileData'])
        response = requests.get(url=url, headers = {"zone_id": configJson["ZONE_ID"]})
        data = response.json()
        # print(json.dumps(data, indent=4))
        # logger.info("raw_json:"+json.dumps(data, indent=4), extra=Logger.log_extras())
        logger.info("Sending request for system configurations",extra=Logger.log_extras())
        if response.ok:
            date = (datetime.now()).strftime(date_format)
            timestamp=(datetime.now()).strftime(time_format)
            logger.info("Received system configurations",extra=Logger.log_extras())
            global paramConfigJson, configVersion, activeLayout, activeProfile, updatedConfigJson, dynamicConfigJson

            systemParamConfigJson = json.dumps(response.json()[0])
            try :
                for attempts in Retrying(stop=stop_after_attempt(3),wait=wait_exponential(multiplier=2, min=2, max=10), retry=(retry_if_exception_type(AssertionError))):
                    with attempts:

                        url = Base_URL +str(configJson['CBS_URLs']['getErrorData'])
                        response = requests.get(url=url, headers = {"ZONE_ID": configJson["ZONE_ID"]})
                        logger.info("Sending request for error configurations",extra=Logger.log_extras())

                        if response.ok:
                            logger.info("Received error configurations",extra=Logger.log_extras())
                            errorParamConfigJson = response.json()
                            errorParamConfigJson = formatErrorConfigJson(errorParamConfigJson)
                            systemParamConfigJson = json.loads(systemParamConfigJson)

                            systemParamConfigJson["data"].append(errorParamConfigJson) 
                            # logger.info("Final systemParamConfigJson: "+str(systemParamConfigJson), extra=Logger.log_extras())
                            paramConfigJson = json.dumps(systemParamConfigJson)

                            saveYaml(configJson['cbsDataLog'], paramConfigJson)

                            paramConfigJson = json.loads(paramConfigJson)
                            paramConfigJson = array_of_json_to_nested_json((paramConfigJson["data"]))
                            # paramConfigJson = paramConfigJson["data"]


                            configVersion = systemParamConfigJson.get('config_version', '')
                            activeLayout = systemParamConfigJson.get('active_layout_id', -1)
                            # logger.info("systemParamConfigJson: "+json.dumps(systemParamConfigJson,indent=4), extra=Logger.log_extras())

                            updatedConfigJson = paramConfigJson
                            dynamicConfigJson = paramConfigJson
                            
                            global nodes_list, responseTimeout

                            # logger.info("dynamicConfigJson: "+json.dumps(dynamicConfigJson,indent=4), extra=Logger.log_extras())

                            # nodes_list = dynamicConfigJson["RCS_CMS_Node"]["rcs_config_manager"]["rcs_config_manager"]["node_list"]
                            # for nodes in systemParamConfigJson["data"][0]:
                            #     if nodes["config_name"] == "RCS_CMS_Node":
                            #         nodes_list = nodes["nodes"]
                            # logger.info("Nodes list: "+str(nodes_list), extra=Logger.log_extras())
                            for nodes in systemParamConfigJson.get("data"):
                                if nodes["config_name"] == "RCS_CMS_Node":
                                    nodes_list = nodes["nodes"]
                            logger.info("Nodes list: "+str(nodes_list), extra=Logger.log_extras())
                            # responseTimeout = dynamicConfigJson["RCS_CMS_Node"]["rcs_config_manager"]["rcs_config_manager"]["response_time_for_nodes_to_set_configurations"]
                            responseTimeout = 1

                            # res = rcsCmsNode.configure_nodes("set-config")

                            return 1
                        
                        else:
                            logger.error("No success response to get the error data ", extra=Logger.log_extras())

                        assert response.ok

            except Exception as e:
                logger.error("Could not retrieve the error config data received  " + str(e), extra=Logger.log_extras())
       
        else:
            logger.error("No success response to get the data of all files", extra=Logger.log_extras())
    except Exception as e:
        logger.error("CBS not up or could not retrieve the data received  " + str(e), extra=Logger.log_extras())
        raise

def formatErrorConfigJson(errorConfigJson):
    error_handling = errorConfigJson["data"]
    node_list = errorConfigJson["product"]["RCS"]["node_list"]
    errorConfigJson = {
        "config_name": "error_handling",
        "config": {
            "error_handling": error_handling
        },
        "nodes": node_list
    }
    return errorConfigJson

def saveYaml(dirPath,jsonData,date=None,timestamp=None):
    '''
    Arguments passed : 
        dirPath: str -> The path of log directory
        configName: str -> name of config file
        jsonData: dict -> data to be saved in the YAML file
        timestamp: str
    Based on the directory structure of the log file specified in config.json , it saves the data accordingly
    '''
    if not timestamp:
        timestamp = (datetime.now()).strftime(time_format)

    if not date:
        date = (datetime.now()).strftime(date_format)

    if configJson['filenameDir']:
        absDirPath = checkDirExists(os.path.join(srcDir,dirPath,date+'_'+timestamp),True)
        paramFilepath = os.path.join(absDirPath,date+'_'+timestamp+".yaml")
    else:
        absDirPath = checkDirExists(os.path.join(srcDir,dirPath,date+'_'+timestamp),True)
        paramFilepath = os.path.join(absDirPath, date+'_'+timestamp+".yaml")
    try:
        yamlFile = open(paramFilepath, "w")
        yaml.dump(jsonData,yamlFile)
        yamlFile.close()
        logger.info("Yaml file saved successfully "+str(paramFilepath), extra=Logger.log_extras())
        return paramFilepath
    except :
        logger.error("Yaml file could not be saved "+str(paramFilepath), extra=Logger.log_extras())
        return None

def checkDirExists(relativeDirPath, createDir):
    ''''
    Arguments passed : 
        createDir : bool 
        relativeDirPath: str

        if createDir = False 
            it returns a boolean whether file exists or not
        else
            it creates the directory structure for the path passed and returns the absolute path
    '''

    dirPath = os.path.join(srcDir,relativeDirPath)
    fileExists = os.path.exists(dirPath)
    if createDir and not fileExists:
        os.makedirs(dirPath)
    elif not createDir and fileExists:
        return True
    elif not createDir and not fileExists:
        return False
    return dirPath

def loadDefaultSystemConfig():
    # This function is used when the system is not able to load the config.json file
    global configJson
    configJson = {
    "fastApi":{
        "host":"localhost",
        "port":8000
    },
    "logFileDir":"log/system/",
    "cbsDataLog":"log/cbsData",
    "loadParamFiles":"log/config",
    "maxThreads":20,
    "threadTimeout":200,
    "retryTimer":10,
    "filenameDir":True,
    "CBS_URLs":{
        "baseUrl":"http://localhost:8001/",
        "getFileNamesEndpoint":"get-config-list",
        "getFileData":"/cms/v2/rcs/get-rcs-config",
        "getErrorData": "/cms/v2/error/fetch-errors-list"
    },
    "retryAPI": {
        "stop_after_attempt" : 90,
        "multiplier": 2,
        "min": 20,
        "max": 80
    },
    "ZONE_ID": "-1"
}

def handle_shutdown(signum,frame):
    logger.error("Received stop signal. Shutting the Config manager", extra=Logger.log_extras())
    rclpy.shutdown()

def main(args=None):

    try :
        global app, rcsCmsNode,configJson, paramtersInitialized
        signal.signal(signal.SIGINT, handle_shutdown)
        logger.info('Initialising the system', extra=Logger.log_extras())
        try:
            loadConfig()
            logger.info("Loaded system config", extra=Logger.log_extras())
        except:
            loadDefaultSystemConfig()
            logger.error('Could not load config so default config loaded', extra=Logger.log_extras())

        uvicorn_thread = threading.Thread(target=lambda: uvicorn.run(app, host=configJson['fastApi']['host'], port=configJson['fastApi']['port'], log_level="info"), daemon=True)
        uvicorn_thread.start()
        logger.debug ("Uvicorn api thread is created" ,extra=Logger.log_extras())

        rclpy.init(args=args)

        rcsCmsNode = RCS_CMS_Node()
        rcsCmsNode.create_subscriber_session_even_update() # background subscriber for topic 'session_event_update'
        ros2_node_thread = threading.Thread(target=rclpy.spin, args=(rcsCmsNode, ), daemon=True)
        ros2_node_thread.start()
        logger.debug ("ROS2 node thread is created",extra=Logger.log_extras() )

        try :
            res = getAllFilesData()
            if res == 1:
                paramtersInitialized = True
                rcsCmsNode.createService() # this is happening after the init (maybe we do not need this)
            else:
                logger.info("Result of get all file data "+str(res), extra=Logger.log_extras())
        except Exception as error :
            logger.error ('Exception occurred in getAllFilesData - main -> '+ str(error), extra=Logger.log_extras())
       
        rcsCmsNode.spin()
        rcsCmsNode.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        ros2_node_thread.join()
        uvicorn_thread.join()
    
    except Exception as e :
        logger.error('Exception occurred in main -> '+ str(e), extra=Logger.log_extras())


if __name__ == "__main__":
    main()



# runBashScript -> 1 -> All the parameters set to node
# runBashScript -> 2 -> Node not found
# runBashScript -> 3 -> Some error while loading the parameters to node 

# setParameters -> 4 -> Could not save load_YAML file

# getAllFilesData -> 5 -> CBS not up or could not retrieve the data received
# getAllFilesData -> 6 -> No success response to get the data of all files
# getAllFilesData -> 7 -> Could not save the received data
