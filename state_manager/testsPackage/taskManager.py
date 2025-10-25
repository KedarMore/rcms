from state_manager.src.state_manager.stateManager import StateManager
import rclpy
import threading
from rclpy.node import Node
from state_manager import logStatement, logLevel

def testFunctions(object, functionName, arguments=None):
    
    if not arguments:
        functionObj = getattr(object, functionName)
        print(functionObj())
        return
    dataTypes = [1082387826, 0.45678, 'stringHere', ['thisIsArray',10,3.5], {'thisIs':'dictionary'}, ('str',2), (2,4), None]
    dataTypes = dataTypes + arguments
    functionObj = getattr(object, functionName)
    for dataType in dataTypes:
        logStatement(logLevel['info'], 'testingFor: '+ str(dataType)+' of type: '+ str(type(dataType)))
        try:
            logStatement(logLevel['info'], 'Result obtained: '+ str(functionObj(dataType)))
        except Exception as e:
            logStatement(logLevel['error'], 'Exception: '+str(e))
            print()
    
def main(args=None):
    rclpy.init(args=args)
    
    taskManager = Node(node_name='taskManager', allow_undeclared_parameters=True,)
    thread = threading.Thread(target=rclpy.spin, args=(taskManager, ), daemon=True)
    thread.start()
    # setLogger()
    stateManagerObject = StateManager(taskManager)
    rate = taskManager.create_rate(2)
    
    logStatement(logLevel['info'], 'test is_initialized')
    testFunctions(stateManagerObject, 'is_initialized')
    logStatement(logLevel['info'], 'is_initialized sucessfull')
    
    logStatement(logLevel['info'], 'test in_bounds')
    testFunctions(stateManagerObject, 'in_bounds', [(2,4)])
    logStatement(logLevel['info'], 'in_bounds sucessfull')
    
    logStatement(logLevel['info'], 'test get_robots')
    testFunctions(stateManagerObject, 'get_robots', ['queue_1'])
    logStatement(logLevel['info'], 'get_robots sucessfull')
    
    logStatement(logLevel['info'], 'test get_queue_entries')
    testFunctions(stateManagerObject, 'get_queue_entries', [1])
    logStatement(logLevel['info'], 'get_queue_entries sucessfull')
    
    logStatement(logLevel['info'], 'test get_queue_entry')
    testFunctions(stateManagerObject, 'get_queue_entry', [1])
    logStatement(logLevel['info'], 'get_queue_entry sucessfull')
    
    logStatement(logLevel['info'], 'test get_feeder_pose')
    testFunctions(stateManagerObject, 'get_feeder_pose', [1])
    logStatement(logLevel['info'], 'get_feeder_pose sucessfull')
    
    logStatement(logLevel['info'], 'test detect_angle')
    testFunctions(stateManagerObject, 'detect_angle', [(0,0,0,0),(1,0,0,0),(0,2,)])
    logStatement(logLevel['info'], 'detect_angle sucessfull')   
    try:
        while rclpy.ok():   
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    