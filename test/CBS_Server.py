from fastapi import FastAPI
import uvicorn
import os
import yaml
import json


app = FastAPI()

@app.get('/get-config-list')
def get_parameters():
    print("received a request to get list of data")
    file_names = [
        'bin_vs_pincode' ,
        'bin_vs_height', 
        # 'grid_map.yaml', 
        # 'feeder_data.yaml', 
        # 'init_pose.yaml', 
        # 'load_unload_delay.yaml', 
        # 'map_synthesis.yaml', 
        # 'em.yaml', 
        # 'queue_data.yaml', 
        # 'robot_info.yaml', 
        # 'system_params.yaml', 
        # 'grid_map_nav.yaml', 
        # 'rejection_trolleys.yaml', 
        # 'binID_vs_binName.yaml', 
        # 'rack_data.yaml', 
        # 'xy_vs_id.yaml'
    ]
    return ({'filenames':file_names},)


@app.get('/cms/v2/rcs/get-rcs-config')
def get_parameters():
    print("received a request to get the data of all files")
    # dirPath = '/home/unbox/unbox_ws/src/rcs-config-manager/src/testing/testing/configs'
    # print(os.getcwd())
    # print(Path(__file__).resolve().parent.parent)
    dirPath = os.path.join(os.path.dirname(__file__),'cms_config')
    # print(os.path.dirname(__file__))
    print("dirPath",dirPath)
    fileData = {}
    for file_name in os.listdir(dirPath):
        file = open(os.path.join(dirPath,file_name), 'r')
        configuration = yaml.safe_load(file)
        fileData[file_name] = configuration

    response_data = {
        "status": True,
        "data":[]
    }

    for filename in fileData:
        # if filename == 'xy_vs_id.yaml':
        #     json_data = {
        #         "file_name": filename,
        #         "config":{
        #             "xy_vs_id":fileData[filename]
        #         },
        #         "nodes": [
        #                 "/stateManager","Test_Node2"
        #             ]
        #     }
        # else:
        filename_ = filename.split('.')[0]
        json_data = {
            "config_name": filename_,
            "config":fileData[filename],
            "nodes": [
                    "path_planner_node","ecs_node","client_path_planner_node","execution_manager_node","turn_safety_node"
                ]
        }
        response_data['data'].append(json_data)

    # response_data = {
    #     "status":True,
    #     "data":[
    #         {
    #             "file_name": "map_synthesis",
    #             "config": {
    #                 "map_synthesis": {
    #                 "dump_grids_direction": "vertical",
    #                 "dump_in_reverse": True,
    #                 "grid_size": 0.6,
    #                 "height": 36,
    #                 "offset": 0.3,
    #                 "width": 16
    #                 }
    #             },
    #             "nodes": [
    #                 "scriptA","Test_Node2"
    #             ]
    #         },
    #         {
    #             "file_name": "queue_data",
    #             "config": {
    #                 "queue_data": {
    #                 "dump_grids_dir": "vertical",
    #                 "dump_in_rev": True,
    #                 "g_size": 0.6,
    #                 "height_": 36,
    #                 "offset_": 0.3,
    #                 "width_": 16
    #                 }
    #             },
    #             "nodes": [
    #                 "Test_Node1","Test_Node2"
    #             ]
    #         }

    #     ]
        
    # }
    return response_data

# @app.get('/get-config')
# def get_parameters_data(file_name:str):
#     print("received a request to get the data of a file "+str(file_name))
#     response_data = {
#         "status":True,
#         "data":
#             {
#                 "file_name": file_name,
#                 "config": {
#                     "map_synthesis": {
#                     "dump_grids_direction": "vertical",
#                     "dump_in_reverse": True,
#                     "grid_size": 0.6,
#                     "height": 36,
#                     "offset": 0.3,
#                     "width": 16
#                     }
#                 },
#                 "nodes": [
#                     "Test_Node1","Test_Node2"
#                 ]
#             }
#         }

#     return response_data

 



if __name__ == '__main__':
    uvicorn.run(app=app,host='0.0.0.0',port=8001)






