import datetime
import glob
import os
from datetime import datetime
date_format = '%d-%m-%Y'
time_format = '%H:%M:%S'

configJson = {

        "fastApi":{
            "host":"localhost",
            "port":8000
        },
        "cbsDataLog":"log/cbsData",
        "loadParamFiles":"log/loadParams",
        "logFileDir":"log/system/",
        "filenameDir":False,
        "CBS_URLs":{
            "baseUrl":"http://localhost:8001/",
            "getFileNamesEndpoint":"get-config-list",
            "getFileData":"get-config"
        }
    }
srcDir = '/home/shreya/unbox_ws/src/rcs_config_manager/'

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
    print(srcDir,"srcDir")
    dirPath = os.path.join(srcDir,relativeDirPath)
    print("here dir path",dirPath)
    fileExists = os.path.exists(dirPath)
    if createDir and not fileExists:
        os.makedirs(dirPath)
    elif not createDir and fileExists:
        return True
    elif not createDir and not fileExists:
        return False
    return dirPath
def getRecentFile(baseDirPath,filename):
    '''
    Arguments passed :
        filename: str -> name of the config file

        This function gives the absolute path of the latest log file saved for the config 
        If no such file exists, the function will return None
    '''
    date = (datetime.now()).strftime("%d-%m-%Y")
    print("baseDirPath",baseDirPath)
    print("filename",filename)
    if configJson['filenameDir']: 
        dirPath=os.path.join(baseDirPath,date,filename)
    else:
        dirPath=os.path.join(baseDirPath,date)
    print("dirPath",dirPath)
    if checkDirExists(dirPath,False):
        # if configJson['filenameDir']: 
        #     print('filename/timestamp.yaml')
        #     fileType = r'/*yaml'
        # else:
        #     print(' filename_timestamp.yaml')
        #     fileType = '/'+filename+r'_*.yaml'
        # print("File type "+str(fileType))
        # print("File type ",fileType)
        # try:  
        #     files = glob.glob(dirPath + fileType)
        #     print('List of files: '+str(files))
        #     print('List of files: '+str(files))
        #     if not files:
        #         return None
        # except Exception as e:
        #     print('Getting exception here '+str(e))
        # result = max(files, key=os.path.getctime)
        # return result


        if configJson["filenameDir"]:
            print("In the if")
            list_of_files = os.listdir(dirPath)
            print(list_of_files)

            if list_of_files :
                min_timestamp = datetime.strptime((list_of_files[0].replace('.yaml','')),time_format)
                recentFile = list_of_files[0]
                print(min_timestamp,"min timestamp")
                for filename_itr in list_of_files:
                    timestamp_ = datetime.strptime((filename_itr.replace('.yaml','')),time_format)
                    print("timestamp",timestamp_)
                    if timestamp_ > min_timestamp:
                        min_timestamp = timestamp_
                        recentFile = filename_itr
                        print(min_timestamp,"min timestamp")
            print(min_timestamp,"min timestamp")
            print(recentFile,"RecentFile")
        else:
            print("In the else")
            list_of_files = os.listdir(dirPath)
            print(list_of_files)

            if list_of_files :
                min_timestamp = datetime.strptime((list_of_files[0].replace('.yaml','').split('_'))[-1],time_format)
                recentFile = None
                print(min_timestamp,"min timestamp")
                for filename_itr in list_of_files:
                    if filename in filename_itr:
                        timestamp_ = datetime.strptime((filename_itr.replace('.yaml','').split('_')[-1]),time_format)
                        print("timestamp",timestamp_)
                        if timestamp_ > min_timestamp:
                            min_timestamp = timestamp_
                            recentFile = filename_itr
                            print(min_timestamp,"min timestamp")
                print(min_timestamp,"min timestamp")
                print(recentFile,"RecentFile")               


    else:
        print("The directory does not exist to get configs")
        return None


# print((datetime.date()).strftime("%d-%m-%Y"))
if __name__ == '__main__':
    getRecentFile("log/cbsData","allConfigData")
