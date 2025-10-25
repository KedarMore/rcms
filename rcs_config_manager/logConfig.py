import os
import logging
import logging.config
ROOT_DIR = os.getcwd()
logFileDir = "log/system/"
filename ={
    'DEBUG':'debug.log',
    'INFO':'info.log',
    'WARN':'warn.log',
    'ERROR':'error.log'
}




def configureLogger(path):
    
    path = os.path.join(ROOT_DIR, logFileDir)

    LOGGING = {
        'version':1,
        'disable_existing_loggers':False,
        'formatters':{
            'simpleRe':{
                # 'format':'{levelname} [{asctime}] {module} {process:d} {thread:d} {message}',
                'format': '%(levelname)3s [%(asctime)s] %(process)s [%(threadName)s] %(message)s',
                'style':'%',
            }
        },
        'handlers':{
            'file':{
                'level':'DEBUG',
                'class':'logging.FileHandler',
                'filename':os.path.join(path, filename['DEBUG']),
                'formatter':'simpleRe',
            },
            'file1':{
                'level':'INFO',
                'class':'logging.FileHandler',
                'filename':os.path.join(path, filename['INFO']),
                'formatter':'simpleRe',
            },
            'file2':{
                'level':'WARN',
                'class':'logging.FileHandler',
                'filename':os.path.join(path, filename['WARN']),
                'formatter':'simpleRe',
            },
            'file3':{
                'level':'ERROR',
                'class':'logging.FileHandler',
                'filename':os.path.join(path, filename['ERROR']),
                'formatter':'simpleRe',
            },
        },
        'loggers':{
            'cmsNode':{
                'handlers':['file','file1','file2','file3'],
                'level':'DEBUG'
            }
            # 'fast_api':{
            #     'handlers':['file4'],
            #     'level':'DEBUG'
            # }
        }
    }


    if not os.path.exists(path):
        os.makedirs(path)
    logging.config.dictConfig(LOGGING,)
    logger = logging.getLogger(__name__)
    logger.debug("Configured logging")
    print("Configured logging")

