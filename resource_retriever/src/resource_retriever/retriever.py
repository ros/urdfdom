import roslib; roslib.load_manifest('resource_retriever2')
import rospy
import subprocess
import urlgrabber, string

PACKAGE_PREFIX = 'package://'

def rospack_find(package):
    process = subprocess.Popen(['rospack', 'find', package], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (stdout, stderr) = process.communicate()
    if len(stderr) > 0:
        raise Exception(stderr)
    else:
        return string.strip(stdout)

def get(url):
    mod_url = url
    if url.find(PACKAGE_PREFIX) == 0:
        mod_url = url[len(PACKAGE_PREFIX):]
        pos = mod_url.find('/')
        if pos == -1:
            raise Exception("Could not parse package:// format into file:// format for "+url)

        package = mod_url[0:pos]
        mod_url = mod_url[pos:]
        package_path = rospack_find(package)

        mod_url = "file://" + package_path + mod_url;

    return urlgrabber.urlopen(mod_url)	

