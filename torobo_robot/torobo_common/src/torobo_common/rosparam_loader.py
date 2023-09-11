import rospy
import collections

def get_controller_list(controller_list_param_name="controller_list", sorting=True):
    controller_list = rospy.get_param(controller_list_param_name, None)
    if(sorting):
        controller_list.sort()
    return controller_list

def get_torobo_controller_names(torobo_controller_config_param_name="torobo_controller_config", sorting=True):
    names = []
    configs = rospy.get_param(torobo_controller_config_param_name, None)
    if (configs is None):
        return names
    for config in configs:
        names.append(config)
    if(sorting):
        names.sort()
    return names

def get_controller_list_from_torobo_controller_config(torobo_controller_name, torobo_controller_config_param_name="torobo_controller_config", sorting=True):
    controller_list = []
    configs = rospy.get_param(torobo_controller_config_param_name, None)
    if (configs is None):
        return controller_list
    if(torobo_controller_name in configs):
        v = configs[torobo_controller_name]
        for l in v:
            if "controller_name" in l:
                controller_list.append(l["controller_name"])
    if(sorting):
        controller_list.sort()
    return controller_list

def get_joint_names(controller_param_name):
    joint_names = rospy.get_param(controller_param_name + "/joints", [])
    if joint_names == []:
        joint_names = rospy.get_param(controller_param_name + "/joint", [])
    if type(joint_names) == str:
        joint_names = [joint_names]
    return joint_names

def get_controller_name_joint_names_dict(controller_list_param_name="controller_list", sorting=True):
    dic = collections.OrderedDict()
    controller_list = get_controller_list(controller_list_param_name, sorting)
    for controller in controller_list:
        joint_names = get_joint_names(controller)
        if(joint_names == []):
            continue
        dic[controller] = get_joint_names(controller)
    return dic

if __name__ == '__main__':
    rospy.init_node("rosparam_loader_node", anonymous=True)
    print(get_controller_list())
    torobo_controller_names = get_torobo_controller_names()
    print(torobo_controller_names)
    for torobo_controller_name in torobo_controller_names:
        controller_list = get_controller_list_from_torobo_controller_config(torobo_controller_name)
        for controller in controller_list:
            print(controller, get_joint_names(controller))
