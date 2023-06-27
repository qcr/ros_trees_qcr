import rospy
import yaml

from ros_trees.leaves_ros import SubscriberLeaf, ServiceLeaf
from armer_msgs.msg import ManipulatorState

### --------------------------------  Subscriber Leafs ----------------------------------- ###
class IsErrored(SubscriberLeaf):

    def __init__(self,
                 name='Checking arm for errors',
                 timeout=2,
                 topic_name='/arm/state',
                 topic_class=ManipulatorState,
                 result_fn=None,
                 *args,
                 **kwargs):
        super(IsErrored, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_class=topic_class,
            result_fn=result_fn if result_fn else self.result_fn,
            *args,
            **kwargs)
        self.timeout = timeout
        self.ts = None

    def result_fn(self):
        if (self._cached_data.errors and
                ManipulatorState.ESTOP == ManipulatorState.ESTOP):
            self.ts = None
            return True

        if self._cached_data.errors > 0:
            if self.ts == None:
                self.ts = rospy.get_time()

            return rospy.get_time() - self.ts > self.timeout

        self.ts = None
        return False


class IsContacting(SubscriberLeaf):

    def __init__(self,
                 name='Is Contacting',
                 topic_name='/arm/state',
                 topic_class=ManipulatorState,
                 result_fn=None,
                 *args,
                 **kwargs):
        super(IsContacting, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_class=topic_class,
            result_fn=result_fn if result_fn else self.result_fn,
            *args,
            **kwargs)

    def result_fn(self):
        return any(self._cached_data.cartesian_contact)


class GetEEPose(SubscriberLeaf):

    def __init__(self,
                 name='Get EE Pose',
                 topic_name='/arm/state',
                 topic_class=ManipulatorState,
                 result_fn=None,
                 *args,
                 **kwargs):
        super(GetEEPose,
              self).__init__(name=name,
                             topic_name=topic_name,
                             topic_class=topic_class,
                             result_fn=lambda leaf: leaf._cached_data.ee_pose
                             if leaf._cached_data else None,
                             *args,
                             **kwargs)


class GetJointPoses(SubscriberLeaf):

    def __init__(self,
                 name='Get EE Pose',
                 topic_name='/arm/state',
                 topic_class=ManipulatorState,
                 result_fn=None,
                 *args,
                 **kwargs):
        super(GetJointPoses, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_class=topic_class,
            result_fn=lambda leaf: leaf._cached_data.joint_poses
            if leaf._cached_data else None,
            *args,
            **kwargs)
        
######### ADDED BY DG [TESTING]
class RobotWorkspaceCheck(SubscriberLeaf):
    """
    This Leaf Checks (Emergency) Workspace Bound Breaks
    - Returns True if a bound break has occurred! (otherwise False)
    """
    def __init__(self, 
        name="Robot State Query Leaf", 
        topic_name="/arm/state", 
        topic_class=ManipulatorState, 
        expiry_time=None, 
        timeout=3, 
        param_name=None,
        config_path=None,
        save=False,
        *args, **kwargs):
        super().__init__(name=name, 
            topic_name=topic_name, 
            topic_class=topic_class, 
            expiry_time=expiry_time, 
            timeout=timeout, 
            save=save, 
            eval_fn=self.eval_fn,
            result_fn=self.result_fn,
            *args, **kwargs)
        self.name = name
        self.current_ee_pose = None
        self.param_name=param_name
        self.config_path=config_path

    def callback(self, msg):
        self.current_ee_pose = msg.ee_pose.pose
        return super().callback(msg)

    def result_fn(self):
        # Take current end-effector pose from callback and save as a pose object
        return self.current_ee_pose

    def eval_fn(self, state):
        """
        Override of the default eval_fn 
        - Used to check arm state and apply different functions
        """
        ee_pos_x = state.position.x
        ee_pos_y = state.position.y
        ee_pos_z = state.position.z

        path = rospy.get_param(self.param_name, self.config_path)
        with open(path, 'r') as handle:
            config = yaml.load(handle, Loader=yaml.SafeLoader)

        workspace = config['workspace'] if 'workspace' in config else None

        if workspace == None:
            rospy.logerr(f"[{self.name}] WORKSPACE COULD NOT BE DEFINED! Please check the {self.config_path} file")
            return False

        min_x = workspace[0]['min'][0]['x']
        min_y = workspace[0]['min'][0]['y']
        min_z = workspace[0]['min'][0]['z']

        max_x = workspace[1]['max'][0]['x']
        max_y = workspace[1]['max'][0]['y']
        max_z = workspace[1]['max'][0]['z']

        # Check that cartesian position of end-effector is within defined constraints. 
        # NOTE: the following bounds are based from the base_link which is the origin point. 
        # Assumed main constraint is z-axis plane (added a y-axis/End-Point) plane termination condition as well
        # NOTE: this is assuming Left-to-Right motion w.r.t Robot base_link
        # NOTE: testing change (original: 0.65)
        if(ee_pos_x <= min_x or ee_pos_x >= max_x or \
            ee_pos_y <= min_y or ee_pos_y >= max_y or \
                ee_pos_z <= min_z or ee_pos_z >= max_z):

            print(f"[{self.name}] ROBOT EXCEEDED DEFINED BOUNDARY!!!")
            # Returns TRUE to state that a boundary break occurs
            # NOTE: this should be handled with a selector to terminate the remaining sequence
            return True
        else:
            return False

### --------------------------------  Service Leafs ----------------------------------- ###
class SetCartesianImpedance(ServiceLeaf):

    def __init__(self, name='Set Impedance', *args, **kwargs):
        super(SetCartesianImpedance,
              self).__init__(name=name,
                             service_name='/arm/set_cartesian_impedance',
                             *args,
                             **kwargs)
