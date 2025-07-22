import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import nav_msgs.msg
from rcl_interfaces.msg import SetParametersResult
import rcl_interfaces.msg
import rcl_interfaces.srv
import rclpy
import mavros_msgs
from rclpy.node import Node
import geometry_msgs
import sensor_msgs
import sensor_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
from .location import geodetic_to_enu, enu_to_geodetic
import numpy as np
import nav_msgs
import rcl_interfaces
from scipy.spatial.transform import Rotation as R


qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


from enum import Enum
class State(Enum):
    PULL_PARAM = 0
    CHG_PARAM = 1
    CHG_PARAM2 = 5
    
    PUSH_RALLY = 2
    CLEAR_RALLY = 4
    PUSH_WP = 3
    CLEAR_WP = 6

class BaseNode(Node):
    def __init__(self, node_name="main_node",logger_set = True):
        Node.__init__(self, node_name)
        self.logger_set = logger_set
        
        # 订阅各种服务和话题
        self.rallypoint_push_client = self.create_client(
            mavros_msgs.srv.WaypointPush, "/mavros/rallypoint/push"
        )
        self.rallypoint_clear_client = self.create_client(
            mavros_msgs.srv.WaypointClear, "/mavros/rallypoint/clear"
        )
        self.rallypoint_pull_client = self.create_client(
            mavros_msgs.srv.WaypointPull, "/mavros/rallypoint/pull"
        )
        self.state_sub = self.create_subscription(
            mavros_msgs.msg.State, "/mav"
            "ros/state", self.state_cb, qos_profile
        )
        self.arming_client = self.create_client(
            mavros_msgs.srv.CommandBool, "/mavros/cmd/arming"
        )
        self.global_position_sub = self.create_subscription(
            sensor_msgs.msg.NavSatFix,
            "/mavros/global_position/global",
            self.global_position_cb,
            qos_profile,
        )
        self.set_mode_client = self.create_client(
            mavros_msgs.srv.SetMode, "/mavros/set_mode"
        )
        self.waypoint_push_client = self.create_client(
            mavros_msgs.srv.WaypointPush, "/mavros/mission/push"
        )
        self.parameter_set_client = self.create_client(
            mavros_msgs.srv.ParamSetV2, "/mavros/param/set"
        )
        self.home_sub = self.create_subscription(
            mavros_msgs.msg.HomePosition,
            "/mavros/home_position/home",
            self.home_cb,
            qos_profile,
        )
        self.waypoint_reached_sub = self.create_subscription(
            mavros_msgs.msg.WaypointReached,
            "/mavros/mission/reached",
            self.waypoint_reached_cb,
            qos_profile,
        )
        self.local_position_sub = self.create_subscription(
            nav_msgs.msg.Odometry,
            "/mavros/global_position/local",
            self.local_position_cb,
            qos_profile,
        )
        self.parameter_pull_client = self.create_client(
            mavros_msgs.srv.ParamPull,
            "/mavros/param/pull",
        )
        
        self.parameters_set_client = self.create_client(
            rcl_interfaces.srv.SetParameters,
            "/mavros/param/set_parameters",
        )
        
        self.parameters_get_client = self.create_client(
            rcl_interfaces.srv.GetParameters,
            "/mavros/param/get_parameters",
        )
        
        self.waypoint_clear_client = self.create_client(
            mavros_msgs.srv.WaypointClear,
            "/mavros/mission/clear",
        )
        
        # 当前位置信息
        self.global_position = []
        self.local_position = []
        self.home_altitude = 0
        # 当前状态
        self.state = mavros_msgs.msg.State()
        self.rotate = None
        # home点坐标,无人机解锁位置
        self.home = None
        # 当前集结点
        self.current_rallypoint = None
        # 当前航点的数量,用于判断当前航点任务是否已经完成
        self.current_waypoint_num = None
        self.current_reached_waypoint = None
        self.new_mission = False
        
        # 拉取集结点
        self.rallypoint_new_pull_req = False
        self.rallypoint_pull_success = False
        
        # 清空集结点
        self.rallypoint_new_clear_req = False
        self.rallypoint_clear_success = False
        
        # 上传集结点 
        self.rallypoint_new_push_req = False
        self.rallypoint_push_success = False
        
        # 上传航点 
        self.waypoint_new_push_req = False
        self.waypoint_push_success = False
        
        # 清空航点
        self.waypoint_new_clear_req = False
        self.waypoint_clear_success = False
        
        # 拉取参数
        self.parameter_new_pull_req = False
        self.parameter_pull_success = False
        
        # 修改参数
        self.parameter_new_chg_req = False
        self.parameter_chg_success = False
        
        # 设置参数
        self.parameters_new_set_req = False
        self.parameters_set_success = False
        
        # 设置模式
        self.mode_new_set_req = False
        self.mode_set_success = False
        
        # 获取参数
        self.parameters_new_get_req = False
        self.parameters_get_success = False
        self.parameters_get_ret = None

class CallBackNode(BaseNode):
    def parameters_get_cb(self, response):
        result = response.result()
        self.parameters_new_get_req = False
        for val in result.values:
            if val.type == 0: 
                self.parameters_get_success = False
                if self.logger_set: self.get_logger().info("参数获取不成功")
                self.parameters_get_ret = None
                return
        self.parameters_get_success = True
        if self.logger_set: self.get_logger().info("参数获取成功")
        print("获取到参数: ", result.values)
        self.parameters_get_ret = result.values
        print(self.parameters_get_ret)
    def waypoint_clear_cb(self, response):
        if response.result().success:
            if self.logger_set: self.get_logger().info("航点清空成功")
            self.waypoint_clear_success = True
        else: 
            self.waypoint_clear_success = False
            if self.logger_set: self.get_logger().info("航点清空不成功")
        self.waypoint_new_clear_req = False
        
    def parameter_pull_cb(self, response):
        if response.result().success:
            if self.logger_set: self.get_logger().info(f"参数拉取成功, 总共有{response.result().param_received}个参数")
            self.parameter_pull_success = True
        else:
            self.parameter_pull_success = False 
            if self.logger_set: self.get_logger().info("参数拉取不成功")
        self.parameter_new_pull_req = False

    def parameter_set_cb(self, response):
        if response.result().success:
            if self.logger_set: self.get_logger().info("参数修改成功")
            self.parameter_chg_success = True
        else: 
            if self.logger_set: self.get_logger().info("参数修改不成功")
            self.parameter_chg_success = False
        self.parameter_new_chg_req = False
    
    def parameters_set_cb(self, response:rcl_interfaces.srv.SetParameters.Response):
        tmp = response.result().results
        print(tmp)
        flag = True
        for i in tmp:
            if i.successful == False:
                flag = False
        if flag:
            if self.logger_set: self.get_logger().info("参数设置成功")
            self.parameters_set_success = True
        else: 
            if self.logger_set: self.get_logger().info("参数设置不成功")
            self.parameters_set_success = False
        self.parameters_new_set_req = False
    
    def rallypoint_pull_cb(self, response):
        if response.result().success:
            if self.logger_set: self.get_logger().info("集结点拉取成功")
            self.rallypoint_pull_success = True
        else: 
            self.rallypoint_pull_success = False
            if self.logger_set: self.get_logger().info("集结点拉取不成功")
        self.rallypoint_new_pull_req = False

    def rallypoint_clear_cb(self, response):
        if response.result().success:
            if self.logger_set: self.get_logger().info("集结点清空成功")
            self.rallypoint_clear_success = True
        else: 
            self.rallypoint_clear_success = False
            if self.logger_set: self.get_logger().info("集结点清空不成功")
        self.rallypoint_new_clear_req = False

    def rallypoint_push_cb(self, response):
        if response.result().success:
            if self.logger_set: self.get_logger().info("集结点上传成功")
            self.rallypoint_push_success = True
        else: 
            self.rallypoint_push_success = False
            if self.logger_set: self.get_logger().info("集结点上传不成功")
        self.rallypoint_new_push_req = False

    def waypoint_push_cb(self, response):
        if response.result().success:
            if self.logger_set: self.get_logger().info("航点上传成功")
            self.waypoint_push_success = True
        else: 
            self.waypoint_push_success = False
            if self.logger_set: self.get_logger().info("航点上传不成功")
        self.waypoint_new_push_req = False

    def state_cb(self, state:mavros_msgs.msg.State):
        self.state = state

    def arm_cb(self, response):
        ret = response.result()
        if ret.success:
            if self.logger_set: self.get_logger().info("解锁成功，即将起飞..")
        else:
            if self.logger_set: self.get_logger().info("解锁不成功，即将重试..")

    def set_mode_cb(self, response):
        ret = response.result()
        if ret.mode_sent:
            if self.logger_set: self.get_logger().info("更改模式成功")
            self.mode_set_success = True
        else:
            if self.logger_set: self.get_logger().info("更改模式失败")
            self.mode_set_success = False
        self.mode_new_set_req = False

    def global_position_cb(self, global_position: sensor_msgs.msg.NavSatFix):
        if self.home is None: return;
        self.global_position = [float(global_position.latitude), float(global_position.longitude), global_position.altitude - self.home_altitude]
    def home_cb(self, response: mavros_msgs.msg.HomePosition):
        self.home = [response.geo.latitude, response.geo.longitude, 0]
        self.home_altitude = response.geo.altitude

    def waypoint_reached_cb(self, response: mavros_msgs.msg.WaypointReached) -> None:
        self.current_reached_waypoint = response.wp_seq
    def local_position_cb(self, local_position: nav_msgs.msg.Odometry) -> None:
        self.local_position = [float(local_position.pose.pose.position.x), float(local_position.pose.pose.position.y), float(local_position.pose.pose.position.z)]        
        self.rotate = R.from_quat([local_position.pose.pose.orientation.x, local_position.pose.pose.orientation.y, local_position.pose.pose.orientation.z, local_position.pose.pose.orientation.w])
        

class WayPointShit(CallBackNode):
    # 生成mavros消息
    def generate_waypoint(
        self,
        x,
        y,
        z,
        frame=3,
        command=16,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        is_current=False,
        autocontinue=True,
    ):
        res = mavros_msgs.msg.Waypoint()
        res.frame = 3
        res.command = command
        res.param1 = param1
        res.param2 = param2
        res.param3 = param3
        res.param4 = param4
    
        res.x_lat = x
        res.y_long = y
        res.z_alt = z
        res.is_current = is_current
        res.autocontinue = autocontinue
        return res

    """
    东北天坐标传入，最后上传时转化为经纬度
    """

    def generate_straight_line_waypoints(self, start: list, end: list, increase=30.0):
        x0, y0, z0 = start
        x1, y1, z1 = end
        # 求出单位向量，60米为一个增量
        orientation = np.array([x1 - x0, y1 - y0, z1 - z0])
        delta = increase * (orientation / np.linalg.norm(orientation))
        ret = mavros_msgs.srv.WaypointPush.Request()
        ret.waypoints.append(
            self.generate_waypoint(*enu_to_geodetic(x0, y0, z0, *self.home))
        )
        start_cp = np.array(start)
        end_cp = np.array(end)
        delta_cp = np.array(delta)

        # print(start_cp - end_cp + delta_cp)
        dist = (int)(np.linalg.norm(start_cp - end_cp) / increase)
        while dist:
            ret.waypoints.append(
                self.generate_waypoint(
                    *enu_to_geodetic(
                        x0 + delta[0], y0 + delta[1], z0 + delta[2], *self.home
                    )
                )
            )
            x0 += delta[0]
            y0 += delta[1]
            z0 += delta[2]
            dist -= 1
        ret.waypoints.append(
            self.generate_waypoint(*enu_to_geodetic(x1, y1, z1, *self.home))
        )
        return ret.waypoints

    # 参数： 起点，终点，夹角 <= 180，方向（顺逆）(优劣弧)
    def generate_curve_line_waypoints(
        self, start: list, end: list, alpha: float, direction: bool, increase=30.0
    ):
        # 生成旋转矩阵
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])

        x0, y0, z0 = start
        x1, y1, z1 = end

        ret = mavros_msgs.srv.WaypointPush.Request()
        ret.waypoints.append(
            self.generate_waypoint(*enu_to_geodetic(x0, y0, z0, *self.home))
        )

        # 求出起点到终点的距离
        orientation = np.array([x1 - x0, y1 - y0])
        dis = np.linalg.norm(orientation)

        # 特判180度
        if alpha == np.pi:
            alpha = (178.0 / 180.0) * np.pi

        # 单位向量
        orientation = orientation / np.linalg.norm(orientation)
        # 求出半径，正弦定理
        beta = (np.pi - alpha) / 2.0
        radius = (dis / np.sin(alpha)) * np.sin(beta)
        print("radius: ", radius)

        # 生成旋转矩阵，求出指向圆心的向量
        rotate = gen_rotate(-beta)
        vector_radius = radius * (rotate.dot(orientation))
        p0 = np.array([x0, y0])

        # 30米为一个增量，求出递增角度，和递增高度
        delta = increase / radius
 
        sum = 0.0
        nxt_height = z0
        # 优弧
        if direction == True:
            delta_height = (z1 - z0) / (alpha / delta)
            while sum + delta <= alpha:
                sum += delta
                rotate = gen_rotate(-sum)
                tmp = rotate.dot((-vector_radius))
                tmp = p0 + vector_radius + tmp
                if (delta_height >=0 and nxt_height + delta_height <= z1) or (delta_height <= 0 and nxt_height + delta_height <= z1):
                    nxt_height += delta_height
                else:
                    nxt_height = z1
                ret.waypoints.append(
                    self.generate_waypoint(
                        *enu_to_geodetic(tmp[0], tmp[1], nxt_height, *self.home)
                    )
                )
            if sum != alpha:
                ret.waypoints.append(
                    self.generate_waypoint(*enu_to_geodetic(x1, y1, z1, *self.home))
                )
        # 劣弧
        else:
            alpha = 2.0 * np.pi - alpha
            delta_height = (z1 - z0) / (alpha / delta)
            while sum + delta <= alpha:
                sum += delta
                rotate = gen_rotate(sum)
                tmp = rotate.dot((-vector_radius))
                tmp = p0 + vector_radius + tmp
                if (delta_height >=0 and nxt_height + delta_height <= z1) or (delta_height <= 0 and nxt_height + delta_height >= z1):
                    nxt_height += delta_height
                else:
                    nxt_height = z1
                ret.waypoints.append(
                    self.generate_waypoint(
                        *enu_to_geodetic(tmp[0], tmp[1], nxt_height, *self.home)
                    )
                )
            if sum != alpha:
                ret.waypoints.append(
                    self.generate_waypoint(*enu_to_geodetic(x1, y1, z1, *self.home))
                )
        return ret.waypoints
    
        # 参数： 起点，终点，夹角 <= 180，方向（顺逆）(优劣弧)
    def generate_curve_line_waypoints_radius(
        self, start: list, end: list, radius: float, direction: bool, increase=30.0
    ):
        dist = np.linalg.norm(np.array(start[:-1]) - np.array(end[:-1]))
        print(1. - ((dist ** 2.) / (2. * (radius ** 2))))
        theta = np.arccos(1. - ((dist ** 2.) / (2. * (radius ** 2))))
        
        alpha = (theta * 180. / np.pi)
        print("alpha: " ,alpha)
        return self.generate_curve_line_waypoints(
            start, end, theta, direction, increase
        )
    
    def waypoint_straight_line_push(self, end: list):
        while not self.waypoint_push_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待上传航点服务上线...")
        req = mavros_msgs.srv.WaypointPush.Request()
        start = list(geodetic_to_enu(*self.current_rallypoint, *self.home))
        # req.waypoints.append(self.generate_waypoint(*start))
        # req.waypoints.append(self.generate_waypoint(*start,command=18,param2=1))
        ret = self.generate_straight_line_waypoints(start, end)
        req.waypoints.extend(ret)
        # print("?")
        self.waypoint_push_client.call_async(req).add_done_callback(
            self.waypoint_push_cb
        )
        self.rallypoint_push(list(enu_to_geodetic(*end, *self.home)))
        self.current_waypoint_num = len(req.waypoints) - 1
        
    def waypoint_push(self,req:mavros_msgs.srv.WaypointPush.Request):
        while not self.waypoint_push_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待上传航点服务上线...")
        if self.logger_set: self.get_logger().info("尝试上传航点...")
        self.waypoint_push_client.call_async(req).add_done_callback(
            self.waypoint_push_cb
        )
        # self.rallypoint_push(list(req.waypoints[len(req.waypoints)-1]))
        self.current_waypoint_num = len(req.waypoints) - 1


    def waypoint_curve_line_push(self, end: list, alpha: float, direction: bool):
        while not self.waypoint_push_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待上传航点服务上线...")
        if self.logger_set: self.get_logger().info("尝试上传航点...")
            
        req = mavros_msgs.srv.WaypointPush.Request()
        print(self.current_rallypoint)
        start = list(geodetic_to_enu(*self.current_rallypoint, *self.home))
        ret = self.generate_curve_line_waypoints(start, end, alpha, direction)
        req.waypoints.extend(ret)
        self.waypoint_push_client.call_async(req).add_done_callback(
            self.waypoint_push_cb
        )
        self.rallypoint_push(list(enu_to_geodetic(*end, *self.home)))
        self.current_waypoint_num = len(req.waypoints) - 1

    
    def waypoint_clear(self):
        while not self.waypoint_push_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待清空航点服务上线...")
        if self.logger_set:  self.get_logger().info("尝试清空航点...")
        req = mavros_msgs.srv.WaypointClear.Request()
        self.waypoint_clear_client.call_async(req).add_done_callback(
            self.waypoint_clear_cb
        )
    def set_mode(self, mode):
        while not self.set_mode_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待set_mode服务上线...")
        if self.logger_set: self.get_logger().info("set_mode服务已上线即将改为 %s 模式..." % mode)
        request = mavros_msgs.srv.SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        # print(request, request._custom_mode)
        self.set_mode_client.call_async(request).add_done_callback(self.set_mode_cb)

class RallyPointShit(CallBackNode):
    def rallypoint_pull(self):
        while not self.rallypoint_pull_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待拉取集结点服务上线...")
        req = mavros_msgs.srv.WaypointClear.Request()
        self.rallypoint_clear_client.call_async(req).add_done_callback(
            self.rallypoint_pull_cb
        )

    def rallypoint_clear(self):
        while not self.rallypoint_clear_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待清空集结点服务上线...")
        if self.logger_set: self.get_logger().info("尝试清空集结点")
        req = mavros_msgs.srv.WaypointClear.Request()
        self.rallypoint_clear_client.call_async(req).add_done_callback(
            self.rallypoint_clear_cb
        )

    def rallypoint_push(self, target: list) -> None:
        while not self.rallypoint_push_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待上传集结点服务上线...")
        if self.logger_set: self.get_logger().info("尝试上传集结点")
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(*target, command=5100))
        self.rallypoint_push_client.call_async(req).add_done_callback(
            self.rallypoint_push_cb
        )

class ParameterShit(CallBackNode):
    
    def pull_parameter(self, force_pull: bool) -> None:
        while not self.parameter_pull_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待拉取参数服务上线...")
        if self.logger_set: self.get_logger().info("尝试拉取参数")
        req = mavros_msgs.srv.ParamPull.Request()
        req.force_pull = force_pull
        self.parameter_pull_client.call_async(req).add_done_callback(
            self.parameter_pull_cb
        )
        
    def get_parameters(self, names: list) -> any:
        while not self.parameters_get_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待获取参数服务上线...")
        if self.logger_set: self.get_logger().info(f"尝试获取参数{names}")
        req = rcl_interfaces.srv.GetParameters.Request()
        req.names = names
        self.parameters_get_client.call_async(req).add_done_callback(
            self.parameters_get_cb
        )
    def chg_parameter(self, name: str, value: any) -> None:
        while not self.parameter_set_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待修改参数服务上线...")
        req = mavros_msgs.srv.ParamSetV2.Request()
        req.param_id = name
        req.force_set = True
        if self.logger_set: self.get_logger().info(f"尝试修改{name}参数")
        if type(value) == type(False):
            req.value.bool_value = value
            req.value.type = 1
        elif type(value) == type(1):
            req.value.integer_value = value
            req.value.type = 2
        elif type(value) == type(1.0):
            req.value.double_value = value
            req.value.type = 3
        elif type(value) == type("s"):
            req.value.string_value = value
            req.value.type = 4
        else:
            if self.logger_set: self.get_logger().warn(f"不支持的参数类型{(type(value))}")
            return
        self.parameter_set_client.call_async(req).add_done_callback(
            self.parameter_set_cb
        )
    def get_param_value(self, value):
        req = rcl_interfaces.msg.ParameterValue()
        if type(value) == type(False):
            req.bool_value = value
            req.type = 1
        elif type(value) == type(1):
            req.integer_value = value
            req.type = 2
        elif type(value) == type(1.0):
            req.double_value = value
            req.type = 3
        elif type(value) == type("s"):
            req.string_value = value
            req.type = 4
        return req
    def chg_parameters(self, names: list, values: list) -> None:
        while not self.parameters_set_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待修改参数服务上线...")
        req = rcl_interfaces.srv.SetParameters.Request()
        if self.logger_set: self.get_logger().info(f"尝试修改{names}参数")
        for name, value in zip(names, values):
            tmp = rcl_interfaces.msg.Parameter()
            tmp.name = name
            tmp.value = self.get_param_value(value)
            req.parameters.append(tmp)
        self.parameters_set_client.call_async(req).add_done_callback(
            self.parameters_set_cb
        )

class TakeOffShit(CallBackNode):
    def arm(self):
        while not self.arming_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待解锁服务上线...")
        request = mavros_msgs.srv.CommandBool.Request()
        request.value = True
        self.arming_client.call_async(request).add_done_callback(self.arm_cb)


    def takeoff_process(self, position: list) -> None:
        while not self.waypoint_push_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待上传航点服务上线...")
        req = mavros_msgs.srv.WaypointPush.Request()
        target = list(enu_to_geodetic(*position, *self.home))
        req.waypoints.append(self.generate_waypoint(*target, command=22))
        req.waypoints.append(self.generate_waypoint(*target, command=22))
        self.waypoint_push_client.call_async(req).add_done_callback(
            self.waypoint_push_cb
        )
        self.rallypoint_push(target)

    def land_process(self) -> None:
        while not self.waypoint_push_client.wait_for_service(1.0):
            if self.logger_set: self.get_logger().info("等待上传航点服务上线...")
        req = mavros_msgs.srv.WaypointPush.Request()
        # target=list(enu_to_geodetic(*position,*self.home));
        req.waypoints.append(self.generate_waypoint(*self.home, command=21))
        req.waypoints.append(self.generate_waypoint(*self.home, command=21))
        self.waypoint_push_client.call_async(req).add_done_callback(
            self.waypoint_push_cb
        )
        # self.rallypoint_push(target)