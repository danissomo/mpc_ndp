import math
class trajectPlanner(object):
    def __init__(self, debug = False):
        self.debug = debug
        if self.debug:
            print(trajectPlanner)
            print()
        super().__init__()

    def calc_point_c(self, point1,point2):
        max_speed = 0.1*0.008
        l = 0
        for i in range(3):
            l+= (point2[i] - point1[i])*(point2[i] - point1[i])
        l = math.sqrt(l)
        #print(l/max_speed)
        return math.ceil(l/max_speed)


    def a_PointToPoint(self, point1,  point2):
        point_count = self.calc_point_c(point1, point2)
        if len(point1) != len(point2):
            print("Error: different point length")
            return
        if point_count <= 0:
            print("Error: point count < 0")
        result = []
        for i in range(point_count):
            result.append(self.__param_line(point1, point2, i/(point_count-1)))

        if self.debug:
            print(self.a_PointToPoint)
            print(point1)
            print(point2)
            print(point_count)
            print()

        return result




    def __param_line(self, point1, point2, t):
        result = []
        a = (point2[0]-point1[0])/(math.cosh(1)-1)
        b = (point2[1]-point1[1])/(math.sinh(1))
        result.append(a*math.cosh(t)-a+point1[0])
        result.append(b*math.sinh(t)+point1[1])
        for i in range(2,len(point1)):
            result.append(point1[i]+(point2[i]-point1[i])*t)

        if self.debug:
            print(self.__param_line)
            print(t)
            print(result)
            print()
        return result

class rtde_kinematic:
    def __init__(self, rtde_c, debug = False):
        self.rtde_c = rtde_c
        self.debug = debug


    def get_joint_pose(self, cart, start_pose):
        result = []
        if len(cart) == 0: 
            print("Error: cart has zero lenth")
            return

        result.append(self.rtde_c.getInverseKinematics(cart[0], start_pose))
        for i in range(1,len(cart)):
            q =self.rtde_c.getInverseKinematics(cart[i], result[i-1])
            if not q: 
                print("Error: can't find ik solve")
                exit()
            result.append(q)

        if self.debug:
            print(self.get_joint_pose)
            print(cart)
            print(result)
        return result
        
    