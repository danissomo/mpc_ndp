import rtde_control
import rtde_receive

def array_sum(vec1, vec2):
    result = []
    for i in range(vec1.__len__()):
        result.append(vec1[i]+vec2[i])
    return result