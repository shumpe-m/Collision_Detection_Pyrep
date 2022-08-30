import time
import os.path as osp
from pyrep import PyRep
from pyrep.objects.shape import Shape
from pyrep.objects.object import Object
from pyrep.const import PrimitiveShape
import numpy as np
import quaternion

class PyrepEnv:
    def __init__(self, scene_file: str, headless=True):
        self.pr = PyRep()
        self.pr.launch(scene_file, headless)
        self.dim_action, self.dim_state = None, None
        # self.pr.start()

    def _get_state(self):
        raise NotImplementedError()

    def reset(self):
        raise NotImplementedError()

    def step(self, action):
        raise NotImplementedError()

    def shutdown(self):
        self.pr.stop()
        self.pr.shutdown()


class CollisionDetectionEnv(PyrepEnv):
    def __init__(self, scene_file: str, headless: bool, num_obj: int):

        super(CollisionDetectionEnv, self).__init__(scene_file, headless)
        self.num_obj = num_obj
        self.done = False
        self.num_arrange = 0

        # self.obj_size_min, self.obj_size_max = [0.02, 0.02, 0.15], [0.8, 0.8, 0.15]
        self.position_min, self.position_max = [-0.002, -0.002, 0.75], [0.002, 0.002, 0.75]
        self.orientation_min, self.orientation_max =[0.261, 0, 0.261], [0.785, 0.785, 0.785]

        self.pr.start()
        [self.pr.step() for _ in range(200)]
        #self.initial_num_objects, _ = self._num_objects(0)
    
    '''
    # scene 内のオブジェクトの数と名前を返す
    def _num_objects(self, count):
        done = True
        while done:
            ob = Object.get_object_name(count)
            done = False if ob == '' else True
            print(ob)
            count += 1
        return count - 1, Object.get_object_name(count-2)
    '''

    def _get_state(self, obj_name):
        return Shape(obj_name).get_position(), Shape(obj_name).get_orientation()


    def _collision_detection(self):
        # terminalで'yes'が入力されるまで繰り返す
        while not self.done:
            print('----Create a Arrangement----')
            # num_obj の個数生成
            for obj in range(self.num_obj):
                # size = list(np.random.uniform(self.obj_size_min, self.obj_size_max))
                pos = list(np.random.uniform(self.position_min, self.position_max))
                ori = list(np.random.uniform(self.orientation_min, self.orientation_max))
                # 上からCuboidを落とす
                self._create_object(pos = pos, ori = ori, color = [0.1, 0.7, 0.1], static = False, obj = obj)
                pos_0, ori_0 = self._get_state('Obj' + str(obj))
                # Cuboidを消去してstatic=Trueにする。
                Object.remove(Shape('Obj' + str(obj)))
                color = [0.1, 0.1, 1] if self.num_obj == obj + 1 else [0.1, 1, 0.1]
                self._create_object(pos = pos_0, ori = ori_0, color = color, static = True, obj = obj)
            response = input('>The arrangement is Ok? (yes or no)')
            if response == 'yes':
                self.done = True
            else:
                self.reset()
                
        [self.pr.step() for _ in range(100)]

    def _create_object(self, pos, ori, color, static, obj):
        size = [0.02, 0.05, 0.15]
        Arrangement_obj = Shape.create(type = PrimitiveShape.CUBOID,
                                       mass = 0.01,
                                       size = size,
                                       color = color,
                                       static = static, respondable=True)
        Arrangement_obj.set_name('Obj' + str(obj))
        Arrangement_obj.set_bullet_friction(1.)
        Arrangement_obj.set_position(pos)
        Arrangement_obj.set_orientation(ori)
        [self.pr.step() for _ in range(100)]
        
    def reset(self):
        self.pr.stop()
        self.pr.start()

if __name__ == '__main__':
    scene = 'assets/collision_detection.ttt'
    abs_scene = osp.join(osp.dirname(osp.abspath(__file__)), scene)
    env = CollisionDetectionEnv(scene_file=abs_scene, headless=False, num_obj=10)

    env._collision_detection()

    env.shutdown()
