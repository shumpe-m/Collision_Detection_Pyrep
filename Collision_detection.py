import time

import os.path as osp
from pyrep import PyRep
from pyrep.objects.shape import Shape
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

    def _get_state(self):
        return Shape('Cuboid').get_position(), Shape('Cuboid').get_orientation()


    def _collision_detection(self):
        while not self.done:
            print('----Create a Arrangement----')
            for obj in range(self.num_obj):
                # size = list(np.random.uniform(self.obj_size_min, self.obj_size_max))
                pos = list(np.random.uniform(self.position_min, self.position_max))
                ori = list(np.random.uniform(self.orientation_min, self.orientation_max))
                self._create_object(pos, ori, color = [0.1, 1, 0.1], static = False)
                pos_0, ori_0 = self._get_state()
                pos_0 = pos_0[np.newaxis, :]
                ori_0 = ori_0[np.newaxis, :]
                self.pos = np.copy(pos_0) if obj == 0 else np.concatenate([self.pos, pos_0], 0)
                self.ori = np.copy(ori_0) if obj == 0 else np.concatenate([self.ori, ori_0], 0)


            response = input('>The arrangement is Ok? (yes or no)')
            if response == 'yes':
                self.pr.stop()
                self.pr.start()
                self.done = True
            else:
                self.reset()
        print('pos:', self.pos)
        print('ori:', self.ori)
        self._create_arrangement(self.pos, self.ori)


    def _create_object(self, pos, ori, color, static):
        size = [0.02, 0.045, 0.2]
        pos = list(np.random.uniform(self.position_min, self.position_max))
        ori = list(np.random.uniform(self.orientation_min, self.orientation_max))
        Arrangement_obj = Shape.create(type = PrimitiveShape.CUBOID,
                                       mass = 0.0001,
                                       size = size,
                                       color = color,
                                       static = static, respondable=True)
        Arrangement_obj.set_position(pos)
        Arrangement_obj.set_orientation(ori)
        [self.pr.step() for _ in range(500)]

    def _create_arrangement(self, pos, ori):
        for Arr_pos, Arr_ori in zip(pos[:self.num_arrange + 1], ori[:self.num_arrange + 1]):
            color = [0.1, 1, 0.1] if self.num_obj == self.num_arrange + 1 else [0.1, 0.1, 1]
            self._create_object(pos, ori, color, static = True)
            self.num_arrange += 1


    def reset(self):
        self.num_arrange = 0
        self.pos = np.array([])
        self.ori = np.array([])
        self.pr.stop()
        self.pr.start()


if __name__ == '__main__':
    scene = 'assets/collision_detection.ttt'
    abs_scene = osp.join(osp.dirname(osp.abspath(__file__)), scene)

    env = CollisionDetectionEnv(scene_file=abs_scene, headless=False, num_obj=10)
    done = False
    step = 0

    env._collision_detection()

    env.shutdown()

