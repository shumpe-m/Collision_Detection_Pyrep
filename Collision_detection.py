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
        self.position_min, self.position_max = [-0.001, -0.001, 0.75], [0.001, 0.001, 0.75]
        self.orientation_min, self.orientation_max =[0, 0.261, 0.261], [0.785, 0.785, 0.785]
        self.Cuboid_size = [0.05, 0.16, 0.02]
        self.chopstick_size = [0.01, 0.01, 0.10]
        self.cubes = []

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

    def _arrangement(self):
        # terminalで'yes'が入力されるまで繰り返す
        while not self.done:
            print('----Create a Arrangement----')
            # num_obj の個数生成
            for num_cube in range(self.num_obj):
                # size = list(np.random.uniform(self.obj_size_min, self.obj_size_max))
                pre_pos = list(np.random.uniform(self.position_min, self.position_max))
                pre_ori = list(np.random.uniform(self.orientation_min, self.orientation_max))
                # 上からCuboidを落とす
                self._create_cuboid(pos = pre_pos, ori = pre_ori, color = [0.1, 0.7, 0.1], static = False)
                [self.pr.step() for _ in range(100)]
                pos, ori = self._get_state('Obj')
                # Cuboidを消去してstatic=Trueにする。
                color = [0.1, 0.1, 1] if self.num_obj == num_cube + 1 else [0.1, 1, 0.1]
                self._create_cuboid(pos = pos, ori = ori, color = color, static = True, obj = num_cube)
                Object.remove(Shape('Obj'))
                [self.pr.step() for _ in range(40)]
                cube = np.concatenate(([pos],[ori]), 0)
                self.cubes = cube[np.newaxis, :, :] if num_cube == 0 else np.concatenate((self.cubes, [cube]), 0)
            response = input('>The arrangement is Ok? (yes or no)')
            if response == 'yes':
                self.done = True
            else:
                self.cubes = []
            self.reset()
                

    def _create_cuboid(self, pos = None, ori = None, color = [0.1, 1, 0.1], static = False, obj = ''):
        cuboid = Shape.create(type = PrimitiveShape.CUBOID,
                                       mass = 0.01,
                                       size = self.Cuboid_size,
                                       color = color,
                                       static = static, respondable=True)
        cuboid.set_name('Obj' + str(obj))
        cuboid.set_bullet_friction(1.)
        cuboid.set_position(pos)
        cuboid.set_orientation(ori)
        

    def _collision_detection(self):
        self.division = 40
        for num_cube in range(self.num_obj):
            self._create_cuboid(pos = self.cubes[num_cube][0], ori = self.cubes[num_cube][1], color = [0.1, 0.1, 1.], static = True, obj = num_cube)
            if num_cube != 0:
                Shape('Obj' + str(num_cube-1)).set_color([0.1, 1., 0.1])
            for d in range(self.division):
                for num_chop in range(2):
                    chop_name = 'chopstick' + str(num_chop)
                    pos, ori = self._chopstick_position(num_cube, num_chop, d)
                    if d == 0:
                        self._create_chopsticks(pos = pos, ori = ori, color = [0.1, 0.1, 1], static = True, obj = num_chop)
                    else:
                        Shape(chop_name).set_position(pos)
                        
                    if Shape(chop_name).check_collision():
                        Shape(chop_name).set_color([1., 0.1, 0.1])
                        print('cube No.', num_cube, '>Contacted grasp point:' , pos)
                    else:
                        Shape(chop_name).set_color([0.1, 0.1, 1])
                [self.pr.step() for _ in range(50)]
            Object.remove(Shape('chopstick0'))
            Object.remove(Shape('chopstick1'))
        [self.pr.step() for _ in range(1000)]

    
        
    def _create_chopsticks(self, pos, ori, color, static, obj):
        chopstick = Shape.create(type = PrimitiveShape.CYLINDER,
                                       mass = 0.01,
                                       size = self.chopstick_size,
                                       color = color,
                                       static = static, respondable=True)
        chopstick.set_name('chopstick' + str(obj))
        chopstick.set_bullet_friction(1.)
        chopstick.set_position(pos)
        chopstick.set_orientation(ori)
        #[self.pr.step() for _ in range(100)]
    
    def _chopstick_position(self, num_cube, num_chop, d):
        pos_cube, ori_cube = self.cubes[num_cube][0], self.cubes[num_cube][1]
        #print('cuboid:', pos_cube, ori_cube)
        # オブジェクトの角度によって座標が変化
        # wide = キューブの中心から箸の中心までの長さ x
        # grasp y
        # height = 箸の先端がキューブの底面と同じ長さになるように設定する変数 z
        sign = 1 if num_chop == 0 else -1
        wide = sign * (self.Cuboid_size[0] / 2 + self.chopstick_size[1] / 2 + 0.0001)
        grasp = self.Cuboid_size[1] / self.division * (-self.division / 2 + d)
        height = (self.chopstick_size[2] / 2 - self.Cuboid_size[2] / 2 + 0.0001)
        
        rot = self._rot(ori_cube)
        pos = np.dot(rot.T, np.array([wide, grasp, height]))
        
        pos[0] += pos_cube[0]
        pos[1] += pos_cube[1]
        pos[2] += pos_cube[2]
        #print('chopstick:', pos, ori_cube)
        return pos, ori_cube
    
    def _rot(self, ori):
        rot_x = np.array([[1, 0, 0],
                       [0, np.cos(ori[0]), np.sin(ori[0])],
                       [0, -np.sin(ori[0]), np.cos(ori[0])]])
        rot_y = np.array([[np.cos(ori[1]), 0, -np.sin(ori[1])],
                       [0, 1, 0],
                       [np.sin(ori[1]), 0, np.cos(ori[1])]])
        rot_z = np.array([[np.cos(ori[2]), np.sin(ori[2]), 0],
                       [-np.sin(ori[2]), np.cos(ori[2]), 0],
                       [0, 0, 1]])
        rot = rot_z.dot(rot_y).dot(rot_x)
        
        return rot
    
        
    def reset(self):
        self.pr.stop()
        self.pr.start()


if __name__ == '__main__':
    scene = 'assets/collision_detection.ttt'
    abs_scene = osp.join(osp.dirname(osp.abspath(__file__)), scene)
    env = CollisionDetectionEnv(scene_file=abs_scene, headless=False, num_obj = 8)

    env._arrangement()
    env._collision_detection()
    
    #input('press Enter.')

    env.shutdown()
