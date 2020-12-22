#!/usr/bin/env python
import tf
import cma  # import CMA-ES optimization algorithm
import sys
import time
import rospy
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped
import yaml


# ####### LOG:
# - fixing planning frame panda_hand/panda_ee confict
# - adding rotations errors to SEE to force converging to correct solution (multiple correct solutions likely when monitoring translation errors only)
# - reviewed how q1 to q4 are calculated from rotation matrix
# - storing data in a dictionary to reduce the no of lists
# - saving data and prompting user if new data needs to be collected by running the robot
# - reduced goal tolerance callibration to 5 mm
#
# ####### TO DO:
# - adding IF statement to only append data to list if marker detection = sucess  (marker likely to be moved)
# - fixing issue with loading and saving robot positions in yaml file


class Calibrate(object):
    def __init__(self):
        super(Calibrate, self).__init__()
        self.listener = tf.TransformListener()
        self.marker_sub = rospy.Subscriber('/our/test/pose_stamped', PoseStamped, self.get_whycon_pos)
        # self.camera_marker_list = list()

        # self.trans_camera_marker_list = list()
        self.transDict = {}
        self.rotDict = {}

        self.transDict['L0_EE'] = []
        self.transDict['camera_marker1'] = []
        self.transDict['camera_marker2'] = []

        self.rotDict['L0_EE'] = []
        self.rotDict['camera_marker1'] = []
        self.rotDict['camera_marker2'] = []

        self.br = tf.TransformBroadcaster()
        self.robot = Robot()

    def get_whycon_pos(self, msg):
        self.marker_trans = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.marker_rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def find(self, index):
        (trans, rot) = self.get_transform('panda_link0', 'panda_hand')  # was panda_hand
        self.transDict['L0_EE'].append(trans)
        self.rotDict['L0_EE'].append(rot)

        self.feedback(index)

        print "recording images"

        self.camera_marker_list_trans2 = list()
        self.camera_marker_list_rot2 = list()

        for i in range(10):
            self.myprint(".")
            time.sleep(1)
            self.camera_marker_list_trans2.append(self.marker_trans)
            self.camera_marker_list_rot2.append(self.marker_rot)

        # camera_marker_mean_trans1 = np.mean(np.array(self.camera_marker_list_trans1), axis=0)
        # camera_marker_mean_rot1 = np.mean(np.array(self.camera_marker_list_rot1), axis=0)

        camera_marker_mean_trans2 = np.mean(np.array(self.camera_marker_list_trans2), axis=0)
        camera_marker_mean_rot2 = np.mean(np.array(self.camera_marker_list_rot2), axis=0)

        # self.transDict['camera_marker1'].append(camera_marker_mean_trans1)
        # self.rotDict['camera_marker1'].append(camera_marker_mean_rot1)

        self.transDict['camera_marker2'].append(camera_marker_mean_trans2)
        self.rotDict['camera_marker2'].append(camera_marker_mean_rot2)

    def feedback(self, index):
        print 'Calibration Point: ', index

    def get_transform(self, from_tf, to_tf):
        self.listener.waitForTransform(from_tf, to_tf, rospy.Time().now(), rospy.Duration(5.0))
        return self.listener.lookupTransform(from_tf, to_tf, rospy.Time(0))

    def myprint(self, to_print):
        sys.stdout.write(to_print)
        sys.stdout.flush()

    def optimise(self):

        res = cma.fmin(self.objective_function, [0., 0., -0.2,
            0., 0., 0., 1], 0.2)   

        #res = cma.fmin(self.objective_function, [-0.08, 0., 0.,
        #   0.707, 0.0, 0.707, 0.0], 0.2) 

        # closet solution:
        # trans = [-0.19962311 - 0.01034741 - 0.04000562]
        # quat = [-0.60079466 - 0.12068384 - 0.78862378  0.05053437]

        # while not res.stop():
        #     solutions = res.ask()
        #     res.tell(solutions, [cma.ff.rosen(x) for x in solutions])
        #     res.logger.add()  # write data to disc to be plotted
        #     res.disp()
        # res.result_pretty()
        # cma.plot()

        # testVal = self.objective_function(res[0])
        trans = res[0][0:3]
        quat = res[0][3:]
        quat = quat / np.linalg.norm(quat)
        # p_res = self.listener.fromTranslationRotation(trans, quat)

        #### adding extra transformation to fix  rotation between optical and camera link frames
        T_from_opt = self.listener.fromTranslationRotation(trans, quat)        # calib transformation wrt optical frame
        T_optical_camera = self.listener.fromTranslationRotation([0.015, 0.000, 0.001], [0.502, -0.496, 0.500, 0.502]) #transformation from  optical  frame to camera link
        T_final = np.dot(T_from_opt, T_optical_camera)
        trans = T_final[0:3, 3]        
        quat = tf.transformations.quaternion_from_matrix(T_final)     # accepts full transformation matrix

        print '#########Result by optimization: '
        print "trans=", trans
        print "norm quat =", quat
        return trans, quat

    def objective_function(self, x):
        trans = [x[0], x[1], x[2]]
        rot = [x[3], x[4], x[5], x[6]]
        mag = np.linalg.norm(rot)
        rot_normalized = rot / mag
        T = self.listener.fromTranslationRotation(trans, rot_normalized)

        # print "T=", T

        sse = 0

        pos_list = np.zeros((len(self.transDict['L0_EE']), 3))
        orient_list = np.zeros((len(self.transDict['L0_EE']), 4))

        for i in range(len(self.transDict['L0_EE'])):
            trans_xi = self.transDict['L0_EE'][i]
            rot_xi = self.rotDict['L0_EE'][i]
            xi = self.listener.fromTranslationRotation(trans_xi, rot_xi)

            trans_yi = self.transDict['camera_marker2'][i]
            rot_yi = self.rotDict['camera_marker2'][i]
            yi = self.listener.fromTranslationRotation(trans_yi, rot_yi)

            temp = np.dot(T, yi)
            base_marker = np.dot(xi, temp)

            pos_i = base_marker[0:3, 3]
            orient_i = tf.transformations.quaternion_from_matrix(base_marker)     # accepts full transformation matrix

            # x, y, z, q1, q2, q3, q4 = base_marker
            pos_list[i, :] = pos_i
            orient_list[i, :] = orient_i

        sse = np.sum(np.var(pos_list, axis=0))  #+ np.sum(np.var(orient_list, axis=0))      #rotation seem to mess data

        return sse

        print "############SSE:", sse


class Robot(object):
    def __init__(self):
        print "============ Initialising..."
        super(Robot, self).__init__()
        self.load_config()
        moveit_commander.roscpp_initialize(sys.argv)
        self.commander = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm.set_planner_id("FMTkConfigDefault")
        # self.rviz_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        rospy.sleep(2)
        self.arm.set_end_effector_link("panda_hand")    # planning wrt to panda_hand or link8
        self.arm.set_max_velocity_scaling_factor(0.10)  # scaling down velocity
        self.arm.set_max_acceleration_scaling_factor(0.10)  # scaling down velocity
        self.arm.allow_replanning(True)
        self.arm.set_num_planning_attempts(5)
        self.arm.set_goal_position_tolerance(0.01)          # was 0.005
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_planning_time(5)

        # print self.arm.get_current_pose().pose

    def load_config(self):
        self.positions = list()

        self.positions.append([-0.29021561276285274, 0.12636862861117054, -0.12506102137771713, -1.9093128912962363, 0.3444320711910822, 1.6248377451875933, 0.17382591431792424])
        self.positions.append([0.02443942915399869, 0.46673909325348706, -0.39334134471378884, -1.4835145217661272, 0.2850451601589187, 1.3330224422606964, 0.10029918030370026])
        self.positions.append([-0.06838630456464333, -0.7072280792688068, -0.1288322677175379, -2.837287618168613, 0.049362759997422574, 2.1788312390276867, 0.42411726281610407])
        self.positions.append([0.08076317085717853, -0.40339251564260103, 0.6398663307240632, -2.772376327447724, -0.4829939398018415, 2.211275206283002, 1.5666560650418198])
        self.positions.append([0.02479424776032305, 0.39081663928921145, 0.45177425950870176, -1.8504539694033169, -0.4809605723039968, 1.5696443838552725, 1.417458188734464])    
        self.positions.append([-0.3167118737241982, 0.17458424823325977, -0.26764921717439505, -2.174334934569241, 0.3571267512043317, 2.0066584593984818, -0.1724274453971949])
        self.positions.append([-0.3127685887741686, -0.47580423080351897, -0.22790333723836848, -2.9115570040920327, 0.3472240333703558, 2.435488914203594, -0.1046377944192016])
        self.positions.append([-0.24722993707109484, -0.8216488193294458, 1.1042658614944993, -2.987659984472144, 0.2324325354165504, 2.4467499325897086, 1.0148617136809492])
        self.positions.append([0.02501802015099548, 0.15179193521800793, 0.08539240931725132, -2.0550966551596663, -0.09332612091302872, 1.8447079379738776, 0.8660068615823981])
        self.positions.append([-0.1689139768617195, -0.2424827781267333, -0.23425610814596476, -2.3515616356380917, 0.2737893464763959, 1.8512460997746782, 0.17163326147859612])
        self.positions.append([-0.03207843073679689, 0.09983654342199627, 0.6693389059949817, -2.327959250667639, -0.5550821814291838, 1.7713862823446587, 1.6273102427075306])
        self.positions.append([-0.006577562646188756, 0.5791499242029691, 0.5540791301408614, -1.7181691182959835, -0.5912067626839229, 1.3306091501004345, 1.5727480427970568])
        self.positions.append([-0.05050972567367972, -0.36215200300802264, -0.09679696158354535, -2.5477194552838247, 0.05972013647908508, 2.1370160227146036, 0.4134754527157378])
        self.positions.append([-0.09484604250672668, -0.12765091621996705, -0.2170380109875724, -2.583939632958302, 0.07096718591416713, 2.3946779385142856, 0.3275466168470061])
        self.positions.append([0.05497521364584304, 0.12450918447553065, 0.15566315450242196, -2.024442253380491, -0.14088330700000126, 1.7202385384771557, 0.8532596413831249])
        self.positions.append([0.09266749860027121, -0.3629513006252155, 0.519635454658944, -2.692446547809535, -0.3254121098336246, 2.096764392521144, 1.3391306948628803])
        self.positions.append([0.023905320650652834, 0.032159521633091846, -0.6034049986871599, -2.380025409884517, 0.44415809449173294, 2.1055873489072456, -0.1721770448933045])
        self.positions.append([0.1726646456895461, -0.5441621516007153, 0.17820556396559661, -2.7008028399355557, -0.15254146122768117, 2.1617343814287553, 1.129569414878292])
        self.positions.append([0.062649716052047, 0.1882825902900268, -0.3590408138266781, -1.944538020718793, -0.01098458359374173, 1.8102688684993316, -0.11783595360860494])
        self.positions.append([0.11671516002479353, -0.6484696519219212, -0.06651313175025744, -2.0192508261831184, -0.052139262103440734, 1.375730055862003, 0.7360559495836246])
        self.positions.append([0.1426238866798547, -0.5860076728452421, 0.3211208654454048, -2.0084537695177658, -0.06914829441090453, 1.3098458598984613, 0.9154166242343693])
        self.positions.append([0.10681669097406822, 0.055031178102160495, 0.2558272637643817, -1.3252099332642138, -0.0663180694381396, 0.9839646829917865, 0.9859419324195474])



    def move(self):
        print "============ Moving..."
        # print self.arm.plan()
        self.arm.go()

    def set_joint_positions(self, joints):
        self.arm.set_joint_value_target(joints)
        return self


def main():
    rospy.init_node('test', anonymous=True)
    cal = Calibrate()

    text = raw_input("============ Do you want to recollect calibration data?")
    if text == "Y" or text == "y":
        for i, pos in enumerate(cal.robot.positions):
            cal.robot.set_joint_positions(pos).move()
            time.sleep(2)
            cal.find(i)
            time.sleep(1)

        with open('transFile.yaml', 'w') as stream:
           yaml.dump(cal.transDict, stream)

        with open('rotFile.yaml', 'w') as stream:
           yaml.dump(cal.rotDict, stream)
    else:
        with open('transFile.yaml', 'r') as stream:
            cal.transDict = yaml.load(stream)

        with open('rotFile.yaml', 'r') as stream:
            cal.rotDict = yaml.load(stream)

    (t, r) = cal.optimise()
    print('t',t)
    print('r',r)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        cal.br.sendTransform((t[0], t[1], t[2]*0.01), (r[0], r[1], r[2], r[3]), rospy.Time.now(), 'camera_link', 'panda_hand')
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print e
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
