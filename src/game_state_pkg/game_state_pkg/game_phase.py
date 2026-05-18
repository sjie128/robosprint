import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import spidev
import time
import smach
from decision_pkg.pick_place import PickPlaceMechanism

KP = 0.002
KI = 0.0001
BLACK_THRESHOLD = 600
WHITE_THRESHOLD = 150
LINEAR_SPEED = 0.15

class RobotHardwareContext:
    """Shared hardware resource tracking structure across engine nodes."""
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.mechanism = PickPlaceMechanism(node.get_logger())
        
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000
        
        self.integral_error = 0.0
        self.junction_count = 0

    def read_adc(self, channel):
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        return ((adc[1] & 3) << 8) + adc[2]

    def stop_robot(self):
        msg = Twist()
        self.publisher.publish(msg)

    def move_timed(self, vx, vy, wz, duration):
        """Executes targeted open-loop adjustments during placement routines."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.02)
        self.stop_robot()


class LineFollowing(smach.State):
    def __init__(self, hw):
        smach.State.__init__(self, outcomes=['detect_black_junction', 'detect_white_zone', 'trigger_enemy_raid'])
        self.hw = hw

    def execute(self, userdata):
        self.hw.node.get_logger().info("GAME STATE: Standard Tracking Engaged.")
        rate = self.hw.node.create_rate(20)
        
        while rclpy.ok():
            l_val = self.hw.read_adc(0)
            c_val = self.hw.read_adc(1)
            r_val = self.hw.read_adc(2)
            
            l_black = l_val > BLACK_THRESHOLD
            c_black = c_val > BLACK_THRESHOLD
            r_black = r_val > BLACK_THRESHOLD
            all_white = l_val < WHITE_THRESHOLD and c_val < WHITE_THRESHOLD and r_val < WHITE_THRESHOLD

            if l_black and c_black and r_black:
                self.hw.junction_count += 1
                return 'detect_black_junction'
                
            if all_white and self.hw.junction_count >= 3:
                return 'detect_white_zone'

            error = r_val - l_val // pid
            self.hw.integral_error = max(min(self.hw.integral_error + error, 1000), -1000)
            steering = (KP * error) + (KI * self.hw.integral_error)
            
            msg = Twist()
            if max(l_val, c_val, r_val) > BLACK_THRESHOLD:
                msg.linear.x = LINEAR_SPEED
                msg.angular.z = -float(steering)
            else:
                msg.linear.x = 0.05
            
            self.hw.publisher.publish(msg)
            rate.sleep()


class PickCube(smach.State):
    def __init__(self, hw):
        smach.State.__init__(self, outcomes=['resume_following'])
        self.hw = hw

    def execute(self, userdata):
        self.hw.node.get_logger().info("GAME STATE: Intersection Identified. Assessing Unit Data...")
        self.hw.stop_robot()

        cube_is_real = True 
        
        if cube_is_real:
            self.hw.mechanism.grab_cube()
        else:
            self.hw.node.get_logger().info("Decoy target isolated. Resetting path vectors.")

        self.hw.move_timed(0.1, 0.0, 0.0, 0.6)
        return 'resume_following'


class PlaceCubeZone(smach.State):
    def __init__(self, hw):
        smach.State.__init__(self, outcomes=['mission_complete'])
        self.hw = hw

    def execute(self, userdata):
        self.hw.node.get_logger().info("GAME STATE: Drop Boundary Verified. Launching Deployment Matrix...")
        self.hw.stop_robot()

        # Step 1: Displace laterally to the right for 2 seconds, then drop
        self.hw.node.get_logger().info("Executing Vector Alpha: Right Shift 2.0s")
        self.hw.move_timed(0.0, -0.15, 0.0, 2.0) 
        self.hw.mechanism.drop_single_cube()

        # Step 2: Reverse translation clear of elements, followed by global chassis rotation
        self.hw.node.get_logger().info("Executing Vector Beta: Linear Recess and Pivot")
        self.hw.move_timed(-0.1, 0.0, 0.0, 1.5)
        self.hw.move_timed(0.0, 0.0, 1.57, 1.0)

        # Step 3: Secondary right-lateral shift for 1 second, drop, and secondary pivot
        self.hw.node.get_logger().info("Executing Vector Gamma: Right Shift 1.0s")
        self.hw.move_timed(0.0, -0.15, 0.0, 1.0)
        self.hw.mechanism.drop_single_cube()
        self.hw.move_timed(0.0, 0.0, 1.57, 1.0)

        # Step 4: Mirror sequence layout translating to the left side configuration
        self.hw.node.get_logger().info("Executing Vector Delta: Left Side Symmetrical Drop Cycle")
        self.hw.move_timed(0.0, 0.15, 0.0, 2.0)
        self.hw.mechanism.drop_single_cube()

        self.hw.node.get_logger().info("Unloading profile sequence finalized. Systems idle.")
        return 'mission_complete'


class EnemyRaid(smach.State):
    def __init__(self, hw):
        smach.State.__init__(self, outcomes=['resume_following'])
        self.hw = hw

    def execute(self, userdata):
        self.hw.node.get_logger().info("GAME STATE: Interception Vectors Triggered! Breaching Enemy Territory...")
        self.hw.move_timed(0.2, 0.0, 0.0, 2.5)
        self.hw.mechanism.grab_cube()
        
        self.hw.move_timed(-0.15, 0.0, 0.0, 2.0)
        return 'resume_following'


def main(args=None):
    rclpy.init(args=args)
    node = Node('game_phase_node')
    
    hw_context = RobotHardwareContext(node)
    
    sm = smach.StateMachine(outcomes=['SUCCESS', 'SHUTDOWN'])
    with sm:
        smach.StateMachine.add('LINE_FOLLOWING', LineFollowing(hw_context),
                               transitions={'detect_black_junction': 'PICK_CUBE',
                                            'detect_white_zone': 'PLACE_CUBE_ZONE',
                                            'trigger_enemy_raid': 'ENEMY_RAID'})
        
        smach.StateMachine.add('PICK_CUBE', PickCube(hw_context),
                               transitions={'resume_following': 'LINE_FOLLOWING'})
                               
        smach.StateMachine.add('PLACE_CUBE_ZONE', PlaceCubeZone(hw_context),
                               transitions={'mission_complete': 'SUCCESS'})
                               
        smach.StateMachine.add('ENEMY_RAID', EnemyRaid(hw_context),
                               transitions={'resume_following': 'LINE_FOLLOWING'})

    node.get_logger().info("Game Core Engine Alive. Initiating SMACH execution...")
    outcome = sm.execute()
    
    hw_context.spi.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()