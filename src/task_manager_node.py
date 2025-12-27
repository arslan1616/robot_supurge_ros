#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager_node', anonymous=True)
        rospy.loginfo("Task Manager Node baslatildi.")

        # Kendi dosya yolunu buraya tam olarak yazdigindan emin ol
        default_path = "/home/ibrahim16/rgiris/src/robot_supurge_ros/config/mission.yaml"
        mission_file_path = rospy.get_param('~mission_file', default_path)
        
        self.mission_data = self.load_mission(mission_file_path)
        if not self.mission_data:
            rospy.signal_shutdown("Gorev dosyasi yuklenemedi.")
            return

        # Action Client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("move_base action sunucusuna baglaniliyor...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action sunucusuna baglanildi!")

        # QR Verisi ve Hareket
        self.qr_data = None
        self.qr_subscriber = rospy.Subscriber('/qr_code', String, self.qr_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.task_report = {}
        rospy.loginfo("Sistem hazir. Gorev dongusu bekleniyor...")

    def qr_callback(self, message):
        self.qr_data = message.data

    def load_mission(self, file_path):
        try:
            with open(file_path, 'r') as stream:
                return yaml.safe_load(stream)
        except Exception as e:
            rospy.logerr(f"Gorev dosyasi okunurken hata: {e}")
            return None

    def execute_recovery_behavior(self):
        """
        [Gorev 5.1 & Iyilestirme]
        Robot navigasyonda sıkısırsa veya hedefe gidemezse
        kendi etrafinda donerek kurtulmaya calisir.
        """
        rospy.logwarn("!!! NAVIGASYON SIKISTI !!! Kurtarma manevrasi: 360 derece donuluyor...")
        msg = Twist()
        msg.angular.z = 0.5 # Saniyede 0.5 radyan donus hizi
        
        # Yaklasik 12-13 saniye donerek tam tur atmasini sagla
        rate = rospy.Rate(10)
        for _ in range(125):
            if rospy.is_shutdown(): break
            self.cmd_vel_pub.publish(msg)
            rate.sleep()
            
        # Dur
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Kurtarma manevrasi bitti. Tekrar deneniyor...")

    def search_for_qr_maneuver(self):
        """
        [Gorev 5.2]
        QR kod gorunmuyorsa robot kafasini saga sola cevirir
        ve hafifce geri-ileri giderek 'arama' yapar.
        """
        rospy.loginfo("QR hemen bulunamadi. Arama manevrasi baslatiliyor...")
        rate = rospy.Rate(10)
        
        # Hareketler listesi: [HizX, DonusZ, Sure(adim)]
        # 1. Saga bak, 2. Sola bak, 3. Geri git, 4. Ileri git
        maneuvers = [
            (0.0, -0.4, 15), # Saga don
            (0.0,  0.4, 30), # Sola don (daha fazla donup diger tarafa baksin)
            (0.0, -0.4, 15), # Ortaya gel
            (-0.1, 0.0, 10), # Hafif geri git
            (0.1,  0.0, 10)  # Hafif ileri git
        ]

        for linear_x, angular_z, steps in maneuvers:
            if self.qr_data: # Eger hareket sirasinda QR gorurse hemen dur
                rospy.loginfo("HAREKET SIRASINDA QR BULUNDU!")
                self.cmd_vel_pub.publish(Twist())
                return True
                
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            
            for _ in range(steps):
                self.cmd_vel_pub.publish(msg)
                rate.sleep()
        
        self.cmd_vel_pub.publish(Twist()) # Dur
        return False

    def go_to_pose(self, pose_data, retry_allowed=True):
        """Hedefe gider. Basarisiz olursa 'execute_recovery_behavior' cagirip tekrar dener."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose_data['position']['x']
        goal.target_pose.pose.position.y = pose_data['position']['y']
        goal.target_pose.pose.orientation.z = pose_data['orientation']['z']
        goal.target_pose.pose.orientation.w = pose_data['orientation']['w']
        
        rospy.loginfo(f"Hedef Gonderiliyor -> X:{goal.target_pose.pose.position.x:.2f} Y:{goal.target_pose.pose.position.y:.2f}")
        
        self.move_base_client.send_goal(goal)
        
        # 45 saniye zaman asimi [Gorev 5.3]
        finished = self.move_base_client.wait_for_result(rospy.Duration(45.0))

        if finished:
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hedefe VARILDI.")
                return True
            else:
                rospy.logwarn(f"Hedefe varilamadi. State: {state}")
        else:
            rospy.logwarn("Zaman asimi (Timeout) oldu. Navigasyon iptal ediliyor.")
            self.move_base_client.cancel_goal()

        # Eger buraya geldiysek basarisiz olmusuzdur. Kurtarma (Recovery) deneyelim mi?
        if retry_allowed:
            self.execute_recovery_behavior()
            rospy.loginfo("Simdi hedefe gitmeyi TEKRAR deniyorum...")
            # Recursive cagri (bu sefer retry_allowed=False yapiyoruz ki sonsuz donguye girmesin)
            return self.go_to_pose(pose_data, retry_allowed=False)
        
        return False

    def verify_qr_code(self, expected_qr):
        rospy.loginfo(f"QR Kontrolu: Beklenen '{expected_qr}'")
        self.qr_data = None # Eski veriyi temizle
        
        # 1. Asama: Durup bekle (3 saniye)
        timeout = rospy.Time.now() + rospy.Duration(3.0)
        while rospy.Time.now() < timeout:
            if self.qr_data:
                if self.qr_data.strip() == expected_qr.strip():
                    return True
            rospy.sleep(0.2)
        
        # 2. Asama: Bulamadiysa Arama Manevrasini Baslat [Gorev 5.2]
        if self.search_for_qr_maneuver():
            # Manevra sirasinda veya sonunda buldu mu tekrar bakalim
            if self.qr_data and self.qr_data.strip() == expected_qr.strip():
                return True
        
        rospy.logerr("QR Kodu tum cabalara ragmen bulunamadi.")
        return False

    def execute_cleaning(self, waypoints):
        rospy.loginfo(f"Temizlik Rotasi Basliyor ({len(waypoints)} nokta).")
        for i, pose in enumerate(waypoints):
            rospy.loginfo(f"--- Temizlik Noktasi {i+1} ---")
            # Temizlik noktasina giderken de navigasyon kurtarma ozelligini kullansin
            if self.go_to_pose(pose, retry_allowed=True):
                rospy.sleep(1) # Temizlik yapiyormus gibi bekle
            else:
                rospy.logwarn(f"Temizlik noktasi {i+1} es gecildi.")

    def run(self):
        rospy.sleep(1)
        if not self.mission_data: return

        for room in self.mission_data.get('rooms', []):
            room_name = room.get('name', 'Bilinmeyen')
            rospy.loginfo("__________________________________________________")
            rospy.loginfo(f"GOREV BASLIYOR: {room_name.upper()}")
            
            # 1. ADIM: Odaya Git
            entry_pose = room['entry_pose']
            if self.go_to_pose(entry_pose, retry_allowed=True):
                
                # 2. ADIM: QR Kontrol
                qr_text = room.get('qr_data', '')
                if self.verify_qr_code(qr_text):
                    rospy.loginfo(f">>> {room_name} DOGRULANDI! Temizlige geciliyor... <<<")
                    
                    # 3. ADIM: Temizlik
                    cleaning_poses = room.get('cleaning_poses', [])
                    self.execute_cleaning(cleaning_poses)
                    
                    self.task_report[room_name] = "SUCCESS"
                else:
                    self.task_report[room_name] = "SKIPPED (QR Fail)"
            else:
                rospy.logerr(f"{room_name} girisine ulasilamadi!")
                self.task_report[room_name] = "FAIL (Nav Error)"

        rospy.loginfo("\n********** GENEL GOREV RAPORU **********")
        for oda, durum in self.task_report.items():
            rospy.loginfo(f"{oda}: {durum}")
        rospy.loginfo("****************************************")

if __name__ == '__main__':
    try:
        tm = TaskManager()
        tm.run()
    except rospy.ROSInterruptException:
        pass
