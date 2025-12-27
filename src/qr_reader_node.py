#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyzbar import pyzbar
from std_msgs.msg import String

class QRReader:
    def __init__(self):
        # ROS Düğümünü (Node) başlat
        rospy.init_node('qr_reader_node', anonymous=True)
        
        # Görüntüleri ROS'tan OpenCV formatına çevirmek için bir köprü oluştur
        self.bridge = CvBridge()
        
        # QR koddan okunan veriyi yayınlamak için bir Publisher oluştur
        # /qr_code topic'ine String tipinde mesajlar yayınlayacak
        self.qr_pub = rospy.Publisher('/qr_code', String, queue_size=10)
        
        # Robotun kamera görüntüsüne abone ol (subscribe)
        # /camera/rgb/image_raw topic'inden Image tipinde mesajlar geldikçe
        # self.image_callback fonksiyonunu çağır
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        rospy.loginfo("QR Reader Node baslatildi. Kamera goruntusu bekleniyor...")

    def image_callback(self, data):
        try:
            # Gelen ROS Image mesajını OpenCV görüntüsüne çevir
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Görüntüdeki QR kodlarını (veya barkodları) bul
        barcodes = pyzbar.decode(cv_image)
        
        # Eğer bir veya daha fazla QR kodu bulunduysa
        if barcodes:
            # Bulunan her bir QR kodu için döngüye gir
            for barcode in barcodes:
                # QR kodun içindeki veriyi string'e çevir
                barcode_data = barcode.data.decode("utf-8")
                rospy.loginfo("QR Kodu Bulundu! Veri: %s" % barcode_data)
                
                # Okunan veriyi /qr_code topic'ine yayınla
                self.qr_pub.publish(barcode_data)
                
                # --- Görselleştirme için (opsiyonel) ---
                # QR kodun etrafına bir dikdörtgen çiz
                (x, y, w, h) = barcode.rect
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                
                # QR kodun verisini resmin üzerine yaz
                text = "{}".format(barcode_data)
                cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                # -----------------------------------------

        # (Opsiyonel) Kameradan gelen görüntüyü bir pencerede göster
        # Projenin çalışması için zorunlu değil, sadece test amaçlı
        cv2.imshow("Kamera Goruntusu", cv_image)
        cv2.waitKey(1)

    def run(self):
        # Düğümün kapanana kadar çalışmasını sağla
        rospy.spin()

if __name__ == '__main__':
    try:
        qr_reader = QRReader()
        qr_reader.run()
    except rospy.ROSInterruptException:
        pass
