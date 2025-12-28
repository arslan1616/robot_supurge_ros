
# Süpürge Robotu Projesi: Oda Bazlı Temizlik ve QR Doğrulama

**Hazırlayan:** İbrahim Arslan
**Öğrenci No:** 211229048
**Ders:** Robotiğe Giriş
**Tarih:** Aralık 2025

## 1. Proje Tanımı ve Amacı
Bu proje, KTÜN Robotiğe Giriş dersi final uygulaması kapsamında geliştirilmiştir. Projenin temel amacı; Gazebo simülasyon ortamında çalışan bir TurtleBot3 robotunun, evin haritasını çıkardıktan sonra (SLAM) belirli odaları otonom olarak gezmesi, oda girişlerindeki QR kodları kamera aracılığıyla okuyarak konumunu doğrulaması ve her oda için belirlenen temizlik rotalarını takip etmesidir.

Proje, otonom mobil robotlarda haritalama, lokalizasyon, navigasyon, görüntü işleme ve görev planlama (task management) yeteneklerini birleştiren bütünleşik bir sistem sunmaktadır.

## 2. Kullanılan Yöntem ve Teknolojiler
Proje geliştirme sürecinde aşağıdaki teknoloji ve paketler kullanılmıştır:

* **İşletim Sistemi ve Ara Yazılım:** Ubuntu 20.04 üzerinde ROS Noetic.
* **Simülasyon Ortamı:** Gazebo ve TurtleBot3 (Waffle Pi modeli).
* **Navigasyon Stack:** Haritalama için `gmapping`, lokalizasyon için `amcl`, yol planlama ve hareket kontrolü için `move_base` paketleri.
* **Görüntü İşleme:** Kamera verisinden QR kod tespiti ve çözümlenmesi için `cv_bridge` ve `pyzbar` kütüphaneleri.
* **Programlama:** Görev yöneticisi ve görüntü işleme düğümleri Python 3 ile geliştirilmiştir.

## 3. Kurulum Talimatları

Projenin çalıştırılabilmesi için gerekli ortamın hazırlanması ve bağımlılıkların yüklenmesi gerekmektedir.

**1. Çalışma Alanının Hazırlanması:**
Proje dosyaları `catkin_ws` dizini içerisine klonlanmalıdır:
```bash
cd ~/catkin_ws/src
git clone [https://github.com/arslan1616/robot_supurge_ros.git](https://github.com/arslan1616/robot_supurge_ros.git)

```

**2. Bağımlılıkların Yüklenmesi:**
Simülasyon ve görüntü işleme için gerekli paketler terminal üzerinden yüklenmelidir:

```bash
sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-navigation
sudo apt-get install python3-pip ros-noetic-cv-bridge
pip3 install pyzbar opencv-python

```

**3. Derleme İşlemi:**

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

```

## 4. Çalıştırma Adımları

Tüm sistemi (Simülasyon, Navigasyon, QR Okuyucu ve Görev Yöneticisi) başlatmak için tek bir launch dosyası hazırlanmıştır. Aşağıdaki komut ile sistem ayağa kaldırılır:

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch robot_supurge_ros main.launch

```

*Not: Simülasyonun tam olarak yüklenmesi ve robotun ilk göreve başlaması sistem performansına bağlı olarak birkaç saniye sürebilir.*

## 5. Algoritma ve Senaryo Akışı

Robotun davranış mimarisi `task_manager_node.py` içerisinde bir durum makinesi (State Machine) mantığıyla kurgulanmıştır. Akış şu şekildedir:

1. **Başlangıç (Initialization):** Robot "Salon" odasında başlar. `mission.yaml` dosyasından hedef koordinatlar ve görev parametreleri okunur.
2. **Navigasyon (Go to Room):** Robot, sıradaki odanın giriş koordinatına gitmek için `move_base` eylemini kullanır. Eğer robot hedefe ulaşamazsa veya sıkışırsa, kendi etrafında dönerek kurtarma manevrası (recovery behavior) gerçekleştirir ve tekrar dener.
3. **QR Doğrulama (Verification):** Odaya varıldığında kamera verisi analiz edilir.
* Beklenen QR kod (örneğin "ROOM=KITCHEN") okunursa görev onaylanır.
* QR kod ilk bakışta görülemezse, robot sağa-sola dönerek ve ileri-geri manevra yaparak ortamı tarar (Search Maneuver).


*(Aşağıdaki görselde robotun Gazebo ortamında QR kodu başarıyla tespit ettiği an görülmektedir)*
4. **Temizlik (Cleaning):** Doğrulama başarılı ise robot odaya girer ve tanımlanmış alt hedefleri (waypoint) sırayla gezerek temizlik simülasyonunu tamamlar.


<img width="1281" height="935" alt="image" src="https://github.com/user-attachments/assets/f459692f-ac6c-406c-9aed-6ae783561f43" />


5. **Raporlama:** Tüm odalar tamamlandığında sistem terminal üzerinden hangi odaların başarıyla tamamlandığını, hangilerinin atlandığını raporlar.

   
<img width="1247" height="950" alt="image" src="https://github.com/user-attachments/assets/649d3e27-2cb8-455b-87e6-1d1eb9301eef" />

   

## 6. Dosya Yapısı

Proje dizini aşağıdaki hiyerarşiye sahiptir:

* **config/**: Görev tanımları ve koordinatların bulunduğu `mission.yaml` dosyasını içerir.
* **launch/**: Sistemi başlatan `main.launch` ve alt sistemleri (navigasyon, simülasyon) çağıran dosyaları barındırır.
* **maps/**: SLAM algoritması ile oluşturulmuş `map.yaml` ve `map.pgm` harita dosyaları buradadır.
* **models/**: Gazebo ortamına eklenen QR kod model dosyaları (SDF ve texture).
* **src/**:
* `qr_reader_node.py`: Kamera görüntüsünü işleyen düğüm.
* `task_manager_node.py`: Robotun ana karar mekanizması ve görev yöneticisi.



## 7. Demo ve Sonuçlar

Projenin başarıyla tamamlandığını gösteren terminal çıktısı ve RViz görüntüsü aşağıda sunulmuştur. Görüldüğü üzere robot tüm odaları (Salon, Mutfak, Yatak Odası, Koridor) başarıyla gezmiş, QR kodları doğrulamış ve "SUCCESS" raporunu üretmiştir.

### Proje Demo Videosu

Projenin çalışmasını, otonom navigasyonunu ve hata yönetimi senaryolarını içeren detaylı demo videosuna aşağıdaki bağlantıdan ulaşabilirsiniz:

> **Video Linki:** [YouTube üzerinden izlemek için tıklayınız](https://youtu.be/C8LBD32-8Zk)


