# Dokumen Referensi Rumus Matematika dan Algoritma

Dokumen ini mendokumentasikan seluruh persamaan matematika, model geometri, dan algoritma sensor fusion yang digunakan dalam sistem **Optical Flow Drone**.

---

## 1. Estimasi Sikap Wahana (Attitude Estimation)

Sikap wahana (Roll, Pitch, dan Yaw) diestimasi dengan menggabungkan pembacaan sensor akselerometer, giroskop (pada MPU6050), dan kompas (HMC5883L) menggunakan *Complementary Filter*.

### A. Sudut Kemiringan dari Akselerometer
Akselerometer mengukur vektor gravitasi untuk menghitung sudut kemiringan statis (*tilt*):

$$\text{roll}_{\text{accel}} = \operatorname{atan2}(a_y, a_z) \cdot \left(\frac{180}{\pi}\right)$$

$$\text{pitch}_{\text{accel}} = \operatorname{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right) \cdot \left(\frac{180}{\pi}\right)$$

*Dimana:*
* $a_x, a_y, a_z$ adalah percepatan dalam satuan $g$ (gravitasi) pada sumbu tubuh drone.

---

### B. Integrasi Giroskop
Kecepatan sudut dari giroskop diintegrasikan terhadap waktu ($dt$) untuk mendapatkan akumulasi perubahan sudut:

$$\text{roll}_{\text{gyro}, t} = \text{roll}_{t-1} + \omega_x \cdot dt$$

$$\text{pitch}_{\text{gyro}, t} = \text{pitch}_{t-1} + \omega_y \cdot dt$$

$$\text{yaw}_{\text{gyro}, t} = \text{yaw}_{t-1} + \omega_z \cdot dt$$

*Dimana:*
* $\omega_x, \omega_y, \omega_z$ adalah kecepatan sudut (dalam derajat per detik / dps) pada masing-masing sumbu.
* $dt$ adalah interval waktu antar pembacaan sensor ($t - (t-1)$).

---

### C. Blending Complementary Filter
Data giroskop (tinggi frekuensi/sensitif perubahan cepat) dan akselerometer/kompas (rendah frekuensi/stabil jangka panjang) digabungkan menggunakan koefisien filter $\alpha = 0.96$:

#### Roll & Pitch Fusion:
$$\text{roll}_t = \alpha \cdot (\text{roll}_{t-1} + \omega_x \cdot dt) + (1 - \alpha) \cdot \text{roll}_{\text{accel}}$$

$$\text{pitch}_t = \alpha \cdot (\text{pitch}_{t-1} + \omega_y \cdot dt) + (1 - \alpha) \cdot \text{pitch}_{\text{accel}}$$

#### Yaw & Compass Fusion:
$$\text{yaw}_t = \alpha \cdot (\text{yaw}_{t-1} + \omega_z \cdot dt) + (1 - \alpha) \cdot \text{heading}_{\text{compass}}$$

---

## 2. Estimasi Gerak Visual (Visual Motion Estimation)

Untuk mengestimasi pergeseran piksel antar bingkai gambar (*frame*), digunakan pencocokan model geometri 2D Affine parsial (4 derajat kebebasan: translasi, rotasi, skala seragam) setelah penarikan vektor fitur menggunakan *Dense DIS (Dense Inverse Search) Optical Flow*.

### Model Affine Partial 2D:
Matriks transformasi $M$ yang dicocokkan menggunakan RANSAC adalah:

$$\begin{bmatrix} x_{\text{new}} \\ y_{\text{new}} \end{bmatrix} = \begin{bmatrix} s \cos\theta & -s \sin\theta \\ s \sin\theta & s \cos\theta \end{bmatrix} \begin{bmatrix} x_{\text{old}} \\ y_{\text{old}} \end{bmatrix} + \begin{bmatrix} t_x \\ t_y \end{bmatrix}$$

*Dimana:*
* $t_x, t_y$ adalah pergeseran piksel (translasi) pada citra kamera.
* $s$ adalah perubahan skala citra (karena ketinggian).
* $\theta$ adalah perubahan sudut rotasi citra (rotasi yaw kamera).

Dari matriks hasil estimasi $M$, parameter gerakan diekstraksi sebagai berikut:
$$t_x = M_{0, 2}, \quad t_y = M_{1, 2}$$
$$s = \sqrt{M_{0, 0}^2 + M_{1, 0}^2}$$
$$\theta = \operatorname{atan2}(M_{1, 0}, M_{0, 0})$$

---

## 3. Kompensasi Sudut Kemiringan (Tilt Compensation)

Ketika drone melakukan gerakan pitch atau roll, kamera ikut miring dan menghasilkan aliran optik semu (*apparent optical flow*) seolah-olah drone bergeser secara lateral. Hal ini dieliminasi menggunakan data orientasi dari IMU.

### A. Proyeksi Perubahan Sudut ke Piksel
Pergeseran sudut roll ($\Delta\phi$) dan pitch ($\Delta\theta$) diproyeksikan ke perpindahan piksel teoritis pada sensor kamera:

$$d_{\text{reticle}, x} = (\text{roll}_t - \text{roll}_{t-1}) \cdot K_{\text{roll}}$$

$$d_{\text{reticle}, y} = -(\text{pitch}_t - \text{pitch}_{t-1}) \cdot K_{\text{pitch}}$$

*Dimana:*
* $K_{\text{roll}}, K_{\text{pitch}}$ adalah rasio skala piksel per derajat kemiringan yang diperoleh dari kalibrasi HUD.

### B. Pengurangan Aliran Optik Semu
Mengurangi pergeseran sudut terproyeksi yang dikalibrasi ($\text{scale}_x, \text{scale}_y$) dari nilai translasi hasil *affine fitting* ($t_x, t_y$):

$$t_{x, \text{compensated}} = t_x - (\text{scale}_x \cdot d_{\text{reticle}, x})$$

$$t_{y, \text{compensated}} = t_y - (\text{scale}_y \cdot d_{\text{reticle}, y})$$

---

## 4. Skala Kecepatan Fisik (Physical Velocity Scaling)

Mengonversi translasi piksel kamera terkompensasi ($t_{x, \text{compensated}}, t_{y, \text{compensated}}$) menjadi kecepatan linier fisik dalam satuan meter per detik (m/s).

$$V_{x, \text{body}} = \frac{t_{x, \text{compensated}} \cdot h}{f_x \cdot dt}$$

$$V_{y, \text{body}} = -\frac{t_{y, \text{compensated}} \cdot h}{f_y \cdot dt}$$

*Dimana:*
* $h$ adalah ketinggian drone di atas tanah (dalam meter) yang dibaca dari sensor jarak/rangefinder.
* $f_x, f_y$ adalah panjang fokus lensa kamera dalam satuan piksel:
  $$f_x = \frac{\text{Lebar Frame}}{2 \cdot \tan\left(\frac{\text{FOV Horizontal}}{2}\right)}$$
* $dt$ adalah selang waktu antar bingkai citra (*frame*).

---

## 5. Koreksi Jarak Kamera (Lever-Arm Offset Correction)

Jika kamera tidak diletakkan tepat di pusat rotasi (titik berat) drone, rotasi yaw ($\omega_z$) akan menghasilkan translasi linear tambahan pada kamera. Koreksi dilakukan dengan rumus:

$$v_{\text{offset}, x} = -\omega_z \cdot \left(\frac{C_y}{100.0}\right)$$

$$v_{\text{offset}, y} = \omega_z \cdot \left(\frac{C_x}{100.0}\right)$$

$$V_{x, \text{body, comp}} = V_{x, \text{body}} - v_{\text{offset}, x}$$

$$V_{y, \text{body, comp}} = V_{y, \text{body}} - v_{\text{offset}, y}$$

*Dimana:*
* $C_x, C_y$ adalah jarak koordinat posisi kamera terhadap titik pusat rotasi drone (dalam satuan cm).
* $\omega_z$ adalah kecepatan rotasi yaw dalam radian per detik ($\text{rad/s}$).

---

## 6. Rotasi Bingkai Tubuh ke Dunia (Body-to-World Rotation)

Kecepatan linier tubuh drone ($V_{x, \text{body, comp}}, V_{y, \text{body, comp}}$) diputar menggunakan sudut yaw absolut ($\psi$) dari kompas untuk mendapatkan arah gerak relatif terhadap bumi (Timur dan Utara):

$$V_{x, \text{world}} = V_{x, \text{body, comp}} \cdot \cos(\psi) + V_{y, \text{body, comp}} \cdot \sin(\psi)$$

$$V_{y, \text{world}} = -V_{x, \text{body, comp}} \cdot \sin(\psi) + V_{y, \text{body, comp}} \cdot \cos(\psi)$$

*Dimana:*
* $\psi$ adalah sudut heading kompas dalam radian (Arah Utara = 0 derajat, berputar searah jarum jam).
* $V_{x, \text{world}}$ bernilai positif ke arah **Timur** (East).
* $V_{y, \text{world}}$ bernilai positif ke arah **Utara** (North).

---

## 7. Integrasi Posisi (Dead Reckoning)

Mengintegrasikan kecepatan dunia terhadap selang waktu secara diskrit untuk melacak posisi kumulatif (dalam cm) drone sejak sistem dinyalakan:

$$x_{\text{world}, t} = x_{\text{world}, t-1} + V_{x, \text{world}} \cdot dt \cdot 100$$

$$y_{\text{world}, t} = y_{\text{world}, t-1} + V_{y, \text{world}} \cdot dt \cdot 100$$

---

## 8. Proyeksi Koordinat Global GPS

Untuk sistem yang menyuntikkan data koordinat global GPS tiruan (injektor NMEA), jarak perpindahan dalam meter ($x_{\text{world}}$ dan $y_{\text{world}}$) diproyeksikan dari titik koordinat awal ($\text{lat}_0, \text{lon}_0$):

$$\text{lat}_t = \text{lat}_0 + \left( \frac{y_{\text{world}} / 100}{R} \right) \cdot \left(\frac{180}{\pi}\right)$$

$$\text{lon}_t = \text{lon}_0 + \left( \frac{x_{\text{world}} / 100}{R \cdot \cos(\text{lat}_t \cdot \frac{\pi}{180})} \right) \cdot \left(\frac{180}{\pi}\right)$$

*Dimana:*
* $R$ adalah jari-jari bumi ($R = 6.378.137,0\text{ meter}$).
* $\text{lat}_0, \text{lon}_0$ adalah koordinat awal titik awal penerbangan (ditentukan statis pada konfigurasi).

---

## 9. Daftar Pustaka / Sumber Referensi Akademik

Berikut adalah referensi publikasi ilmiah dan buku teks standar yang mendasari perumusan algoritma di atas:

### A. Estimasi Sikap & Complementary Filter (Bagian 1)
* **Mahony, R., Hamel, T., & Pflimlin, J. M. (2008).** *Nonlinear Complementary Filters on the Special Orthogonal Group.* IEEE Transactions on Automatic Control, 53(5), 1203-1218.
  *(Dasar teori penyaringan komplementer untuk menggabungkan data giroskop frekuensi tinggi dengan akselerometer frekuensi rendah).*
* **Valenti, R. G., Dryanovski, I., & Xiao, J. (2015).** *Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs.* Sensors, 15(8), 19302-19330.
  *(Aplikasi kompensasi kemiringan menggunakan akselerometer).*

### B. DIS Optical Flow & RANSAC Affine Fitting (Bagian 2)
* **Kroeger, T., Timofte, R., Dai, D., & Van Gool, L. (2016).** *Fast Optical Flow using Dense Inverse Search.* In European Conference on Computer Vision (ECCV) (pp. 471-488). Springer, Cham.
  *(Algoritma DIS Optical Flow yang digunakan untuk mendeteksi pergeseran piksel secara cepat pada sistem embedded seperti Raspberry Pi).*
* **Fischler, M. A., & Bolles, R. C. (1981).** *Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography.* Communications of the ACM, 24(6), 381-395.
  *(Metode RANSAC untuk memisahkan inliers gerakan tanah dari outliers derau visual).*

### C. Kompensasi Sudut & Skala Kecepatan Fisik (Bagian 3 & 4)
* **Honegger, D., Meier, L., Tanskanen, P., & Pollefeys, M. (2013).** *An Open Source and Open Hardware Visual Odometry Sensor for Micro Air Vehicles.* In 2013 IEEE International Conference on Robotics and Automation (ICRA) (pp. 4730-4737). IEEE.
  *(Dokumen referensi utama proyek PX4FLOW untuk kompensasi kemiringan berbasis giroskop dan penskalaan kecepatan linear fisik menggunakan altimeter/sonar).*

### D. Koreksi Jarak Kamera & Rotasi Koordinat (Bagian 5 & 6)
* **Farrell, J. A. (2008).** *Aided Navigation: GPS with High Rate Sensors.* McGraw-Hill, Inc.
  *(Persamaan koreksi efek tuas (Lever-Arm Offset Correction) akibat peletakan sensor/kamera yang tidak berada tepat di pusat gravitasi kendaraan).*
* **Diebel, J. (2006).** *Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Matrices.* Stanford University, 58, 1-35.
  *(Matriks rotasi sumbu koordinat dari bingkai tubuh (body-frame) ke bingkai bumi (world-frame/NED)).*

### E. Proyeksi Koordinat Global & NMEA (Bagian 8)
* **Hofmann-Wellenhof, B., Lichtenegger, H., & Wasle, E. (2007).** *GNSS—Global Navigation Satellite Systems: GPS, GLONASS, Galileo, and more.* Springer Science & Business Media.
  *(Rumus konversi jarak linear meter di permukaan bumi menjadi koordinat geodetis lintang (latitude) dan bujur (longitude) menggunakan aproksimasi jari-jari bola bumi).*

