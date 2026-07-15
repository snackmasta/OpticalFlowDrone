# Panduan Teoritis dan Implementasi Complementary Filter

Dokumen ini menjelaskan secara mendalam tentang **Complementary Filter** yang digunakan pada proyek **Optical Flow Drone** untuk menggabungkan data dari sensor giroskop, akselerometer (MPU6050), dan kompas (HMC5883L) guna mendapatkan estimasi sikap (*attitude*) yang akurat dan responsif.

---

## 1. Pendahuluan & Prinsip Kerja

*Complementary Filter* adalah algoritma penyaringan linear sederhana yang menggabungkan pembacaan dari dua sensor berbeda untuk menutupi kelemahan masing-masing sensor:

1. **Giroskop (High-Pass Source)**:
   * **Kelebihan**: Sangat responsif terhadap perubahan sudut cepat (frekuensi tinggi).
   * **Kekurangan**: Mengalami akumulasi kesalahan seiring waktu (*drift*) karena integrasi numerik dari derau kecil (*bias*).
2. **Akselerometer & Kompas (Low-Pass Source)**:
   * **Kelebihan**: Stabil dalam jangka panjang (frekuensi rendah). Akselerometer mengacu pada gravitasi bumi untuk mendeteksi horison datar, sedangkan kompas mengacu pada medan magnet bumi untuk mendeteksi arah Utara.
   * **Kekurangan**: Sangat rentan terhadap derau getaran dan percepatan linear sesaat (frekuensi tinggi).

Prinsip kerja *Complementary Filter* adalah melewatkan data giroskop melalui filter lolos-tinggi (*high-pass*) dan data akselerometer/kompas melalui filter lolos-rendah (*low-pass*), lalu menjumlahkannya.

```
                  ┌──────────────────────┐
Gyro Rate (ω) ───>│ Integrasi & High-Pass│───┐
                  └──────────────────────┘   │
                                             ▼
                                           ( + ) ───> Estimasi Sudut (θ)
                                             ▲
                  ┌──────────────────────┐   │
Accel/Compass ───>│       Low-Pass       │───┘
                  └──────────────────────┘
```

---

## 2. Pemodelan Matematika

### A. Sudut Akselerometer (Roll & Pitch)
Akselerometer mendeteksi komponen gravitasi bumi pada sumbu tubuh drone untuk menentukan sudut kemiringan statis:

$$\theta_{\text{acc}} = \text{pitch}_{\text{acc}} = \operatorname{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right) \cdot \left(\frac{180}{\pi}\right)$$

$$\phi_{\text{acc}} = \text{roll}_{\text{acc}} = \operatorname{atan2}(a_y, a_z) \cdot \left(\frac{180}{\pi}\right)$$

*Dimana:*
* $a_x, a_y, a_z$ adalah percepatan dalam satuan $g$ pada sumbu tubuh drone.

#### Contoh Perhitungan:
Jika sensor akselerometer membaca nilai percepatan:
* $a_x = -0.1736\text{ g}$
* $a_y = 0.0\text{ g}$
* $a_z = 0.9848\text{ g}$

Perhitungan Sudut:
$$\theta_{\text{acc}} = \operatorname{atan2}\left(-(-0.1736), \sqrt{0.0^2 + 0.9848^2}\right) = \operatorname{atan2}(0.1736, 0.9848) \approx 0.1746\text{ rad} \approx 10.0^\circ$$
$$\phi_{\text{acc}} = \operatorname{atan2}(0.0, 0.9848) = 0.0\text{ rad} = 0.0^\circ$$

---

### B. Integrasi Giroskop
Sudut dihitung dengan mengintegrasikan kecepatan sudut terhadap waktu ($dt$):

$$\theta_{\text{gyro}, t} = \theta_{t-1} + \omega_y \cdot dt$$

$$\phi_{\text{gyro}, t} = \phi_{t-1} + \omega_x \cdot dt$$

*Dimana:*
* $\omega_x, \omega_y$ adalah kecepatan sudut giroskop pada sumbu roll dan pitch (dalam derajat per detik / $^\circ/\text{s}$).
* $dt$ adalah selang waktu sampling.

#### Contoh Perhitungan:
Jika pada sudut sebelumnya ($t-1$) adalah $\theta_{t-1} = 9.5^\circ$ dan $\phi_{t-1} = -0.5^\circ$, dengan kecepatan sudut $\omega_y = 5.0^\circ/\text{s}$, $\omega_x = -2.0^\circ/\text{s}$, dan $dt = 0.02\text{ s}$:
$$\theta_{\text{gyro}, t} = 9.5^\circ + (5.0^\circ/\text{s} \cdot 0.02\text{ s}) = 9.5^\circ + 0.1^\circ = 9.6^\circ$$
$$\phi_{\text{gyro}, t} = -0.5^\circ + (-2.0^\circ/\text{s} \cdot 0.02\text{ s}) = -0.5^\circ - 0.04^\circ = -0.54^\circ$$

---

### C. Persamaan Filter Utama (The Blend)
Menggabungkan kedua estimasi di atas menggunakan faktor bobot $\alpha$ (konstanta filter). 

Persamaannya dapat dipecah menjadi dua bagian utama:

$$\theta_t = \underbrace{\alpha \cdot (\theta_{t-1} + \omega_y \cdot dt)}_{\text{Bagian Giroskop (96\%)}} + \underbrace{(1 - \alpha) \cdot \theta_{\text{acc}}}_{\text{Bagian Akselerometer (4\%)}}$$

$$\phi_t = \underbrace{\alpha \cdot (\phi_{t-1} + \omega_x \cdot dt)}_{\text{Bagian Giroskop (96\%)}} + \underbrace{(1 - \alpha) \cdot \phi_{\text{acc}}}_{\text{Bagian Akselerometer (4\%)}}$$

#### Penjelasan Komponen:
1. **Bagian Giroskop (Kiri)**: $\alpha \cdot (\theta_{t-1} + \omega_y \cdot dt)$
   * $(\theta_{t-1} + \omega_y \cdot dt)$ adalah **sudut baru hasil integrasi giroskop** (kecepatan sudut dikali selang waktu $dt$, ditambahkan ke sudut sebelumnya).
   * Nilai ini dikalikan dengan $\alpha$ (`0.96`), yang artinya kita **mempercayai giroskop sebesar 96%** untuk mendeteksi perubahan sudut jangka pendek yang cepat dan sensitif.

2. **Bagian Akselerometer (Kanan)**: $(1 - \alpha) \cdot \theta_{\text{acc}}$
   * $\theta_{\text{acc}}$ adalah **sudut yang dihitung dari vektor gravitasi akselerometer**.
   * Nilai ini dikalikan dengan $(1 - \alpha)$ (`0.04`), yang artinya kita **mempercayai akselerometer sebesar 4%** pada setiap siklus pembaruan. Porsi kecil ini berfungsi sebagai "jangkar" jangka panjang untuk menghilangkan pergeseran (*drift*) giroskop secara perlahan.

*Dimana:*
* $\alpha$ adalah konstanta filter (pada kode disetel ke `0.96`).


#### Contoh Perhitungan (Menggabungkan Hasil A dan B):
* $\theta_{\text{gyro}} = 9.6^\circ$ dan $\phi_{\text{gyro}} = -0.54^\circ$
* $\theta_{\text{acc}} = 10.0^\circ$ dan $\phi_{\text{acc}} = 0.0^\circ$

Blending Sikap Akhir:
$$\theta_t = 0.96 \cdot 9.6^\circ + 0.04 \cdot 10.0^\circ = 9.216^\circ + 0.4^\circ = 9.616^\circ$$
$$\phi_t = 0.96 \cdot (-0.54^\circ) + 0.04 \cdot 0.0^\circ = -0.5184^\circ + 0.0^\circ = -0.5184^\circ$$

---

## 3. Implementasi Kode Program Python

Implementasi riil dari rumus-rumus di atas dapat ditemukan pada file [optical_flow/sensor_readers.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py). Berikut adalah potongan kode (*code snippet*) utama yang berjalan pada *thread* pembaca IMU (`imu_reader`):

### A. Konversi Nilai Mentah Sensor ke Unit Fisik
Sebelum filter dijalankan, pembacaan register mentah MPU6050 dikonversi menjadi satuan gravitasi ($g$) dan derajat per detik ($^{\circ}/\text{s}$), serta dikoreksi terhadap nilai bias gyro rata-rata:

```python
# Konstanta Konversi
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

# Konversi akselerometer
xaccel_g = ax_raw / ACCEL_LSB_PER_G
yaccel_g = ay_raw / ACCEL_LSB_PER_G
zaccel_g = az_raw / ACCEL_LSB_PER_G

# Konversi giroskop dengan pengurangan bias kalibrasi
xgyro_dps = (gx_raw / GYRO_LSB_PER_DPS) - gyro_bias["x"]
ygyro_dps = (gy_raw / GYRO_LSB_PER_DPS) - gyro_bias["y"]
zgyro_dps = (gz_raw / GYRO_LSB_PER_DPS) - gyro_bias["z"]
```

### B. Perhitungan Sudut Akselerometer (Akselerometer Tilt)
Sudut akselerometer dihitung menggunakan fungsi `accel_to_roll_pitch`:

```python
def accel_to_roll_pitch(ax_g, ay_g, az_g):
    magnitude = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
    if magnitude < 0.1:
        return 0.0, 0.0

    ax_g /= magnitude
    ay_g /= magnitude
    az_g /= magnitude

    roll_deg = math.degrees(math.atan2(ay_g, az_g))
    pitch_deg = math.degrees(math.atan2(-ax_g, math.sqrt((ay_g * ay_g) + (az_g * az_g))))
    return normalize_angle_deg(roll_deg), normalize_angle_deg(pitch_deg)
```

### C. Integrasi Giroskop dan Blending Complementary Filter
Langkah integrasi temporal $dt$ dan penggabungan filter utama dijalankan secara periodik:

```python
# Menghitung selang waktu dt
dt = now - last_imu_ts

if 0 < dt < 0.1:
    # 1. Integrasi sudut giroskop (Roll, Pitch, Yaw)
    roll_gyro_deg = attitude_state["roll_deg"] + (xgyro_dps * dt)
    pitch_gyro_deg = attitude_state["pitch_deg"] + (ygyro_dps * dt)
    yaw_gyro_deg = attitude_state["yaw_deg"] + (zgyro_dps * dt)

    # 2. Blending Complementary Filter utama (Roll & Pitch)
    attitude_state["roll_deg"] = normalize_angle_deg(
        (COMPLEMENTARY_FILTER_ALPHA * roll_gyro_deg)
        + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * roll_accel_deg)
    )
    
    attitude_state["pitch_deg"] = normalize_angle_deg(
        (COMPLEMENTARY_FILTER_ALPHA * pitch_gyro_deg)
        + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * pitch_accel_deg)
    )
    
    # 3. Blending Yaw dengan referensi absolute heading dari Kompas HMC5883L
    if compass_heading_deg is not None and compass_age_s <= COMPASS_FRESHNESS_THRESHOLD_S:
        attitude_state["yaw_deg"] = blend_angle_deg(
            yaw_gyro_deg,
            compass_heading_deg,
            1.0 - COMPLEMENTARY_FILTER_ALPHA,
        )
    else:
        attitude_state["yaw_deg"] = normalize_angle_deg(yaw_gyro_deg)
```

---



## 3. Karakteristik Frekuensi (Cut-off Frequency)

Hubungan antara konstanta filter $\alpha$, interval waktu sampling ($dt$), dan konstanta waktu filter ($\tau$) didefinisikan sebagai:

$$\tau = \frac{\alpha \cdot dt}{1 - \alpha}$$

Frekuensi pemotongan (*cut-off frequency* / $f_c$) dari filter ini adalah:

$$f_c = \frac{1}{2\pi\tau} = \frac{1 - \alpha}{2\pi \cdot \alpha \cdot dt}$$

### Contoh Kasus Pada Proyek Kita:
* Frekuensi sampling IMU: $\approx 50\text{ Hz}$ ($dt \approx 0.02\text{ detik}$).
* Konstanta filter: $\alpha = 0.96$.

Maka:
$$\tau = \frac{0.96 \cdot 0.02}{1 - 0.96} = \frac{0.0192}{0.04} = 0.48\text{ detik}$$

$$f_c = \frac{1}{2\pi \cdot 0.48} \approx 0.33\text{ Hz}$$

**Analisis Fisik**: 
Sinyal getaran atau percepatan sesaat di atas $0.33\text{ Hz}$ (seperti getaran motor drone) akan diredam pada jalur akselerometer. Sebaliknya, perubahan sudut lambat (*drift*) di bawah $0.33\text{ Hz}$ pada giroskop akan dikoreksi oleh akselerometer.

---

## 4. Contoh Perhitungan Numerik Langkah Demi Langkah

Berikut adalah simulasi langkah perhitungan filter pada satu siklus pembaruan data sensor.

### Data Awal:
* Sudut sebelumnya pada $t-1$: $\text{roll}_{prev} = 8.0^\circ$, $\text{pitch}_{prev} = 1.0^\circ$
* Waktu pembaruan: $dt = 0.02\text{ s}$
* Pembacaan giroskop (setelah dikurangi bias): $\omega_x = 12.0^\circ/\text{s}$ (roll rate), $\omega_y = -6.0^\circ/\text{s}$ (pitch rate)
* Pembacaan akselerometer (dikonversi ke sudut): $\text{roll}_{\text{acc}} = 10.0^\circ$, $\text{pitch}_{\text{acc}} = -2.0^\circ$

### Langkah 1: Integrasi Giroskop
Menghitung prediksi sudut baru murni dari giroskop:

$$\text{roll}_{\text{gyro}} = 8.0^\circ + (12.0^\circ/\text{s} \cdot 0.02\text{ s}) = 8.24^\circ$$

$$\text{pitch}_{\text{gyro}} = 1.0^\circ + (-6.0^\circ/\text{s} \cdot 0.02\text{ s}) = 0.88^\circ$$

### Langkah 2: Blending dengan Complementary Filter
Menggabungkan hasil prediksi giroskop dengan referensi akselerometer:

$$\text{roll}_{\text{fused}} = 0.96 \cdot 8.24^\circ + 0.04 \cdot 10.0^\circ = 7.9104^\circ + 0.4^\circ = 8.3104^\circ$$

$$\text{pitch}_{\text{fused}} = 0.96 \cdot 0.88^\circ + 0.04 \cdot (-2.0^\circ) = 0.8448^\circ - 0.08^\circ = 0.7648^\circ$$

**Hasil Akhir**: 
Sudut sikap drone hasil fusion adalah **Roll = $8.31^\circ$** dan **Pitch = $0.76^\circ$**. Terlihat bahwa akselerometer menarik sudut ke arah posisi horison sebenarnya secara halus tanpa membuat sudut berubah drastis secara mendadak.

---

## 5. Panduan Tuning dan Troubleshooting

* **Jika Drone Sangat Bergetar dan Estimasi Sudut Berosilasi**:
  * *Penyebab*: Getaran motor merambat ke akselerometer.
  * *Solusi*: Naikkan nilai $\alpha$ (misal ke `0.98`) untuk memperbesar konstanta waktu $\tau$, sehingga filter lebih meredam derau akselerometer.
* **Jika Sudut Terasa Lambat Kembali ke Posisi Datar (Sumbu Merayap)**:
  * *Penyebab*: Koreksi akselerometer terlalu lemah.
  * *Solusi*: Turunkan nilai $\alpha$ (misal ke `0.92` atau `0.94`) untuk memberikan bobot lebih besar pada akselerometer.
* **Kalibrasi Gyro Bias**:
  * Sangat krusial agar drone diam (*stationary*) saat dinyalakan pertama kali agar pembacaan bias $\omega_{\text{bias}}$ dapat diukur dengan tepat dan dikurangkan dari sinyal giroskop aktif: $\omega_{\text{aktif}} = \omega_{\text{raw}} - \omega_{\text{bias}}$.

---

## 6. Daftar Pustaka / Referensi Akademik

Berikut adalah sumber pustaka ilmiah yang dapat digunakan sebagai referensi penulisan untuk *Complementary Filter* pada bagian sikap wahana (*attitude estimation*):

1. **Mahony, R., Hamel, T., & Pflimlin, J. M. (2008).** *Nonlinear Complementary Filters on the Special Orthogonal Group.* IEEE Transactions on Automatic Control, 53(5), 1203-1218.
   *(Referensi utama konsep matematis penyaringan komplementer nonlinear dan pembuktian kestabilan sistem).*
2. **Valenti, R. G., Dryanovski, I., & Xiao, J. (2015).** *Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs.* Sensors, 15(8), 19302-19330.
   *(Pembahasan mendalam mengenai ekstraksi sudut kemiringan akselerometer dan implementasi kompensasi kemiringan).*
3. **Euston, M., Coote, P., Mahony, R., Kim, J., & Hamel, T. (2008).** *A Complementary Filter for Attitude Estimation of a Fixed-Wing UAV.* In 2008 IEEE/RSJ International Conference on Intelligent Robots and Systems (pp. 340-345). IEEE.
   *(Studi kasus penerapan complementary filter pada kendaraan udara nirkabel untuk estimasi sikap).*
4. **Higgins, W. T. (1975).** *A Comparison of Complementary and Kalman Filtering.* IEEE Transactions on Aerospace and Electronic Systems, (3), 321-325.
   *(Paper klasik yang membandingkan performa komparatif, kesederhanaan komputasi, dan respon frekuensi dari Complementary Filter terhadap Kalman Filter).*

