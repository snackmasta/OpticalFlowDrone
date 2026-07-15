# Panduan Perhitungan Konstanta Sistem

Dokumen ini menjelaskan asal-usul, dasar teori, dan perhitungan nilai seluruh **konstanta utama** yang digunakan dalam perangkat lunak **Optical Flow Drone**.

---

## 1. Konstanta Sensor IMU (MPU6050)

Konstanta sensitivitas MPU6050 ditentukan berdasarkan pengaturan rentang penuh (*Full Scale Range*) register konfigurasi sensor yang digunakan.

### A. LSB per G Akselerometer (`ACCEL_LSB_PER_G = 16384.0`)
Akselerometer MPU6050 dikonfigurasi pada rentang terkecil dan paling sensitif, yaitu **$\pm 2\text{ g}$**.
* Menurut lembar spesifikasi (*datasheet*) MPU6050, konversi data 16-bit bertanda (*signed 16-bit integer* dengan rentang $-32768$ hingga $+32767$) untuk skala $\pm 2\text{ g}$ adalah:

$$\text{Sensitivitas Akselerometer} = \frac{2^{15}}{2\text{ g}} = \frac{32768}{2\text{ g}} = 16384\text{ LSB/g}$$

* **Rumus Penggunaan:**
  $$a\text{ (in g)} = \frac{\text{Raw Register Value}}{16384.0}$$

---

### B. LSB per DPS Giroskop (`GYRO_LSB_PER_DPS = 131.0`)
Giroskop dikonfigurasi pada rentang kecepatan sudut penuh sebesar **$\pm 250^\circ/\text{s}$** (derajat per detik / *degrees per second*).
* Konversi data 16-bit bertanda untuk skala $\pm 250^\circ/\text{s}$ adalah:

$$\text{Sensitivitas Giroskop} = \frac{2^{15}}{250^\circ/\text{s}} = \frac{32768}{250^\circ/\text{s}} \approx 131.072\text{ LSB/(}^\circ/\text{s)}$$

* **Rumus Penggunaan:**
  $$\omega\text{ (in dps)} = \frac{\text{Raw Register Value}}{131.0}$$

---

## 2. Konstanta Complementary Filter (`COMPLEMENTARY_FILTER_ALPHA = 0.96`)

Konstanta $\alpha$ dihitung secara matematis menggunakan hubungan antara frekuensi pemotongan derau (*cut-off frequency* / $f_c$), konstanta waktu respon ($\tau$), dan periode sampling sistem ($dt$).

### Rumus Perhitungan $\alpha$:
$$\tau = \frac{1}{2\pi \cdot f_c}$$

$$\alpha = \frac{\tau}{\tau + dt}$$

*Dimana:*
* **Periode sampling ($dt$)**: Dengan kecepatan pembacaan IMU sebesar $50\text{ Hz}$, maka $dt = 1/50 = 0.02\text{ detik}$.
* **Konstanta waktu ($\tau$)**: Waktu yang dibutuhkan filter untuk merespon $\approx 63.2\%$ terhadap perubahan masukan langkah (*step input*). Pada drone, disetel $\tau = 0.48\text{ detik}$ untuk meredam getaran frekuensi tinggi dari motor.

### Perhitungan:
$$\alpha = \frac{0.48}{0.48 + 0.02} = \frac{0.48}{0.50} = 0.96$$

Jika dibalik, frekuensi pemotongan derau ($f_c$) filter ini adalah:
$$f_c = \frac{1}{2\pi \cdot 0.48} \approx 0.33\text{ Hz}$$

---

## 3. Konstanta Sensor Kamera CSI (`CAMERA_HORIZONTAL_FOV_DEG = 62.2`)

Konstanta ini bergantung pada sensor fisik modul kamera yang digunakan.

* **Sensor**: Sony IMX219 (Raspberry Pi Camera Module v2).
* **Field of View (FOV) Horizontal**: Berdasarkan spesifikasi resmi pabrik, sudut pandang horizontal lensa adalah **$62.2^\circ$**.
* **Fungsi**: Digunakan untuk mencari panjang fokus kamera dalam satuan piksel ($f_{\text{px}}$) yang diperlukan dalam kalkulasi konversi gerak piksel ke meter.

### Rumus Panjang Fokus Piksel ($f_{\text{px}}$):
$$f_{\text{px}} = \frac{W_{\text{frame}}}{2 \cdot \tan\left(\frac{\text{FOV}_{\text{horiz}} \cdot \frac{\pi}{180}}{2}\right)}$$

Jika ukuran frame citra yang diproses adalah $W_{\text{frame}} = 640\text{ piksel}$:
$$f_{\text{px}} = \frac{640}{2 \cdot \tan\left(\frac{62.2^\circ}{2}\right)} = \frac{320}{\tan(31.1^\circ)} = \frac{320}{0.6032} \approx 530.5\text{ piksel}$$

---

## 4. Konstanta Penskalaan Gambar (`FLOW_SCALE = 0.5`)

* **Definisi**: Faktor pengecilan dimensi gambar sebelum diproses oleh algoritma DIS Optical Flow.
* **Nilai**: `0.5` ($50\%$).
* **Dasar Perhitungan**:
  Untuk menghemat daya komputasi CPU Raspberry Pi 4 agar mampu memproses gambar pada laju $60\text{ FPS}$, citra masukan beresolusi asli (misalnya $640 \times 480$) diperkecil setengahnya menjadi resolusi pemrosesan $320 \times 240$.
  
$$\text{Jumlah Piksel Baru} = (W \cdot 0.5) \times (H \cdot 0.5) = 0.25 \cdot (W \cdot H)$$

*Hasil*: Mengurangi beban pemrosesan piksel hingga **$75\%$** (hanya memproses $25\%$ data piksel asli).

---

## 5. Konstanta Geodesi Bumi (`EARTH_RADIUS = 6378137.0`)

* **Definisi**: Jari-jari ekuator bumi (semi-major axis) dalam satuan meter.
* **Dasar Referensi**: Sistem Koordinat **WGS-84** (World Geodetic System 1984) yang menjadi standar navigasi GPS global.
* **Fungsi**: Digunakan untuk memproyeksikan perpindahan jarak meter horizontal ($x$ dan $y$) ke perubahan sudut koordinat geodetis global (Latitude dan Longitude).
