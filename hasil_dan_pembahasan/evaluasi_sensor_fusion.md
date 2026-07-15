# Laporan Evaluasi Komprehensif Sistem Fusi Sensor (Sensor Fusion Evaluation Report)

Dokumen ini menyajikan analisis hasil pengujian, evaluasi performa, dan karakteristik teknis dari algoritma fusi sensor inersia (*Inertial Measurement Unit* MPU6050) dan sensor magnetik (*Magnetometer* HMC5883L) menggunakan metode **Filter Komplementer** (*Complementary Filter*) pada drone quadcopter.

---

## 1. Pendahuluan Arsitektur Fusi Sensor

Sistem navigasi estimasi sikap (*attitude estimation*) drone ini memanfaatkan fusi dari tiga sensor utama untuk mendapatkan sudut *Roll* ($\phi$), *Pitch* ($\theta$), dan *Yaw* ($\psi$) secara akurat dan bebas dari hanyatan (*drift*) maupun derau (*noise*). Karakteristik fusi sensor dirancang sebagai berikut:

- **Filter Lolos-Tinggi (High-Pass Filter - HPF)** diterapkan pada integrasi kecepatan sudut giroskop untuk mereduksi galat hanyatan frekuensi rendah akumulatif.
- **Filter Lolos-Rendah (Low-Pass Filter - LPF)** diterapkan pada akselerometer dan magnetometer untuk meredam derau getaran mekanis motor dan interferensi elektromagnetik frekuensi tinggi.
- **Konstanta Pemilah ($\alpha = 0,96$)** membagi porsi bobot fusi sebesar $96\%$ untuk integrasi giroskop (estimasi dinamis cepat) dan $4\%$ untuk acuan absolut akselerometer/magnetometer (koreksi statis lambat).

---

## 2. Analisis Hasil Kalibrasi Bias Giroskop

Sebelum fusi filter dilakukan, giroskop wajib melewati tahapan kalibrasi bias statis selama **10 detik** (1000 sampel pada rate $100\text{ Hz}$). Hasil kalibrasi bias giroskop stasis dirangkum pada **Tabel 1**.

<p align="center"><b>Tabel 1.</b> Konstanta Bias Giroskop Hasil Kalibrasi Statis</p>

| Sumbu Sensor | Nilai Bias Terukur | Unit | Deskripsi Fisik / Penyebab |
|:------------:|:------------------:|:----:|:---------------------------|
| **Sumbu-X (GX)** | $-8,1874$ | dps ($^\circ/\text{s}$) | Deviasi tinggi akibat ketidakjajaran mekanis chip (*mounting tilt*) dan tegangan solder. |
| **Sumbu-Y (GY)** | $+0,5673$ | dps ($^\circ/\text{s}$) | Deviasi rendah (normal) sesuai toleransi pabrik MEMS. |
| **Sumbu-Z (GZ)** | $-0,2714$ | dps ($^\circ/\text{s}$) | Deviasi rendah (normal), merupakan offset konstan pada sumbu vertikal. |

### 2.1. Dampak Kalibrasi Terhadap Integrasi Sudut
Berdasarkan visualisasi data pada berkas [gyro_calibration_plot.png](gyro_calibration_plot.png):
* **Tanpa Kalibrasi**: Galat bias sumbu-X yang kecil ($-8,1874\text{ dps}$) terintegrasi secara linier terhadap waktu ($\theta = \int \omega \, dt$), mengakibatkan estimasi sudut *Roll* hanyut (*drift*) hingga **$-81,87^\circ$** hanya dalam waktu **10 detik stasis**. Hal ini akan membuat flight controller mendeteksi sudut miring palsu yang sangat ekstrem.
* **Dengan Kalibrasi**: Setelah bias dikurangkan, kecepatan sudut stasis berosilasi mulus di sekitar garis $0\text{ dps}$ dengan deviasi standar $\sigma = 0,0150\text{ dps}$. Hasil integrasi sudut setelah kalibrasi tetap stabil horizontal mendekati **$0,00^\circ$** tanpa ada kecenderungan drift.

---

## 3. Evaluasi Performa Fusi Sikap Roll & Pitch

Pengujian dinamis dilakukan selama **20 detik** (968 sampel pada rate $50\text{ Hz}$) untuk menguji performa filter komplementer dalam mereduksi derau dan hanyatan pada sumbu *Roll* dan *Pitch*. Grafik hasil pengujian disimpan di [roll_comparison_plot.png](roll_comparison_plot.png) dan [pitch_comparison_plot.png](pitch_comparison_plot.png).

### 3.1. Akurasi Kuantitatif (RMSE)
Akurasi estimasi filter komplementer terhadap acuan percepatan dinilai menggunakan metrik *Root Mean Square Error* (RMSE) pada **Tabel 2**.

<p align="center"><b>Tabel 2.</b> Metrik RMSE Fusi Roll & Pitch</p>

| Sumbu Sikap | Parameter Uji | RMSE (derajat) | Hasil Analisis |
|:-----------:|:-------------:|:--------------:|:---------------|
| **Roll ($\phi$)** | Roll CF vs Roll Accel | **$5,6931^\circ$** | Filter mampu meredam derau puncak akselerometer sebesar $5,69^\circ$ tanpa kehilangan respon dinamis gerakan. |
| **Pitch ($\theta$)** | Pitch CF vs Pitch Accel | **$3,7642^\circ$** | Meredam getaran transisi cepat akselerometer dengan tingkat presisi penyaringan mencapai $3,76^\circ$. |

### 3.2. Laju Hanyatan (Drift Rate) Integrasi Giroskop
Tanpa fusi akselerometer, laju hanyatan giroskop murni yang terkalibrasi adalah:
* **Sumbu Roll**: $0,2195^\circ/\text{detik}$ (Total hanyatan $4,38^\circ$ dalam 20 detik).
* **Sumbu Pitch**: $0,2342^\circ/\text{detik}$ (Total hanyatan $4,68^\circ$ dalam 20 detik).

Filter komplementer secara sukses menghilangkan akumulasi hanyatan ini dengan terus menarik balik estimasi sudut ke vektor gravitasi akselerometer.

---

## 4. Evaluasi Performa Fusi Sumbu Yaw dengan Magnetometer

Koreksi sumbu *Yaw* ($\psi$) diuji dengan menggabungkan integrasi giroskop Z dengan data arah hadap absolut dari *Magnetometer* HMC5883L. Grafik hasil fusi disajikan di [yaw_comparison_plot.png](yaw_comparison_plot.png).

### 4.1. Analisis Alinyemen Skala NMEA [0, 360]
Sesuai implementasi *flight controller*, data heading dikonversi ke format koordinat global standard $[0, 360]^\circ$. Proses fusi bekerja dengan menyelaraskan arah sudut giroskop (sistem koordinat berlawanan arah jarum jam) dengan kompas (arah jarum jam) menggunakan fungsi inversi:
$$\text{Compass\_Heading\_Deg} = \text{normalize\_angle\_deg}(-\text{Raw\_Heading})$$

Hasil pengujian membuktikan bahwa **Yaw CF** stabil berhimpit pada acuan kompas stasis di sekitar **$257^\circ\text{ s.d. } 258^\circ$**, menghilangkan efek transient awal secara instan berkat alinyemen parameter awal pada awal start program.

### 4.2. Perbandingan Karakteristik Hanyatan Yaw
Perbandingan performa navigasi yaw dirangkum pada **Tabel 3**.

<p align="center"><b>Tabel 3.</b> Komparasi Akurasi dan Stabilitas Sudut Yaw</p>

| Metode Navigasi | Nilai Drift (20 detik) | Laju Drift | Stabilitas Sikap |
|:----------------|:----------------------:|:----------:|:-----------------|
| **Giroskop Z Murni** | $-40,2984^\circ$ | **$2,0152^\circ/\text{s}$** | **Buruk**. Hanyat meluncur jauh tanpa batas acuan. |
| **Magnetometer Mentah** | $0,00^\circ$ (Stasis) | $0,0000^\circ/\text{s}$ | **Bising**. Fluktuasi derau pembacaan mencapai $\pm 3,5^\circ$. |
| **Filter Fusi Kompas (Yaw CF)** | **$0,00^\circ$ (Stasis)** | **$0,0000^\circ/\text{s}$** | **Sangat Baik**. Stabil bebas hanyatan dan bebas derau. |

Metrik akurasi **RMSE Yaw CF terhadap Magnetometer** tercatat sebesar **$2,6879^\circ$**, membuktikan bahwa fusi filter berhasil meredam derau frekuensi tinggi magnetometer sebesar $2,68^\circ$ sambil menjaga kestabilan sudut kemudi drone.

---

## 5. Kesimpulan Evaluasi untuk Skripsi

Berdasarkan pengujian komprehensif di atas, fusi sensor dengan filter komplementer terbukti memenuhi standar kelaikan sistem estimasi posisi otonom drone:
1. **Kalibrasi Awal**: Menghilangkan bias statis $8,18\text{ dps}$ giroskop X untuk mencegah *drift* cepat pada detik-detik awal lepas landas.
2. **Kompensasi Roll/Pitch**: Filter LPF/HPF mampu meredam getaran mekanis frekuensi tinggi pada frame drone.
3. **Koreksi Yaw**: Fusi magnetometer berhasil mencegah akumulasi hanyatan arah hadap ($2,01^\circ/\text{detik}$) yang dapat menyesatkan arah navigasi dead-reckoning aliran optik (*optical flow*). Data heading siap ditransmisikan secara presisi dalam kalimat emulasi serial NMEA `$GPHDT`.
