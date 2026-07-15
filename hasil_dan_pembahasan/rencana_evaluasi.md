# Rencana Evaluasi Hasil dan Pembahasan (Evaluation & Analysis Plan)

Dokumen ini merencanakan metodologi pengujian, skenario evaluasi, metrik performa, dan format analisis data untuk sistem navigasi estimasi posisi otonom berbasis *optical flow* pada drone quadcopter.

---

## 1. Tujuan Evaluasi

Tujuan dari evaluasi ini adalah untuk mengukur keandalan, akurasi, latensi, dan stabilitas estimasi koordinat global sintetis hasil kalkulasi *optical flow* dan fusi sensor inersia sebelum diintegrasikan secara penuh pada wahana terbang. Pengujian difokuskan pada simulasi meja uji (*bench test*) secara statis dan dinamis terkendali.

---

## 2. Skenario Pengujian (Testing Scenarios)

Pengujian dibagi menjadi 5 skenario utama yang dirancang untuk mengisolasi dan memvalidasi setiap fungsi algoritma secara bertahap. Rincian skenario dijabarkan pada **Tabel 1**.

<p align="center"><b>Tabel 1.</b> Ringkasan Skenario Pengujian Sistem</p>

| No | Nama Skenario | Deskripsi Prosedur | Target Validasi |
|:--:|:--------------|:-------------------|:----------------|
| 1 | **Uji Hanyatan Statis (Stasis/Drift Test)** | Drone diletakkan dalam kondisi diam sempurna di atas permukaan bertekstur selama 5 menit. Data posisi ($X, Y$) direkam secara kontinu. | Mengukur akumulasi hanyatan (*drift rate*) posisi akibat derau (*noise*) sensor dan ketidaksempurnaan estimasi aliran piksel stasis. |
| 2 | **Uji Pergerakan Linier Terkontrol (Linear Translation Test)** | Drone digeser secara manual mengikuti jalur pemandu fisik sepanjang 1 meter pada sumbu-X dan sumbu-Y secara bergantian pada ketinggian tetap (1 meter). | Mengukur akurasi metrik penskalaan kecepatan (*metric velocity scaling*) dan integrasi jarak fisik sesungguhnya vs estimasi sensor. |
| 3 | **Uji Kompensasi Kemiringan (Tilt Compensation Test)** | Drone dirotasikan secara rotasi murni (*roll* dan *pitch* murni) tanpa pergeseran linier di atas meja uji. | Memvalidasi keefektifan kompensasi rotasi kamera (*tilt compensation*). Target: Posisi terkompensasi tetap mendekati nol. |
| 4 | **Uji Penskalaan Tinggi (Altitude Scaling Test)** | Drone diletakkan stasis pada ketinggian yang bervariasi (0,5m, 1,0m, dan 1,5m) di atas permukaan uji, kemudian digeser sepanjang 1 meter. | Memverifikasi bahwa data jarak vertikal dari LiDAR TF-Mini digunakan dengan benar sebagai faktor skala metrik kecepatan. |
| 5 | **Uji Keandalan Injeksi GPS NMEA (GPS Injection Reliability)** | Menguji kontinuitas pengiriman kalimat NMEA via UART serial (`/dev/ttyAMA2` @38400 bps) dan memantau status EKF pada *Flight Controller*. | Memastikan *update rate* stabil pada $10\text{ Hz}$, latensi transmisi rendah, dan *Flight Controller* berhasil memperoleh status *RTK Fixed GPS lock* secara emulasi. |

---

## 3. Metrik Evaluasi (Performance Metrics)

Akurasi sistem akan dianalisis secara kuantitatif menggunakan beberapa metrik statistika utama yang ditunjukkan pada **Tabel 2**.

<p align="center"><b>Tabel 2.</b> Metrik Kuantitatif Evaluasi Performa</p>

| No | Metrik Performa | Formula / Rumus | Deskripsi Fungsi |
|:--:|:----------------|:---------------:|:-----------------|
| 1 | **Root Mean Square Error (RMSE)** | $\text{RMSE} = \sqrt{\frac{1}{n} \sum_{i=1}^n (x_i - \hat{x}_i)^2}$ | Mengukur deviasi kuadratik rata-rata antara posisi estimasi ($\hat{x}$) dan posisi referensi fisik ($x$). |
| 2 | **Mean Absolute Error (MAE)** | $\text{MAE} = \frac{1}{n} \sum_{i=1}^n \vert v_i - \hat{v}_i \vert$ | Mengukur rata-rata magnitudo kesalahan absolut pada estimasi kecepatan linier. |
| 3 | **Drift Rate Posisi** | $\text{Drift} = \frac{\Delta\text{Posisi (cm)}}{\Delta t \text{ (detik)}}$ | Mengukur laju akumulasi galat posisi per detik saat drone dalam kondisi stasis. |
| 4 | **Packet Success Rate (PSR)** | $\text{PSR} = \frac{\text{Data Diterima FC}}{\text{Data Dikirim RPi}} \times 100\%$ | Mengukur keandalan link serial UART dan mendeteksi adanya paket data GPS NMEA yang hilang (*drop rate*). |

---

## 4. Rencana Visualisasi dan Pembahasan Data

Hasil pengujian dari kelima skenario di atas akan disajikan dan dibahas pada dokumen laporan pengujian dengan format sebagai berikut:

### 4.1. Analisis Kualitatif & Kuantitatif Kompensasi Rotasi
Menyajikan grafik perbandingan antara *Raw Velocity* (tanpa kompensasi kemiringan) vs *Compensated Velocity* (setelah kompensasi kemiringan). Pembahasan akan fokus pada seberapa efektif giroskop MPU6050 mereduksi galat translasi semu akibat gerakan rotasi drone.

### 4.2. Plot Lintasan 2D (X-Y Scatter Plot)
Memplot koordinat posisi $X$ dan $Y$ hasil kalkulasi *dead reckoning* terhadap lintasan acuan nyata (misal: bentuk kotak 1x1 meter). Grafik ini digunakan untuk memvisualisasikan akumulasi galat posisi (*drift*) dan distorsi geometris lintasan.

### 4.3. Analisis Stabilitas Siklus Pengiriman Data
Menyajikan histogram interval waktu (*time delta*) transmisi serial untuk memverifikasi kestabilan frekuensi injeksi data GPS sintetis $10\text{ Hz}$. Ketidakstabilan waktu kirim (*jitter*) dianalisis dampaknya terhadap estimasi *Extended Kalman Filter* (EKF) pada *Flight Controller*.

---

## 5. Metodologi Pengumpulan Data (Data Collection Methodology)

Prosedur sistematis untuk mengumpulkan data selama pengujian diatur melalui langkah-langkah kerja pada **Tabel 3**.

<p align="center"><b>Tabel 3.</b> Langkah-Langkah Metodologi Pengumpulan Data</p>

| No | Tahap Kerja | Prosedur Pelaksanaan | Output / Output Hasil |
|:--:|:------------|:---------------------|:----------------------|
| 1 | **Persiapan & Kalibrasi Awal** | 1. Drone diletakkan pada titik koordinat mula ($X=0, Y=0$) di atas meja uji.<br>2. Verifikasi pencahayaan konstan (min. 300 lux) dan pemandu fisik (penggaris/grid).<br>3. Jalankan prosedur kalibrasi bias sensor IMU dalam keadaan stasis. | Nilai koreksi *offset* giroskop & akselerometer tersimpan. |
| 2 | **Pencatatan Data Aktivitas** | 1. Aktifkan logger data internal pada *Companion Computer* (RPi).<br>2. Hubungkan link serial GCS (Mission Planner/QGC) untuk logging telemetri FC.<br>3. Jalankan skenario gerakan fisik yang ditentukan secara bergantian. | Rekaman log data paralel (RPi CSV Log & FC Dataflash Log). |
| 3 | **Penyelarasan Waktu (Sync)** | 1. Lakukan manuver sentakan kecil yang terbaca jelas pada sensor inersia dan aliran optik sebelum memulai pengujian.<br>2. Gunakan penanda sentakan (*spike marker*) ini untuk menyelaraskan linimasa log RPi dan FC. | Linimasa data yang sinkron untuk analisis galat. |
| 4 | **Pengekstrakan Data** | 1. Salin file CSV log dari RPi ke laptop analisis.<br>2. Unduh file log internal (.bin/.tlog) dari memori *Flight Controller* menggunakan GCS. | Berkas log mentah siap olah. |

---

## 6. Daftar Data yang Diperlukan (Required Data Checklist)

Untuk melakukan analisis performa sistem secara komprehensif, data yang wajib diperoleh dari pengujian dikelompokkan pada **Tabel 4**.

<p align="center"><b>Tabel 4.</b> Check-list Kebutuhan Data Evaluasi</p>

| Kategori Data | Sumber Data | Parameter Spesifik yang Harus Diambil | Kegunaan Analisis |
|:--------------|:-----------:|:--------------------------------------|:------------------|
| **Data Navigasi Estimasi** | RPi (Shared Memory Log) | `timestamp`, `x_cm`, `y_cm`, `vx`, `vy`, `alt`, `heading` | Data utama hasil perhitungan *optical flow* dan *dead reckoning* untuk dicocokkan dengan referensi. |
| **Data Navigasi Mentah** | RPi (Shared Memory Log) | `x_raw_cm`, `y_raw_cm`, `vx_raw`, `vy_raw` | Sebagai pembanding untuk menguji efektivitas algoritma kompensasi kemiringan (*tilt compensation*). |
| **Sikap Wahana (Attitude)** | RPi & FC | `roll_deg`, `pitch_deg`, `yaw_deg` (dari MPU6050 & EKF FC) | Memvalidasi akurasi fusi filter komplementer dan komparasi dengan filter internal Flight Controller. |
| **Data GPS Emulasi** | FC (Dataflash Log) | `GPS.Lat`, `GPS.Lon`, `GPS.Alt`, `GPS.Spd`, `GPS.Status` | Memvalidasi apakah FC menerima dan memproses data koordinat GPS sintetis dengan benar tanpa kegagalan EKF. |
| **Ground Truth (Acuan Nyata)** | Pengukuran Fisik | Jarak geser fisik (meteran), durasi waktu (stopwatch), kondisi diam stasis | Referensi absolut (nilai $x$ sebenarnya) untuk menghitung RMSE dan laju hanyatan (*drift rate*). |

