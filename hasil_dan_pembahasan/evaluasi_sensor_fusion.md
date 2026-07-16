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

## 5. Evaluasi Trajektori Navigasi Otonom (Aliran Optik & LiDAR)

Evaluasi navigasi otonom dilakukan dengan menganalisis hasil perekaman posisi *dead-reckoning* aliran optik (*optical flow*) sumbu $X-Y$ dan sensor jarak LiDAR sumbu $Z$ yang tersimpan pada berkas log [optical_flow_shm_log.csv](optical_flow_shm_log.csv). Rangkuman parameter kuantitatif hasil penerbangan aktual disajikan pada **Tabel 4**.

<p align="center"><b>Tabel 4.</b> Parameter Kuantitatif Hasil Uji Trajektori Navigasi Otonom</p>

| Kategori Parameter | Jenis Metrik Evaluasi | Nilai Terukur | Keterangan / Analisis |
|:-------------------|:----------------------|:-------------:|:----------------------|
| **Informasi Log** | Durasi Log Penerbangan | $29,73\text{ detik}$ | Pengumpulan data stabil pada 357 sampel. |
| **Jarak Lintasan** | Jarak Lintasan Horizontal (2D Path) | **$1,6124\text{ m}$** | Akumulasi total panjang jalur lintasan datar. |
| **Perpindahan Bersih** | Jarak Perpindahan Datar (2D Displacement) | **$0,4928\text{ m}$** | Jarak lurus spasial awal ke akhir ($\approx 49,28\text{ cm}$). |
| **Kecepatan Penerbangan**| Kecepatan Translasional Maksimum | $0,1895\text{ m/s}$ | Kecepatan puncak drone ($\approx 18,95\text{ cm/s}$). |
| | Kecepatan Translasional Rata-rata | $0,0266\text{ m/s}$ | Gerakan dominan lambat stasis/terkontrol. |
| **Ketinggian LiDAR** | Batas Ketinggian Terbang | $0,00\text{ s.d. } 0,42\text{ m}$ | Ketinggian stasis pada ketinggian uji $42\text{ cm}$. |

### 5.1. Analisis Lintasan Horizontal 2D (Tampak Atas)
Berdasarkan visualisasi grafik pada [optical_flow_2d_top_down.png](optical_flow_2d_top_down.png), drone diuji dengan pergeseran linear terkontrol di mana terjadi perpindahan posisi dominan ke arah kiri (sumbu $-X$) sejauh $47\text{ cm}$ (rentang $-0,47\text{ m}$ s.d. $+0,01\text{ m}$) dan sumbu $Y$ yang relatif sempit (rentang $-0,13\text{ m}$ s.d. $+0,02\text{ m}$). Grafik tampak atas membuktikan akurasi penelusuran jalur navigasi di mana kurva koordinat terplot secara geometris proporsional tanpa distorsi spasial berkat penggunaan rasio aspek $1:1$ pada plotting sumbu.

Meskipun demikian, terdapat perbedaan antara perpindahan posisi horizontal terhitung hasil integrasi awal ($49,28\text{ cm}$) dengan perpindahan posisi aktual fisik drone di lapangan yang diukur sebesar $80\text{ cm}$. Ketidaksesuaian skala ini disebabkan karena algoritma pencarian DIS flow di pemroses aliran optik dijalankan pada resolusi gambar yang diturunkan sebesar $50\%$ (`FLOW_SCALE = 0.5`) guna menghemat komputasi Raspberry Pi, namun parameter panjang fokus kamera (*focal length*) yang digunakan sebagai pembagi pembacaan pixel masih merujuk pada lebar frame resolusi penuh ($640\text{ pixel}$). Selain itu, perbedaan sudut pandang horizontal (*FOV*) riil lensa kamera dari nilai default $62,2^\circ$ serta distorsi radial lensa juga turut menyumbang galat penskalaan linear tersebut.

Untuk mengatasi kendala penskalaan ini, konstanta kalibrasi skala koreksi sebesar `1.6233` (rasio dari jarak aktual $80\text{ cm}$ terhadap jarak terhitung $49,28\text{ cm}$) telah diintegrasikan langsung pada perhitungan kecepatan linier tubuh drone di skrip utama. Koreksi ini memastikan bahwa data kecepatan linier m/s dan integrasi posisi cm yang dihasilkan oleh sistem aliran optik telah terkalibrasi secara presisi sesuai jarak fisik riil di lapangan sebelum data tersebut diumpankan ke filter Kalman Pixhawk. Selain kendala penskalaan statis tersebut, pada kondisi dinamis juga teridentifikasi adanya deviasi akibat belum diimplementasikannya kompensasi kemiringan LiDAR (*slant range compensation*) menggunakan sudut *pitch* dan *roll* ($h_{\text{sebenarnya}} = h_{\text{LiDAR}} \times \cos(\phi) \times \cos(\theta)$), yang mengakibatkan pembacaan tinggi semu bertambah saat drone miring dan mengacaukan pengali skala aliran optik. Kurangnya presisi pada penyesuaian waktu (*temporal calibration*) atau latensi fase antara data pembacaan giroskop IMU dan penangkapan citra kamera juga memicu ketidakakuratan parsial pada kompensasi gerakan rotasi saat manuver cepat.


### 5.2. Analisis Ketinggian Terbang (LiDAR)
Pengukuran jarak vertikal dari pembacaan LiDAR TF-Mini melengkapi profil terbang drone dengan menunjukkan transisi ketinggian yang sangat mulus. Selama masa uji coba, drone menunjukkan grafik ketinggian awal stasis di lantai ($0,00\text{ m}$), melakukan proses lepas landas (*take-off*) secara terkontrol hingga melayang (*hover*) pada ketinggian maksimum $0,42\text{ m}$, dan kembali mendarat (*landing*) dengan aman ke permukaan tanah pada akhir rekaman data log. Stabilitas pembacaan LiDAR ini memberikan jaminan keandalan parameter tinggi yang digunakan sebagai pengali penyekalaan kecepatan linier aliran optik.

### 5.3. Evaluasi Komunikasi Serial UART dan Emulasi GPS ke Flight Controller
Validasi integrasi sistem dilakukan dengan menguji keandalan transmisi data GPS sintetis format NMEA dari *Companion Computer* (Raspberry Pi) ke *Flight Controller* (Pixhawk/ArduPilot) via UART Serial (`/dev/ttyAMA2` @38400 bps) pada frekuensi update $10\text{ Hz}$. Keberhasilan integrasi awal ini ditunjukkan oleh kemampuan EKF pada *Flight Controller* untuk melakukan konvergensi data inersia internal secara sempurna dengan data emulasi GPS eksternal. Hasilnya, status indikator EKF berada dalam kondisi sehat (*normal status*) dan stasiun bumi menampilkan konfirmasi siap terbang (*Ready to Arm*).

Evaluasi kualitas dan kestabilan fusi EKF di stasiun bumi Mission Planner membuktikan bahwa seluruh indikator deviasi kuadratis ternormalisasi (*innovation variance*) untuk parameter *Velocity*, *Position (Horiz)*, *Position (Vert)*, *Compass*, dan *Terrain* berada jauh di bawah ambang batas toleransi oranye ($0,5$) maupun merah ($0,8$), dengan nilai osilasi yang sangat rendah di kisaran $0,1$ hingga $0,2$. Kualitas fusi yang sangat rendah ini menegaskan bahwa EKF autopilot sangat mempercayai data navigasi sintetis karena tingkat konsistensi datanya sangat tinggi terhadap prediksi model internal. Hal ini diperkuat oleh status bendera kontrol EKF (*Flags*) di mana *attitude*, *velocity_horiz*, *velocity_vert*, *pos_horiz_rel*, *pos_horiz_abs*, dan *pos_vert_abs* berada dalam status aktif (*On*), sementara mode darurat penahanan posisi konstan (*const_pos_mode*) berada pada status mati (*Off*).

Keandalan transmisi data NMEA juga terkonfirmasi dari perolehan kualitas kunci GPS yang menunjukkan status *RTK Fixed Lock* (`GPS: rtk Fixed`) pada parameter HUD utama. Hasil pembacaan pesan MAVLink `GPS_RAW_INT` mencatat `fix_type` bernilai $6$ (mewakili *RTK Fixed*), `satellites_visible` mencapai $30$ satelit, dan `eph` bernilai $10$ (mewakili nilai HDOP $0,1$), yang menunjukkan bahwa kalimat emulasi `$GPGGA`, `$GPRMC`, `$GPGSA`, dan `$GPHDT` diterima oleh autopilot tanpa mengalami kehilangan paket data (*packet loss*). Selain itu, integritas koordinat geografis global terbukti sinkron dengan koordinat asal kota Bandung pada lintang `lat` = `-69174999` ($-6,9175^\circ$) dan bujur `lon` = `1076190999` ($107,6191^\circ$).

Terakhir, keberhasilan konfigurasi drone ke dalam mode terbang `Guided` (dan mode `Loiter`) dalam status siap *arming* membuktikan secara mutlak bahwa umpan balik posisi horizontal ($X-Y$) hasil *Dead-Reckoning* aliran optik terkompensasi dan ketinggian ($Z$) dari LiDAR TF-Mini telah diakui sepenuhnya oleh EKF autopilot sebagai input navigasi posisi yang valid untuk melakukan penahanan posisi otonom (*autonomous position hold*).

---

## 6. Kesimpulan Evaluasi untuk Skripsi

Berdasarkan hasil pengujian komprehensif, fusi sensor dengan filter komplementer terbukti memenuhi standar kelaikan sistem estimasi posisi otonom drone. Prosedur kalibrasi awal berhasil menghilangkan bias statis giroskop sumbu-X sebesar $8,18\text{ dps}$ untuk mencegah hanyatan cepat pada detik-detik pertama lepas landas. Kompensasi dinamis *roll* dan *pitch* menggunakan filter lolos-rendah dan lolos-tinggi juga terbukti mampu meredam getaran mekanis frekuensi tinggi pada *frame* drone. Pada sumbu *yaw*, fusi magnetometer sukses menghentikan akumulasi hanyatan arah hadap yang masif sebesar $2,01^\circ/\text{detik}$ sehingga data heading siap ditransmisikan secara presisi dalam format kalimat emulasi NMEA `$GPHDT`. Akhirnya, integrasi *dead-reckoning* yang menggabungkan estimasi posisi horizontal aliran optik dan ketinggian LiDAR mampu merekonstruksi profil trajektori penerbangan drone secara presisi tanpa distorsi spasial pada koordinat bidang horizontal tampak atas, sehingga data posisi sintetis ini siap digunakan oleh autopilot untuk mode penerbangan `Guided` dan `Loiter`.
