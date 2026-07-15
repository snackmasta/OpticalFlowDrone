# BAB II: LANDASAN TEORI

## 2.5. Spesifikasi dan Prinsip Kerja Komponen Elektronika

Untuk merealisasikan sistem estimasi sikap dan posisi wahana tanpa awak (UAV) berbasis *optical flow*, dibutuhkan integrasi beberapa komponen elektronika utama. Arsitektur interkoneksi hardware dan protokol komunikasi antar komponen ditunjukkan pada **Gambar 2.6**.

```mermaid
graph TD
    %% Subgraph Raspberry Pi
    subgraph RPi["Companion Computer: Raspberry Pi"]
        CPU["Broadcom SoC <br> (CPU/GPU)"]
        UART0["UART0 (/dev/serial0) <br> 921600 bps"]
        UART2["UART2 (/dev/ttyAMA2) <br> 38400 bps"]
        I2C1["I2C Bus 1 <br> Fast Mode 400 kHz"]
        CSI["MIPI CSI Interface"]
    end

    %% Subgraph Sensors & FC
    subgraph FC["Flight Controller (Autopilot)"]
        MCU["Microcontroller"]
        Telem1["TELEM1 Port"]
        GPSPort["GPS1/2 Port"]
        PWMOut["PWM Output Channels"]
    end

    IMU["MPU6050 IMU <br> (Accel/Gyro, Addr: 0x68)"]
    Mag["HMC5883L Magnetometer <br> (Compass, Addr: 0x1E)"]
    RF["Rangefinder / TF-Mini <br> (Sensor Jarak)"]
    Cam["Downward Camera <br> (Optical Flow)"]
    
    %% Actuators & Power
    Batt["Baterai Li-Ion <br> (Power Source)"]
    ESC["Electronic Speed Controller <br> (ESC x4)"]
    Motors["Motor Brushless <br> (x4)"]

    %% Connections
    UART0 <-->|"MAVLink Telemetry"| Telem1
    UART2 -->|"Synthetic NMEA GPS"| GPSPort
    I2C1 <-->|"SMBus Protocol"| IMU
    I2C1 <-->|"SMBus Protocol"| Mag
    CSI <--|"MIPI Ribbon Cable"| Cam
    RF -->|"Serial/I2C"| MCU
    
    %% Power & Actuator Connections
    Batt -->|"Main Power"| ESC
    ESC -->|"Power Supply"| MCU
    ESC -->|"Power Supply"| CPU
    PWMOut -->|"PWM Control Signal"| ESC
    ESC -->|"3-Phase AC Power"| Motors
```
<p align="center"><b>Gambar 2.6.</b> Arsitektur interkoneksi hardware dan protokol komunikasi sistem.</p>

---

### 2.5.1. Companion Computer (Raspberry Pi)

*Companion Computer* bertindak sebagai unit pemroses data tingkat tinggi (*high-level computer*) yang memikul beban kalkulasi algoritma dengan kebutuhan daya komputasi intensif secara *real-time*. Pada sistem ini, Raspberry Pi digunakan untuk mengeksekusi algoritma pengolahan citra (*optical flow*), kompensasi kemiringan kinetik, penyaringan fusi data sensor (*sensor fusion*), dan pengelolaan komunikasi data telemetri.

Secara spesifikasi perangkat keras, Raspberry Pi dilengkapi dengan *System on Chip* (SoC) Broadcom yang mengintegrasikan prosesor multi-core berbasis arsitektur ARM Cortex (seperti Quad-core ARM Cortex-A72 pada Raspberry Pi 4) serta memori akses acak (RAM) bertipe LPDDR4 [6]. Kekuatan pemrosesan ini didukung oleh instruksi SIMD (Single Instruction Multiple Data) berupa teknologi ARM NEON yang memfasilitasi percepatan komputasi vektor secara paralel pada tingkat instruksi CPU, sebuah fitur yang sangat krusial untuk mempercepat kalkulasi matriks pada algoritma pemrosesan citra OpenCV [7], [8]. Untuk antarmuka fisik, perangkat ini menyediakan pin GPIO (General Purpose Input/Output) yang mendukung bus komunikasi serial seperti I2C (Inter-Integrated Circuit) dan UART (Universal Asynchronous Receiver-Transmitter), serta antarmuka kamera khusus MIPI CSI (Mobile Industry Processor Interface Camera Serial Interface).

Pada lapisan perangkat lunak, Raspberry Pi menjalankan sistem operasi berbasis Linux kernel yang dikonfigurasi dengan alokasi prioritas tinggi (*real-time priority*). Struktur perangkat lunak sistem ini dibangun menggunakan bahasa pemrograman Python dengan pustaka-pustaka inti meliputi:
1. **OpenCV (Open Source Computer Vision Library)**: Digunakan untuk memproses bingkai citra digital secara sekuensial menggunakan metode Dense Inverse Search (DIS) Optical Flow untuk mendeteksi pergeseran piksel antar bingkai citra [9].
2. **SMBus2**: Digunakan sebagai pustaka driver komunikasi tingkat rendah untuk melakukan operasi baca/tulis register pada sensor MPU6050 dan HMC5883L melalui bus I2C.
3. **PyMAVLink**: Pustaka implementasi protokol MAVLink untuk mengemas dan mengurai paket data serial antara pendamping komputer dengan *flight controller*.

Guna menjamin eksekusi yang responsif dan bebas hambatan latensi (*latency bottleneck*), prinsip kerja sistem perangkat lunak pada *Companion Computer* dirancang menggunakan arsitektur multi-utas (*multi-threading*). Karena interpreter Python memiliki pembatasan *Global Interpreter Lock* (GIL) pada utas CPU tunggal, pembacaan sensor fisik dipisahkan ke dalam beberapa utas mandiri (*worker threads*) yang berjalan di latar belakang secara asinkron. Utas-utas tersebut meliputi utas akuisisi bingkai kamera, utas pembacaan I2C untuk IMU dan kompas, serta utas penerimaan/pengiriman telemetri MAVLink. Komunikasi data antar utas dan proses aplikasi visualisasi dashboard menggunakan mekanisme memori bersama (*shared memory* - SHM) segmen IPC (Inter-Process Communication). Data estimasi posisi relatif hasil fusi kemudian dikonversi menjadi paket data navigasi sintetis yang dikirimkan secara berkala ke *flight controller* dengan baud rate serial tinggi (921600 bps) guna menstabilkan gerak melayang (*hovering*) wahana.

### 2.5.2. Inertial Measurement Unit (MPU6050)

MPU6050 merupakan sensor inersia terintegrasi 6-axis yang menggabungkan giroskop 3-axis dan akselerometer 3-axis dalam satu chip silikon mikroelektromekanis (MEMS). Sensor ini beroperasi pada bus komunikasi I2C dengan alamat dasar `0x68`. Giroskop pada sensor ini memiliki skala pengukuran penuh yang dapat dikonfigurasi mulai dari $\pm 250^\circ/\text{s}$ hingga $\pm 2000^\circ/\text{s}$ dengan tingkat sensitivitas nominal sebesar $131 \text{ LSB/dps}$ pada rentang $\pm 250^\circ/\text{s}$. Sementara itu, akselerometer memiliki batas skala penuh $\pm 2\text{g}$ hingga $\pm 16\text{g}$ dengan sensitivitas nominal $16384 \text{ LSB/g}$ pada rentang $\pm 2\text{g}$ [1].

Prinsip kerja akselerometer didasarkan pada deteksi perubahan kapasitansi elektrostatik yang disebabkan oleh pergeseran massa bukti (*proof mass*) akibat gaya inersia atau gravitasi [10]. Nilai akselerasi linier pada sumbu $X$, $Y$, dan $Z$ ini digunakan untuk mengestimasi sudut kemiringan statis (*tilt estimation*) menggunakan fungsi trigonometri. Di sisi lain, giroskop bekerja dengan memanfaatkan efek Coriolis pada struktur mikro silikon yang digetarkan secara internal untuk mendeteksi laju perubahan sudut [11]. Laju sudut tersebut kemudian diintegrasikan terhadap waktu guna memperoleh sudut orientasi dinamis wahana [12].

### 2.5.3. Digital Compass / Magnetometer (HMC5883L)

HMC5883L adalah sensor medan magnet bumi 3-axis berbasis teknologi *Anisotropic Magnetoresistive* (AMR) yang bertugas menyuplai data orientasi hadap (*heading*) absolut drone. Sensor ini beroperasi pada bus komunikasi I2C dengan alamat default `0x1E` serta mampu mendeteksi kekuatan fluks magnetik dalam rentang skala penuh $\pm 0,88$ hingga $\pm 8,1$ Gauss [2].

Prinsip kerja sensor AMR memanfaatkan karakteristik material feromagnetik tipis yang mengalami perubahan resistansi listrik ketika dikenai medan magnet dari luar. Nilai kuat medan magnet pada sumbu $X$ ($B_x$) dan sumbu $Y$ ($B_y$) dikonversi menjadi sudut arah hadap mentah ($\psi_{\text{raw}}$) menggunakan fungsi arctangen dua parameter yang ditunjukkan pada Persamaan (2.17):

$$\psi_{\text{raw}} = \operatorname{atan2}(B_y, B_x) \quad (2.17)$$

Sudut hadap mentah tersebut selanjutnya dikompensasi secara matematis menggunakan nilai sudut deklinasi magnetik lokal setempat ($\lambda_{\text{dec}}$) untuk menghasilkan estimasi arah hadap sejati terhadap kutub utara geografis bumi, sebagaimana dirumuskan dalam Persamaan (2.18):

$$\psi = \psi_{\text{raw}} + \lambda_{\text{dec}} \quad (2.18)$$

Keterangan variabel untuk Persamaan (2.17) dan Persamaan (2.18) adalah sebagai berikut:
* $\psi$ = sudut arah hadap sejati (*heading*) terhadap kutub utara geografis (derajat atau radian).
* $\psi_{\text{raw}}$ = sudut arah hadap mentah hasil pembacaan sensor (derajat atau radian).
* $B_x$ = nilai densitas fluks medan magnet pada sumbu X (Gauss).
* $B_y$ = nilai densitas fluks medan magnet pada sumbu Y (Gauss).
* $\lambda_{\text{dec}}$ = nilai sudut deklinasi magnetik daerah lokal pengoperasian drone (derajat atau radian).

### 2.5.4. Rangefinder / Sensor Jarak (TF-Mini)

Sensor jarak berupa LiDAR berukuran mikro (seperti TF-Mini) difungsikan untuk mengukur jarak vertikal atau ketinggian drone ($h$) terhadap permukaan tanah di bawahnya secara presisi. Data ketinggian tersebut merupakan parameter utama yang dibutuhkan dalam perhitungan penskalaan fisik dari kecepatan aliran optik (*velocity scaling*). Tanpa adanya data ketinggian metrik yang valid, translasi piksel pada citra kamera tidak dapat dikonversi menjadi satuan kecepatan fisik (meter per detik).

Prinsip kerja TF-Mini didasarkan pada metode *Time-of-Flight* (ToF) atau fase inframerah, di mana sensor memancarkan pulsa cahaya inframerah ke arah permukaan tanah dan mendeteksi selang waktu ($\Delta t$) yang dibutuhkan oleh pulsa tersebut untuk memantul kembali ke elemen penerima (*receiver*). Jarak vertikal dihitung dengan memanfaatkan nilai konstanta kecepatan cahaya ($c$) melalui formulasi matematis pada Persamaan (2.19):

$$h = \frac{c \cdot \Delta t}{2} \quad (2.19)$$

Keterangan variabel untuk Persamaan (2.19) adalah sebagai berikut:
* $h$ = jarak vertikal atau ketinggian wahana terhadap permukaan tanah (meter).
* $c$ = konstanta kecepatan cahaya ($\approx 3 \times 10^8 \text{ m/s}$).
* $\Delta t$ = selang waktu penjalaran pulsa inframerah dari transmitter ke receiver (detik).

### 2.5.5. Flight Controller (Autopilot) & Protokol MAVLink

*Flight Controller* (FC) merupakan sistem tertanam berbasis mikrokontroler *real-time* yang mengendalikan kestabilan sikap dinamis wahana melalui loop kontrol tertutup (*closed-loop control*). FC membaca sensor internalnya sendiri (seperti giroskop primer) untuk mengeksekusi kendali PID pada sumbu *roll*, *pitch*, dan *yaw*.

Hubungan antara FC dengan *Companion Computer* dijembatani oleh protokol komunikasi **MAVLink** (*Micro Air Vehicle Link*). Melalui protokol MAVLink, FC secara pasif mengirimkan data telemetri baterai, status persenjataan motor (*armed/disarmed*), dan mode penerbangan ke Raspberry Pi. Sebaliknya, Raspberry Pi menyuntikkan balik estimasi posisi hasil kalkulasi *dead reckoning* ke estimator filter Kalman (EKF) pada FC, sehingga wahana dapat melakukan navigasi otonom tanpa bergantung pada sinyal GPS satelit [3].

### 2.5.6. Kamera Downward (Optical Flow Sensor)

Kamera yang dipasang menghadap ke bawah (*downward-looking camera*) berfungsi sebagai sensor aliran optik dengan melakukan akuisisi citra permukaan tanah secara berurutan pada laju bingkai (*frame rate*) tinggi yang konstan. Pada sistem ini, digunakan **Raspberry Pi Camera Module v2** dengan resolusi sensor sebesar 8 Megapiksel. Kamera ini menggunakan sensor citra berbasis **Sony IMX219** dengan tipe rana gulung (*rolling shutter*) dan ukuran piksel aktif sebesar $1,12\,\mu\text{m} \times 1,12\,\mu\text{m}$ [13]. Sensor ini mampu menangkap detail tekstur permukaan tanah dengan resolusi spasial yang tinggi, serta memiliki sudut pandang lensa (*Field of View* - FoV) sebesar $62,2^\circ$ secara horizontal dan $48,8^\circ$ secara vertikal.

Untuk mendukung komputasi *real-time* aliran optik, kamera dikonfigurasi untuk bekerja pada resolusi yang lebih rendah namun dengan laju bingkai yang tinggi, seperti $640 \times 480$ piksel pada laju $90\text{ fps}$ atau citra yang kemudian di-*downscale* menjadi $240 \times 240$ piksel. Langkah ini krusial untuk meminimalkan beban komputasi pemrosesan citra pada *Companion Computer* tanpa kehilangan fitur tekstur permukaan yang signifikan.

Prinsip pengiriman data citra dilakukan melalui antarmuka fisik **MIPI CSI-2** (*Mobile Industry Processor Interface Camera Serial Interface 2*). Data *frame* citra ditransmisikan dalam format mentah (*RAW*) atau YUV melalui kabel pita fleksibel 15-pin menuju periferal kontroler CSI pada SoC Broadcom Raspberry Pi. Melalui mekanisme *Direct Memory Access* (DMA), data piksel dari sensor citra langsung ditulis ke memori utama (RAM) Raspberry Pi tanpa membebani siklus CPU. Setelah data citra berada di memori utama, utas akuisisi citra akan menyalin bingkai tersebut ke segmen memori bersama (*shared memory*) untuk diakses secara instan oleh modul algoritma DIS (*Dense Inverse Search*) *Optical Flow*. Algoritma ini memproses intensitas piksel antar bingkai secara berurutan guna mengekstrak vektor pergeseran piksel ($\Delta x, \Delta y$) secara *real-time*.

### 2.5.7. Baterai (Lithium Ion)

Baterai Lithium Ion (Li-Ion) digunakan sebagai penyimpan daya utama untuk menyuplai energi listrik bagi seluruh sistem propulsi dan avionik wahana. Penggunaan sel baterai jenis Li-Ion didasarkan pada keunggulan densitas energi gravitasi yang jauh lebih tinggi dibandingkan jenis Lithium Polymer (LiPo), sehingga sangat cocok untuk memperpanjang durasi terbang wahana (*long-endurance flight*).

Pada sistem ini, digunakan konfigurasi baterai **3S1P (3 Series, 1 Parallel)** menggunakan sel silindris bertipe **18650** (seperti Sony/Murata US18650VTC6 atau Samsung INR18650-30Q) dengan kapasitas nominal sebesar 2600 mAh. Konfigurasi ini menghasilkan spesifikasi tegangan operasional sebagai berikut:
* **Tegangan nominal**: $3 \times 3,6\text{ V} = 10,8\text{ V}$ (atau $11,1\text{ V}$ tergantung jenis bahan kimia sel).
* **Tegangan pengisian penuh (maksimal)**: $3 \times 4,2\text{ V} = 12,6\text{ V}$.
* **Tegangan batas bawah aman (cut-off)**: $3 \times 2,5\text{ V} = 7,5\text{ V}$ (atau diatur pada $9,0\text{ V}$ pada flight controller untuk menjaga kesehatan sel baterai).

Kemampuan pengosongan daya baterai dinyatakan dalam spesifikasi *C-rating*. Arus kontinu maksimal ($I_{\text{max}}$) yang dapat dikeluarkan oleh baterai tanpa merusak struktur kimia sel dirumuskan pada Persamaan (2.20):

$$I_{\text{max}} = C \cdot \text{Kapasitas} \quad (2.20)$$

Nilai laju pengosongan (*C-rating*) ini diperoleh dari rasio antara batas arus kontinu maksimal ($I_{\text{max}}$) yang mampu dialirkan oleh baterai terhadap kapasitas nominal total baterai, seperti yang dirumuskan pada Persamaan (2.21):

$$C = \frac{I_{\text{max}}}{\text{Kapasitas}} \quad (2.21)$$

Sebagai contoh, jika sebuah baterai dengan konfigurasi 3S memiliki kapasitas nominal sebesar $2600\text{ mAh}$ ($2,6\text{ Ah}$) dan batas arus kontinu maksimal ($I_{\text{max}}$) sebesar $40\text{ A}$, maka nilai *C-rating* ($C$) dari baterai tersebut dapat dihitung sebagai berikut:

$$C = \frac{40\text{ A}}{2,6\text{ Ah}} \approx 15,38\text{ C}$$

Keterangan variabel untuk Persamaan (2.20) dan Persamaan (2.21) adalah sebagai berikut:
* $I_{\text{max}}$ = arus batas pengosongan daya kontinu maksimal baterai (Ampere).
* $C$ = nilai kelipatan laju pengosongan daya baterai (*C-rating*).
* $\text{Kapasitas}$ = kapasitas penyimpanan muatan listrik total baterai (Ampere-jam atau Ah).

### 2.5.8. Motor Brushless dan Electronic Speed Controller (ESC)

Motor brushless DC (BLDC) berpasangan dengan *Electronic Speed Controller* (ESC) bertindak sebagai sistem aktuator mekanik utama drone. Motor BLDC dipilih karena memiliki efisiensi daya yang tinggi, rasio daya terhadap berat yang besar, serta keandalan yang tinggi akibat tidak adanya sikat karbon (*brush*). ESC bertugas mengatur laju kecepatan putaran motor berdasarkan sinyal kontrol PWM (*Pulse Width Modulation*) yang diterimanya dari pin output *Flight Controller*.

Pada sistem ini, digunakan unit aktuator dengan spesifikasi fisik dan elektrikal sebagai berikut:
1. **Motor Brushless DarwinFPV 1504 3600KV**: Motor ini mengadopsi konfigurasi elektromagnetik stator dan kutub magnet rotor sebesar **12N14P** (12 *stator slots*, 14 *rotor poles*). Dengan bobot ultra-ringan hanya **9,4 gram**, motor ini mendukung catu daya baterai 3-6S dan memiliki laju kecepatan per volt sebesar **3600 KV**. Kinerja puncaknya mampu menghasilkan daya maksimal sebesar **97,2 Watt** dengan tarikan arus maksimal sebesar **8,1 A** [14].
2. **ESC 20A (2-3S)**: Untuk mengatur kecepatan putaran motor, dipasangkan ESC dengan batas arus kontinu sebesar **20 A** yang mendukung tegangan input baterai **2-3S** (sangat kompatibel dengan konfigurasi baterai 3S Li-Ion yang digunakan). Batas arus 20A ini memberikan margin keamanan (*safety margin*) sebesar $146,9\%$ dari arus maksimal motor ($8,1\text{ A}$), sehingga meminimalkan risiko kerusakan akibat panas berlebih (*overheating*).

Prinsip kerja ESC melibatkan konversi arus searah (DC) dari baterai menjadi arus bolak-balik (AC) tiga fase untuk menyuplai tiga buah kumparan elektromagnetik pada stator motor BLDC secara berurutan. Siklus pergantian fase arus ini (komutasi) diatur secara elektronis oleh mikrokontroler pada ESC dengan mendeteksi posisi rotor melalui gaya gerak listrik balik (*back electromotive force* - BEMF). Lebar pulsa sinyal kendali PWM yang dikirimkan oleh FC bervariasi antara $1000\,\mu\text{s}$ (motor berhenti/putar minimum) hingga $2000\,\mu\text{s}$ (kecepatan motor maksimal) pada frekuensi operasi $50\text{Hz}$ hingga $400\text{Hz}$.

### 2.5.9. Propeler

Propeler bertindak sebagai elemen konversi aerodinamis untuk mengubah energi rotasi dari motor brushless menjadi gaya angkat (*thrust*) dan gaya dorong yang dibutuhkan wahana untuk terbang.

Secara teoritis, besarnya gaya angkat (*thrust*) yang dihasilkan oleh putaran propeler dapat dirumuskan secara aerodinamis melalui Persamaan (2.22):

$$T = C_T \cdot \rho \cdot n^2 \cdot D^4 \quad (2.22)$$

Keterangan variabel untuk Persamaan (2.22) adalah sebagai berikut:
* $T$ = gaya angkat (*thrust*) yang dihasilkan (Newton).
* $C_T$ = koefisien gaya angkat (*thrust coefficient*) propeler (nondimensi, dipengaruhi oleh geometri bilah dan rasio keladangan langkah ulir terhadap diameter).
* $\rho$ = densitas udara ($\approx 1,225\text{ kg/m}^3$ pada tekanan standar permukaan laut).
* $n$ = kecepatan putaran propeler (putaran per detik atau RPS).
* $D$ = diameter nominal propeler (meter).

Pada sistem ini, digunakan propeler bertipe **Gemfan F3015** dengan karakteristik sebagai berikut:
* **Bahan**: Polikarbonat (*polycarbonate*) untuk ketahanan benturan yang baik.
* **Dimensi**: Diameter sebesar **3 inci (76,6 mm)** dengan nilai langkah ulir (*pitch*) sebesar **1,5 inci**.
* **Bobot**: Ultra-ringan sebesar **1,4 gram** per baling-baling.
* **Dudukan (Mounting)**: Tipe **T-mount** untuk dipasangkan secara presisi pada poros motor mikro [15].

### 2.5.10. Protokol NMEA 0183 (National Marine Electronics Association)

Protokol NMEA 0183 merupakan standar spesifikasi kelistrikan dan format data yang didefinisikan oleh *National Marine Electronics Association* untuk memfasilitasi komunikasi data satu arah secara serial antar peralatan elektronika navigasi, khususnya penerima GPS/GNSS (*Global Navigation Satellite System*) [16]. Pada sistem UAV ini, protokol NMEA 0183 digunakan oleh *Companion Computer* untuk mengemulasikan sensor penerima GPS fisik dengan cara menyuntikkan paket koordinat posisi hasil estimasi algoritma *optical flow* ke *Flight Controller* melalui port UART2 pada laju transmisi 38400 bps.

Secara struktural, data NMEA ditransmisikan dalam bentuk kalimat teks ASCII (*ASCII sentences*) yang dapat dibaca secara langsung. Setiap kalimat NMEA memiliki karakteristik format sebagai berikut:
1. Diawali dengan karakter pemulai (`$`).
2. Diikuti oleh kode pengidentifikasi pengirim data (*talker ID*) sepanjang 2 karakter (misalnya `GP` untuk *Global Positioning System* atau `GN` untuk *Global Navigation Satellite System*).
3. Diikuti oleh pengidentifikasi tipe kalimat sepanjang 3 karakter (seperti `GGA` atau `RMC`).
4. Data parameter yang dipisahkan oleh karakter koma (`,` sebagai pembatas field).
5. Diakhiri dengan karakter tanda bintang (`*`) diikuti oleh 2 digit nilai pemeriksaan kesalahan (*checksum*) dalam representasi heksadesimal, serta karakter *Carriage Return* dan *Line Feed* (`<CR><LF>`).

Nilai *checksum* pada kalimat NMEA dihitung dengan melakukan operasi logika XOR (Exclusive OR) pada representasi biner dari semua karakter ASCII yang berada di antara karakter `$` dan `*` (tidak termasuk kedua karakter pembatas tersebut). Formula penghitungan nilai checksum ($C$) ditunjukkan pada Persamaan (2.23):

$$C = \bigoplus_{i=1}^{n} \operatorname{ord}(s_i) \quad (2.23)$$

Keterangan variabel untuk Persamaan (2.23) adalah sebagai berikut:
* $C$ = nilai desimal dari hasil operasi XOR kumulatif yang nantinya dikonversi ke format heksadesimal 2 digit.
* $s_i$ = karakter ke-$i$ pada badan kalimat NMEA.
* $\operatorname{ord}(s_i)$ = nilai ordinal ASCII dari karakter $s_i$.
* $\bigoplus$ = operator logika XOR.

Dalam rancangan sistem penyuntikkan GPS sintetis (*synthetic GPS injector*), terdapat dua jenis kalimat NMEA utama yang wajib dikonstruksi secara berkala dan dikirimkan ke *Flight Controller*:

#### A. Kalimat GPGGA (Global Positioning System Fix Data)
Kalimat `GPGGA` membawa informasi esensial mengenai waktu, koordinat lintang/bujur, kualitas fusi (*fix quality*), jumlah satelit yang terkunci, serta ketinggian wahana di atas permukaan laut. Struktur pesan `GPGGA` yang dikirimkan memiliki susunan format sebagai berikut:

`$GPGGA,hhmmss.ss,llll.lllll,a,yyyyy.yyyyy,a,x,xx,x.x,a.a,M,g.g,M,dd,xxxx*hh<CR><LF>`

Adapun rincian parameter field pada kalimat `GPGGA` ditunjukkan pada Tabel 2.2:

<p align="center"><b>Tabel 2.2.</b> Deskripsi field data kalimat GPGGA.</p>

| No | Nama Field | Deskripsi | Contoh Nilai |
|---|---|---|---|
| 1 | `Sentence ID` | Pengidentifikasi pesan GGA | `GPGGA` |
| 2 | `UTC Time` | Waktu UTC (Format: `hhmmss.ss`) | `123456.00` |
| 3 | `Latitude` | Sudut lintang (Format: `DDMM.MMMMM`) | `-0655.05000` (6° 55.05' S) |
| 4 | `N/S Indicator` | Penunjuk arah lintang (`N` = Utara, `S` = Selatan) | `S` |
| 5 | `Longitude` | Sudut bujur (Format: `DDDMM.MMMMM`) | `10737.14600` (107° 37.146' E) |
| 6 | `E/W Indicator` | Penunjuk arah bujur (`E` = Timur, `W` = Barat) | `E` |
| 7 | `Position Fix` | Indikator kualitas sinyal (`0` = Invalid, `1` = GPS SPS, `4` = RTK Fixed) | `4` (Diatur tinggi untuk melewati validasi autopilot) |
| 8 | `Satellites Used` | Jumlah satelit aktif yang digunakan untuk kalkulasi | `30` |
| 9 | `HDOP` | Efek pengenceran presisi horizontal (*Horizontal Dilution of Precision*) | `0.1` |
| 10| `Altitude` | Ketinggian wahana di atas permukaan geoid (meter) | `10.50` (Nilai dinamis hasil fusi sensor jarak) |
| 11| `Altitude Unit` | Satuan unit ketinggian (`M` = Meter) | `M` |
| 12| `Geoidal Separation` | Ketinggian geoid di atas elipsoid WGS-84 (meter) | `0.0` |
| 13| `Geoidal Unit` | Satuan unit tinggi geoid (`M` = Meter) | `M` |
| 14| `DGPS Age` | Umur data diferensial GPS (detik) | Kosong |
| 15| `Station ID` | ID stasiun referensi DGPS | Kosong |
| 16| `Checksum` | Nilai deteksi kesalahan heksadesimal | `*4C` |

#### B. Kalimat GPRMC (Recommended Minimum Specific GNSS Data)
Kalimat `GPRMC` berisi data navigasi minimum yang direkomendasikan, meliputi waktu, tanggal, status validitas data, koordinat lintang/bujur, serta kecepatan relatif terhadap tanah (*Speed Over Ground* - SOG). Struktur pesan `GPRMC` didefinisikan sebagai berikut:

`$GPRMC,hhmmss.ss,A,llll.lllll,a,yyyyy.yyyyy,a,x.x,x.x,ddmmyy,,,a*hh<CR><LF>`

Adapun rincian parameter field pada kalimat `GPRMC` ditunjukkan pada Tabel 2.3:

<p align="center"><b>Tabel 2.3.</b> Deskripsi field data kalimat GPRMC.</p>

| No | Nama Field | Deskripsi | Contoh Nilai |
|---|---|---|---|
| 1 | `Sentence ID` | Pengidentifikasi pesan RMC | `GPRMC` |
| 2 | `UTC Time` | Waktu UTC (Format: `hhmmss.ss`) | `123456.00` |
| 3 | `Status` | Status keaktifan navigasi (`A` = Valid/Aktif, `V` = Void/Peringatan) | `A` |
| 4 | `Latitude` | Sudut lintang (Format: `DDMM.MMMMM`) | `-0655.05000` |
| 5 | `N/S Indicator` | Penunjuk arah lintang (`N` atau `S`) | `S` |
| 6 | `Longitude` | Sudut bujur (Format: `DDDMM.MMMMM`) | `10737.14600` |
| 7 | `E/W Indicator` | Penunjuk arah bujur (`E` atau `W`) | `E` |
| 8 | `Speed Over Ground` | Kecepatan laju relatif terhadap tanah (knot) | `0.0` |
| 9 | `Track Angle` | Sudut arah pergerakan drone (derajat) | `0.0` |
| 10| `Date` | Tanggal hari UTC (Format: `ddmmyy`) | `140726` |
| 11| `Magnetic Var.` | Sudut variasi magnetik bumi | Kosong |
| 12| `Var. Direction` | Arah sudut variasi magnetik | Kosong |
| 13| `Mode Indicator` | Mode pengoperasian (`A` = Autonomous, `D` = Differential) | `A` |
| 14| `Checksum` | Nilai deteksi kesalahan heksadesimal | `*5A` |

Melalui fusi berkala dari kedua pesan NMEA tersebut, *Flight Controller* dapat memperbarui estimasi lokasinya di dalam ruang navigasi lokal secara otonom seolah-olah mendapat referensi eksternal dari konstelasi satelit GPS yang nyata.

---

## Referensi

[1] InvenSense Inc., "MPU-6000 and MPU-6050 Product Specification Revision 3.4," Document Number: PS-MPU-6000A-00, 2013.

[2] Honeywell International Inc., "3-Axis Digital Compass IC HMC5883L Datasheet," Document Number: Form #900405, 2013.

[3] MAVLink Developer Guide, "MAVLink Common Message Set Specification," [Online]. Available: https://mavlink.io/en/messages/common.html.

[4] J. F. L. M. G. de Queiroz and A. A. R. de Moura, "Experimental characterization of lithium-ion batteries for unmanned aerial vehicle applications," *Journal of Aerospace Technology and Management*, vol. 10, 2018.

[5] T. J. Kennerly, *Brushless DC Motors: Principles and Applications*. New York, NY, USA: CRC Press, 2015.

[6] Raspberry Pi Foundation, "Raspberry Pi 4 Model B Product Brief," 2019.

[7] ARM Limited, "ARM Cortex-A Series Programmer's Guide for ARMv8-A," version 1.0, 2015.

[8] G. Bradski and A. Kaehler, *Learning OpenCV: Computer Vision with the OpenCV Library*. Sebastopol, CA, USA: O'Reilly Media, Inc., 2008.

[9] T. Kroeger, R. Timofte, D. Dai, and L. Van Gool, "Fast Optical Flow using Dense Inverse Search," in *Proceedings of the European Conference on Computer Vision (ECCV)*, 2016, pp. 471-488.

[10] S. Beeby, G. Ensell, M. Kraft, and N. White, *MEMS Mechanical Sensors*. Boston, MA, USA: Artech House, 2004.

[11] A. Lawrence, *Modern Inertial Technology: Navigation, Guidance, and Control*. New York, NY, USA: Springer-Verlag, 1998.

[12] M. S. Grewal, L. R. Weill, and A. P. Andrews, *Global Positioning Systems, Inertial Navigation, and Integration*. New Jersey, NJ, USA: John Wiley & Sons, Inc., 2007.

[13] Raspberry Pi Foundation, "Raspberry Pi Camera Module v2 Datasheet," 2016.

[14] DarwinFPV, "DarwinFPV 1504 3600KV Brushless Motor Datasheet," 2023.

[15] Gemfan Hobby, "Gemfan F3015 3-Blade Propeller Specifications," 2023.

[16] National Marine Electronics Association, "NMEA 0183 Standard for Interfacing Marine Electronic Devices," Version 4.10, 2012.

