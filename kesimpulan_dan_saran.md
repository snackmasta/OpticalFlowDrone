# BAB V: KESIMPULAN DAN SARAN

Bab ini menyajikan kesimpulan menyeluruh dari hasil penelitian rancang bangun sistem estimasi posisi otonom drone berbasis fusi sensor inersial, aliran optik (*optical flow*), dan LiDAR, serta menyertakan saran-saran akademis untuk pengembangan penelitian selanjutnya.

---

## V.1	Kesimpulan
Kesimpulan yang dapat diambil berdasarkan tujuan penelitian yaitu:
1.	Penelitian ini berhasil merancang dan mengimplementasikan sistem estimasi posisi otonom drone berbasis *optical flow* yang dikombinasikan dengan data sensor IMU, LiDAR, dan magnetometer melalui pendekatan *sensor fusion*. Sistem yang dirancang mampu mengukur pergerakan relatif dan menyalurkan data estimasi posisi horizontal serta *heading* yang stabil ke *flight controller* via UART serial dengan emulasi GPS NMEA.
2.	Hasil pengujian menunjukkan bahwa sistem estimasi posisi yang diterapkan mampu menjaga kestabilan posisi drone secara otonom dalam mode terbang *Guided* dan *Loiter* dengan status *Ready to Arm*. Pengujian *closed-loop* stasis membuktikan fusi magnetometer berhasil membuang hanyatan (*drift*) kecepatan sudut Z murni sebesar $2,0152^\circ/\text{s}$ menjadi stabil di kisaran $257^\circ\text{ s.d. } 258^\circ$, dan kalibrasi stasis MPU6050 berhasil mengoreksi bias giroskop X sebesar $-8,1874\text{ dps}$ untuk mencegah *drift* linier. Penerapan konstanta kalibrasi skala `1.2355` berhasil menyelaraskan estimasi perpindahan terhitung sebesar $64,75\text{ cm}$ dengan perpindahan fisik aktual drone sebesar $80\text{ cm}$ di lapangan.

## V.2	Saran
Berdasarkan hasil penelitian yang telah dilakukan, terdapat beberapa saran yang dapat dipertimbangkan untuk pengembangan penelitian selanjutnya, yaitu sebagai berikut.
1.	Penelitian selanjutnya dapat mengembangkan sistem estimasi posisi dengan menggunakan metode fusi yang lebih adaptif, seperti *Extended Kalman Filter* (EKF) lokal pada Raspberry Pi dengan estimasi kovariansi derau adaptif, serta menambahkan kalibrasi temporal IMU-kamera untuk meminimalkan keterlambatan fase data yang memicu anomali kompensasi gerakan miring drone.
2.	Pengujian sistem dapat dilakukan pada kondisi yang lebih bervariasi secara dinamis, seperti menguji ketangguhan loop kontrol autopilot saat mempertahankan posisi (*position hold*) di lingkungan luar ruangan (*outdoor*) dengan gangguan angin aktif, serta menganalisis pengaruh variasi tekstur tanah alami terhadap akurasi estimasi sensor aliran optik.
3.	Penelitian selanjutnya juga dapat melakukan optimalisasi perangkat keras berupa pemasangan peredam getaran mekanis (*vibration damper*) fisik pada sensor IMU untuk meredam derau getaran motor, serta menerapkan koreksi kemiringan sudut LiDAR ($h_{\text{sebenarnya}} = h_{\text{LiDAR}} \times \cos(\phi) \times \cos(\theta)$) untuk menyempurnakan keandalan penskalaan jarak dinamis saat drone melakukan manuver rotasi miring.
