# KostBox

Alat IoT penghitung pemakaian listrik dan air penghuni kos (ESP32, PZEM-004T, YF-S201, LCD 16x2, ThingSpeak).

- **Halaman utama (index.html):** data asli alat KostBox dari ThingSpeak, bisa dibuka di HP lewat kode QR
- **Simulasi interaktif:** `simulasi.html`
- **Firmware ESP32:** `firmware/KostBox_Lomba.ino`

## Aturan KostBox
- Listrik **BOROS** jika daya sesaat > 500 W atau pemakaian hari ini > 500 Wh
- Air **BOROS** jika pemakaian hari ini > 50 liter
- Air **BOCOR** jika air mengalir terus tanpa henti (1 menit di mode lomba, 30 menit di mode nyata)

Universitas Tadulako
