/*
  =====================================================================
  KostBox - Firmware ESP32
  ---------------------------------------------------------------------
  Sensor : PZEM-004T V3.0 (listrik), YF-S201 (air)
  Layar  : LCD 16x2 I2C (modul PCF8574)
  Kirim  : ThingSpeak (data + grafik otomatis) & Telegram (notifikasi)

  Library yang perlu di-install (Library Manager Arduino IDE):
    - PZEM004Tv30      oleh Jakub Mandula
    (LCD I2C memakai driver bawaan di dalam kode ini, tidak perlu library)
  Board: "ESP32 Dev Module" (paket esp32 oleh Espressif)
  =====================================================================
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <time.h>
#include <PZEM004Tv30.h>

// ========================= KONFIGURASI =========================
const char* WIFI_SSID     = "NAMA_WIFI";
const char* WIFI_PASS     = "PASSWORD_WIFI";
const char* ID_KAMAR      = "kamar101";

// ThingSpeak: Channels > (channel Anda) > API Keys > "Write API Key"
const char* THINGSPEAK_API_KEY = "ISI_WRITE_API_KEY";

// Telegram (opsional): kosongkan TELEGRAM_TOKEN jika belum dipakai
const char* TELEGRAM_TOKEN   = "";   // dari @BotFather, contoh "123456:ABC-xyz..."
const char* TELEGRAM_CHAT_ID = "";   // dari @userinfobot, contoh "123456789"

// 1 = data listrik dummy (dipakai selama PZEM belum datang)
// 0 = baca dari PZEM-004T sungguhan
#define MODE_SIMULASI_LISTRIK 1

// 1 = MINIATUR 5V: arus & tegangan DIUKUR SUNGGUHAN lewat resistor shunt
//     (beban: kipas/lampu USB 5V). Mode ini mengalahkan mode simulasi.
// 0 = tidak dipakai
#define MODE_SENSOR_DC 0
#if MODE_SENSOR_DC
  #undef  MODE_SIMULASI_LISTRIK
  #define MODE_SIMULASI_LISTRIK 0
#endif
#define PIN_SHUNT    34    // titik antara beban(-) dan resistor shunt
#define PIN_TEGANGAN 35    // 5V lewat pembagi 20k (atas) + 10k (bawah)
const float SHUNT_OHM        = 1.0;   // nilai resistor shunt (ohm)
const float RASIO_PEMBAGI    = 3.0;   // (20k + 10k) / 10k
const float FAKTOR_KALIBRASI = 1.0;   // samakan dengan watt meter DC bila perlu

// 1 = mode lomba/expo: kirim tiap 20 detik, BOCOR terdeteksi dalam 1 menit
// 0 = mode pemakaian nyata di kos
#define MODE_EXPO 1

// 1 = LCD 16x2 I2C terpasang, 0 = tanpa LCD (tampilan hanya di Serial Monitor)
#define PAKAI_LCD 1

// ===== SATU TOMBOL SIMULASI di D4 (satu kaki ke D4, kaki lain ke GND) =====
//   Tekan singkat          = alat berikutnya:
//       SEMUA MATI -> LAMPU 10W -> TEKO 350W -> HAIR DRYER 350W -> SEMUA (710W) -> SEMUA MATI ...
//   Tahan +-2 detik, lepas = pakai air +10 liter (simulasi mandi/mencuci)
//   Tahan 5 detik, lepas   = RESET hitungan hari ini (untuk mengulang demo)
//   (LCD memberi petunjuk saat tombol ditahan)
#define PIN_TOMBOL 4
const float AIR_PER_TEKAN_L = 10.0;
const unsigned long TAHAN_AIR_MS   = 1500;
const unsigned long TAHAN_RESET_MS = 5000;
bool tombolDitahan = false;

// ===== ATURAN BOROS =====
// Listrik BOROS jika: daya yang sedang dipakai > 500 W  (banyak alat dicolok bersamaan)
//                atau total pemakaian hari ini > 500 Wh (0,5 kWh)
// Air BOROS jika    : total pemakaian air hari ini > 50 liter
#if MODE_SENSOR_DC
const float BATAS_DAYA_W            = 0.8;    // miniatur 5V: kipas USB ~1 W dianggap beban besar
#else
const float BATAS_DAYA_W            = 500.0;  // watt (daya sesaat)
#endif
const float BATAS_LISTRIK_HARIAN_WH = 500.0;  // Wh per hari
const float BATAS_AIR_HARIAN_L      = 50.0;   // liter per hari

#if MODE_EXPO
const unsigned long INTERVAL_KIRIM_MS = 20000UL;              // 20 detik (ThingSpeak gratis minimal 15 detik)
const unsigned long LAMA_BOCOR_MENIT  = 1;
#else
const unsigned long INTERVAL_KIRIM_MS = 5UL * 60UL * 1000UL;  // 5 menit
const unsigned long LAMA_BOCOR_MENIT  = 30;   // air mengalir terus-menerus selama ini -> BOCOR
#endif
const unsigned long INTERVAL_SIMPAN_MS = 5UL * 60UL * 1000UL; // simpan ke flash tiap 5 menit

const float PULSA_PER_LITER = 450.0;  // YF-S201 ~7,5 Hz per L/menit -> ~450 pulsa/liter (WAJIB dikalibrasi)
const long ZONA_WAKTU_DETIK = 8 * 3600;    // WITA (UTC+8)

// ============================ PIN ==============================
#define PIN_FLOW  27   // sinyal YF-S201 (lewat pembagi tegangan!)
#define PZEM_RX   16   // ESP32 RX2  <- TX PZEM
#define PZEM_TX   17   // ESP32 TX2  -> RX PZEM
// LCD I2C: SDA -> GPIO21, SCL -> GPIO22
#define LCD_ALAMAT 0x27   // jika layar kosong, coba 0x3F
const unsigned long GANTI_HALAMAN_MS = 3000;  // pergantian halaman LCD

PZEM004Tv30 pzem(Serial2, PZEM_RX, PZEM_TX);
#if PAKAI_LCD
  // Driver LCD 16x2 I2C (PCF8574) bawaan, TANPA library tambahan,
  // supaya tidak bentrok dengan library LiquidCrystal_I2C yang terpasang ganda.
  class LcdI2C {
    uint8_t alamat, lampu = 0x08;
    void kirimRaw(uint8_t d) { Wire.beginTransmission(alamat); Wire.write(d | lampu); Wire.endTransmission(); }
    void pulsa(uint8_t d)    { kirimRaw(d | 0x04); delayMicroseconds(1); kirimRaw(d & ~0x04); delayMicroseconds(60); }
    void tulis4(uint8_t d)   { kirimRaw(d); pulsa(d); }
    void kirim(uint8_t v, uint8_t mode) { tulis4((v & 0xF0) | mode); tulis4(((v << 4) & 0xF0) | mode); }
    void perintah(uint8_t v) { kirim(v, 0); }
  public:
    LcdI2C(uint8_t a, int, int) : alamat(a) {}
    void init() {
      delay(50); kirimRaw(0); delay(100);
      tulis4(0x30); delayMicroseconds(4500);
      tulis4(0x30); delayMicroseconds(4500);
      tulis4(0x30); delayMicroseconds(200);
      tulis4(0x20);                       // mode 4-bit
      perintah(0x28);                     // 2 baris, huruf 5x8
      perintah(0x0C);                     // layar nyala, kursor mati
      perintah(0x01); delay(3);           // bersihkan layar
      perintah(0x06);                     // tulis ke kanan
    }
    void backlight() { lampu = 0x08; kirimRaw(0); }
    void setCursor(int kolom, int baris) { perintah(0x80 | (kolom + (baris ? 0x40 : 0x00))); }
    void print(const char* t) { while (*t) kirim((uint8_t)*t++, 0x01); }
    void print(const String& t) { print(t.c_str()); }
  };
  LcdI2C lcd(LCD_ALAMAT, 16, 2);
#else
  // Pengganti LCD kosong: kode tetap jalan tanpa library LCD
  struct LcdKosong {
    void init() {}
    void backlight() {}
    void setCursor(int, int) {}
    void print(const String&) {}
    void print(const char*) {}
  } lcd;
#endif
Preferences pref;

volatile uint32_t pulsaAir = 0;
uint32_t pulsaTotal = 0;      // total pulsa sejak nyala/reset, untuk kalibrasi
void IRAM_ATTR isrFlow() { pulsaAir++; }

struct DataKostBox {
  float tegangan = 0, arus = 0, daya = 0;
  float energiTotal = 0, energiAwalHari = 0;  // kWh
  float debit = 0;                             // liter/menit
  float airTotal = 0, airAwalHari = 0;         // liter
  bool  pzemOk = false;
  String statusListrik = "HEMAT";
  String statusAir = "NORMAL";
} d;

int hariTersimpan = -1;
unsigned long sejakDayaTinggi = 0, sejakAirMengalir = 0;
unsigned long terakhirBaca = 0, terakhirKirim = 0, terakhirSimpan = 0, terakhirCekWifi = 0;
String infoKirim = "belum kirim";

// ===================== PENYIMPANAN (FLASH) =====================
void muatData() {
  pref.begin("kostbox", false);
  d.airTotal       = pref.getFloat("airTotal", 0);
  d.airAwalHari    = pref.getFloat("airAwal", 0);
  d.energiAwalHari = pref.getFloat("kwhAwal", 0);
  hariTersimpan    = pref.getInt("hari", -1);
#if MODE_SIMULASI_LISTRIK || MODE_SENSOR_DC
  d.energiTotal    = pref.getFloat("kwhSim", 0);
#endif
}

void simpanData() {
  pref.putFloat("airTotal", d.airTotal);
  pref.putFloat("airAwal", d.airAwalHari);
  pref.putFloat("kwhAwal", d.energiAwalHari);
  pref.putInt("hari", hariTersimpan);
#if MODE_SIMULASI_LISTRIK || MODE_SENSOR_DC
  pref.putFloat("kwhSim", d.energiTotal);
#endif
}

// ========================= SENSOR ==============================
void tulisBaris(int baris, const String& teks);   // dideklarasikan di bagian LAYAR
// Daya tiap alat (ubah sesuai label watt alat yang dipakai saat demo)
const float DAYA_LAMPU = 10.0;
const float DAYA_TEKO  = 350.0;
const float DAYA_HAIR  = 350.0;

const int   JUMLAH_SKENARIO = 5;
int bebanSim = 0;                       // mulai dari SEMUA MATI
const float DAYA_SIM[JUMLAH_SKENARIO] = {0.0, DAYA_LAMPU, DAYA_TEKO, DAYA_HAIR, DAYA_LAMPU + DAYA_TEKO + DAYA_HAIR};
const char* NAMA_SIM[JUMLAH_SKENARIO] = {"SEMUA MATI", "LAMPU 10W", "TEKO 350W", "HAIRDRYER 350W", "SEMUA 710W"};
String alasanListrik = "";

void tampilPesan(const String& baris1, const String& baris2) {
#if PAKAI_LCD
  tulisBaris(0, baris1);
  tulisBaris(1, baris2);
  delay(1200);
#endif
}

void resetHarian() {
  noInterrupts(); pulsaAir = 0; interrupts();
  pulsaTotal = 0;
  d.airTotal = 0;  d.airAwalHari = 0;
  d.energiAwalHari = d.energiTotal;
  sejakAirMengalir = 0;
  simpanData();
  Serial.println("\n>>> RESET: hitungan listrik & air hari ini kembali ke 0\n");
}

// Satu tombol: tekan singkat = alat berikutnya, tahan 2 detik = air +10 L, tahan 5 detik = reset
void tambahAir() {
  d.airTotal += AIR_PER_TEKAN_L;
  float hariIni = d.airTotal - d.airAwalHari;
  Serial.printf("\n>>> Pakai air +%.0f L, total hari ini %.1f L\n\n", AIR_PER_TEKAN_L, hariIni);
  tampilPesan("Pakai air +" + String(AIR_PER_TEKAN_L, 0) + " L", "Hari ini:" + String(hariIni, 0) + " L");
}

void cekTombol() {
  static unsigned long mulaiTekan = 0, terakhirLepas = 0;
  static int petunjuk = 0;          // 0 = belum, 1 = sudah tampil "air", 2 = sudah tampil "reset"
  bool low = digitalRead(PIN_TOMBOL) == LOW;
  unsigned long now = millis();

  if (low && !tombolDitahan) {
    if (now - terakhirLepas < 200) return;           // abaikan pantulan tombol
    tombolDitahan = true; mulaiTekan = now; petunjuk = 0;
  } else if (low && tombolDitahan) {
    unsigned long lama = now - mulaiTekan;
    if (lama >= TAHAN_RESET_MS && petunjuk < 2) {
      petunjuk = 2; tulisBaris(0, "Lepas sekarang:"); tulisBaris(1, "RESET hari ini");
    } else if (lama >= TAHAN_AIR_MS && petunjuk < 1) {
      petunjuk = 1; tulisBaris(0, "Lepas sekarang:"); tulisBaris(1, "Pakai air +10 L");
    }
  } else if (!low && tombolDitahan) {
    tombolDitahan = false; terakhirLepas = now;
    unsigned long lama = now - mulaiTekan;
    if (lama < 50) return;                            // terlalu singkat, dianggap derau
    if (lama >= TAHAN_RESET_MS) {
      bebanSim = 0;
      resetHarian();
      tampilPesan("RESET DEMO", "Hitungan = 0");
    } else if (lama >= TAHAN_AIR_MS) {
      tambahAir();
    } else {
      bebanSim = (bebanSim + 1) % JUMLAH_SKENARIO;
      Serial.printf("\n>>> Alat yang dicolok sekarang: %s (%.0f W)\n\n", NAMA_SIM[bebanSim], DAYA_SIM[bebanSim]);
      tampilPesan("Dicolok:", NAMA_SIM[bebanSim]);
    }
  }
}

float bacaMilivolt(int pin) {
  uint32_t total = 0;
  for (int i = 0; i < 64; i++) total += analogReadMilliVolts(pin);
  return total / 64.0;
}

void bacaListrik(float dtDetik) {
#if MODE_SENSOR_DC
  d.pzemOk = true;
  float vShunt = bacaMilivolt(PIN_SHUNT) / 1000.0;                    // volt
  float vSumber = bacaMilivolt(PIN_TEGANGAN) / 1000.0 * RASIO_PEMBAGI;  // volt
  float arus = vShunt / SHUNT_OHM * FAKTOR_KALIBRASI;
  if (arus < 0.02) arus = 0;                 // abaikan derau saat beban mati
  d.tegangan = vSumber;
  d.arus     = arus;
  d.daya     = (vSumber - vShunt) * arus;    // daya yang dipakai beban
  if (d.daya < 0) d.daya = 0;
  d.energiTotal += d.daya * dtDetik / 3600000.0;
#elif MODE_SIMULASI_LISTRIK
  d.pzemOk   = true;
  d.tegangan = 220.0 + random(-30, 31) / 10.0;
  float dasar = DAYA_SIM[bebanSim];
  d.daya     = (dasar > 0) ? dasar * (1.0 + random(-20, 21) / 1000.0) : 0;  // fluktuasi +-2%
  d.arus     = d.daya / d.tegangan;
  d.energiTotal += d.daya * dtDetik / 3600000.0;  // W x detik -> kWh
#else
  float v = pzem.voltage();
  if (isnan(v)) {                // PZEM tidak menjawab
    d.pzemOk = false;
    d.tegangan = 0; d.arus = 0; d.daya = 0;
    return;
  }
  d.pzemOk   = true;
  d.tegangan = v;
  d.arus     = pzem.current();
  d.daya     = pzem.power();
  float e    = pzem.energy();    // kWh kumulatif, tersimpan di dalam PZEM
  if (isnan(d.arus)) d.arus = 0;
  if (isnan(d.daya)) d.daya = 0;
  if (!isnan(e)) d.energiTotal = e;
#endif
}

void bacaAir(float dtDetik) {
  noInterrupts();
  uint32_t p = pulsaAir;
  pulsaAir = 0;
  interrupts();
  pulsaTotal += p;
  float liter = p / PULSA_PER_LITER;
  d.debit = (dtDetik > 0) ? liter / dtDetik * 60.0 : 0;
  d.airTotal += liter;
}

// Reset hitungan "hari ini" setiap ganti hari (butuh jam dari NTP)
void cekGantiHari() {
  if (!d.pzemOk) return;
  struct tm t;
  if (!getLocalTime(&t, 0)) return;   // jam NTP belum siap
  if (t.tm_yday != hariTersimpan) {
    hariTersimpan    = t.tm_yday;
    d.energiAwalHari = d.energiTotal;
    d.airAwalHari    = d.airTotal;
    simpanData();
  }
  if (d.energiAwalHari > d.energiTotal) d.energiAwalHari = d.energiTotal; // energi PZEM di-reset
}

// Klasifikasi berbasis aturan di perangkat (AI di cloud bisa menyusul)
void tentukanStatus() {
  unsigned long now = millis();
  float listrikHariIniWh = (d.energiTotal - d.energiAwalHari) * 1000.0;
  float airHariIniL      = d.airTotal - d.airAwalHari;

  // --- Listrik: BOROS jika daya sesaat > 500 W atau total hari ini > 500 Wh ---
  if (d.daya > BATAS_DAYA_W) {
    d.statusListrik = "BOROS";
    alasanListrik = "daya " + String(d.daya, 0) + " W melebihi " + String(BATAS_DAYA_W, 0) + " W";
  } else if (listrikHariIniWh > BATAS_LISTRIK_HARIAN_WH) {
    d.statusListrik = "BOROS";
    alasanListrik = "pemakaian hari ini " + String(listrikHariIniWh, 0) + " Wh melebihi " + String(BATAS_LISTRIK_HARIAN_WH, 0) + " Wh";
  } else {
    d.statusListrik = "HEMAT";
    alasanListrik = "";
  }

  // --- Air ---
  bool mengalir = d.debit > 0.2;
  if (mengalir) {
    if (sejakAirMengalir == 0) sejakAirMengalir = now;
  } else {
    sejakAirMengalir = 0;
  }
  unsigned long lamaAir = sejakAirMengalir ? now - sejakAirMengalir : 0;

  struct tm t;
  bool diniHari = getLocalTime(&t, 0) && t.tm_hour >= 1 && t.tm_hour < 5;
  if (lamaAir >= LAMA_BOCOR_MENIT * 60000UL || (diniHari && lamaAir >= 5UL * 60000UL))
    d.statusAir = "BOCOR";                       // mengalir terus tanpa henti
  else if (airHariIniL > BATAS_AIR_HARIAN_L)
    d.statusAir = "BOROS";                       // pemakaian harian melewati batas
  else if (mengalir)
    d.statusAir = "MENGALIR";
  else
    d.statusAir = "NORMAL";
}

// ========================== LAYAR ==============================
// LCD 16x2 menampilkan 4 halaman bergantian tiap 3 detik:
// 1) Listrik  2) Air  3) Status  4) Koneksi
String pas16(String t) {              // potong/isi spasi agar tepat 16 karakter
  if (t.length() > 16) t = t.substring(0, 16);
  while (t.length() < 16) t += ' ';
  return t;
}

void tulisBaris(int baris, const String& teks) {
  lcd.setCursor(0, baris);
  lcd.print(pas16(teks));
}

void layarAwal() {
#if !PAKAI_LCD
  Serial.println("[LCD] PAKAI_LCD = 0, LCD tidak diaktifkan");
  return;
#endif
  Wire.beginTransmission(LCD_ALAMAT);
  if (Wire.endTransmission() != 0) {
    Serial.printf("[LCD] Tidak menjawab di alamat 0x%02X. Mencari alamat lain:\n", LCD_ALAMAT);
    for (byte a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) Serial.printf("[LCD]   ditemukan perangkat di 0x%02X -> isi LCD_ALAMAT dengan ini\n", a);
    }
  } else {
    Serial.printf("[LCD] Terdeteksi di alamat 0x%02X\n", LCD_ALAMAT);
  }
  lcd.init();
  lcd.backlight();
  tulisBaris(0, "    KostBox");
  tulisBaris(1, String(ID_KAMAR) + " memulai");
}

void perbaruiLayar() {
#if !PAKAI_LCD
  return;
#endif
  if (tombolDitahan) return;          // jangan timpa petunjuk saat tombol ditahan
  static unsigned long terakhirGanti = 0;
  static int halaman = 0;
  if (millis() - terakhirGanti >= GANTI_HALAMAN_MS) {
    terakhirGanti = millis();
    halaman = (halaman + 1) % 4;
  }

  switch (halaman) {
    case 0:  // Listrik
      if (d.pzemOk) {
        String tanda = MODE_SIMULASI_LISTRIK ? String(" S") : String("");
#if MODE_SENSOR_DC
        tulisBaris(0, "P:" + String(d.daya, 2) + "W I:" + String(d.arus, 2) + "A");
#else
        tulisBaris(0, "P:" + String(d.daya, 1) + "W " + String(d.tegangan, 0) + "V" + tanda);
#endif
      } else {
        tulisBaris(0, "PZEM ERROR");
      }
#if MODE_SENSOR_DC
      tulisBaris(1, "Hr:" + String((d.energiTotal - d.energiAwalHari) * 1000.0, 2) + " Wh");
#else
      tulisBaris(1, "Hr:" + String((d.energiTotal - d.energiAwalHari) * 1000.0, 1) + " Wh");
#endif
      break;
    case 1:  // Air
      tulisBaris(0, "Air:" + String(d.debit, 2) + " L/m");
      tulisBaris(1, "Hr:" + String(d.airTotal - d.airAwalHari, 1) + " L");
      break;
    case 2:  // Status
      tulisBaris(0, "Listrik:" + d.statusListrik);
      tulisBaris(1, "Air:" + d.statusAir);
      break;
    default: // Koneksi
      tulisBaris(0, WiFi.status() == WL_CONNECTED ? String("WiFi OK") : String("WiFi PUTUS"));
      tulisBaris(1, infoKirim);
      break;
  }
}

// ===================== TAMPILAN SERIAL ========================
void tampilSerial() {
  static unsigned long terakhir = 0;
  if (millis() - terakhir < 2000) return;   // tiap 2 detik agar mudah dibaca
  terakhir = millis();
  Serial.println("========== KostBox " + String(ID_KAMAR) + " ==========");
#if MODE_SIMULASI_LISTRIK
  Serial.printf(" Alat dicolok     : %s  (simulasi)\n", NAMA_SIM[bebanSim]);
#endif
#if MODE_SENSOR_DC
  Serial.printf(" Daya (miniatur)  : %.2f W   Arus: %.3f A   Tegangan: %.2f V\n", d.daya, d.arus, d.tegangan);
  Serial.printf(" Energi hari ini  : %.3f Wh\n", (d.energiTotal - d.energiAwalHari) * 1000.0);
#else
  Serial.printf(" Daya             : %.1f W  (batas %.0f W)   Tegangan: %.1f V\n", d.daya, BATAS_DAYA_W, d.tegangan);
  Serial.printf(" Energi hari ini  : %.1f Wh  (batas %.0f Wh/hari)\n",
                (d.energiTotal - d.energiAwalHari) * 1000.0, BATAS_LISTRIK_HARIAN_WH);
#endif
  Serial.printf(" Status listrik   : %s %s\n", d.statusListrik.c_str(),
                alasanListrik.length() ? ("(" + alasanListrik + ")").c_str() : "");
  Serial.printf(" Debit air        : %.2f L/menit\n", d.debit);
  Serial.printf(" Air hari ini     : %.2f L  (batas %.0f L/hari, pulsa: %lu)\n",
                d.airTotal - d.airAwalHari, BATAS_AIR_HARIAN_L, (unsigned long)pulsaTotal);
  Serial.printf(" Status air       : %s\n", d.statusAir.c_str());
  Serial.printf(" WiFi             : %s | %s\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "PUTUS", infoKirim.c_str());
  Serial.println();
}

// ==================== PERINTAH SERIAL ==========================
// Ketik di Serial Monitor lalu Enter:
//   r = reset hitungan air & energi ke 0 (untuk mulai uji/demo)
void cekPerintahSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();
  if (c == 'r' || c == 'R') resetHarian();
}

// ======================= THINGSPEAK ============================
// Field 1-8 ThingSpeak:
// 1 Daya (W) | 2 Tegangan (V) | 3 kWh hari ini | 4 kWh total
// 5 Debit (L/menit) | 6 Air hari ini (L) | 7 Status listrik | 8 Status air
// Status listrik: 0 HEMAT, 1 TINGGI, 2 BOROS
// Status air    : 0 NORMAL, 1 MENGALIR, 2 BOCOR, 3 BOROS
String angka(float v, int desimal) {
  return isnan(v) ? String("0") : String(v, desimal);
}

int kodeListrik() {
  if (d.statusListrik == "BOROS")  return 2;
  if (d.statusListrik == "TINGGI") return 1;
  return 0;
}

int kodeAir() {
  if (d.statusAir == "BOROS")    return 3;
  if (d.statusAir == "BOCOR")    return 2;
  if (d.statusAir == "MENGALIR") return 1;
  return 0;
}

bool kirimThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) {
    infoKirim = "gagal: offline";
    return false;
  }
  String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY);
  url += "&field1=" + angka(d.daya, 2);
  url += "&field2=" + angka(d.tegangan, 2);
  url += "&field3=" + angka(d.energiTotal - d.energiAwalHari, 6);
  url += "&field4=" + angka(d.energiTotal, 6);
  url += "&field5=" + angka(d.debit, 2);
  url += "&field6=" + angka(d.airTotal - d.airAwalHari, 1);
  url += "&field7=" + String(kodeListrik());
  url += "&field8=" + String(kodeAir());

  WiFiClient client;
  HTTPClient http;
  http.begin(client, url);
  int kode = http.GET();
  String balasan = (kode == 200) ? http.getString() : String("");
  http.end();

  // ThingSpeak membalas nomor entri (>0) bila berhasil, "0" bila ditolak (mis. terlalu cepat)
  bool ok = (kode == 200 && balasan.toInt() > 0);
  if (ok) infoKirim = "Kirim OK #" + balasan;
  else    infoKirim = "gagal: " + String(kode) + " " + balasan;
  Serial.println("[ThingSpeak] " + infoKirim);
  return ok;
}

// ======================== TELEGRAM =============================
String urlEncode(const String& teks) {
  String hasil = "";
  const char* hex = "0123456789ABCDEF";
  for (size_t i = 0; i < teks.length(); i++) {
    char c = teks[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
      hasil += c;
    } else {
      hasil += '%';
      hasil += hex[((unsigned char)c >> 4) & 0x0F];
      hasil += hex[(unsigned char)c & 0x0F];
    }
  }
  return hasil;
}

bool kirimTelegram(const String& pesan) {
  if (strlen(TELEGRAM_TOKEN) == 0 || WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) +
               "/sendMessage?chat_id=" + String(TELEGRAM_CHAT_ID) +
               "&text=" + urlEncode(pesan);
  http.begin(client, url);
  int kode = http.GET();
  http.end();
  Serial.println("[Telegram] kode " + String(kode));
  return kode == 200;
}

// Kirim notifikasi hanya saat status BERUBAH menjadi BOROS / BOCOR
void cekNotifikasi() {
  static String listrikSebelum = "HEMAT", airSebelum = "NORMAL";

  if (d.statusListrik == "BOROS" && listrikSebelum != "BOROS") {
    kirimTelegram("KostBox " + String(ID_KAMAR) + ": listrik BOROS! " + alasanListrik + ".");
  }
  if (d.statusAir == "BOROS" && airSebelum != "BOROS") {
    kirimTelegram("KostBox " + String(ID_KAMAR) + ": air BOROS! Pemakaian hari ini " +
                  angka(d.airTotal - d.airAwalHari, 1) + " L, melewati batas " +
                  angka(BATAS_AIR_HARIAN_L, 0) + " L/hari.");
  }
  if (d.statusAir == "BOCOR" && airSebelum != "BOCOR") {
    kirimTelegram("KostBox " + String(ID_KAMAR) + ": air terus mengalir (" +
                  angka(d.debit, 1) + " L/menit). Kemungkinan keran lupa ditutup atau bocor!");
  }
  listrikSebelum = d.statusListrik;
  airSebelum     = d.statusAir;
}

// ========================== WIFI ===============================
void hubungkanWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Menghubungkan WiFi");
  unsigned long mulai = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - mulai < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" terhubung, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" gagal, akan dicoba lagi otomatis");
    Serial.printf("Kode status WiFi: %d (1=nama WiFi tidak ditemukan, 4=password salah, 6=belum tersambung)\n", WiFi.status());
    Serial.println("Jaringan WiFi yang terlihat oleh ESP32:");
    int n = WiFi.scanNetworks();
    bool ketemu = false;
    for (int i = 0; i < n; i++) {
      Serial.printf("  - \"%s\"  sinyal %d dBm\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
      if (WiFi.SSID(i) == WIFI_SSID) ketemu = true;
    }
    if (n <= 0) Serial.println("  (tidak ada jaringan terdeteksi)");
    Serial.println(ketemu ? ">> Nama hotspot terlihat. Kemungkinan password salah."
                          : ">> Nama hotspot TIDAK terlihat. Cek hotspot menyala, pita 2.4 GHz, dan ejaan nama.");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ====================== SETUP & LOOP ===========================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_FLOW, INPUT_PULLDOWN);   // cegah pulsa palsu saat sensor belum terpasang
  pinMode(PIN_TOMBOL, INPUT_PULLUP);
#if MODE_SENSOR_DC
  analogSetPinAttenuation(PIN_SHUNT, ADC_0db);      // rentang kecil, lebih teliti untuk mV
  analogSetPinAttenuation(PIN_TEGANGAN, ADC_11db);  // rentang sampai ~3 V
#endif
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), isrFlow, FALLING);

#if PAKAI_LCD
  Wire.begin(21, 22);
#endif
  layarAwal();

  muatData();
  hubungkanWiFi();
  configTime(ZONA_WAKTU_DETIK, 0, "pool.ntp.org", "time.google.com");

  unsigned long now = millis();
  terakhirBaca   = now;
  terakhirSimpan = now;
  terakhirKirim  = now - INTERVAL_KIRIM_MS + 20000;  // kiriman pertama ~20 detik setelah nyala
}

void loop() {
  cekPerintahSerial();
#if MODE_SIMULASI_LISTRIK
  cekTombol();
#endif
  unsigned long now = millis();

  // Baca sensor & perbarui layar tiap 1 detik
  if (now - terakhirBaca >= 1000) {
    float dt = (now - terakhirBaca) / 1000.0;
    terakhirBaca = now;

    bacaListrik(dt);
    bacaAir(dt);
    cekGantiHari();
    tentukanStatus();
    cekNotifikasi();
    perbaruiLayar();

    tampilSerial();
  }

  // Kirim ke ThingSpeak
  if (now - terakhirKirim >= INTERVAL_KIRIM_MS) {
    terakhirKirim = now;
    kirimThingSpeak();
  }

  // Simpan total air/energi ke flash (agar tidak hilang saat listrik padam)
  if (now - terakhirSimpan >= INTERVAL_SIMPAN_MS) {
    terakhirSimpan = now;
    simpanData();
  }

  // Sambung ulang WiFi bila putus
  if (WiFi.status() != WL_CONNECTED && now - terakhirCekWifi >= 30000) {
    terakhirCekWifi = now;
    WiFi.reconnect();
  }
}
