# ⛵ Saildrone & Yelkenli Dinamiği Simülasyonu - İyileştirme, Modelleme ve Otonom Seyrüsefer Görev Şartnamesi

Bu belge, bu depodaki (`saildrone_sim`) mevcut Saildrone ve klasik yelkenli katamaran simülasyonunun incelenmesi, fiziksel/mantıksal hatalarının giderilmesi, eksiksiz matematiksel modellerinin çıkartılması, noktadan noktaya seyrüsefer (waypoint navigation) kabiliyetinin test edilmesi ve akademik/teknik bir rapor hazırlanması amacıyla hazırlanmıştır.

---

## 📌 1. Proje Özeti ve Mevcut Durum

Mevcut projede şu dosyalar bulunmaktadır:
1. `sim.py`: Tekne 3-DOF (Surge, Sway, Yaw) hareket denklemlerini, rüzgar/akış vektörlerini, aerodinamik polar eğrilerini (NACA 0012, NACA 0015, Kumaş Cambered) ve hidrodinamik direnç/dümen kuvvetlerini `scipy.integrate.solve_ivp` ile çözen temel fizik motoru.
2. `app.py`: Parametrelerin (NACA profili, rota açısı, yelken açısı, dümen açısı, simülasyon süresi) değiştirilebildiği ve teknelerin yörüngeleri ile telemetrilerini gösteren Streamlit tabanlı arayüz.
3. `requirements.txt`: Gerekli Python kütüphaneleri (`numpy`, `matplotlib`, `scipy`, `streamlit`).

---

## 🎯 2. Claude Code İçin Temel Görevler

Lütfen aşağıdaki 4 görevi sırasıyla ve eksiksiz olarak yerine getir:

---

### Görev 1: Fizik ve Mantık Hatalarının Denetlenmesi ve Düzeltilmesi

`sim.py` dosyasındaki mevcut fizik ve aerodinamik/hidrodinamik mantığı baştan sona incele. Özellikle şu kritik konuları doğrula ve gerekiyorsa düzelt:
1. **Görünen Rüzgar (Apparent Wind) ve Vektörel İzdüşümler:**
   - Tekne hızı $\vec{V}_b = [u \cos\psi - v \sin\psi, u \sin\psi + v \cos\psi]$ ile gerçek rüzgar $\vec{V}_w$ arasındaki bağıl hız vektörü $\vec{V}_{aw} = \vec{V}_w - \vec{V}_b$.
   - Kaldırma (Lift) ve Sürükleme (Drag) vektörlerinin yönelimleri.
   - Hücum açısının ($\alpha$) işaretinin ve doğrultusunun sancak/iskele kontralarda (port/starboard tack) sürekliliği ve fiziksel tutarlılığı.
2. **Saildrone Kanat & Kuyruk (Wing-Tail) Moment Dengesi:**
   - Saildrone rijit ana kanadı 360° serbestçe dönebilen bir direk üzerine kuruludur.
   - Kanadın arkasındaki aerodinamik kuyruk (tail-vane) ana kanada göre açılandırılarak aerodinamik moment sıfırlandığında ($M_{pivot} = 0$), ana kanat rüzgara göre optimum hücum açısında ($\alpha \approx 12^\circ$) kilitlenir.
   - Bu davranışın matematiksel olarak doğru modellendiğinden emin ol.
3. **Salma ve Gövde Hidrodinamiği (Keel Lift vs. Sway Damping):**
   - Gerçek yelkenlilerde salma (keel / daggerboard) sadece karesel sürtünme üretmez; tekne yanal kayma açısı (drift / leeway angle $\beta \approx \arctan(v/u)$) yaptıkça suya karşı **hidrodinamik kaldırma kuvveti (yanal itiş)** üreterek yelkenin yanal kuvvetini dengeler.
   - Salmanın hidrodinamik taşıma ve direnç karakteristiğinin fiziksel gerçekçiliğini teyit et/geliştir.
4. **Kör Nokta (No-Go Zone / In Irons):**
   - Tekne doğrudan rüzgarın geldiği yöne ($\pm 25^\circ - 35^\circ$) giderken ileri itiş üretemez. Bu geçişin yumuşak ve fiziksel sınırlarla uyumlu olduğunu doğrula.

---

### Görev 2: Kapsamlı Matematiksel Modelin Çıkartılması

Tüm sistemin diferansiyel denklemlerini ve cebirsel modellerini net, eksiksiz ve akademik standartta (LaTeX notasyonu ile) çıkart:
1. **Koordinat Sistemleri ve Dönüşüm Matrisleri:**
   - Dünya Sabit Eksen Takımı (Inertial Frame / NED / Earth Frame: $X, Y$).
   - Tekne Gövde Eksen Takımı (Body-fixed Frame: $u$ - İleri/Surge, $v$ - Yanal/Sway, $r$ - Dönüş/Yaw).
   - Gövde $\leftrightarrow$ Dünya hız ve kuvvet dönüşüm bağıntıları.
2. **3-DOF Doğrusal Olmayan Hareket Denklemleri:**
   $$m (\dot{u} - v r) = X_{total}$$
   $$m (\dot{v} + u r) = Y_{total}$$
   $$I_z \dot{r} = N_{total}$$
3. **Aerodinamik Model (Yelken / Kanat):**
   - Görünen rüzgar hızı ($V_{aw}$) ve hücum açısı ($\alpha$) denklemleri.
   - NACA 0012, NACA 0015 ve Bombeli Kumaş yelken için $C_L(\alpha)$ ve $C_D(\alpha)$ formülasyonu ($C_{D} = C_{D0} + k \cdot C_L^2$ indüklenmiş direnç ve stall modeli).
4. **Hidrodinamik Model (Gövde, Salma, Dümen):**
   - İleri yönlü gövde direnci ($F_{drag, u}$).
   - Salma yanal taşıma ve direnç kuvvetleri ($F_{keel, v}$).
   - Dümen yan kuvveti ve dümen yaw momenti ($F_{rudder}, M_{rudder}$).
   - Dönüş sönümleme momenti ($M_{hull, r}$).

---

### Görev 3: Otonom Noktadan Noktaya Seyrüsefer Testi (Waypoint Navigation)

Saildrone'un **belirtilen bir başlangıç konumundan $(x_0, y_0)$ istenilen bir hedef konuma $(x_{target}, y_{target})$ otonom olarak gidip gidemeyeceğini** test edecek bir seyrüsefer ve otopilot algoritması geliştir:

1. **Güdüm Algoritması (Guidance & Line-of-Sight - LOS):**
   - Hedefe olan mesafe ve hedef kerteriz açısı hesabı:
     $$\psi_{des} = \arctan2(y_{target} - y, x_{target} - x)$$
2. **Tramola (Tacking / Orsalama) Durum Makinesi (Rüzgara Karşı Seyir):**
   - **Kritik Gereksinim:** Eğer hedef rüzgarın geldiği yöndeyse (Kör Nokta / Upwind Dead Zone içindeyse), hiçbir yelkenli doğrudan hedefe gidemez!
   - Saildrone'un hedefe ulaşabilmesi için rüzgara $40^\circ - 45^\circ$ açılarla zigzaglar çizerek (tramola / tacking yaparak, "beating upwind") ilerleyen bir rota durum makinesi (tacking logic / corridor bounds) geliştir.
3. **Dümen Kontrolcüsü (Autopilot - Heading Controller):**
   - Hedef baş açısı ($\psi_{des}$) ile anlık baş açısı ($\psi$) arasındaki hatayı dümen açısına ($\delta_r$) dönüştüren bir kontrolcü (ör. PID veya doyumlu oransal kontrolcü):
     $$\delta_r = \text{clip}(K_p (\psi_{des} - \psi) - K_d r, -\delta_{max}, \delta_{max})$$
4. **Test Senaryoları:**
   - **Senaryo A (Apaz Seyri - Beam Reach):** Rüzgara dik yöndeki bir hedefe gidiş.
   - **Senaryo B (Pupa Seyri - Downwind):** Rüzgar yönündeki bir hedefe gidiş.
   - **Senaryo C (Orsa / Rüzgar Üstü - Upwind Beating):** Rüzgarın tam geldiği yöndeki bir hedefe tramola atarak (zigzag yaparak) tırmanış testi.
   - **Kabul Kriteri:** Teknenin belirlenen kabul yarıçapı içine ($R_{accept} \le 5\text{ m}$) başarıyla girip girmediği, varış süresi ve rota verimliliği.

---

### Görev 4: Test ve Matematiksel Modeller Raporunun Hazırlanması

Tüm bu çalışmaları kapsayan, hocaya ve akademik sunumlara sunulabilecek nitelikte kapsamlı bir Markdown raporu oluştur:
- **Dosya Adı:** `MATHEMATICAL_MODEL_AND_TEST_REPORT.md`
- **İçerik:**
  1. Giriş ve Saildrone Sistem Mimarisi (Makaralı klasik yelken vs. Serbest döner kanat-kuyruk karşılaştırması).
  2. Çıkartılan Eksiksiz Matematiksel Modeller (Tüm diferansiyel denklemler, aerodinamik ve hidrodinamik formüller, parametre tablosu).
  3. Otonom Seyrüsefer ve Güdüm Algoritması (LOS ve Tramola durum makinesinin açıklaması).
  4. Test Sonuçları ve Başarım Analizi (Senaryo A, B, C için başlangıç-hedef koordinatları, rota grafikleri, varış süreleri ve telemetri verileri).
  5. Sonuç ve Öneriler (Saildrone'un hareket özgürlüğü avantajının matematiksel ve simülatif kanıtı).

---

## 🛠️ 3. Beklenen Çıktılar

1. Güncellenmiş / doğrulanmış `sim.py`.
2. Otonom seyrüsefer ve hedef takibi sağlayan yeni bir navigasyon modülü (örneğin `navigation.py`).
3. Senaryoları çalıştırıp grafiklerini kaydeden bir test scripti (örneğin `test_waypoints.py`).
4. Eksiksiz teknik rapor: `MATHEMATICAL_MODEL_AND_TEST_REPORT.md`.
