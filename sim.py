import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# --- TEKNE VE ÇEVRE PARAMETRELERİ ---
m = 100.0       # Tekne kütlesi (kg)
Iz = 150.0      # Atalet momenti (kg.m^2)
rho_air = 1.2   # Hava yoğunluğu (kg/m^3)
rho_water = 1000# Su yoğunluğu (kg/m^3)

A_sail = 5.0    # Yelken/Kanat alanı (m^2)
cp_sail = 0.0   # Yelkenin döndürme etkisini (kontrolsüz fırıldak olmayı) iptal ediyoruz ki sadece dümen döndürsün.

hull_drag_coeff = 25.0  # Gövde ileri yön direnç katsayısı
hull_sway_coeff = 2000.0 # Gövde yanal direnç (salma/keel etkisi - kaymayı önler)
hull_yaw_coeff = 300.0  # Dönme sönümleme

# Sabit Rüzgar: Kuzeyden Güneye doğru esiyor (-y yönünde)
W_speed = 6.0   # m/s (yaklaşık 12 knot)
V_w_x = 0.0
V_w_y = -W_speed

def lift_drag(alpha_deg, profile_name="NACA 0012 (Simetrik)"):
    """
    Gerçekçi kanat/yelken aerodinamik polar modeli.
    CL: Taşıma katsayısı, CD: Sürükleme katsayısı
    """
    abs_alpha = abs(alpha_deg)
    sign_alpha = np.sign(alpha_deg) if alpha_deg != 0 else 1.0
    
    if "0012" in profile_name:
        stall_angle = 13.0
        CL_alpha = 0.085
        CD0 = 0.015
        k_ind = 0.08
        CL_max = CL_alpha * stall_angle  # ~1.10
    elif "0015" in profile_name:
        stall_angle = 15.0
        CL_alpha = 0.080
        CD0 = 0.020
        k_ind = 0.08
        CL_max = CL_alpha * stall_angle  # ~1.20
    else:
        # Kumaş yelken (Cambered/Bombeli)
        stall_angle = 16.0
        CL_alpha = 0.075
        CD0 = 0.040
        k_ind = 0.09
        CL_max = 1.45
        
    if abs_alpha <= stall_angle:
        if "Kumaş" in profile_name or "Cambered" in profile_name:
            # Bombeli kumaş yelken sıfır hücum açısında bile pozitif lift üretir
            CL = (0.35 + CL_alpha * abs_alpha) * sign_alpha
        else:
            CL = CL_alpha * alpha_deg
        # Gerçekçi indüklenmiş sürükleme (Induced drag: CD = CD0 + k * CL^2)
        CD = CD0 + k_ind * (abs(CL)**2)
    else:
        # Stall sonrası yumuşak düşüş ve ayrılmış akış direnci
        decay = (stall_angle / abs_alpha)**0.8
        CL = CL_max * decay * sign_alpha
        CD = CD0 + k_ind * (CL_max**2) + 1.0 * (np.sin(np.radians(abs_alpha - stall_angle))**2)
        
    return abs(CL), CD

def boat_dynamics(t, state, is_saildrone, theta_input, rudder_angle, profile_name="NACA 0012 (Simetrik)"):
    x, y, psi, u, v, r = state
    
    # 1. Kinematik - Teknenin Dünya'ya göre hızı
    boat_v_x = u * np.cos(psi) - v * np.sin(psi)
    boat_v_y = u * np.sin(psi) + v * np.cos(psi)
    
    # 2. Görünen Rüzgar (Apparent Wind)
    aw_x = V_w_x - boat_v_x
    aw_y = V_w_y - boat_v_y
    aw_speed = np.sqrt(aw_x**2 + aw_y**2)
    if aw_speed < 1e-4:
        return [0, 0, 0, 0, 0, 0]
        
    # Rüzgar akışının birim vektörü (Havanın aktığı yön)
    ew_x = aw_x / aw_speed
    ew_y = aw_y / aw_speed
    
    # Rüzgarın geliş açısı ve tekneye göre bağıl açısı
    aw_dir = np.arctan2(aw_y, aw_x)
    aw_coming_from = aw_dir + np.pi
    gamma_rel = (aw_coming_from - psi + np.pi) % (2 * np.pi) - np.pi
    gamma_rel_deg = np.degrees(gamma_rel)
    
    # 3. Kör Nokta (No-Go Zone / In Irons): Rüzgara doğrudan karşı (+/- 28 derece) seyir yapılamaz
    if abs(gamma_rel_deg) < 28.0:
        CL, CD = 0.0, 0.06
        nl_x, nl_y = 0.0, 0.0
    else:
        # 4. Hücum Açısı (alpha) Hesabı
        if is_saildrone:
            # Saildrone kuyruk kanadı sayesinde rüzgara göre daima optimum ~12 derece hücum açısını korur
            alpha_deg = 12.0
        else:
            # Klasik yelkende sancak açısı elle sabitlenmiştir (theta_input)
            alpha_deg = abs(gamma_rel_deg) - abs(np.degrees(theta_input))
            if alpha_deg < 0:
                alpha_deg = 0.0  # Yelken pırpırlanır (flapping / backwinded)
                
        CL, CD = lift_drag(alpha_deg, profile_name)
        
        # Lift yönü: Hava akışına diktir. 
        # Rüzgar iskeleden (soldan, gamma_rel > 0) geliyorsa Lift sancağa (sağa) yönelir.
        if gamma_rel > 0:
            nl_x = -ew_y
            nl_y = ew_x
        else:
            nl_x = ew_y
            nl_y = -ew_x
            
    # 5. Yelken Aerodinamik Kuvvetleri
    q = 0.5 * rho_air * (aw_speed**2) * A_sail
    F_lift_x = q * CL * nl_x
    F_lift_y = q * CL * nl_y
    F_drag_x = q * CD * ew_x
    F_drag_y = q * CD * ew_y
    
    F_sail_x = F_lift_x + F_drag_x
    F_sail_y = F_lift_y + F_drag_y
    
    # Dünya ekseninden Tekne eksenine (u: ileri, v: yanal) çevir
    F_sail_u = F_sail_x * np.cos(psi) + F_sail_y * np.sin(psi)
    F_sail_v = -F_sail_x * np.sin(psi) + F_sail_y * np.cos(psi)
    
    # 6. Gövde Direnç Kuvvetleri (Hidrodinamik)
    F_hull_u = -hull_drag_coeff * u * abs(u)
    F_hull_v = -hull_sway_coeff * v * abs(v)
    
    # Dümen Kuvveti ve Momenti (Daha gerçekçi dönüş dinamiği)
    # Su akış hızı (u) ile orantılı dümen yan kuvveti ve momenti
    F_rudder_v = -35.0 * u * abs(u) * np.sin(rudder_angle)
    M_rudder = -F_rudder_v * 1.5
    
    # Gövde dönme sönümlemesi (doğrusal + karesel sönümleme: kontrolsüz spin atmayı önler)
    M_hull_r = -600.0 * r * abs(r) - 350.0 * r
    
    # 7. Toplam İvmeler (F = m*a)
    Sum_F_u = F_sail_u + F_hull_u
    Sum_F_v = F_sail_v + F_hull_v + F_rudder_v
    Sum_M = (F_sail_v * cp_sail) + M_hull_r + M_rudder
    
    u_dot = (Sum_F_u / m) + r * v
    v_dot = (Sum_F_v / m) - r * u
    r_dot = Sum_M / Iz
    
    x_dot = boat_v_x
    y_dot = boat_v_y
    psi_dot = r
    
    return [x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot]

def run_experiment_1():
    """ 
    Deney 1: Sabit rüzgar (-y yönünde), dümen=0, başlangıçta hareketsiz.
    Farklı rüzgar açılarına (45, 90, 135 derece) göre teknenin aldığı yollar.
    """
    t_span = (0, 30) # 30 saniyelik simülasyon
    t_eval = np.linspace(0, 30, 300)
    
    # Test edilecek tekne yönleri: (Rüzgar -y yönünde, yani 270 veya -90 dereceden geliyor gibi düşünebiliriz. 
    # Rüzgar vektörü [0, -W].
    # Kuzeye (0) bakan tekne için rüzgar tam karşıdan (kafadan) gelir.
    # Doğuya (90 derece) bakan tekne için rüzgar iskeleden (yandan) gelir.
    
    # Tekne yönleri: 45 (Dar apaz), 90 (Apaz), 135 (Geniş apaz)
    headings = [np.radians(45), np.radians(90), np.radians(135)]
    heading_labels = ["45° (Dar Apaz)", "90° (Apaz)", "135° (Geniş Apaz)"]
    
    plt.figure(figsize=(10, 8))
    
    # 1. Klasik Yelken (Sancak Açısı = 15 derece sabit)
    theta_fixed = np.radians(15)
    
    colors = ['r', 'g', 'b']
    
    for i, psi0 in enumerate(headings):
        state0 = [0, 0, psi0, 0, 0, 0]
        
        # Klasik Yelken Simülasyonu
        sol_classic = solve_ivp(boat_dynamics, t_span, state0, t_eval=t_eval, 
                                args=(False, theta_fixed, 0.0, "NACA 0012 (Simetrik)"), method='RK45')
        plt.plot(sol_classic.y[0], sol_classic.y[1], '--', color=colors[i], 
                 label=f'Klasik Yelken - {heading_labels[i]}')
        
        # Saildrone Simülasyonu (Kanat kendi kendine optimum açıyı buluyor)
        sol_drone = solve_ivp(boat_dynamics, t_span, state0, t_eval=t_eval, 
                              args=(True, 0.0, 0.0, "NACA 0012 (Simetrik)"), method='RK45')
        plt.plot(sol_drone.y[0], sol_drone.y[1], '-', color=colors[i], linewidth=2,
                 label=f'Saildrone - {heading_labels[i]}')
        
    plt.title("Deney 1: Farklı Rotalarda Seyir (Sabit Rüzgar: Yukarıdan Aşağıya)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axvline(0, color='black', linewidth=0.5)
    plt.axhline(0, color='black', linewidth=0.5)
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    
    # Rüzgar okları çizimi (Görsellik için)
    for x_wind in np.linspace(-10, 10, 5):
        plt.arrow(x_wind, 5, 0, -3, head_width=0.5, head_length=1, fc='cyan', ec='cyan', alpha=0.5)
    plt.text(0, 6, "RÜZGAR YÖNÜ", color='c', ha='center')
    
    plt.show()

if __name__ == "__main__":
    print("Simülasyon başlatılıyor... Lütfen grafik penceresinin açılmasını bekleyin.")
    run_experiment_1()
