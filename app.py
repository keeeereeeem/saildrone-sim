import streamlit as st
import numpy as np
import matplotlib
matplotlib.use('Agg') # GUI hatasını önler
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from sim import boat_dynamics, W_speed

st.set_page_config(page_title="Saildrone & Yelken Simülatörü", layout="wide")

# Tek ekrana sığdırmak için kompakt CSS
st.markdown("""
<style>
    .block-container {
        padding-top: 0.8rem !important;
        padding-bottom: 0rem !important;
        padding-left: 2rem !important;
        padding-right: 2rem !important;
        max-width: 100% !important;
    }
    h2, h3 {
        font-size: 1.15rem !important;
        margin-top: 0rem !important;
        margin-bottom: 0.4rem !important;
        font-weight: 600 !important;
    }
    .stSlider, .stSelectbox {
        margin-bottom: -0.3rem !important;
    }
    div[data-testid="stMetricValue"] {
        font-size: 1.15rem !important;
    }
    div[data-testid="stMetricLabel"] {
        font-size: 0.78rem !important;
    }
    div[data-testid="stMetricDelta"] {
        font-size: 0.75rem !important;
    }
    hr {
        margin: 0.3rem 0 !important;
    }
</style>
""", unsafe_allow_html=True)

st.markdown("### ⛵ Yelkenli Katamaran ve Saildrone Manevra Simülatörü")

# --- UI KONTROLLERİ ---
col1, col2 = st.columns([1, 2.2], gap="medium")

with col1:
    st.subheader("⚙️ Kontrol Paneli")
    
    # 1. Profil Seçimi
    naca_profile = st.selectbox("Kanat / Yelken Profili", ["NACA 0012 (Simetrik)", "NACA 0015", "Kumaş Yelken (Cambered)"])
    
    # 2. Tekne Yönü
    boat_heading_deg = st.slider("Tekne Yönü (Derece)", 0, 360, 45, help="0°: Doğu (Apaz), 45°: Kuzeydoğu (Orsa), 90°: Kuzey (Rüzgara Karşı / Kör Nokta), 270°: Güney (Pupa).")
    
    # 3. Sancak Yelken Açısı
    theta_deg = st.slider("Sancak Yelken Açısı (Derece)", -90, 90, 15, help="Yelkenin tekneye açısı (Klasik yelken için).")
    
    # 4. Dümen Açısı
    rudder_deg = st.slider("Dümen (Derece)", -45, 45, 0, help="Dönüş açısı.")
    
    # 5. Simülasyon Süresi
    sim_time = st.slider("Simülasyon Süresi (sn)", 10, 60, 30)

with col2:
    st.subheader("📊 Rota ve Performans Çıktısı")
    
    boat_heading = np.radians(boat_heading_deg)
    theta_rad = np.radians(theta_deg)
    rudder_rad = np.radians(rudder_deg)
    
    t_span = (0, sim_time)
    t_eval = np.linspace(0, sim_time, sim_time * 10)
    state0 = [0, 0, boat_heading, 0, 0, 0]
    
    # Çözüm: Klasik
    sol_classic = solve_ivp(boat_dynamics, t_span, state0, t_eval=t_eval, 
                            args=(False, theta_rad, rudder_rad, naca_profile), method='RK45')
                            
    # Çözüm: Saildrone
    sol_drone = solve_ivp(boat_dynamics, t_span, state0, t_eval=t_eval, 
                            args=(True, 0.0, rudder_rad, naca_profile), method='RK45')
                            
    # Metrik Hesaplamaları
    dist_c = np.hypot(sol_classic.y[0, -1], sol_classic.y[1, -1])
    dist_d = np.hypot(sol_drone.y[0, -1], sol_drone.y[1, -1])
    speed_c_knot = sol_classic.y[3, -1] * 1.94384
    speed_d_knot = sol_drone.y[3, -1] * 1.94384
    
    # Kompakt Çizim
    fig, ax = plt.subplots(figsize=(5.0, 2.5), dpi=100)
    
    ax.plot(sol_classic.y[0], sol_classic.y[1], 'r--', linewidth=2, label='Klasik Yelken (Sabit Açı)')
    ax.plot(sol_drone.y[0], sol_drone.y[1], 'b-', linewidth=2, label='Saildrone (Otomatik Trim)')
    
    # Bitiş noktaları
    ax.plot(sol_classic.y[0, -1], sol_classic.y[1, -1], 'ro', markersize=5)
    ax.plot(sol_drone.y[0, -1], sol_drone.y[1, -1], 'bo', markersize=5)
    
    # Tekne başlangıç yönü oku
    ax.arrow(0, 0, 4*np.cos(boat_heading), 4*np.sin(boat_heading), 
             head_width=0.8, color='green', alpha=0.6, label='Başlangıç Yönü')
    
    # Rüzgar vektörleri
    for xw in np.linspace(-15, 15, 7):
        ax.arrow(xw, 5, 0, -2.5, head_width=0.7, color='cyan', alpha=0.45)
    
    ax.set_title(f"Rüzgar: {W_speed} m/s (Yukarıdan Aşağıya)", fontsize=9, pad=4)
    ax.set_xlabel("X (m)", fontsize=8, labelpad=2)
    ax.set_ylabel("Y (m)", fontsize=8, labelpad=2)
    ax.tick_params(axis='both', labelsize=8)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="lower right", fontsize=7.5, framealpha=0.85)
    ax.axis('equal')
    plt.tight_layout(pad=0.5)
    
    st.pyplot(fig)
    
    # Kompakt Canlı Telemetri Kartları
    m_col1, m_col2, m_col3 = st.columns(3)
    with m_col1:
        st.metric(label="⛵ Klasik Mesafe", value=f"{dist_c:.1f} m", delta=f"{speed_c_knot:.1f} kts")
    with m_col2:
        st.metric(label="🤖 Saildrone Mesafe", value=f"{dist_d:.1f} m", delta=f"{speed_d_knot:.1f} kts")
    with m_col3:
        fark = dist_d - dist_c
        st.metric(label="🚀 Saildrone Kazancı", value=f"{'+' if fark>=0 else ''}{fark:.1f} m", 
                  delta=f"%{((dist_d/dist_c - 1)*100):.0f} Fazla" if dist_c > 0.5 else "Tam Kontrol")
    
    st.caption("💡 **Özet:** Klasik yelkende açı elle sabitlenirken; Saildrone serbest kanat ve kuyruğuyla optimum hücum açısını daima otomatik koruyarak üstün hareket özgürlüğü sağlar.")
