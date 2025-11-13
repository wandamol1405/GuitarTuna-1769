import serial
import numpy as np
from scipy.signal import butter, filtfilt, medfilt
from scipy.fft import rfft, rfftfreq

# ------------------ Config ------------------
PORT = 'COM9'
BAUD = 9600
N_expected = 2048      # muestras esperadas
fs = 10000             # sampling rate real en Hz

# Valores de calibracion (ponelos desde tu medicion)
OFFSET = 2048          # ejemplo: valor medio en silencio (ADC 12-bit)
SIGMA = 15             # desviacion estandar medida en silencio

# Hampel params
hampel_k = 4           # umbral en sigma
hampel_window = 11     # ventana impar

# Bandpass (ajustá según tu instrumento)
lowcut = 80.0    # Hz (eliminar sub-bajos)
highcut = 0.49*fs # Hz (o menos si tu interés es solo frecuencia fundamental baja)

# ------------------------------------------------


def butter_bandpass(lowcut, highcut, fs, order=4):
    nyq = 0.5 * fs
    lown = lowcut / nyq
    highn = highcut / nyq

    # Limitar dentro del rango permitido
    lown = max(0.0001, min(lown, 0.9999))
    highn = max(0.0001, min(highn, 0.9999))

    if lown >= highn:
        raise ValueError("⚠️ lowcut debe ser menor que highcut después de normalizar.")

    b, a = butter(order, [lown, highn], btype='band')
    return b, a


def hampel_filter(x, window_size=11, n_sigmas=4):
    # Devuelve arreglo donde outliers reemplazados por mediana de ventana
    L = len(x)
    k = (window_size - 1) // 2
    x_out = x.copy()
    for i in range(L):
        start = max(0, i - k)
        end = min(L, i + k + 1)
        window = x[start:end]
        med = np.median(window)
        mad = np.median(np.abs(window - med))
        if mad == 0:
            continue
        threshold = n_sigmas * 1.4826 * mad  # factor para aproximar sigma
        if abs(x[i] - med) > threshold:
            x_out[i] = med
    return x_out

def parabolic_interpolation(mag, peak_idx):
    # Interpolacion parabólica para refinar pico
    if peak_idx <= 0 or peak_idx >= len(mag)-1:
        return peak_idx
    alpha = mag[peak_idx-1]
    beta  = mag[peak_idx]
    gamma = mag[peak_idx+1]
    p = 0.5 * (alpha - gamma) / (alpha - 2*beta + gamma)
    return peak_idx + p

def estimate_freq_fft(x, fs):
    N = len(x)
    X = np.abs(rfft(x))
    freqs = rfftfreq(N, 1/fs)
    peak = np.argmax(X)
    peak_refined = parabolic_interpolation(X, peak)
    freq = np.interp(peak_refined, np.arange(len(freqs)), freqs)
    return freq, X, freqs

def estimate_freq_autocorr(x, fs, min_f=50, max_f=5000):
    # Autocorrelación simple para detectar periodo
    x = x - np.mean(x)
    corr = np.correlate(x, x, mode='full')
    corr = corr[corr.size//2:]
    # buscar primer pico después de lag 0
    d = np.diff(corr)
    start = np.where(d > 0)[0]
    if start.size == 0:
        return None
    # limitar lags por frecuencia maxima/minima
    min_lag = int(fs / max_f) if max_f>0 else 1
    max_lag = int(fs / min_f) if min_f>0 else len(corr)-1
    # encontrar primer máximo en rango
    lag = np.argmax(corr[min_lag:max_lag]) + min_lag
    if lag == 0:
        return None
    return fs / lag

# --------- Abrir puerto y loop principal ----------
ser = serial.Serial(PORT, BAUD, timeout=2)

b_bp, a_bp = butter_bandpass(lowcut, highcut, fs, order=4)

while True:
    line = ser.readline().decode(errors='ignore').strip()
    if not line:
        continue
    print("Recibido:", line[:40])
    if line.upper().startswith("START"):
        data_line = ser.readline().decode(errors='ignore').strip()
        # limpiar coma final
        if data_line.endswith(','):
            data_line = data_line[:-1]
        try:
            raw = np.array(list(map(int, data_line.split(','))), dtype=float)
        except Exception as e:
            print("Error parseando:", e)
            continue
        if len(raw) != N_expected:
            print(f"Warning: muestras recibidas {len(raw)} != esperado {N_expected}")

        # 1) Restar offset (calibracion)
        x = raw - OFFSET

        # 2) Rechazo de picos: primera pasada con hampel usando mad (robusto)
        x = hampel_filter(x, window_size=hampel_window, n_sigmas=hampel_k)

        # 3) Filtro mediano ligero para eliminar impulsos residuales
        x = medfilt(x, kernel_size=3)

        # 4) Bandpass digital con filtfilt para no introducir desfase
        try:
            x = filtfilt(b_bp, a_bp, x)
        except Exception as e:
            print("Error en filtfilt (posible N pequeño):", e)

        # 5) Estimación de frecuencia
        freq_fft, mag, freqs = estimate_freq_fft(x, fs)
        freq_ac = estimate_freq_autocorr(x, fs, min_f=50, max_f=3000)

        print(f"Freq FFT: {freq_fft:.2f} Hz | Freq Autocorr: {freq_ac if freq_ac is not None else 'N/A'}")

        # enviar resultado al micro si queres
        ser.write(f"FREQ:{freq_fft:.2f}\n".encode())
