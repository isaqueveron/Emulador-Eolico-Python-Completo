import numpy as np

def gerar_vento_dia_normal(t, tempo, h):
    v_media_diaria = 2.5 + 1.5 * np.sin(2 * np.pi * (t / tempo) - np.pi/2)
    v_rajada_lenta = 1.9 * np.sin(2 * np.pi * (t / 3600))
    v_base = v_media_diaria + v_rajada_lenta
    
    I = 0.10 
    sigma = I * v_base
    tempo_correlacao = 20.0  
    phi = np.exp(-h / tempo_correlacao)
    
    if not hasattr(gerar_vento_dia_normal, "last_noise"):
        gerar_vento_dia_normal.last_noise = 0 
    
    random_shock = np.random.normal(0, sigma * np.sqrt(1 - phi**2))
    noise = phi * gerar_vento_dia_normal.last_noise + random_shock
    gerar_vento_dia_normal.last_noise = noise
    v_final = v_base + noise
    return max(0, v_final), v_base

# Parâmetros da simulação
tempo_total = 60*3  # segundos
h = 0.02           # passo de tempo
t_vetor = np.arange(0, tempo_total + h, h)

# Resetando o estado do ruído da função (importante se você rodar várias vezes no mesmo console)
if hasattr(gerar_vento_dia_normal, "last_noise"):
    del gerar_vento_dia_normal.last_noise

# Gerando a lista de valores de vento (v_final)
lista_vento = []
for t in t_vetor:
    v_instantaneo, _ = gerar_vento_dia_normal(t, tempo_total, h)
    lista_vento.append(v_instantaneo)

# Verificações rápidas
print(f"Total de pontos gerados: {len(lista_vento)}")
print(f"Vento médio no período: {np.mean(lista_vento):.2f} m/s")
print(f"Vento máximo atingido: {np.max(lista_vento):.2f} m/s")

# Para SALVAR
np.save('vetor_vento.npy', lista_vento)