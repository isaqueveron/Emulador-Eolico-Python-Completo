import time
import numpy as np
import matplotlib.pyplot as plt

from parametros import *
from modelo_aerogerador import TurbinaVirtual
from driver_torquimetro import Torquimeter
from driver_inversor_cfw11 import Inverter
from pid_module import PIDController 
from init_serial_devices import selecionar_porta

class RegistroDeEmulacao:
    def __init__(self, duracao_s, passo_hardware, passo_simulacao):
        # Tamanho total estimado baseado no tempo total de emulação
        self.n_hw = int(duracao_s / passo_hardware) + 1
        self.n_sim = int(duracao_s / passo_simulacao) + 1
        
        # Índices de controle para preenchimento
        self.idx_hw = 0
        self.idx_sim = 0

        self.tempos_simulacao_digital = np.zeros(self.n_sim)
        self.velocidades_angulares_gerador_digital = np.zeros(self.n_sim)
        self.torques_aerodinamicos_digitais = np.zeros(self.n_sim)
        self.potencias_eixo_alta_velocidade_digitais = np.zeros(self.n_sim)
        self.coeficientes_potencia_digitais = np.zeros(self.n_sim)

        self.tempos_hardware_fisico = np.zeros(self.n_hw)
        self.comandos_esforco_inversor = np.zeros(self.n_hw)
        self.velocidades_angulares_reais_rpm = np.zeros(self.n_hw)
        self.torques_referencia_calculados = np.zeros(self.n_hw)
        self.torques_medidos_sensor = np.zeros(self.n_hw)
        self.potencias_medidas_sensor = np.zeros(self.n_hw)
        self.latencias_comunicacao_ms = np.zeros(self.n_hw)
        self.coeficientes_potencia_reais = np.zeros(self.n_hw)

def calcular_velocidade_vento_degraus(tempo_decorrido_segundos):
    if tempo_decorrido_segundos < 10.0: return 0.0
    if tempo_decorrido_segundos < 30.0: return 2.0
    if tempo_decorrido_segundos < 50.0: return 3.0
    if tempo_decorrido_segundos < 70.0: return 4.0
    if tempo_decorrido_segundos < 90.0: return 5.0
    if tempo_decorrido_segundos < 110.0: return 6.0
    if tempo_decorrido_segundos < 130.0: return 7.0
    if tempo_decorrido_segundos < 150.0: return 4.0
    if tempo_decorrido_segundos < 190.0: return 2.0
    return 0.0

def aplicar_filtro_passa_baixa(valor_atual, valor_anterior_filtrado, passo_tempo, constante_tempo):
    fator_suavizacao = passo_tempo / (constante_tempo + passo_tempo)
    return fator_suavizacao * valor_atual + (1.0 - fator_suavizacao) * valor_anterior_filtrado

def configurar_interface_tempo_real():
    plt.ion() 
    figura, (eixo_torque, eixo_potencia) = plt.subplots(2, 1, figsize=(10, 8))
    figura.canvas.manager.set_window_title('Monitoramento em Tempo Real - Bancada de Emulação')

    linha_torque_referencia, = eixo_torque.plot([], [], 'r--', label='Torque Ref (Modelo)')
    linha_torque_real, = eixo_torque.plot([], [], 'k-', label='Torque Real (Eixo)')
    eixo_torque.set_ylabel("Torque [N.m]")
    eixo_torque.legend(loc='upper right')
    eixo_torque.grid(True)

    linha_potencia_virtual, = eixo_potencia.plot([], [], 'purple', label='Potência Eixo Virtual')
    linha_potencia_real, = eixo_potencia.plot([], [], 'black', linestyle=':', label='Potência Eixo Real')
    eixo_potencia.set_xlabel("Tempo [s]")
    eixo_potencia.set_ylabel("Potência [W]")
    eixo_potencia.legend(loc='upper right')
    eixo_potencia.grid(True)

    visor_digital = eixo_potencia.text(0.5, 0.85, '', transform=eixo_potencia.transAxes, 
                                       fontsize=16, fontweight='bold', color='darkgreen',
                                       ha='center', bbox=dict(facecolor='white', alpha=0.8, edgecolor='green'))
    plt.tight_layout()
    return figura, eixo_torque, eixo_potencia, linha_torque_referencia, linha_torque_real, linha_potencia_virtual, linha_potencia_real, visor_digital

def plotar_graficos_analise_final(registros):
    h = registros.idx_hw
    s = registros.idx_sim

    # --- PROCESSAMENTO DE DADOS (Pós-Emulação) ---
    # Cria arrays locais para armazenar as versões filtradas
    torques_filtrados_plot = np.zeros(h)
    potencias_filtradas_plot = np.zeros(h)
    cp_filtrado_plot = np.zeros(h) # Novo array para o Cp filtrado
    
    if h > 0:
        torques_filtrados_plot[0] = registros.torques_medidos_sensor[0]
        potencias_filtradas_plot[0] = registros.potencias_medidas_sensor[0]
        cp_filtrado_plot[0] = registros.coeficientes_potencia_reais[0] # Inicializa o Cp
        
        # Reconstrói os sinais filtrados usando os parâmetros globais
        for i in range(1, h):
            torques_filtrados_plot[i] = aplicar_filtro_passa_baixa(
                registros.torques_medidos_sensor[i], 
                torques_filtrados_plot[i-1], 
                PASSO_HARDWARE_SEGUNDOS, 
                0.05 # Constante de tempo fixa para o torque apenas visual
            )
            potencias_filtradas_plot[i] = aplicar_filtro_passa_baixa(
                registros.potencias_medidas_sensor[i], 
                potencias_filtradas_plot[i-1], 
                PASSO_HARDWARE_SEGUNDOS, 
                CONSTANTE_TEMPO_FILTRO_SENSOR_POTENCIA
            )
            # Filtra o Cp usando a mesma constante de tempo da potência
            cp_filtrado_plot[i] = aplicar_filtro_passa_baixa(
                registros.coeficientes_potencia_reais[i],
                cp_filtrado_plot[i-1],
                PASSO_HARDWARE_SEGUNDOS,
                CONSTANTE_TEMPO_FILTRO_SENSOR_POTENCIA
            )

    # --- PLOTAGEM DOS GRÁFICOS ---
    plt.figure("Seguimento de Hardware")
    plt.subplot(2, 1, 1)
    # Sinal Bruto em Cinza Claro (Fundo)
    plt.plot(registros.tempos_hardware_fisico[:h], registros.torques_medidos_sensor[:h], color='lightgray', alpha=0.7, linewidth=1, label='Torque Bruto (Real)')
    # Sinal Filtrado (Sobreposto)
    plt.plot(registros.tempos_hardware_fisico[:h], torques_filtrados_plot, 'k', linewidth=1.5, label='Torque Filtrado')
    plt.plot(registros.tempos_hardware_fisico[:h], registros.torques_referencia_calculados[:h], 'r--', label='Torque Ref')
    plt.ylabel('N.m'); plt.legend(); plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(registros.tempos_hardware_fisico[:h], registros.comandos_esforco_inversor[:h], 'b', label='Esforço Controle (u)')
    plt.ylabel('Ref Inversor'); plt.legend(); plt.grid(True)

    plt.figure("Latência da Comunicação Serial")
    plt.plot(registros.tempos_hardware_fisico[:h], registros.latencias_comunicacao_ms[:h], color='teal', alpha=0.5, label='Latência')
    plt.ylabel('Latência [ms]'); plt.legend(); plt.grid(True)

    plt.figure("Validação Potência")
    # Sinal Bruto em Cinza Claro
    plt.plot(registros.tempos_hardware_fisico[:h], registros.potencias_medidas_sensor[:h], color='lightgray', alpha=0.7, linewidth=1, label='Potência Bruta (Real)')
    # Sinal Filtrado e Modelo
    plt.plot(registros.tempos_simulacao_digital[:s], registros.potencias_eixo_alta_velocidade_digitais[:s], 'purple', label='Potência Modelo')
    plt.plot(registros.tempos_hardware_fisico[:h], potencias_filtradas_plot, 'k--', alpha=0.9, linewidth=1.5, label='Potência Filtrada')
    plt.ylabel('Watts [W]'); plt.legend(); plt.grid(True)

    plt.figure("Validação Velocidade Angular")
    plt.plot(registros.tempos_simulacao_digital[:s], registros.velocidades_angulares_gerador_digital[:s], 'purple', label='Velocidade Modelo')
    plt.plot(registros.tempos_hardware_fisico[:h], registros.velocidades_angulares_reais_rpm[:h], 'k--', alpha=0.7, label='Velocidade Real')
    plt.ylabel('[RPM]'); plt.legend(); plt.grid(True)

    plt.figure("Eficiência Aerodinâmica")
    plt.ylim(0,1)
    plt.plot(registros.tempos_simulacao_digital[:s], registros.coeficientes_potencia_digitais[:s], 'purple', linewidth=1.5, zorder=3, label="Cp Modelo")
    # Sinal Bruto em Cinza Claro
    plt.plot(registros.tempos_hardware_fisico[:h], registros.coeficientes_potencia_reais[:h], color='lightgray', alpha=0.7, linewidth=1, zorder=1, label="Cp Bruto (Real)")
    # Sinal Filtrado
    plt.plot(registros.tempos_hardware_fisico[:h], cp_filtrado_plot, 'k--', linewidth=1.5, zorder=2, label="Cp Filtrado")
    
    plt.axhline(y=0.593, color='r', linestyle='--', zorder=4, label="Limite de Betz")
    plt.ylabel("Cp [-]"); plt.legend(); plt.grid(True)
    
    plt.show()

# --- INICIALIZAÇÃO PRINCIPAL ---
turbina_digital = TurbinaVirtual(
    CONSTANTE_VELOCIDADE_GERADOR_V_RAD_S, RESISTENCIA_ARMADURA_OHMS, INDUTANCIA_ARMADURA_HENRIES,
    RAIO_TURBINA_METROS, INERCIA_TURBINA_KG_M2, COEFICIENTE_ATRITO_TURBINA,
    TAXA_VARIACAO_TENSAO_ARMADURA_V_S, VELOCIDADE_MAXIMA_PAS_GRAUS_S, TORQUE_MAXIMO_MOTOR_NM,
    RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS, INERCIA_GERADOR_KG_M2, COEFICIENTE_ATRITO_GERADOR,
    modelo_coeficiente_potencia=0
)

torquimetro_fisico = Torquimeter(Port=selecionar_porta("Torquimetro"), Baudrate=230400, Timeout=0.003) 
inversor_motor = Inverter(Port=selecionar_porta("Inversor"), ADR=1, Baudrate=57600)

inversor_motor.ActivateMotor()
time.sleep(0.1)
inversor_motor.SendReferenceAngularVelocity(0)
time.sleep(1)

controlador_mppt_velocidade = PIDController(kp=20.0, ki=5.0, kd=0.0, out_min=TENSAO_MINIMA_INVERSOR_V, out_max=TENSAO_MAXIMA_INVERSOR_V)
controlador_seguimento_torque = PIDController(kp=400.0, ki=300.0, kd=0.0, out_min=0.0, out_max=1000.0) 

registros = RegistroDeEmulacao(TEMPO_TOTAL_EMULACAO_SEGUNDOS,PASSO_HARDWARE_SEGUNDOS,PASSO_SIMULACAO_SEGUNDOS)
figura, eixo_torque, eixo_potencia, linha_torque_referencia, linha_torque_real, linha_potencia_virtual, linha_potencia_real, visor_digital = configurar_interface_tempo_real()

motor_em_soft_start = True
comando_velocidade_inversor = 0.0
torque_aerodinamico_referencia_nm = 0.0
velocidade_vento_atual_m_s = 0.0
potencia_sensor_filtrada_w = 0.0
velocidade_angular_sensor_filtrado_rad_s = 0.0
velocidade_angular_alvo_filtrada_rad_s = 0.0
tensao_armadura_alvo_filtrada_volts = 0.0
index_vetor_vento = 0

tempo_inicio_absoluto = time.perf_counter()
tempo_ultima_simulacao_digital = tempo_inicio_absoluto
tempo_ultimo_controle_hardware = tempo_inicio_absoluto
tempo_ultima_atualizacao_graficos = tempo_inicio_absoluto

print(f"Emulação Iniciada por {TEMPO_TOTAL_EMULACAO_SEGUNDOS}s...")

try:
    while (time.perf_counter() - tempo_inicio_absoluto) < TEMPO_TOTAL_EMULACAO_SEGUNDOS:
        tempo_atual = time.perf_counter()
        tempo_decorrido = tempo_atual - tempo_inicio_absoluto

            
        if (tempo_atual - tempo_ultima_simulacao_digital) >= PASSO_SIMULACAO_SEGUNDOS: 
            
            if PERFIL_VENTO == 'ESCADA': velocidade_vento_atual_m_s = calcular_velocidade_vento_degraus(tempo_decorrido)
            if PERFIL_VENTO == 'NATURAL': 
                try: 
                    velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento]
                    index_vetor_vento += 1
                except: 
                    velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento - 1]
                    index_vetor_vento = 100
                    
            velocidade_angular_alvo_rad_s = (velocidade_vento_atual_m_s * TSR_IDEAL / RAIO_TURBINA_METROS)
            velocidade_angular_alvo_filtrada_rad_s = aplicar_filtro_passa_baixa(velocidade_angular_alvo_rad_s, velocidade_angular_alvo_filtrada_rad_s, PASSO_SIMULACAO_SEGUNDOS, CONSTANTE_TEMPO_FILTRO_VELOCIDADE_ALVO)

            if velocidade_vento_atual_m_s >= VELOCIDADE_VENTO_MINIMA_M_S:
                if turbina_digital.esta_em_inicializacao:
                    turbina_digital.velocidade_angular_turbina_rad_s += INCREMENTO_VELOCIDADE_ANG_SOFTSTART_SIMULACAO_RAD_S
                    tensao_armadura_alvo_volts = turbina_digital.velocidade_angular_gerador_rad_s * turbina_digital.constante_velocidade_gerador
                    tensao_armadura_alvo_filtrada_volts = tensao_armadura_alvo_volts
                    turbina_digital.tensao_armadura_volts = tensao_armadura_alvo_volts
                    
                    if turbina_digital.velocidade_angular_turbina_rad_s >= VELOCIDADE_EIXO_FIM_SOFT_START_PERC_ALVO * velocidade_angular_alvo_filtrada_rad_s: 
                        turbina_digital.esta_em_inicializacao = False
                    
                    controlador_mppt_velocidade.compute(velocidade_angular_alvo_filtrada_rad_s, turbina_digital.velocidade_angular_turbina_rad_s, PASSO_SIMULACAO_SEGUNDOS, True)

                else:
                    tensao_armadura_calculada_volts = controlador_mppt_velocidade.compute(velocidade_angular_alvo_filtrada_rad_s, turbina_digital.velocidade_angular_turbina_rad_s, PASSO_SIMULACAO_SEGUNDOS, True)
                    tensao_armadura_alvo_filtrada_volts = aplicar_filtro_passa_baixa(tensao_armadura_calculada_volts, tensao_armadura_alvo_filtrada_volts, PASSO_SIMULACAO_SEGUNDOS, CONSTANTE_TEMPO_FILTRO_VELOCIDADE_ALVO)
                    tensao_armadura_alvo_volts = tensao_armadura_alvo_filtrada_volts

            if velocidade_vento_atual_m_s >= VELOCIDADE_VENTO_MINIMA_M_S:
                turbina_digital.esta_em_inicializacao = True
                turbina_digital.velocidade_angular_turbina_rad_s -= INCREMENTO_VELOCIDADE_ANG_SOFTSTART_SIMULACAO_RAD_S
                tensao_armadura_alvo_volts = turbina_digital.velocidade_angular_gerador_rad_s * turbina_digital.constante_velocidade_gerador
                tensao_armadura_alvo_filtrada_volts = tensao_armadura_alvo_volts
                turbina_digital.tensao_armadura_volts = tensao_armadura_alvo_volts 
                controlador_mppt_velocidade.compute(velocidade_angular_alvo_filtrada_rad_s, turbina_digital.velocidade_angular_turbina_rad_s, PASSO_SIMULACAO_SEGUNDOS, True)

            ANGULO_PAS_REFERENCIA_RADIANOS = 0.0

            turbina_digital.executar_passo_simulacao(velocidade_vento_atual_m_s, tensao_armadura_alvo_volts, ANGULO_PAS_REFERENCIA_RADIANOS, PASSO_SIMULACAO_SEGUNDOS)

            registros.tempos_simulacao_digital[registros.idx_sim] = tempo_decorrido
            registros.velocidades_angulares_gerador_digital[registros.idx_sim] = turbina_digital.velocidade_angular_gerador_rad_s * 60.0 / (2.0 * np.pi)
            registros.torques_aerodinamicos_digitais[registros.idx_sim] = turbina_digital.torque_aerodinamico_pas_nm / RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS
            registros.potencias_eixo_alta_velocidade_digitais[registros.idx_sim] = turbina_digital.potencia_eixo_alta_velocidade_w
            
            potencia_disponivel_vento = 0.5 * turbina_digital.densidade_ar_kg_m3 * turbina_digital.area_varrida_turbina_m2 * (velocidade_vento_atual_m_s**3) if velocidade_vento_atual_m_s > 0 else 1.0
            registros.coeficientes_potencia_digitais[registros.idx_sim] = (turbina_digital.potencia_eixo_alta_velocidade_w / potencia_disponivel_vento)
            
            registros.idx_sim += 1

            tempo_ultima_simulacao_digital += PASSO_SIMULACAO_SEGUNDOS

        if (tempo_atual - tempo_ultimo_controle_hardware) >= PASSO_HARDWARE_SEGUNDOS:
            tempo_inicio_comunicacao = time.perf_counter()
            torquimetro_fisico.ReadRaw() 

            torque_real_lido_nm = torquimetro_fisico.Torque_calibrated 
            potencia_real_lida_w = torquimetro_fisico.Potencia_calculated
            velocidade_angular_real_rad_s = torquimetro_fisico.RPM_calibrated * (2.0 * np.pi) / 60.0

            if potencia_real_lida_w < POTENCIA_MECANICA_MAXIMA_W:
                inversor_motor.SendReferenceAngularVelocity(comando_velocidade_inversor)
            else:
                print("Limite de potência atingido na bancada!")
                break

            potencia_sensor_filtrada_w = aplicar_filtro_passa_baixa(potencia_real_lida_w, potencia_sensor_filtrada_w, PASSO_HARDWARE_SEGUNDOS, CONSTANTE_TEMPO_FILTRO_SENSOR_POTENCIA)
            velocidade_angular_sensor_filtrado_rad_s = aplicar_filtro_passa_baixa(velocidade_angular_real_rad_s, velocidade_angular_sensor_filtrado_rad_s, PASSO_HARDWARE_SEGUNDOS, CONSTANTE_TEMPO_FILTRO_SENSOR_VELOCIDADE)
            torque_aerodinamico_referencia_nm = turbina_digital.calcular_torque_aerodinamico(velocidade_vento_atual_m_s, velocidade_angular_sensor_filtrado_rad_s / RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS, 0.0) / RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS

            if velocidade_vento_atual_m_s >= VELOCIDADE_VENTO_MINIMA_M_S:
                if motor_em_soft_start:
                    comando_velocidade_inversor += INCREMENTO_VELOCIDADE_ANG_SOFT_START_EIXO_RPM 
                    if potencia_sensor_filtrada_w >= POTENCIA_EIXO_FIM_SOFT_START_W: 
                        motor_em_soft_start = False

                else: comando_velocidade_inversor = controlador_seguimento_torque.compute(torque_aerodinamico_referencia_nm, torque_real_lido_nm, PASSO_HARDWARE_SEGUNDOS)

            if velocidade_vento_atual_m_s < VELOCIDADE_VENTO_MINIMA_M_S: 
                torque_aerodinamico_referencia_nm = 0.0
                controlador_seguimento_torque.compute(torque_aerodinamico_referencia_nm, torque_real_lido_nm, PASSO_HARDWARE_SEGUNDOS)
                motor_em_soft_start = True
                comando_velocidade_inversor -= INCREMENTO_VELOCIDADE_ANG_SOFT_START_EIXO_RPM

                
            registros.tempos_hardware_fisico[registros.idx_hw] = tempo_decorrido
            registros.comandos_esforco_inversor[registros.idx_hw] = comando_velocidade_inversor
            registros.velocidades_angulares_reais_rpm[registros.idx_hw] = torquimetro_fisico.RPM_calibrated
            registros.torques_referencia_calculados[registros.idx_hw] = torque_aerodinamico_referencia_nm
            registros.torques_medidos_sensor[registros.idx_hw] = torque_real_lido_nm
            registros.potencias_medidas_sensor[registros.idx_hw] = potencia_real_lida_w
            registros.latencias_comunicacao_ms[registros.idx_hw] = (time.perf_counter() - tempo_inicio_comunicacao) * 1000.0
            
            potencia_disponivel_vento_hardware = 0.5 * turbina_digital.densidade_ar_kg_m3 * turbina_digital.area_varrida_turbina_m2 * (velocidade_vento_atual_m_s**3) if velocidade_vento_atual_m_s > 0 else 1.0
            registros.coeficientes_potencia_reais[registros.idx_hw] = (potencia_real_lida_w/ potencia_disponivel_vento_hardware)
            
            registros.idx_hw += 1

            tempo_ultimo_controle_hardware = tempo_atual

        if (tempo_atual - tempo_ultima_atualizacao_graficos) >= TAXA_ATUALIZACAO_GRAFICOS_SEGUNDOS:
            linha_torque_referencia.set_data(registros.tempos_hardware_fisico[:registros.idx_hw], 
                                             registros.torques_referencia_calculados[:registros.idx_hw])
            
            linha_torque_real.set_data(registros.tempos_hardware_fisico[:registros.idx_hw], 
                                        registros.torques_medidos_sensor[:registros.idx_hw])
            
            linha_potencia_virtual.set_data(registros.tempos_simulacao_digital[:registros.idx_sim], 
                                            registros.potencias_eixo_alta_velocidade_digitais[:registros.idx_sim])
            
            linha_potencia_real.set_data(registros.tempos_hardware_fisico[:registros.idx_hw], 
                                         registros.potencias_medidas_sensor[:registros.idx_hw])
            
            if registros.idx_hw > 0:
                valor_potencia_atual = registros.potencias_medidas_sensor[registros.idx_hw - 1]
                visor_digital.set_text(f'POTÊNCIA NO EIXO: {valor_potencia_atual:.2f} W')
            
            for eixo in [eixo_torque, eixo_potencia]:
                eixo.relim()
                eixo.autoscale_view()
            
            plt.pause(0.001)
            tempo_ultima_atualizacao_graficos = tempo_atual

finally:
    inversor_motor.SendReferenceAngularVelocity(0)
    time.sleep(0.1)
    inversor_motor.StopMotor()
    plt.ioff()
    print("Emulação Finalizada. Gerando gráficos de análise...")
    plotar_graficos_analise_final(registros)