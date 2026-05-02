import numpy as np

class TurbinaVirtual:    
    def __init__(self,
                 constante_velocidade,
                 resistencia_armadura,
                 indutancia_armadura,
                 raio_turbina,
                 inercia_turbina,
                 coeficiente_atrito_turbina,
                 taxa_variacao_tensao_armadura,
                 velocidade_angular_maxima_pas,
                 torque_maximo_motor,
                 relacao_transmissao_caixa_engrenagens = 1.0,
                 inercia_gerador = 0.0,
                 coeficiente_atrito_gerador = 0.0,
                 corrente_armadura_inicial = 0.0,
                 velocidade_angular_turbina_inicial = 0.0,
                 modelo_coeficiente_potencia = 0):
        
        self.esta_em_inicializacao = True
        self.modelo_coeficiente_potencia = modelo_coeficiente_potencia
        self.torque_maximo_motor_nm = torque_maximo_motor
        
        self.constante_velocidade_gerador = constante_velocidade
        self.resistencia_armadura_ohms = resistencia_armadura
        self.indutancia_armadura_henries = indutancia_armadura
        self.relacao_transmissao_caixa_engrenagens = relacao_transmissao_caixa_engrenagens
        self.densidade_ar_kg_m3 = 1.2754 
        self.raio_turbina_metros = raio_turbina
        self.area_varrida_turbina_m2 = np.pi * raio_turbina**2
        self.inercia_equivalente_sistema = inercia_turbina + (inercia_gerador * relacao_transmissao_caixa_engrenagens**2)

        self.inverso_indutancia_armadura = 1 / self.indutancia_armadura_henries
        self.inverso_inercia_sistema = 1 / self.inercia_equivalente_sistema

        self.coeficiente_atrito_turbina = coeficiente_atrito_turbina
        self.coeficiente_atrito_gerador = coeficiente_atrito_gerador
        self.coeficiente_atrito_equivalente = self.coeficiente_atrito_turbina + (self.coeficiente_atrito_gerador * (self.relacao_transmissao_caixa_engrenagens**2))

        self.taxa_variacao_tensao_armadura_v_s = taxa_variacao_tensao_armadura
        self.taxa_variacao_angulo_pas_rad_s = np.deg2rad(velocidade_angular_maxima_pas)        

        self.corrente_armadura_amperes = corrente_armadura_inicial
        self.velocidade_angular_turbina_rad_s = velocidade_angular_turbina_inicial
        self.angulo_pas_radianos = 0.0
        self.tensao_armadura_volts = 0.0
        self.velocidade_angular_gerador_rad_s = 0.0
        
        self.torque_eletromagnetico_gerador_nm = 0.0
        self.torque_aerodinamico_pas_nm = 0.0
        self.torque_atrito_pas_nm = 0.0
        self.torque_atrito_gerador_nm = 0.0
        self.torque_refletido_carga_nm = 0.0

        self.potencia_perdas_efeito_joule_w = 0.0
        self.potencia_perdas_atrito_pas_w = 0.0
        self.potencia_perdas_atrito_gerador_w = 0.0
        self.potencia_absorvida_vento_w = 0.0
        self.potencia_eixo_alta_velocidade_w = 0.0
        self.potencia_gerada_w = 0.0
        self.potencia_inercial_w = 0.0
        self.potencia_eletrica_entregue_w = 0.0
        self.erro_balanco_energetico_w = 0.0

        self.angulo_pas_anterior_radianos = 0.0
        self.tensao_armadura_anterior_volts = 0.0

    def calcular_coeficiente_potencia(self, tip_speed_ratio, angulo_pas_radianos):
        if self.modelo_coeficiente_potencia == 0:
            angulo_pas_graus = np.rad2deg(angulo_pas_radianos)
            constante_1, constante_2, constante_3, constante_5, constante_6 = 0.5, 116.0, 0.4, 5.0, 21.0
            
            denominador_termo_1 = tip_speed_ratio + 0.08 * angulo_pas_graus
            termo_1 = 1.0 / denominador_termo_1 if denominador_termo_1 != 0 else 0.0
            termo_2 = 0.035 / (angulo_pas_graus**3 + 1.0)
            
            lambda_inverso = termo_1 - termo_2
            coeficiente = constante_1 * (constante_2 * lambda_inverso - constante_3 * angulo_pas_graus - constante_5) * np.exp(-constante_6 * lambda_inverso)
            
        elif self.modelo_coeficiente_potencia == 1:
            fator_seno = np.sin(((np.pi * tip_speed_ratio) / 15.0 - 0.3 * angulo_pas_radianos))
            coeficiente = ((0.44 - 0.167 * angulo_pas_radianos) * fator_seno) - 0.16 * tip_speed_ratio * angulo_pas_radianos
            
        return max(0.0, coeficiente)

    def calcular_torque_aerodinamico(self, velocidade_vento_m_s, velocidade_angular_turbina_rad_s, angulo_pas_radianos):
        if velocidade_vento_m_s <= 0.1 or velocidade_angular_turbina_rad_s == 0.0: 
            return 0.0
        
        tip_speed_ratio = (self.raio_turbina_metros * velocidade_angular_turbina_rad_s) / velocidade_vento_m_s
        coeficiente_potencia = self.calcular_coeficiente_potencia(tip_speed_ratio, angulo_pas_radianos)
    
        potencia_disponivel_vento = 0.5 * self.densidade_ar_kg_m3 * self.area_varrida_turbina_m2 * (velocidade_vento_m_s**3)
        torque_calculado = (potencia_disponivel_vento * coeficiente_potencia) / velocidade_angular_turbina_rad_s
        torque_maximo_permitido = self.torque_maximo_motor_nm * self.relacao_transmissao_caixa_engrenagens
        
        return min(torque_calculado, torque_maximo_permitido)
        
    def calcular_derivada_corrente_armadura(self, corrente_atual, velocidade_turbina_atual, tensao_armadura_aplicada):
        corrente_atual = max(0.0, corrente_atual)
        velocidade_turbina_atual = max(0.0, velocidade_turbina_atual)

        forca_eletromotriz = self.constante_velocidade_gerador * (velocidade_turbina_atual * self.relacao_transmissao_caixa_engrenagens)
        queda_tensao_resistor = self.resistencia_armadura_ohms * corrente_atual
        
        derivada = self.inverso_indutancia_armadura * (forca_eletromotriz - queda_tensao_resistor - tensao_armadura_aplicada)

        if self.esta_em_inicializacao:
            return min(derivada, 0.0)
        return derivada

    def calcular_derivada_velocidade_angular(self, corrente_atual, velocidade_turbina_atual, torque_aerodinamico_atual):
        corrente_atual = max(0.0, corrente_atual)
        velocidade_turbina_atual = max(0.0, velocidade_turbina_atual)
        
        torque_eletromagnetico_referencia = (self.constante_velocidade_gerador * corrente_atual) * self.relacao_transmissao_caixa_engrenagens
        torque_friccao = self.coeficiente_atrito_equivalente * velocidade_turbina_atual
        
        aceleracao_turbina_teorica = self.inverso_inercia_sistema * (torque_aerodinamico_atual - torque_eletromagnetico_referencia - torque_friccao)

        limite_aceleracao_gerador_rad_s2 = 100.0 * (2.0 * np.pi / 60.0) 
        limite_aceleracao_turbina_rad_s2 = limite_aceleracao_gerador_rad_s2 / self.relacao_transmissao_caixa_engrenagens

        return np.clip(aceleracao_turbina_teorica, -limite_aceleracao_turbina_rad_s2, limite_aceleracao_turbina_rad_s2)

    def executar_passo_simulacao(self, velocidade_vento_m_s, tensao_armadura_alvo_volts, angulo_pas_alvo_radianos, passo_tempo_segundos):
        variacao_maxima_tensao = self.taxa_variacao_tensao_armadura_v_s * passo_tempo_segundos
        diferenca_tensao = tensao_armadura_alvo_volts - self.tensao_armadura_anterior_volts
        self.tensao_armadura_volts = self.tensao_armadura_anterior_volts + np.clip(diferenca_tensao, -variacao_maxima_tensao, variacao_maxima_tensao)
        self.tensao_armadura_anterior_volts = self.tensao_armadura_volts

        variacao_maxima_angulo = self.taxa_variacao_angulo_pas_rad_s * passo_tempo_segundos
        diferenca_angulo = angulo_pas_alvo_radianos - self.angulo_pas_anterior_radianos
        self.angulo_pas_radianos = self.angulo_pas_anterior_radianos + np.clip(diferenca_angulo, -variacao_maxima_angulo, variacao_maxima_angulo)
        self.angulo_pas_anterior_radianos = self.angulo_pas_radianos

        torque_aerodinamico = self.calcular_torque_aerodinamico(velocidade_vento_m_s, self.velocidade_angular_turbina_rad_s, self.angulo_pas_radianos)

        k1_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura(self.corrente_armadura_amperes, self.velocidade_angular_turbina_rad_s, self.tensao_armadura_volts)
        k1_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular(self.corrente_armadura_amperes, self.velocidade_angular_turbina_rad_s, torque_aerodinamico)
        
        k2_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura(self.corrente_armadura_amperes + 0.5 * k1_corrente, self.velocidade_angular_turbina_rad_s + 0.5 * k1_velocidade, self.tensao_armadura_volts)
        k2_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular(self.corrente_armadura_amperes + 0.5 * k1_corrente, self.velocidade_angular_turbina_rad_s + 0.5 * k1_velocidade, torque_aerodinamico)
        
        k3_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura(self.corrente_armadura_amperes + 0.5 * k2_corrente, self.velocidade_angular_turbina_rad_s + 0.5 * k2_velocidade, self.tensao_armadura_volts)
        k3_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular(self.corrente_armadura_amperes + 0.5 * k2_corrente, self.velocidade_angular_turbina_rad_s + 0.5 * k2_velocidade, torque_aerodinamico)
        
        k4_corrente = passo_tempo_segundos * self.calcular_derivada_corrente_armadura(self.corrente_armadura_amperes + k3_corrente, self.velocidade_angular_turbina_rad_s + k3_velocidade, self.tensao_armadura_volts)
        k4_velocidade = passo_tempo_segundos * self.calcular_derivada_velocidade_angular(self.corrente_armadura_amperes + k3_corrente, self.velocidade_angular_turbina_rad_s + k3_velocidade, torque_aerodinamico)

        self.corrente_armadura_amperes += (k1_corrente + 2.0 * k2_corrente + 2.0 * k3_corrente + k4_corrente) / 6.0
        self.corrente_armadura_amperes = max(0.0, self.corrente_armadura_amperes)
        
        self.velocidade_angular_turbina_rad_s += (k1_velocidade + 2.0 * k2_velocidade + 2.0 * k3_velocidade + k4_velocidade) / 6.0
        self.velocidade_angular_turbina_rad_s = max(0.0, self.velocidade_angular_turbina_rad_s)

        self.velocidade_angular_gerador_rad_s = self.velocidade_angular_turbina_rad_s * self.relacao_transmissao_caixa_engrenagens

        self.torque_aerodinamico_pas_nm = torque_aerodinamico
        self.torque_eletromagnetico_gerador_nm = self.constante_velocidade_gerador * self.corrente_armadura_amperes
        self.torque_atrito_pas_nm = self.coeficiente_atrito_turbina * self.velocidade_angular_turbina_rad_s
        self.torque_atrito_gerador_nm = self.coeficiente_atrito_gerador * self.velocidade_angular_gerador_rad_s
        self.torque_refletido_carga_nm = self.torque_eletromagnetico_gerador_nm * self.relacao_transmissao_caixa_engrenagens
        
        self.potencia_perdas_efeito_joule_w = self.resistencia_armadura_ohms * (self.corrente_armadura_amperes**2)
        self.potencia_perdas_atrito_pas_w = self.torque_atrito_pas_nm * self.velocidade_angular_turbina_rad_s
        self.potencia_perdas_atrito_gerador_w = self.torque_atrito_gerador_nm * self.velocidade_angular_gerador_rad_s        
        
        self.potencia_absorvida_vento_w = self.torque_aerodinamico_pas_nm * self.velocidade_angular_turbina_rad_s
        self.potencia_eixo_alta_velocidade_w = self.potencia_absorvida_vento_w - self.potencia_perdas_atrito_pas_w
        self.potencia_gerada_w = self.torque_eletromagnetico_gerador_nm * self.velocidade_angular_gerador_rad_s 
        
        torque_aceleracao = (self.torque_aerodinamico_pas_nm - self.torque_atrito_pas_nm - (self.torque_atrito_gerador_nm * self.relacao_transmissao_caixa_engrenagens) - self.torque_refletido_carga_nm)
        
        self.potencia_inercial_w = torque_aceleracao * self.velocidade_angular_turbina_rad_s
        self.potencia_eletrica_entregue_w = self.potencia_gerada_w - self.potencia_perdas_efeito_joule_w

        soma_perdas = self.potencia_perdas_atrito_pas_w + self.potencia_perdas_atrito_gerador_w + self.potencia_gerada_w + self.potencia_inercial_w
        self.erro_balanco_energetico_w = self.potencia_absorvida_vento_w - soma_perdas