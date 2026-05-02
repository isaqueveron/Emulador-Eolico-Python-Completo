# Bancada de Emulação de Turbina Eólica Integrada em Microrrede

Este repositório contém o ecossistema de software desenvolvido para o projeto de pesquisa **"BANCADA DE EMULAÇÃO DE TURBINA EÓLICA INTEGRADA EM MICRORREDE"**. O sistema utiliza Python para realizar a emulação em tempo real de uma turbina eólica, integrando modelos matemáticos avançados com hardware físico (motor, inversor e transdutor de torque) para validar algoritmos de controle e gestão de energia[cite: 9, 10].

---

## 📋 Visão Geral do Sistema

O projeto é dividido em módulos que gerenciam desde a simulação da física do vento e da aerodinâmica das pás até o controle determinístico de hardware via comunicação serial[cite: 9, 10]. O núcleo do sistema é a **Turbina Virtual**, um gêmeo digital que calcula o torque teórico esperado, enquanto o sistema de controle garante que o motor físico replique esse comportamento no eixo[cite: 10].

### Fluxograma de Operação
A emulação opera em dois loops principais sincronizados:
1. **Loop Digital (Simulação)**: Processa a física da turbina (modelo de $C_p$, inércia, torque aerodinâmico) a cada 20ms[cite: 7, 10].
2. **Loop de Hardware (Controle)**: Realiza a leitura dos sensores e o ajuste do inversor de frequência a cada 1ms, garantindo latência mínima no seguimento de torque[cite: 7, 9].

---

## 🛠️ Componentes do Projeto

### 1. Modelo de Turbina Virtual (`TurbinaVirtual`)
Implementa o modelo dinâmico do aerogerador, considerando:
* **Cálculo de $C_p$**: Modelos matemáticos para eficiência aerodinâmica baseada no *Tip Speed Ratio* ($\lambda$) e ângulo de passo ($\beta$)[cite: 10].
* **Dinâmica de Eixo**: Integração numérica (Runge-Kutta de 4ª ordem) para velocidades angulares e correntes de armadura[cite: 10].
* **Parâmetros Configuráveis**: Inércia equivalente, coeficientes de atrito e constantes de velocidade do gerador[cite: 7, 10].

### 2. Comunicação com Transdutor (`Torquimeter`)
Baseado na biblioteca `LCTSfunctions`, gerencia a comunicação serial (RS232/485) com o sensor de torque T25[cite: 11].
* **Protocolo**: Telegramas com controle de *byte stuffing* e *checksum* ponderado[cite: 11].
* **Aquisição**: Leitura de Torque (N.m), RPM e cálculo de potência mecânica no eixo[cite: 11].

### 3. Controle do Inversor (`Inverter`)
Módulo `LCINVfunctions` para interface com inversores WEG (ex: CFW11) via protocolo serial proprietário[cite: 12].
* **Comandos**: Ativação do motor, parada de emergência e ajuste de referência de velocidade angular[cite: 12].

### 4. Controlador PID (`PIDController`)
Controlador com lógica de **Anti-Windup** (integração condicional) para evitar a saturação dos termos integrais durante transientes bruscos de vento[cite: 13].

---

## 📂 Estrutura de Arquivos

| Arquivo | Descrição |
| :--- | :--- |
| `main_emulacao.py` | Script principal que coordena o loop de tempo real e interface[cite: 9]. |
| `modelo_aerogerador.py` | Classe da turbina virtual e dinâmica física[cite: 10]. |
| `driver_torquimetro.py` | Implementação da classe `Torquimeter` para o sensor T25[cite: 11]. |
| `driver_inversor_cfw11.py` | Implementação da classe `Inverter` para controle do motor[cite: 12]. |
| `parametros.py` | Constantes físicas e configurações de hardware (Inércia, $R_a$, $L_a$)[cite: 7]. |
| `gerar_vento.py` | Script para geração de perfis de vento sintéticos ou naturais[cite: 8]. |

---

## ⚙️ Configuração do Hardware

Para o funcionamento correto, o hardware deve seguir as especificações:
* **Baudrate Torquímetro**: 230.400 bps[cite: 7, 11].
* **Baudrate Inversor**: 57.600 bps[cite: 12].
* **Passo de Hardware**: 1ms ($0,001s$) para garantir estabilidade do controle[cite: 7].
* **Filtros**: Aplicados filtros passa-baixa nos sinais de torque e potência para mitigar ruídos[cite: 9].

---

## 🚀 Como Executar

1. **Instale as dependências**:
   ```bash
   pip install numpy matplotlib pyserial
