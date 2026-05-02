# Bancada de Emulação de Turbina Eólica Integrada em Microrrede

Este repositório contém o ecossistema de software desenvolvido para o projeto de pesquisa **"BANCADA DE EMULAÇÃO DE TURBINA EÓLICA INTEGRADA EM MICRORREDE"**. O sistema utiliza Python para realizar a emulação em tempo real de uma turbina eólica, integrando modelos matemáticos avançados com hardware físico (motor, inversor e transdutor de torque) para validar algoritmos de controle e gestão de energia.

---

## 📋 Visão Geral do Sistema

O projeto é dividido em módulos que gerenciam desde a simulação da física do vento e da aerodinâmica das pás até o controle determinístico de hardware via comunicação serial. O núcleo do sistema é a **Turbina Virtual**, um gêmeo digital que calcula o torque teórico esperado, enquanto o sistema de controle garante que o motor físico replique esse comportamento no eixo.

### Fluxograma de Operação
A emulação opera em dois loops principais sincronizados:
1. **Loop Digital (Simulação)**: Processa a física da turbina (modelo de $C_p$, inércia, torque aerodinâmico) a cada 20ms.
2. **Loop de Hardware (Controle)**: Realiza a leitura dos sensores e o ajuste do inversor de frequência a cada 1ms, garantindo latência mínima no seguimento de torque.

---

## 🛠️ Componentes do Projeto

### 1. Modelo de Turbina Virtual (`TurbinaVirtual`)
Implementa o modelo dinâmico do aerogerador, considerando:
* **Cálculo de $C_p$**: Modelos matemáticos para eficiência aerodinâmica baseada no *Tip Speed Ratio* ($\lambda$) e ângulo de passo ($\beta$).
* **Dinâmica de Eixo**: Integração numérica (Runge-Kutta de 4ª ordem) para velocidades angulares e correntes de armadura.
* **Parâmetros Configuráveis**: Inércia equivalente, coeficientes de atrito e constantes de velocidade do gerador.

### 2. Comunicação com Transdutor (`Torquimeter`)
Baseado na biblioteca `LCTSfunctions`, gerencia a comunicação serial (RS232/485) com o sensor de torque T25.
* **Protocolo**: Telegramas com controle de *byte stuffing* e *checksum* ponderado.
* **Aquisição**: Leitura de Torque (N.m), RPM e cálculo de potência mecânica no eixo.

### 3. Controle do Inversor (`Inverter`)
Módulo `LCINVfunctions` para interface com inversores WEG (ex: CFW11) via protocolo serial proprietário.
* **Comandos**: Ativação do motor, parada de emergência e ajuste de referência de velocidade angular.

### 4. Controlador PID (`PIDController`)
Controlador com lógica de **Anti-Windup** (integração condicional) para evitar a saturação dos termos integrais durante transientes bruscos de vento.

---

## 📂 Estrutura de Arquivos

| Arquivo | Descrição |
| :--- | :--- |
| `main_emulacao.py` | Script principal que coordena o loop de tempo real e interface. |
| `modelo_aerogerador.py` | Classe da turbina virtual e dinâmica física. |
| `driver_torquimetro.py` | Implementação da classe `Torquimeter` para o sensor T25. |
| `driver_inversor_cfw11.py` | Implementação da classe `Inverter` para controle do motor. |
| `parametros.py` | Constantes físicas e configurações de hardware (Inércia, $R_a$, $L_a$). |
| `gerar_vento.py` | Script para geração de perfis de vento sintéticos ou naturais. |

---

## ⚙️ Configuração do Hardware

Para o funcionamento correto, o hardware deve seguir as especificações:
* **Baudrate Torquímetro**: 230.400 bps.
* **Baudrate Inversor**: 57.600 bps.
* **Passo de Hardware**: 1ms ($0,001s$) para garantir estabilidade do controle.
* **Filtros**: Aplicados filtros passa-baixa nos sinais de torque e potência para mitigar ruídos.

---

## 🚀 Como Executar

1. **Instale as dependências**:
   ```bash
   pip install numpy matplotlib pyserial
