-----

# Emulador de Turbina Eólica: Bancada de Ensaios em Tempo Real (`WindSimHIL`)

[](https://www.python.org/)
[](https://opensource.org/licenses/MIT)

-----

## Visão Geral

O projeto `WindSimHIL` é um ecossistema de software em Python projetado para transformar uma bancada de motores em um **Emulador de Turbina Eólica de Alta Fidelidade**. Ele integra um modelo matemático dinâmico de uma turbina eólica (Gêmeo Digital) com hardware físico através de comunicação serial em tempo real. 

O sistema permite a emulação de diferentes perfis de vento e o estudo do comportamento mecânico e elétrico de aerogeradores, abstraindo a complexidade do controle de hardware e da integração numérica para o pesquisador.

-----

## Contexto de Utilização no Projeto

No âmbito da pesquisa de microrredes, este software atua como o núcleo de processamento de uma bancada HIL (*Hardware-in-the-loop*). Ele utiliza o driver do inversor **WEG CFW11** para acionar o motor de indução e a biblioteca de leitura do **transdutor de torque T25 (Interface Inc)** para fechar o loop de controle. 

A lógica implementada garante que o torque medido no eixo físico siga o torque calculado pelo modelo aerodinâmico virtual, permitindo testes de algoritmos de **MPPT** e análise de dinâmica de carga com precisão laboratorial.

-----

## Drivers de Comunicação e Integração de Hardware

A robustez da emulação depende da integração direta com os componentes físicos através de drivers de baixo nível desenvolvidos especificamente para este protocolo:

### 1. Driver do Inversor de Frequência (`LCINVfunctions`)
* **Hardware:** WEG CFW11.
* **Papel no Projeto:** Atua como o **atuador** do sistema. O driver converte as referências de velocidade angular calculadas pelo modelo virtual em telegramas de bytes (baseados em STX/ETX e BCC) que o inversor compreende.
* **Uso:** É utilizado para enviar comandos de *setpoint* de velocidade e monitorar o status de operação do motor que simula o rotor eólico.

### 2. Driver do Transdutor de Torque (`LCTSfunctions`)
* **Hardware:** Interface Inc. T25 (ou Lorenz Messtechnik).
* **Papel no Projeto:** Atua como o **sensor de feedback**. Ele realiza a leitura em tempo real do torque e do RPM efetivos no eixo da bancada.
* **Uso:** O driver abstrai o protocolo proprietário de telegramas binários, entregando valores decimais limpos para o controlador PID, permitindo o ajuste dinâmico da carga e a verificação da potência mecânica real.

-----

## Funcionalidades Implementadas

### 1. Núcleo de Simulação Digital (`modelo_aerogerador.py`)
* **Solução Numérica RK4**: Implementação do método de Runge-Kutta de 4ª ordem para integração das equações diferenciais de velocidade e corrente.
* **Modelo de Coeficiente de Potência ($C_p$)**: Suporte a modelos matemáticos de eficiência aerodinâmica baseados em $\lambda$ (Tip-Speed Ratio).
* **Dinâmica de Transmissão**: Modelagem incluindo inércia da turbina, inércia do gerador e relação de caixa de engrenagens.

### 2. Algoritmos de Controle (`pid_module.py`)
* **Controlador PID com Anti-Windup**: Utiliza *integração condicional* para evitar a saturação do termo integral, garantindo que o sistema não "oscile" ao atingir os limites físicos do inversor.
* **Sincronização HIL**: Gerenciamento de tempos distintos entre o passo de simulação digital e o passo de comunicação serial.

### 3. Interface e Logs (`main.py` & `init_serial_devices.py`)
* **Auto-Detecção Serial**: Varredura automática de portas COM para facilitar a conexão dos dispositivos.
* **Logger de Dados**: Registro otimizado com `numpy` de todas as variáveis (Torque, RPM, Potência e Erro).
* **Visualização Dinâmica**: Plotagem em tempo real de Referência vs. Medição Real via `matplotlib`.

-----

## Estrutura do Repositório

| Arquivo | Função Principal |
| :--- | :--- |
| `main.py` | Orquestrador do loop principal, interface gráfica e logs. |
| `driver_inversor_cfw11.py` | Driver de comunicação com o inversor WEG. |
| `driver_torquimetro.py` | Driver de comunicação com o sensor de torque T25. |
| `modelo_aerogerador.py` | Motor de física da turbina e do gerador elétrico. |
| `pid_module.py` | Lógica de controle PID com proteção de saturação. |
| `init_serial_devices.py` | Utilitário de inicialização de portas seriais. |

-----

## Contato

* **Isaque Verona** - [GitHub Profile](https://github.com/isaqueveron)

-----

## Referências

* Manual de Comunicação Serial WEG CFW11.
* Protocolo de Comunicação Transdutores de Torque Interface Inc.
* Modelagem Dinâmica de Turbinas Eólicas de Velocidade Variável.

**Key-words:** wind turbine, HIL, PID, anti-windup, WEG CFW11, torque sensor, emulação, tempo real.

**Updates:**
* **v2.0 (Abril/2026):** Integração completa dos drivers de hardware com loop de controle fechado e interface gráfica em tempo real.
