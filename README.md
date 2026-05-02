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

## Funcionalidades Implementadas

### 1. Núcleo de Simulação Digital (`modelo_aerogerador.py`)
* **Solução Numérica RK4**: Implementação do método de Runge-Kutta de 4ª ordem para integração das equações diferenciais de velocidade angular e corrente.
* **Modelo de Coeficiente de Potência ($C_p$)**: Suporte a múltiplos modelos matemáticos de eficiência aerodinâmica baseados em $\lambda$ (Tip-Speed Ratio).
* **Dinâmica de Transmissão**: Modelagem completa incluindo inércia da turbina, inércia do gerador, coeficientes de atrito viscoso e relação de caixa de engrenagens.
* **Cálculo de Perdas**: Estimativa em tempo real de perdas por efeito Joule e perdas mecânicas.

### 2. Algoritmos de Controle (`pid_module.py`)
* **Controlador PID com Anti-Windup**: Implementação robusta que utiliza *integração condicional* para evitar a saturação do termo integral quando a saída atinge os limites de tensão/corrente do hardware.
* **Sincronização HIL**: Gerenciamento de tempos distintos entre o passo de simulação digital e o passo de comunicação com o hardware.

### 3. Gerenciamento de Hardware e Dados (`main.py` & `init_serial_devices.py`)
* **Auto-Detecção Serial**: Interface amigável para listagem e seleção de portas COM para o inversor e o torquímetro.
* **Logger de Dados**: Classe `RegistroDeEmulacao` otimizada com `numpy` para armazenar todas as variáveis de estado (torque, potência, velocidade, erro de controle) sem perda de performance.
* **Visualização Dinâmica**: Plotagem em tempo real utilizando `matplotlib` para monitoramento de torque de referência vs. torque real e potência de saída.

-----

## Estrutura do Repositório

| Arquivo | Função Principal |
| :--- | :--- |
| `main.py` | Orquestrador do loop principal, interface gráfica e logs. |
| `modelo_aerogerador.py` | Motor de física da turbina e do gerador elétrico. |
| `pid_module.py` | Lógica de controle PID com proteção de saturação. |
| `init_serial_devices.py` | Utilitário de inicialização e varredura de portas seriais. |
| `parametros.py` | Centralização de todas as constantes físicas e de controle. |

-----

## Como Utilizar

1. **Configuração de Hardware**: Certifique-se de que o inversor e o torquímetro estão conectados.
2. **Dependências**:
   ```bash
   pip install numpy matplotlib pyserial
