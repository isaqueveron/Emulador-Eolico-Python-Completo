# Biblioteca de Comunicação do Transdutor de Torque Rotativo (`LCTSfunctions`)

[![Python](https://img.shields.io/badge/python-3.x-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![GitHub Commits](https://img.shields.io/github/commit-activity/m/isaque-verona/LCTSfunctions)](https://github.com/isaque-verona/LCTSfunctions/commits/main)

---

## Visão Geral

A biblioteca `LCTSfunctions` é um conjunto de módulos Python desenvolvido para facilitar a comunicação serial com transdutores de torque rotativos, especificamente o **modelo T25 da Interface Inc**. Esta biblioteca de alto nível abstrai as complexidades do protocolo de comunicação proprietário do fabricante, permitindo que pesquisadores e engenheiros interajam com o dispositivo de forma eficiente para aquisição e controle de dados[cite: 1].

---

## Contexto de Utilização no Projeto "Bancada de Emulação de Turbina Eólica Integrada em Microrrede"

No âmbito do projeto de pesquisa **"BANCADA DE EMULAÇÃO DE TURBINA EÓLICA INTEGRADA EM MICRORREDE"**, a biblioteca desempenha um papel crucial na interface com o transdutor de torque T25[cite: 1]. 

*   **Leitura em Tempo Real**: Permite a leitura precisa de torque e RPM gerados pelo sistema motor-gerador[cite: 1].
*   **Controle Contínuo**: Viabiliza a coleta de dados fundamental para o ajuste do motor emulador, garantindo a reprodução fiel de uma turbina real[cite: 1].
*   **Gêmeo Digital**: Serve como um dos pilares para a validação de algoritmos avançados de controle e gestão de energia em laboratório[cite: 1].

#### Configuração do Transdutor instalado no Emulador
*   **Dados**: 8 bits[cite: 1]
*   **Parada**: 1 bit[cite: 1]
*   **Paridade**: Sem paridade[cite: 1]
*   **Baudrate**: 230400[cite: 1]
*   **Timeout**: 3 ms[cite: 1]
*   **Silêncio entre mensagens**: 10μs (importante para o tempo de amostragem)[cite: 1]

---

## Fluxograma de Alto Nível do Processo de Comunicação

O processo de comunicação segue o fluxo abaixo:

1.  **Inicialização**: Criação de uma instância da classe `Torquimeter`.
2.  **Chamada de Método**: Invocação de um método (ex: `ReadRaw`).
3.  **Envio de Telegrama**: Conversão do comando em telegrama de bytes enviado via serial.
4.  **Aguardando Resposta**: Loop de aguardo de dados na porta serial.
5.  **Leitura e Processamento**: Limpeza de dados e verificação de integridade (*checksums*).
6.  **Extração**: Transformação dos parâmetros recebidos.
7.  **Atualização**: Atribuição dos novos valores aos atributos do objeto.
8.  **Retorno**: Disponibilização dos dados processados ou `None`.

---

## Formato do Telegrama

Os telegramas seguem o padrão de bytes definido pelo fabricante[cite: 1, 4]:

*   **`STX` (0x02)**: Início de texto (repetido duas vezes).
*   **Command byte**: Operação a ser realizada.
*   **Receiver (RX) address**: Endereço do destino.
*   **Transmitter (TX) address**: Endereço de origem.
*   **Number of parameter bytes**: Quantidade de bytes de dados.
*   **Parameters**: Dados opcionais específicos do comando.
*   **Checksum**: Soma de verificação simples.
*   **Weighted checksum**: Soma de verificação ponderada.

---

## Comandos Disponíveis

Constantes hexadecimais implementadas na biblioteca[cite: 1, 4]:

*   `STX = 0x02`
*   `SCMD_ReadRaw = 0x41` (Leitura de Torque e RPM)
*   `SCMD_ReadStatus = 0x42` / `SCMD_ReadStatusShort = 0x43`
*   `SCMD_ReadConfig = 0x44` / `SCMD_WriteConfig = 0x46`
*   `SCMD_WriteFullStroke = 0x45` (Sinal de fundo de escala)
*   `SCMD_RestartDevice = 0x4B`

---

## Instalação

Requer Python instalado e a biblioteca `pyserial`[cite: 1].

1.  **Clone o repositório:**
    ```bash
    git clone https://github.com/seu-usuario/LCTSfunctions.git
    cd LCTSfunctions
    ```
2.  **Instale as dependências:**
    ```bash
    pip install pyserial
    ```

---

## Como Usar

### Classe `Torquimeter`

Gerencia a conexão serial e as operações com o transdutor[cite: 1, 4].

#### `__init__(self, Port, Tm_max, Rpm_max, Baudrate=230400, Timeout=0.003)`
*   `Port`: Porta serial (ex: 'COM3' ou '/dev/ttyUSB0').
*   `Tm_max`: Torque máximo nominal do sensor.
*   `Rpm_max`: Rotação máxima nominal do sensor.

#### Principais Métodos[cite: 1, 4]
*   **`ReadRaw()`**: Retorna lista com `[Canal0, Canal1, Torque_Calibrado, RPM_Calibrado, FullstrokeFlag, OverloadFlag]`.
*   **`Hello()`**: Verifica conexão com o sensor.
*   **`WriteFullStroke(bool)`**: Ativa/desativa sinal de calibração.
*   **`RestartDevice()`**: Reinicia o hardware do transdutor.

### Exemplo Básico de Implementação

```python
from LCTSfunctions import Torquimeter

try:
    # Inicialização para o modelo T25 (Ex: 100 N.m, 3000 RPM)
    torquimetro = Torquimeter(Port='COM3', Tm_max=100.0, Rpm_max=3000.0)
    
    dados = torquimetro.ReadRaw()
    if dados:
        print(f"Torque: {dados[2]:.2f} N.m | RPM: {dados[3]:.2f}")
        
except Exception as e:
    print(f"Erro: {e}")
finally:
    torquimetro.serialport.close()
