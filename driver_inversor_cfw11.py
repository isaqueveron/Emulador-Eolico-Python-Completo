"""
=======================================
Inverter Communication Library (:mod:`LCINVfunctions`)
=======================================
__author      : Isaque Verona
__v0_date     : 12/04/2026
__last_update : 12/04/2026
__version     : v.1.0

High-Level Flowchart (:mod:`Inverter Communication Process`):
--------------------------------------------------------

    1.  A --> B{Initialize Inverter};
    2.  B --> C{Call an Inverter method (e.g., ReadParameter)};
    3.  C --> D[Build Telegram (BytearrayCommands)];
    4.  D --> E[Send Telegram (Methods.SendTelegram)];
    5.  E --> F{Set isReceiving to True};
    6.  F --> G{Loop for tries};
    7.  G --> H{Read from Serial Port (Methods.ReadFrom)};
    8.  H -- Data Received --> I{Check Integrity (Methods.CheckBCC)};
    9.  I -- Success --> J{Process Response (Methods.ReceiveReadResponse)};
    10. J --> K{Update Inverter attributes (last_value)};
    11. K --> L{Set isReceiving to False};
    12. L --> M[Return Value];
    13. I -- Failure --> N{Set isReceiving to False};
    14. N --> O[Return None/False];
    15. H -- No Data --> N;
    16. M --> P[End];

Telegram format:
---------------
    ♦ STX (0x02)
    ♦ ADR (Inverter Address + 64)
    ♦ COD (Read -> 3Ch | Write -> 3Dh)
    ♦ NUM (Number of parameters - fixed at 0x01)
    ♦ DMR or DMW (Data parameter or value)
    ♦ ETX (0x03)
    ♦ BCC (XOR Checksum)
"""

import serial

# BytearrayCommands Bytes
STX         = 0x02
ETX         = 0x03
COD_READ    = 0x3C
COD_WRITE   = 0x3D

class Inverter:

    def __init__(self, Port: str, ADR=1, Baudrate=57600, Timeout=0.0):
        """
        Inicializa o objeto do inversor

        Args:
            Port: porta serial conectada ao inversor
            ADR: endereco do inversor na rede (valores de 1 a 30)
            Baudrate: velocidade de transmissao de dados
            Timeout: tempo maximo que um sistema espera por dados
        """
        # ADR = P0308 + 64 (Ex: ADR 1 = 65 ou 'A') 
        self.adr_byte = (ADR + 64) & 0xFF 
        self.serialport = serial.Serial(port=Port, baudrate=Baudrate, timeout=Timeout)
        self.serialport.read_all()
        self.isReceiving = False
        # last read values
        self.last_value = None
        self.last_changed_parameter = None
        self.last_changed_parameter_value = None

    def ReadParameter(self, parameter: int, tries = 1) -> int|None:

        """
        Reads an inverter parameter.
        
        Args:
            parameter (int): Numero do parametro (ex: 2 para P0002).
        Returns:
            (int): Valor do parametro lido ou None.
        """
        telegram = BytearrayCommands.ReadParam(self.adr_byte, parameter)
        #print("Telegrama enviado: ", telegram)
        Methods.SendTelegram(self.serialport, telegram)
        self.isReceiving = True
        data = None
        for attempt in range(tries):
            try: 
                code_received = Methods.ReadFrom(self.serialport)
                #print("Telegrama recebido: ", code_received)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveReadResponse(code_received)
                except:data = None
            else: data = None
        self.isReceiving = False
        return data

    def WriteParameter(self, parameter: int, value: int, tries = 1) -> bool:
        """
        Escreve um valor em um parametro.
        Args:
            parameter (int): Numero do parametro (ex: 2 para P0002).
            value (int): valor a ser escrito.
        Returns:
            (bool): True or False.
        """
        telegram = BytearrayCommands.WriteParam(self.adr_byte, parameter, value)
        #print("Telegrama enviado: ", telegram)
        Methods.SendTelegram(self.serialport, telegram)
        self.isReceiving = True
        success = False
        for attempt in range(tries):
            try:
                code_received = Methods.ReadFrom(self.serialport)
                #print("Telegrama recebido: ", code_received)
            except:code_received = None
            if code_received != None:
                if 0x06 in code_received:
                    success = True
                    self.last_changed_parameter = parameter
                    self.last_changed_parameter_value = value
            else: success = False
            self.isReceiving = False
        return success

    def SendReferenceAngularVelocity(self, referencia_rpm):
        """
        Envia a referencia de velocidade angular que o controle em malha 
        aberta interno do inversor tentara atingir via curva V/f.
        Args:
            referencia_rpm (int): Referencia em RPM.
        Returns:
            (bool): True or False.
        """
        bit = int((referencia_rpm*8192)/1800)
        return Inverter.WriteParameter(self,683,bit)
    
    def ActivateMotor(self):
        return Inverter.WriteParameter(self,682,23)
    
    def StopMotor(self):
        return Inverter.WriteParameter(self,682,22)
        

class Methods:

    def SendTelegram(SerialPort: object, tg: bytearray) -> None:
        SerialPort.write(bytes(tg))
    
    def ReadFrom(SerialPort: object) -> bytearray|None:
        
        """
        Read a telegram from the serialport
        """
        
        data = SerialPort.readline()
        if data != b'': 
            return data
        else: return None

    def CalcBCC(tg: list[int]) -> int:
        """
        Calcula o Checksum longitudinal (BCC) usando OU EXCLUSIVO (XOR).
        """
        bcc = 0
        for byte in tg:
            bcc ^= byte
        return bcc

    def CheckBCC(tg: bytearray) -> bool:
        content = list(tg[:-1])
        calculated = Methods.CalcBCC(content)
        return calculated == tg[-1]
        #return True

    def ReceiveReadResponse(code_received: bytearray) -> int | None:
        """
        Processa a resposta de LEITURA (Exemplo 1 da imagem).
        Estrutura esperada: [ADR] [VALOR_HI] [VALOR_LO] [BCC]
        """
        
        if not(Methods.CheckBCC(code_received)):
            print("Erro de Checksum (BCC)")
            return None
        
        hi_byte = code_received[1]
        lo_byte = code_received[2]
        # 1. hex(hi) -> '0x4' -> zfill garante '0x04'
        # 2. [2:] remove o '0x'
        hi_str = hex(hi_byte)[2:].zfill(2)
        lo_str = hex(lo_byte)[2:].zfill(2)
        
        combined_value = int(hi_str + lo_str, 16)
        return combined_value


class BytearrayCommands:
    
    def ReadParam(adr: int, param: int) -> bytearray:
        """
        Monta telegrama de leitura: STX + ADR + COD + NUM + DMR + ETX + BCC
        """
        # decomposicao de um valor inteiro de 16 bits (param) 
        # em dois bytes individuais de 8 bits (phi, plo).
        phi = (param >> 8) & 0xFF
        plo = param & 0xFF
        # COD '<' (3Ch), ler parametro 
        body = [STX, adr, COD_READ, 0x01, phi, plo, ETX]
        bcc = Methods.CalcBCC(body)
        telegram = bytearray(body + [bcc])
        return telegram

    def WriteParam(adr: int, param: int, value: int) -> bytearray:
        """
        Monta telegrama de escrita: STX + ADR + COD + NUM + DMW + ETX + BCC
        """
        # decomposicao de dois valores inteiros de 16 bits (param e value) 
        # em quatro bytes individuais de 8 bits (phi, plo, vhi, vlo).
        phi = (param >> 8) & 0xFF
        plo = param & 0xFF
        vhi = (value >> 8) & 0xFF
        vlo = value & 0xFF
        # COD '=' (3Dh), escrever parametro sem salvar na EEPROM
        body = [STX, adr, COD_WRITE, 0x01, phi, plo, vhi, vlo, ETX]
        bcc = Methods.CalcBCC(body)
        telegram = bytearray(body + [bcc])
        return telegram