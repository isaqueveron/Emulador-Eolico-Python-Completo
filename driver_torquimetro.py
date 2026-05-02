"""
=======================================
Rotary Torque Transducer Communication Library (:mod:`LCTSfunctions`)
=======================================
__author      : Isaque Verona
__v0_date     : 11/07/2025
__last_update : 12/04/2026
__version     : v.2.0

High-Level Flowchart (:mod:`Torquimeter Communication Process`):
--------------------------------------------------------

    1.  A --> B{Initialize Torquimeter};
    2.  B --> C{Call a Torquimeter method (e.g., ReadRaw)};
    3.  C --> D[Send Telegram (Methods.SendTelegram)];
    4.  D --> E{Set isReceiving to True};
    5.  E --> F{Loop while isReceiving};
    6.  F --> G{Read from Serial Port (Methods.ReadFrom)};
    7.  G --> H{Is code_received None?};
    8.  H -- Yes --> I{Set isReceiving to False};
    9.  I --> J{Return None};
    10. H -- No --> K{Try to Process Received Telegram (Methods.ReceiveTg)};
    11. K -- Success --> L{Extract and Process Data};
    12. L --> M{Update Torquimeter attributes};
    13. M --> N{Set isReceiving to False};
    14. N --> J;
    15. K -- Failure --> O{Set isReceiving to False};
    16. O --> J;
    17. J --> P[End];

Telegram format:
---------------
    ♦ STX 
    ♦ STX 
    ♦ Command byte
    ♦ Receiver (RX) address
    ♦ Transmitter (TX) address
    ♦ Number of parameter bytes
    ♦ Parameters (optional)
    ♦ Checksum
    ♦ Weighted checksum
"""

import serial
from math import pi

# BytearrayCommands Bytes
STX =                           0x02
SCMD_ACK =                      0x06
SCMD_NACK =                     0x15
SCMD_Hello =                    0x40
SCMD_ReadRaw =                  0x41
SCMD_ReadStatus =               0x42
SCMD_ReadStatusShort =          0x43
SCMD_ReadConfig =               0x44
SCMD_WriteFullStroke =          0x45
SCMD_WriteConfig =              0x46
SCMD_RestartDevice  =           0x4B
SCMD_GotoSpecialMode =          0x5a

class Torquimeter:

    def __init__(self ,Port:str, Tm_max = 100, Rpm_max = 30000, 
                 Baudrate = 230400, Timeout = 0.003, byte_resolution = 25000):
        
        self.serialport = serial.Serial(port=Port,baudrate=Baudrate,timeout=Timeout) #inicializes the serial port
        self.serialport.read_all() #read trash from the buffer
        self.isReceiving = False
        self.Overloadflag = False
        self.Tm_max = Tm_max # device max torque
        self.Rpm_max = Rpm_max # device max rpm
        self.byte_resolution = byte_resolution # max value in bytes
        #last read values
        self.data = []
        self.MesurementChannel_0 = 0.0
        self.MesurementChannel_1 = 0.0
        self.Torque_calibrated   = 0.0
        self.RPM_calibrated      = 0.0
        self.Potencia_calculated = 0.0
        self.FullstrokeFlag      = 0.0

    def ReadRaw(self, tries = 1) -> list|None:

        """
        Sensors send the latest calibrated and uncalibrated measurement values. 
        Used for calibrating the sensor or measuring in normal mode.
        With sensors or interfaces using one channel only both channels have the same value.

        Args:
            (None)
        Returns:
            (list): MesurementChannel_0,MesurementChannel_1,
                    Torque_calibrated,RPM_calibrated,
                    FullstrokeFlag, Overloadflag]
        """

        Methods.SendTelegram(self.serialport,BytearrayCommands.ReadRaw()) #sends the command "ReadRaw"
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:
                code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            #print(code_received)
            if code_received != None:
                try:
                    Data = Methods.TransformData(Methods.GetRaw(Methods.ReceiveTg(code_received)))
                    self.Overloadflag = Data[1]
                    data = Data[0]
                except:data = None
                if data != None: 
                    self.data = data
                    self.MesurementChannel_0 = data[0]
                    self.MesurementChannel_1 = data[1]
                    self.Torque_calibrated   = -data[2]*self.Tm_max/self.byte_resolution -2.12#TORQUE
                    self.RPM_calibrated      = data[3]*self.Rpm_max/self.byte_resolution #RPM
                    self.FullstrokeFlag      = data[4]
                    self.Potencia_calculated = self.Torque_calibrated * self.RPM_calibrated * (2 * pi / 60)
                else: data = None
            else: data = None
        self.isReceiving = False 
        return (self.MesurementChannel_0,self.MesurementChannel_1,
                    self.Torque_calibrated,self.RPM_calibrated,
                    self.FullstrokeFlag, self.Overloadflag)
    
    def Hello(self, tries = 1) -> list|None:

        """
        Sensors can be configured to send this message after power up.
        This only makes sense in point to point applications because 
        there is no protection against collisions with the RS485.
        The main purpose if this message is to help you to debug your side of the system. 
        
        Args:
            (None)
        Returns:
            (list)    
        """

        Methods.SendTelegram(self.serialport,BytearrayCommands.Hello())
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveTg(code_received)
                    #Methods.TranslateData(data)
                except:
                    data = None
            else: data = None
        self.isReceiving = False 
        return data

    def ReadStatus(self, tries = 1) -> list|None: 
      
        """
        Send a detailed status report. 
        It includes some internal information about the healthiness of the sensor.
        The number of parameters can change with various devices.   
            
        Args:
            (None)
        Returns:
            (list)    
        """

        Methods.SendTelegram(self.serialport,BytearrayCommands.ReadStatus())
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveTg(code_received)
                    #Methods.TranslateData(data)
                except:
                    data = None
            else: data = None
        self.isReceiving = False 
        return data
    
    def ReadStatusShort(self, tries = 1) -> list|None:
        
        """
        Send a short version of the status report. 
            
        Args:
            (None)

        Returns:
            (list)
        """

        Methods.SendTelegram(self.serialport,BytearrayCommands.ReadStatusShort())
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveTg(code_received)
                    #Methods.TranslateData(data)
                except:
                    data = None
            else: data = None
        self.isReceiving = False 
        return data
        
    def ReadConfig(self, parameter: int, tries = 1) -> list|None: # parameter: Block number
        
        """
        Reads a configuration block. 
        There are several blocks containing different data.
            
        Args:
            parameter (int): Block number.

        Returns:
            (list)
        """
        
        Methods.SendTelegram(self.serialport,BytearrayCommands.ReadConfig(parameter))
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveTg(code_received)
                    #Methods.TranslateData(data)
                except:
                    data = None
            else: data = None
        self.isReceiving = False 
        return data
        
    def WriteConfig(self, parameter: list[int], tries = 1)->list|None: #parameter: Block number + 32 bytes
        
        """
        Writes a configuration block. 
        Blocks 0,1,128,129 can only be written once in the production of the sensor
            
        Args:
            parameters (list[int]): Block number + 32 bytes.

        Returns:
            (list)
        """

        Methods.SendTelegram(self.serialport,BytearrayCommands.WriteConfig(parameter))
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveTg(code_received)
                    #Methods.TranslateData(data)
                except:
                    data = None
            else: data = None
        self.isReceiving = False 
        return data
        
    def WriteFullStroke(self, parameter: bool, tries = 1)->list|None: #parameter: on/off
        
        """
        Sets the sensor into the check mode where it sends a 100% signal.
        Note: There must not be any torque applied to the sensor, 
        it would add false information to the 100% signal. 
            
        Args:
            (bool): On or Off.
        Returns:
            (list)
        """

        Methods.SendTelegram(self.serialport,BytearrayCommands.WriteFullStroke(parameter))
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveTg(code_received)
                    #Methods.TranslateData(data)
                except:
                    data = None
            else: data = None
        self.isReceiving = False 
        return data
        
    def RestartDevice(self, tries = 1) -> list|None:

        """
        Resets the device. It responses with a 'hello' 
        even if it is permitted in configuration block or not
            
        Args:
            (None)
        Returns:
            (list)
        """

        Methods.SendTelegram(self.serialport,BytearrayCommands.RestartDevice())
        self.isReceiving = True
        data = None
        for attempt in range(tries): #loop for receiving
            try:code_received=Methods.ReadFrom(self.serialport)
            except: code_received = None
            if code_received != None:
                try:
                    data = Methods.ReceiveTg(code_received)
                    #Methods.TranslateData(data)
                except:
                    data = None
            else: data = None
        self.isReceiving = False 
        return data
    

#metodos para manipulacao do telegram
class Methods:

    def SendTelegram(SerialPort: object, tg: bytearray) -> None:
        
        """
        Write the telegram to the serialport
        """

        SerialPort.write(bytes(tg))
    
    def ReadFrom(SerialPort: object) -> bytearray|None:
        data = SerialPort.readline()
        if data != b'': 
            return data
        else: return None
    
    def CleanTg(tg: bytearray) -> bytearray:
        
        """
        Clean the special caracters from the end (line-feed,carriage return) 
        and STXs from the start
        """

        global STX
        if tg[-2:]==b'\r\n': # "trim"
            tg = tg[:-2]
        if tg[0]==STX:
            tg = tg[1:]
        if tg[0]==STX:
            tg = tg[1:]              
        return tg

    def ToHex(parameters: list[int]) -> list[str]: # func to trasnform data to hex values
        
        """
        Transform a list off int to hex. Used for concatenate
        two bytes an form the read value.

        Args:
            parameters(list[int])
        Returns:
            (list[str])
        """

        i=0
        while i < len(parameters):
            parameters[i] = hex(parameters[i])
            i+=1
        return parameters
    
    def CalcChecksums(tg: list[int]) -> list[int]:
        checksum = 0
        wchecksum = 0
        # O manual indica somar todos os bytes APÓS os STXs iniciais 
        for itm in tg:
            checksum = (checksum + itm) & 0xFF
            wchecksum += checksum
            if wchecksum > 0xFF: # Carry end-around [cite: 116, 120]
                wchecksum += 1
            wchecksum &= 0xFF
        return [checksum, wchecksum]

    def CheckChecksums(tg: bytearray) -> bool:
        # 1. Deve-se primeiro remover o Byte Stuffing (0x02 0x02 -> 0x02)
        # 2. O Checksum é validado sobre o telegrama "limpo" (sem STXs de início)
        # tg aqui já deve estar sem os STX iniciais e sem o stuffing
        payload = list(tg[:-2])
        received_cs = list(tg[-2:])
        return Methods.CalcChecksums(payload) == received_cs
        
    def TransformData(RawData:list) -> list[list,bool]: #func exclusive for the command ReadRaw
            
        """
        Does the transformations of the 
        measurementes sent by the sensor to 
        its respectives positive or negative numbers
        
        Args: 
            RawData(list): Iterable.

        Returns: 
            (list[list,bool]): Useful Data and overload flag
        """   

        overload = False
        i = 0
        while i < len(RawData):
            if RawData[i] >= 0 and RawData[i] <= 32767 : # is > 0 positive
                None
            elif RawData[i] >= 0x7fff and RawData[i] <= 0xffff: # is < 0 negative
                RawData[i]=RawData[i]-65536
            if i>1 and abs(RawData[i]) >= 25000: #only check for overload in the calibrated channels
                overload = True
                RawData[i] = 25000 #overload  
            i+=1
            UsefulData = RawData

        return [UsefulData,overload]

    def GetRaw(command_para:list[int,list[int]]) -> list[int]:
        """
        Func that need to be called to extract the data from the ReceiveTg.
        Exclusive for the command ReadRaw. A saida dessa funcao ainda nao e a medida real
        pois precisa ser convertido entre numeros positivos e negativos.
        
        Args:
            (list[int,list[int]]): List with the received command and parameters list
        Returns:
            RawData(list[int]): List with the byte mesurements in int decimal (0-65536)
            [MesurementChannel_0,MesurementChannel_1,
            CalibratedValCha_0,CalibratedValCha_1,
            FullstrokeFlag]
        """

        global SCMD_ReadRaw
        if command_para == None: return None
        command = command_para[0]
        
        # Transforma em hex e garante 2 digitos para cada byte
        params = [hex(p)[2:].zfill(2) for p in command_para[1]]
        
        if command == SCMD_ReadRaw:
            # Concatena os pares de bytes corretamente
            MesurementChannel_0 = int(params[0] + params[1], 16)
            MesurementChannel_1 = int(params[2] + params[3], 16)
            CalibratedValCha_0  = int(params[4] + params[5], 16)
            CalibratedValCha_1  = int(params[6] + params[7], 16)
            FullstrokeFlag      = int(params[8], 16)
            
            RawData = [MesurementChannel_0, MesurementChannel_1,
                    CalibratedValCha_0, CalibratedValCha_1,
                    FullstrokeFlag]
            return RawData
        return None
    
    def Unstuff(tg: bytearray) -> list:
        """
        Removes the byte stuffing from the received telegram. 
        According to the protocol, if 0x02 appears in the data, 
        it is sent as 0x02 0x02. This function reverts it to a single 0x02.
        
        Args:
            tg (bytearray): The raw bytearray received from the serial port.
        Returns:
            unstuffed (list): List of integers with single 0x02 values.
        """
        raw = list(tg)
        unstuffed = []
        skip = False
        for i in range(len(raw)):
            if skip:
                skip = False
                continue
            unstuffed.append(raw[i])
            # Se encontrar 02 02, pula o próximo
            if raw[i] == 0x02 and i+1 < len(raw) and raw[i+1] == 0x02:
                skip = True
        return unstuffed

    def ReceiveTg(code_received: bytearray) -> list:
        """
        Processes the received bytearray, handles byte stuffing, 
        and validates checksums before extracting the command and parameters.

        Args:
            code_received (bytearray): Bytearray read from the serial port.
        Returns:
            (list[int, list[int]]): Command and unstuffed parameters list.
        """
        if not code_received: return None
        
        # 1. Localiza o início real (STX)
        try:
            start_idx = code_received.index(0x02)
            # Pula os STXs iniciais (podem ser um ou dois)
            while start_idx < len(code_received) and code_received[start_idx] == 0x02:
                start_idx += 1
            # O telegrama útil começa após os STXs
            tg_to_process = code_received[start_idx:]
        except ValueError:
            return None

        # 2. Faz o Unstuffing (importante para o Checksum bater!)
        clean_data = Methods.Unstuff(tg_to_process)
        
        # 3. Valida Checksum sobre os dados desdobrados
        # Os últimos dois bytes são sempre os checksums
        payload = clean_data[:-2]
        received_cs = clean_data[-2:]
        
        if Methods.CalcChecksums(payload) == received_cs:
            command = clean_data[0]
            num_params = clean_data[3]
            parameters = clean_data[4:4+num_params]
            return [command, parameters]
        else:
            #print(f"BAD CHECK SUMS: {clean_data}")
            return None

class BytearrayCommands:
    
    def Hello() -> bytearray:
        
        """
        Sensors can be configured to send this message after power up.
        This only makes sense in point to point applications because 
        there is no protection against collisions with the RS485.
        The main purpose if this message is to help you to debug your side of the system. 
            
        Args:
            (None)

        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_Hello
        rx      =   0x00 #receiver
        tx      =   0x01 #transmiter
        command =   SCMD_Hello
        
        # create the telegram to send
        telegram = [STX,STX,command,rx,tx,0]  #(stx,stx,command,rx,tx,number_of_parameters)
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram

    def ReadRaw() -> bytearray:
        
        """
        Sensors send the latest calibrated and uncalibrated measurement values. 
        Used for calibrating the sensor or measuring in normal mode.
        With sensors or interfaces using one channel only both channels have the same value.
            
        Args:
            (None)

        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_ReadRaw
        rx      =   0x01 #receiver
        tx      =   0xff #transmiter
        command =   SCMD_ReadRaw
        
        # create the telegram to send
        telegram = [STX,STX,command,rx,tx,0]  #(stx,stx,command,rx,tx,number_of_parameters)
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram

    def ReadStatus() -> bytearray:
        
        """
        Send a detailed status report. 
        It includes some internal information about the healthiness of the sensor.
        The number of parameters can change with various devices.   
            
        Args:
            (None)

        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_ReadStatus
        rx      =   0x01 #receiver
        tx      =   0xff #transmiter
        command =   SCMD_ReadStatus
        
        # create the telegram to send
        telegram =[STX,STX,command,rx,tx,0]  #(stx,stx,command,rx,tx,number_of_parameters)
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram

    def ReadStatusShort() -> bytearray:
        
        """
        Send a short version of the status report. 
            
        Args:
            (None)

        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_ReadStatusShort
        rx      =   0x01 #receiver
        tx      =   0xff #transmiter
        command =   SCMD_ReadStatusShort
        
        # create the telegram to send
        telegram = [STX,STX,command,rx,tx,0]  #(stx,stx,command,rx,tx,number_of_parameters)
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram

    def ReadConfig(PARAMETER: int) -> bytearray: #parameter: Block number
        
        """
        Reads a configuration block. 
        There are several blocks containing different data.
            
        Args:
            (int): Block number.

        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_ReadConfig
        rx      =   0x01 #receiver
        tx      =   0xff #transmiter
        command =   SCMD_ReadConfig
        
        # create the telegram to send
        telegram = [STX,STX,command,rx,tx,0x01,PARAMETER]  #(stx,stx,command,rx,tx,number_of_parameters, parameter)   
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram

    def WriteConfig(PARAMETERS: list[int]=[]) -> bytearray: #parameter: Block number + 32 bytes
        
        """
        Writes a configuration block. 
        Blocks 0,1,128,129 can only be written once in the production of the sensor
            
        Args:
            parameters (list[int]): Block number + 32 bytes.

        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_WriteConfig
        rx      =   0x01 #receiver
        tx      =   0xff #transmiter
        command =   SCMD_WriteConfig
        nbr_par =   len(PARAMETERS)
        
        # create the telegram to send
        telegram = [STX,STX,command,rx,tx,nbr_par]  #(stx,stx,command,rx,tx,number_of_parameters)
        for i in PARAMETERS:
            telegram.append(i)
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram

    def WriteFullStroke(PARAMETER: bool) -> bytearray: #parameter: on/off
        
        """
        Sets the sensor into the check mode where it sends a 100% signal.
        Note: There must not be any torque applied to the sensor, 
        it would add false information to the 100% signal. 
            
        Args:
            (bool): On or Off.
        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_WriteFullStroke 
        rx      =   0x01 #receiver
        tx      =   0xff #transmiter
        command =   SCMD_WriteFullStroke

        # create the telegram to send
        telegram = [STX,STX,command,rx,tx,0x01,int(PARAMETER)]  #(stx,stx,command,rx,tx,number_of_parameters,parameter)
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram

    def RestartDevice() -> bytearray:
        
        """
        Resets the device. It responses with a 'hello' 
        even if it is permitted in configuration block or not
            
        Args:
            (None)
        Returns:
            (bytearray): The bytearray to be sent.
        """

        global STX, SCMD_RestartDevice 
        rx      =   0x01 #receiver
        tx      =   0xFF #transmiter
        command =   SCMD_RestartDevice 
        
        # create the telegram to send
        telegram = [STX,STX,command,rx,tx,0] #(stx,stx,command,rx,tx,number_of_parameters)
        checksums = Methods.CalcChecksums(telegram[2:]) # calls function to calculate the check sums excluding stx
        telegram = list(telegram)
        telegram = bytearray(telegram+checksums) #transform the list of ints in a byte array for sending
        return telegram #return the telegram