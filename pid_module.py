class PIDController:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.erro_int = 0.0
        self.erro_ant = 0.0

        self.out_min = out_min
        self.out_max = out_max

    def compute(self, setpoint, measured_value, dt, active=True):
        if not active:
            return 0.0 # Mantém a integral onde parou ou resete se preferir

        erro = setpoint - measured_value
        
        # --- Termo Proporcional ---
        P = self.kp * erro
        
        # --- Termo Derivativo ---
        D = self.kd * (erro - self.erro_ant) / dt
        
        # --- Cálculo da Saída Temporária (Sem a integral nova) ---
        saida_sem_I = P + (self.ki * self.erro_int) + D
        
        # --- Lógica de ANTI-WINDUP (Conditional Integration) ---
        # Só atualiza a integral se a saída total estiver dentro dos limites
        # Isso simula o seu "if Tensao_min < Ea < Tensao_max"
        if self.out_min < saida_sem_I < self.out_max:
            self.erro_int += erro * dt
            
        # --- Termo Integral Final ---
        I = self.ki * self.erro_int
        
        # Saída final saturada
        output = P + I + D
        output_clipped = max(self.out_min, min(output, self.out_max))
        
        self.erro_ant = erro
        return output_clipped

    def reset(self):
        self.erro_int = 0.0
        self.erro_ant = 0.0