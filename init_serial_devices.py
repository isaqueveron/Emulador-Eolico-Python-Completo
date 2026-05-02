import serial
import serial.tools.list_ports

# --- Inicialização da Porta Serial ---
def selecionar_porta(mensagem):
    """Lista as portas e pede ao usuário para selecionar uma."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("Nenhuma porta serial detectada.")
        return None
    print(f"\n--- {mensagem} ---")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    while True:
        try:
            escolha = int(input("Digite o número da porta desejada: "))
            if 0 <= escolha < len(ports):
                return ports[escolha].device
            else:
                print("Número inválido.")
        except ValueError:
            print("Entrada inválida. Digite um número.")

