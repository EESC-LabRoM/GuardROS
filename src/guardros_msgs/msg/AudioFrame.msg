# AudioFrame.msg
#
# Mensagem para transportar um quadro de áudio no GuardROS.
#
# Esta mensagem foi pensada para ser suficientemente genérica
# para transportar tanto:
# - áudio recebido do robô
# - áudio capturado do microfone local para envio ao robô
#
# O campo "data" contém os bytes brutos do áudio.
# A interpretação correta depende do campo "encoding".

# Bytes brutos do áudio.
uint8[] data

# Taxa de amostragem em Hz.
# Exemplo: 8000, 16000, 48000
uint32 sample_rate

# Número de canais.
# Exemplo:
# 1 = mono
# 2 = estéreo
uint8 channels

# Codificação usada nos bytes.
# Exemplos:
# "pcm_u8"
# "pcm_s16le"
string encoding

# Quantidade de amostras por canal presentes neste frame.
# Exemplo:
# se for mono com 1024 amostras, então samples_per_channel = 1024
uint32 samples_per_channel