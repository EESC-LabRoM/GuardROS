# AudioMode.msg
#
# Mensagem para definir o modo de operação do áudio no GuardROS.

# Modos disponíveis.
uint8 MODE_SILENCE=1
uint8 MODE_LISTEN=2
uint8 MODE_TALK=3

# Modo atualmente desejado.
uint8 mode