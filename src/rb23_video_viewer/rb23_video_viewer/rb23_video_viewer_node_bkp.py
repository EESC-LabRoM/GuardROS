#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_video_viewer_node.py

Nó ROS 2 responsável por visualizar o vídeo comprimido (JPEG)
publicado pelo driver do RollerBot 23.

Arquitetura desejada:
    RB23 -> rb23_driver_node -> /rb23/camera/image/compressed -> rb23_video_viewer_node

Responsabilidades deste nó:
1. Assinar o tópico de imagem comprimida publicado pelo driver.
2. Receber os bytes JPEG.
3. Decodificar o JPEG usando OpenCV.
4. Exibir a imagem em uma janela na tela.

Por que separar este nó do driver?
----------------------------------
Porque no ROS é uma boa prática separar responsabilidades:
- o driver cuida da comunicação com o robô
- este nó cuida apenas da interface visual

Isso facilita manutenção, depuração e futuras expansões, como:
- gravação do vídeo
- processamento com OpenCV
- detecção de objetos
- integração com dashboards

Melhoria visual implementada:
-----------------------------
Este nó usa a técnica de "letterbox/pillarbox":
- a imagem é redimensionada preservando sua proporção original
- ela é centralizada dentro de uma área fixa
- áreas excedentes são preenchidas com preto

Isso evita:
- distorção horizontal
- distorção vertical
- janelas desajeitadas demais
"""

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


# ============================================================
# CONFIGURAÇÕES
# ============================================================

# Nome da janela OpenCV que aparecerá na tela.
WINDOW_NAME = "RB23 Camera Viewer"

# Tópico ROS onde o driver publicará o vídeo JPEG.
DEFAULT_IMAGE_TOPIC = "/rb23/camera/image/compressed"

# Período do timer de exibição.
# 0.03 s corresponde aproximadamente a 33 Hz de atualização visual.
DISPLAY_PERIOD = 0.03

# Tamanho fixo da área de exibição.
# Escolhemos 720x405 por ser uma dimensão confortável e compatível
# com proporção 16:9, muito apropriada para vídeo.
WINDOW_WIDTH = 720
WINDOW_HEIGHT = 405


# ============================================================
# FUNÇÕES AUXILIARES DE IMAGEM
# ============================================================

def letterbox_frame(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    """
    Redimensiona a imagem preservando a proporção e a coloca centralizada
    dentro de uma área fixa (canvas) com fundo preto.

    Isso evita distorção e mantém a apresentação visual organizada.

    Estratégia:
    1. Mede o tamanho original da imagem.
    2. Calcula a escala máxima que permite a imagem caber na área desejada.
    3. Redimensiona a imagem preservando a proporção.
    4. Cria um canvas preto de tamanho fixo.
    5. Centraliza a imagem dentro desse canvas.

    O resultado é semelhante ao comportamento de players de vídeo:
    - se faltar largura, aparecem barras laterais
    - se faltar altura, aparecem barras superior/inferior

    Parâmetros:
    -----------
    frame:
        Imagem original OpenCV (BGR).
    target_width:
        Largura da área final desejada.
    target_height:
        Altura da área final desejada.

    Retorno:
    --------
    canvas:
        Imagem final já pronta para exibição.
    """
    if frame is None:
        return np.zeros((target_height, target_width, 3), dtype=np.uint8)

    original_height, original_width = frame.shape[:2]

    # Proteção simples contra casos anômalos.
    if original_width <= 0 or original_height <= 0:
        return np.zeros((target_height, target_width, 3), dtype=np.uint8)

    # Escala máxima que permite à imagem caber dentro da área fixa.
    scale = min(target_width / original_width, target_height / original_height)

    # Novo tamanho preservando proporção.
    new_width = max(1, int(original_width * scale))
    new_height = max(1, int(original_height * scale))

    # Redimensiona usando INTER_AREA, geralmente bom para redução.
    resized = cv2.resize(
        frame,
        (new_width, new_height),
        interpolation=cv2.INTER_AREA,
    )

    # Canvas preto fixo onde a imagem será centralizada.
    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)

    # Calcula deslocamentos para centralização.
    x_offset = (target_width - new_width) // 2
    y_offset = (target_height - new_height) // 2

    # Copia a imagem redimensionada para o centro do canvas.
    canvas[
        y_offset:y_offset + new_height,
        x_offset:x_offset + new_width
    ] = resized

    return canvas


# ============================================================
# CLASSE PRINCIPAL
# ============================================================

class RB23VideoViewerNode(Node):
    """
    Nó ROS 2 para exibir o vídeo do RB23.

    Este nó:
    - assina um tópico do tipo sensor_msgs/CompressedImage
    - armazena o frame mais recente recebido
    - mostra esse frame em uma janela OpenCV

    Observação:
    Preferimos exibir a imagem em um timer, e não diretamente dentro
    do callback ROS, porque isso tende a deixar o comportamento mais
    estável e organizado.
    """

    def __init__(self) -> None:
        super().__init__("rb23_video_viewer_node")

        # --------------------------------------------------------
        # PARÂMETRO ROS
        # --------------------------------------------------------
        # Permite trocar o tópico de vídeo futuramente sem editar o código.
        self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC)
        self.image_topic = str(self.get_parameter("image_topic").value)

        # --------------------------------------------------------
        # ESTADO INTERNO
        # --------------------------------------------------------
        # Usamos um lock porque o callback do subscriber e o timer
        # podem acessar os mesmos dados em instantes diferentes.
        self.frame_lock = threading.Lock()

        # Frame mais recente decodificado.
        self.latest_frame: Optional[np.ndarray] = None

        # Contadores úteis para depuração.
        self.frames_received = 0
        self.frames_decoded = 0
        self.decode_failures = 0

        # --------------------------------------------------------
        # SUBSCRIBER
        # --------------------------------------------------------
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10,
        )

        # --------------------------------------------------------
        # TIMER DE EXIBIÇÃO
        # --------------------------------------------------------
        self.display_timer = self.create_timer(
            DISPLAY_PERIOD,
            self.display_timer_callback,
        )

        # --------------------------------------------------------
        # JANELA OPENCV
        # --------------------------------------------------------
        # Criamos a janela uma única vez e já definimos um tamanho
        # inicial mais agradável para a tela.
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

        self.get_logger().info(
            f"RB23 video viewer iniciado | tópico={self.image_topic} | "
            f"janela={WINDOW_WIDTH}x{WINDOW_HEIGHT}"
        )

    # =========================================================
    # CALLBACK DO TÓPICO DE IMAGEM
    # =========================================================

    def image_callback(self, msg: CompressedImage) -> None:
        """
        Callback executado sempre que chega uma nova mensagem JPEG.

        Etapas:
        1. Ler os bytes do campo msg.data
        2. Converter para vetor NumPy
        3. Decodificar o JPEG para imagem BGR do OpenCV
        4. Armazenar o frame mais recente
        """
        self.frames_received += 1

        try:
            # Converte os bytes JPEG para um vetor NumPy de uint8.
            np_buffer = np.frombuffer(msg.data, dtype=np.uint8)

            # Decodifica o JPEG em imagem colorida BGR.
            frame = cv2.imdecode(np_buffer, cv2.IMREAD_COLOR)

            # Se o OpenCV não conseguiu decodificar, contamos a falha e saímos.
            if frame is None:
                self.decode_failures += 1
                return

            # Armazena o frame mais recente de forma protegida.
            with self.frame_lock:
                self.latest_frame = frame

            self.frames_decoded += 1

        except Exception as exc:
            self.decode_failures += 1
            self.get_logger().warn(f"Falha ao decodificar frame JPEG: {exc}")

    # =========================================================
    # TIMER DE EXIBIÇÃO
    # =========================================================

    def display_timer_callback(self) -> None:
        """
        Timer responsável por exibir o frame mais recente na janela.

        Esta separação é útil porque:
        - o callback ROS fica responsável por receber dados
        - o timer fica responsável por atualizar a interface visual
        """
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        # Se ainda não chegou nenhum frame, apenas mantemos a janela viva.
        if frame is None:
            cv2.waitKey(1)
            return

        # Aplica a apresentação elegante:
        # a imagem é encaixada num canvas fixo sem distorção.
        display_frame = letterbox_frame(
            frame,
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
        )

        # Exibe o frame final.
        cv2.imshow(WINDOW_NAME, display_frame)

        # Mantém a janela responsiva.
        key = cv2.waitKey(1) & 0xFF

        # Opcional: se o usuário apertar q ou ESC, fechamos o ROS.
        if key in (27, ord("q"), ord("Q")):
            self.get_logger().info("Encerrando viewer por comando do teclado.")
            rclpy.shutdown()

    # =========================================================
    # ENCERRAMENTO
    # =========================================================

    def close(self) -> None:
        """
        Fecha a janela OpenCV de forma organizada.
        """
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


# ============================================================
# MAIN
# ============================================================

def main(args=None) -> None:
    """
    Ponto de entrada do nó ROS 2.
    """
    rclpy.init(args=args)
    node = RB23VideoViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()