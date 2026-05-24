#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
keyboard_teleop_node.py

Nó ROS 2 para teleoperação por teclado do GuardROS / RB23.

Objetivo:
---------
Ler o teclado em tempo real e publicar comandos ROS 2 que serão
consumidos por outros nós do sistema, principalmente:
- rb23_driver_node
- rb23_audio_node

Arquitetura atual:
------------------
teclado -> tópicos ROS -> driver / áudio -> RB23

Tópicos publicados:
-------------------
- /rb23/cmd_vel      -> geometry_msgs/msg/Twist
- /rb23/force_rpi    -> std_msgs/msg/Bool
- /rb23/cam_stable   -> std_msgs/msg/Bool
- /rb23/cam_angle    -> std_msgs/msg/Float32
- /rb23/audio_mode   -> guardros_msgs/msg/AudioMode

Características:
----------------
1. Aceita teclas WASD e também as setas direcionais para movimento.
2. Publica Twist em /rb23/cmd_vel.
3. Permite controlar funções adicionais do robô:
   - force_rpi
   - estabilização da câmera
   - ângulo da câmera
4. Permite selecionar o modo de áudio:
   - silêncio bilateral
   - ouvir o robô
   - falar com o robô
5. Exibe ao usuário, no terminal, as funções de cada tecla.
6. Funciona no estilo "carro de videogame":
   - enquanto a tecla é detectada, o comando continua ativo
   - ao soltar a tecla, o nó volta a publicar velocidade zero
7. Usa curses para leitura de teclado em tempo real no terminal Linux.
8. Foi escrito de forma didática e muito comentada para facilitar estudo
   e manutenção futura.

Observação importante:
----------------------
Este nó não conversa diretamente com o robô.
Ele publica apenas em tópicos ROS.
Quem conversa com o RB23 é o rb23_driver_node.
"""

import curses
import time

import rclpy
from geometry_msgs.msg import Twist
from guardros_msgs.msg import AudioMode
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class KeyboardTeleopNode(Node):
    """
    Nó ROS 2 responsável por:
    - capturar teclas do terminal
    - transformar essas teclas em comandos ROS
    - publicar velocidade em /rb23/cmd_vel
    - publicar comandos auxiliares do RB23 em tópicos próprios
    - publicar o modo de áudio em /rb23/audio_mode
    """

    def __init__(self) -> None:
        super().__init__("rb23_keyboard_teleop_node")

        # =========================================================
        # PARÂMETROS ROS
        # =========================================================
        # Estes parâmetros permitem ajustar o comportamento do teclado
        # sem alterar o código-fonte.
        self.declare_parameter("linear_speed", 0.30)
        self.declare_parameter("angular_speed", 1.00)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("key_hold_timeout", 0.18)

        # Parâmetros de câmera.
        self.declare_parameter("cam_angle_step", 0.10)
        self.declare_parameter("cam_angle_min", -1.50)
        self.declare_parameter("cam_angle_max", 1.50)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.key_hold_timeout = float(self.get_parameter("key_hold_timeout").value)

        self.cam_angle_step = float(self.get_parameter("cam_angle_step").value)
        self.cam_angle_min = float(self.get_parameter("cam_angle_min").value)
        self.cam_angle_max = float(self.get_parameter("cam_angle_max").value)

        # =========================================================
        # PUBLISHERS ROS
        # =========================================================
        # Publicador de velocidade padrão ROS para robôs móveis.
        # Convenção adotada:
        #   linear.x  -> velocidade linear (frente / ré)
        #   angular.z -> velocidade angular (giro)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/rb23/cmd_vel",
            10
        )

        # Publicadores de comandos auxiliares do RB23.
        self.force_rpi_pub = self.create_publisher(
            Bool,
            "/rb23/force_rpi",
            10
        )

        self.cam_stable_pub = self.create_publisher(
            Bool,
            "/rb23/cam_stable",
            10
        )

        self.cam_angle_pub = self.create_publisher(
            Float32,
            "/rb23/cam_angle",
            10
        )

        # Publicador do modo de áudio.
        # Este tópico será consumido pelo nó de áudio.
        self.audio_mode_pub = self.create_publisher(
            AudioMode,
            "/rb23/audio_mode",
            10
        )

        # =========================================================
        # ESTADO INTERNO DE MOVIMENTO
        # =========================================================
        # Momento da última tecla de movimento detectada.
        # Isso será usado para implementar o efeito "parar ao soltar".
        self.last_motion_key_time = 0.0

        # Velocidades atualmente desejadas.
        self.current_linear = 0.0
        self.current_angular = 0.0

        # =========================================================
        # ESTADO INTERNO DOS COMANDOS AUXILIARES
        # =========================================================
        # Valores iniciais coerentes com o estado base usado no driver.
        self.force_rpi = False
        self.cam_stable = True
        self.cam_angle = 0.0

        # =========================================================
        # ESTADO INTERNO DO ÁUDIO
        # =========================================================
        # Inicialmente escolhemos silêncio bilateral.
        self.audio_mode = AudioMode.MODE_SILENCE

        # =========================================================
        # INFORMAÇÕES PARA EXIBIÇÃO NA TELA
        # =========================================================
        self.last_key_name = "-"
        self.last_action = "aguardando comando"

        # Flag usada para controlar a saída do loop principal.
        self.running = True

        # =========================================================
        # TIMER DE PUBLICAÇÃO
        # =========================================================
        # Mesmo quando o robô está parado, é útil continuar publicando
        # velocidade zero para manter o fluxo de comandos consistente.
        publish_period = 1.0 / self.publish_rate_hz
        self.publish_timer = self.create_timer(
            publish_period,
            self.publish_cmd_vel
        )

        self.get_logger().info("Nó de teclado inicializado com sucesso.")

        # Publica o modo inicial de áudio assim que o nó inicia.
        self.publish_audio_mode()

    # =========================================================
    # FUNÇÕES AUXILIARES GERAIS
    # =========================================================

    def clamp(self, value: float, low: float, high: float) -> float:
        """
        Limita um valor ao intervalo [low, high].
        """
        return max(low, min(value, high))

    def audio_mode_to_text(self, mode: int) -> str:
        """
        Converte o código numérico do modo de áudio em texto amigável.
        """
        if mode == AudioMode.MODE_SILENCE:
            return "silêncio bilateral"
        if mode == AudioMode.MODE_LISTEN:
            return "ouvir o robô"
        if mode == AudioMode.MODE_TALK:
            return "falar com o robô"
        return f"desconhecido ({mode})"

    # =========================================================
    # FUNÇÕES AUXILIARES DE MOVIMENTO
    # =========================================================

    def set_motion(
        self,
        linear: float,
        angular: float,
        key_name: str,
        action: str
    ) -> None:
        """
        Atualiza o estado atual do comando de movimento.

        Parâmetros:
        -----------
        linear:
            Velocidade linear desejada no eixo x.

        angular:
            Velocidade angular desejada no eixo z.

        key_name:
            Nome amigável da tecla pressionada.

        action:
            Descrição textual da ação.
        """
        self.current_linear = linear
        self.current_angular = angular
        self.last_key_name = key_name
        self.last_action = action
        self.last_motion_key_time = time.time()

    def stop_motion(
        self,
        key_name: str = "-",
        action: str = "parado"
    ) -> None:
        """
        Zera imediatamente o comando de movimento.
        """
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_key_name = key_name
        self.last_action = action

    def update_key_timeout(self) -> None:
        """
        Implementa o comportamento de "tecla mantida pressionada".

        Se passar mais tempo que o limite configurado sem detectar
        uma nova tecla de movimento, o comando é zerado.
        """
        if self.last_motion_key_time <= 0.0:
            return

        elapsed = time.time() - self.last_motion_key_time

        if elapsed > self.key_hold_timeout:
            self.stop_motion(action="parado (tecla solta)")

    def publish_cmd_vel(self) -> None:
        """
        Publica periodicamente a mensagem Twist em /rb23/cmd_vel.
        """
        # Antes de publicar, verificamos se o comando expirou.
        self.update_key_timeout()

        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular

        self.cmd_vel_pub.publish(msg)

    # =========================================================
    # FUNÇÕES AUXILIARES DOS COMANDOS AUXILIARES
    # =========================================================

    def publish_force_rpi(self) -> None:
        """
        Publica o estado atual de force_rpi.
        """
        msg = Bool()
        msg.data = self.force_rpi
        self.force_rpi_pub.publish(msg)

    def publish_cam_stable(self) -> None:
        """
        Publica o estado atual de estabilização da câmera.
        """
        msg = Bool()
        msg.data = self.cam_stable
        self.cam_stable_pub.publish(msg)

    def publish_cam_angle(self) -> None:
        """
        Publica o ângulo atual da câmera.
        """
        msg = Float32()
        msg.data = float(self.cam_angle)
        self.cam_angle_pub.publish(msg)

    def publish_audio_mode(self) -> None:
        """
        Publica o modo atual de áudio.
        """
        msg = AudioMode()
        msg.mode = int(self.audio_mode)
        self.audio_mode_pub.publish(msg)

    def set_audio_mode(self, mode: int, key_name: str) -> None:
        """
        Atualiza e publica o modo de áudio.
        """
        self.audio_mode = mode
        self.last_key_name = key_name
        self.last_action = f"áudio -> {self.audio_mode_to_text(self.audio_mode)}"
        self.publish_audio_mode()

    # =========================================================
    # TRATAMENTO DAS TECLAS
    # =========================================================

    def handle_key(self, ch: int) -> None:
        """
        Converte teclas em comandos de movimento e comandos auxiliares.

        Mapeamento:
        -----------
        Movimento:
        - w ou seta para cima     -> frente
        - s ou seta para baixo    -> ré
        - a ou seta para esquerda -> girar à esquerda
        - d ou seta para direita  -> girar à direita
        - x ou espaço             -> parada imediata

        Comandos auxiliares:
        - f -> alterna force_rpi
        - c -> alterna cam_stable
        - u -> aumenta cam_angle
        - j -> diminui cam_angle

        Comandos de áudio:
        - 1 -> silêncio bilateral
        - 2 -> ouvir o robô
        - 3 -> falar com o robô

        Encerramento:
        - ESC -> encerrar
        """
        # Ignoramos alguns códigos especiais do curses que não são
        # comandos úteis.
        if ch in (curses.KEY_MOUSE, curses.KEY_RESIZE):
            return

        # ---------------------------------------------------------
        # MOVIMENTO
        # ---------------------------------------------------------

        # Frente
        if ch in (ord("w"), ord("W"), curses.KEY_UP):
            self.set_motion(
                linear=+self.linear_speed,
                angular=0.0,
                key_name="w / ↑",
                action="frente"
            )

        # Ré
        elif ch in (ord("s"), ord("S"), curses.KEY_DOWN):
            self.set_motion(
                linear=-self.linear_speed,
                angular=0.0,
                key_name="s / ↓",
                action="ré"
            )

        # Giro à esquerda
        elif ch in (ord("a"), ord("A"), curses.KEY_LEFT):
            self.set_motion(
                linear=0.0,
                angular=+self.angular_speed,
                key_name="a / ←",
                action="girar à esquerda"
            )

        # Giro à direita
        elif ch in (ord("d"), ord("D"), curses.KEY_RIGHT):
            self.set_motion(
                linear=0.0,
                angular=-self.angular_speed,
                key_name="d / →",
                action="girar à direita"
            )

        # Parada imediata
        elif ch in (ord("x"), ord("X"), ord(" ")):
            self.stop_motion(
                key_name="x / espaço",
                action="parada imediata"
            )

        # ---------------------------------------------------------
        # COMANDOS AUXILIARES DO RB23
        # ---------------------------------------------------------

        # Alterna force_rpi
        elif ch in (ord("f"), ord("F")):
            self.force_rpi = not self.force_rpi
            self.last_key_name = "f"
            self.last_action = f"force_rpi -> {self.force_rpi}"
            self.publish_force_rpi()

        # Alterna estabilização da câmera
        elif ch in (ord("c"), ord("C")):
            self.cam_stable = not self.cam_stable
            self.last_key_name = "c"
            self.last_action = f"cam_stable -> {self.cam_stable}"
            self.publish_cam_stable()

        # Aumenta ângulo da câmera
        elif ch in (ord("u"), ord("U")):
            self.cam_angle = self.clamp(
                self.cam_angle + self.cam_angle_step,
                self.cam_angle_min,
                self.cam_angle_max
            )
            self.last_key_name = "u"
            self.last_action = f"cam_angle -> {self.cam_angle:.2f}"
            self.publish_cam_angle()

        # Diminui ângulo da câmera
        elif ch in (ord("j"), ord("J")):
            self.cam_angle = self.clamp(
                self.cam_angle - self.cam_angle_step,
                self.cam_angle_min,
                self.cam_angle_max
            )
            self.last_key_name = "j"
            self.last_action = f"cam_angle -> {self.cam_angle:.2f}"
            self.publish_cam_angle()

        # ---------------------------------------------------------
        # COMANDOS DE ÁUDIO
        # ---------------------------------------------------------

        # Silêncio bilateral
        elif ch == ord("1"):
            self.set_audio_mode(AudioMode.MODE_SILENCE, "1")

        # Ouvir o robô
        elif ch == ord("2"):
            self.set_audio_mode(AudioMode.MODE_LISTEN, "2")

        # Falar com o robô
        elif ch == ord("3"):
            self.set_audio_mode(AudioMode.MODE_TALK, "3")

        # ---------------------------------------------------------
        # ENCERRAMENTO
        # ---------------------------------------------------------

        elif ch == 27:
            self.last_key_name = "ESC"
            self.last_action = "encerrando"
            self.running = False

    # =========================================================
    # LEITURA ROBUSTA DAS SETAS
    # =========================================================

    def get_next_key(self, stdscr) -> int:
        """
        Lê uma tecla do terminal e tenta interpretar corretamente
        as setas direcionais.

        Em muitos terminais Linux, as setas chegam como sequências
        de escape iniciadas por ESC (27). Esta função trata isso
        explicitamente.
        """
        ch = stdscr.getch()

        # Se não for início de sequência ESC, devolvemos diretamente.
        if ch != 27:
            return ch

        # Pequeno atraso para permitir a chegada dos bytes seguintes
        # da sequência de escape.
        time.sleep(0.002)

        next1 = stdscr.getch()
        if next1 == -1:
            return 27

        if next1 != 91:
            return 27

        next2 = stdscr.getch()
        if next2 == -1:
            return 27

        if next2 == 65:
            return curses.KEY_UP
        if next2 == 66:
            return curses.KEY_DOWN
        if next2 == 67:
            return curses.KEY_RIGHT
        if next2 == 68:
            return curses.KEY_LEFT

        return 27

    # =========================================================
    # INTERFACE TEXTUAL
    # =========================================================

    def render_screen(self, stdscr) -> None:
        """
        Desenha a interface textual no terminal.
        """
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()

        lines = [
            "================ GUARDROS - TELEOP POR TECLADO ================",
            "",
            "Movimento:",
            "  w ou ↑  : frente",
            "  s ou ↓  : ré",
            "  a ou ←  : girar à esquerda",
            "  d ou →  : girar à direita",
            "  x/space : parada imediata",
            "",
            "Comandos auxiliares do RB23:",
            "  f       : alterna force_rpi",
            "  c       : alterna estabilização da câmera",
            "  u       : aumenta ângulo da câmera",
            "  j       : diminui ângulo da câmera",
            "",
            "Áudio:",
            "  1       : silêncio bilateral",
            "  2       : ouvir o robô",
            "  3       : falar com o robô",
            "",
            "Encerramento:",
            "  ESC     : sair",
            "  Ctrl+C  : sair",
            "",
            f"Velocidade linear configurada : {self.linear_speed:.3f}",
            f"Velocidade angular configurada: {self.angular_speed:.3f}",
            f"Timeout de tecla              : {self.key_hold_timeout:.3f} s",
            f"Passo do ângulo da câmera     : {self.cam_angle_step:.2f}",
            "",
            f"Última tecla                  : {self.last_key_name}",
            f"Última ação                   : {self.last_action}",
            "",
            f"linear.x atual                : {self.current_linear:.3f}",
            f"angular.z atual               : {self.current_angular:.3f}",
            f"force_rpi atual               : {self.force_rpi}",
            f"cam_stable atual              : {self.cam_stable}",
            f"cam_angle atual               : {self.cam_angle:.2f}",
            f"audio_mode atual              : {self.audio_mode_to_text(self.audio_mode)}",
            "",
            "Publicando em /rb23/cmd_vel ...",
            "Publicando em /rb23/audio_mode ...",
        ]

        for i, line in enumerate(lines):
            if i >= max_y - 1:
                break

            try:
                stdscr.addnstr(i, 0, line, max_x - 1)
            except curses.error:
                # Em terminais pequenos, algumas linhas podem não caber.
                pass

        try:
            stdscr.refresh()
        except curses.error:
            pass

    # =========================================================
    # LOOP PRINCIPAL DO CURSES
    # =========================================================

    def curses_loop(self, stdscr) -> None:
        """
        Loop principal da interface por teclado.
        """
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)
        curses.noecho()
        curses.cbreak()

        last_draw = 0.0
        draw_period = 0.05

        while self.running and rclpy.ok():
            try:
                # Processa callbacks ROS sem bloquear o loop.
                rclpy.spin_once(self, timeout_sec=0.0)

                ch = self.get_next_key(stdscr)
                if ch != -1:
                    self.handle_key(ch)

                now = time.time()
                if now - last_draw >= draw_period:
                    self.render_screen(stdscr)
                    last_draw = now

                time.sleep(0.01)

            except KeyboardInterrupt:
                self.last_key_name = "Ctrl+C"
                self.last_action = "encerrando por interrupção do teclado"
                self.running = False
                break

            except Exception as exc:
                self.get_logger().error(f"Erro no loop curses: {exc}")
                time.sleep(0.05)


def main(args=None) -> None:
    """
    Função principal do nó.
    """
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        curses.wrapper(node.curses_loop)

    except KeyboardInterrupt:
        # Caso uma interrupção ainda escape do loop interno,
        # seguimos para o encerramento normal.
        pass

    finally:
        # Antes de destruir o nó, tentamos publicar um último comando
        # de parada apenas se o contexto ROS ainda estiver válido.
        if rclpy.ok():
            try:
                zero_msg = Twist()
                node.cmd_vel_pub.publish(zero_msg)

                # Também republicamos o modo atual de áudio
                # para deixar o estado explícito no encerramento.
                node.publish_audio_mode()

                time.sleep(0.05)
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()