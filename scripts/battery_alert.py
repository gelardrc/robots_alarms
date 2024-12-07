#!/usr/bin/env python
# -*- coding: utf-8 -*-
## Código de teste para Wavefront ##
import rospy
import sys
from sensor_msgs.msg import BatteryState
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

bateria = 1.0  # Inicializa a variável bateria com 0.0

def battery_cb(msg):
    """Callback da bateria que atualiza o valor global da bateria."""
    global bateria
    bateria = msg.percentage
    #srospy.loginfo("Bateria: {:.2f}%".format(bateria * 100))

def main(id, header):
    global bateria


    battery_alert = float(rospy.get_param(rospy.get_name()+"/battery_percent",0.5))

    rospy.logwarn("Battery percentage:{}".format(battery_alert))

    

    # Inscreve no tópico de bateria
    rospy.Subscriber(header + '/mavros/battery', BatteryState, battery_cb)

    # Publisher para o Marker no RViz
    pub = rospy.Publisher(header + '/battery_marker', Marker, queue_size=10)
    
    pub_status = rospy.Publisher(header + '/battery_status', Bool, queue_size=10)


    # Frequência de publicação (1 Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Cria o Marker para o texto
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.id = int(id)
        marker.text = "Battery alert {}: {:.2f}%!".format(header,bateria * 100)  # Exibe a porcentagem de bateria

        # Definindo a posição do texto no RViz
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0  # Eleva o texto para facilitar a visualização

        # Definindo a escala do texto (tamanho da fonte)
        marker.scale.z = 1.0

        # Definindo a cor do texto (RGBA)
        marker.color.r = 1.0  # Cor vermelha
        marker.color.g = 0.0  # Cor verde
        marker.color.b = 0.0  # Sem componente azul
        marker.color.a = 1.0  # Transparência total (1.0 é opaco)

        # Publica o Marker no RViz
        if bateria < battery_alert:
            pub.publish(marker)
            pub_status.publish(Bool(True))
            rospy.logwarn_once("Battery alert !!!")
            rospy.logwarn_once("Sending....")

        else:
            pub_status.publish(Bool(False))

        # Aguarda até a próxima iteração
        rate.sleep()

if __name__ == '__main__':
    # Verifica os argumentos da linha de comando
    if len(sys.argv) < 2:
        str_id = "1"
        header = ""
    else:
        str_id = sys.argv[1]  # Primeiro argumento
        header = "/uav" + sys.argv[1]

    # Inicializa o nó ROS
    rospy.init_node("battery_uav_" + str_id, anonymous=False)

    # Executa o loop principal
    try:
        main(id=str_id, header=header)
    except rospy.ROSInterruptException:
        pass
