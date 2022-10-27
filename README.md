# Etapa_4

El archivo "meta_etapa_4_2_UPS.py" realiza el mapeo del lugar y al final regresa al punto de donde se hizo el spawn del robot mediante calculo del angulo y desfase de la posicion actual del robot y el punto de origen. Para esto se usó un suscriptor al /hsrb/odom del robot que nos da la posición actual del robot no respecto al mapa.


En el archivo "meta_etapa_4_control.py" de igual manera realiza el mapeo de la zona y al final se usa una una funcion de control "calculate_control" y "get_robot_pose" para poder obtener las coordenadas del robot.
Para esto de igual manera se modific[o el .launch "stage04_UPS_1.launch" agregando unas lineas de c[odigo para poder obtener transformadas hacia el mapa miestra se va mapeando y se usa la transformada generada en el mapa en el punto donde se hizo el spawn del robot y así obtener el punto de origen.

Los dos códigos funcionan para realizar el mapeo del mapeo del lugar. En el .launch enviado se encuentra puesto para que trabaje junto con el "meta_etapa_4_control.py"

Este enlace de youtube corresponde al video grabado del gazebo y Rviz.
https://www.youtube.com/watch?v=ZnjGPL6RmCc&t=67s

Igual ponemos el enlace de la grabación del Gazebo.
https://youtu.be/Uums6F3-bq4


Se realizaron pruebas de las simulacion con un punto de origen del robot en x=-3 y=2. Los enlaces de los videos son los siguientes:
Mapeo Gazebo: https://www.youtube.com/watch?v=L9MpGds97uQ
