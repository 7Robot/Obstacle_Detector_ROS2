# Détecteur de cluster

Ce noeud a pour but de détecter des clusters de points à partir d'un scan Lidar et de les renvoyer en tant qu'obstacle circulaire.

### Entrées :
L'entrée se fait par des messages de type `sensor_msgs::msg::LidarScan` envoyé sur le topic `scan`.

### Sortie : 
La sortie se fait comme des messages de type `cdf::msgs::Obstacles` sur le topic `raw_obstacles`.