# mavbase
repository for basic mav and swarm functions

# Guia da Classe MAV

Aqui pode ser encontrada a documentação completa da classe MAV. 

## Sobre a classe

A classe MAV tem o objetivo de compilar as principais funções de controle de drones, usando ROS. O **arquivo original** pode ser encontrado em ```mavbase/src/mavbase``` .

## Mensagens

### Posição 

#### Posição local
        self.drone_pose = PoseStamped()
        self.goal_pose = PoseStamped()
Mensagens do tipo ```geometry_msgs```, usadas na publicão e recepção de posições locais. 

Documentação oficial: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html

#### Posição global
        self.global_pose = NavSatFix()
Mensagem do tipo ```sensor_msgs```, usada para receber posições golbais.

Documentação oficial: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html

        self.gps_target = GeoPoseStamped()
Mensagem do tipo ```geographic_msgs```, usada na publicação de posições globais. 

Documentação oficial: http://docs.ros.org/jade/api/geographic_msgs/html/msg/GeoPoseStamped.html

### Velocidade 
        self.goal_vel = TwistStamped()
Mensagem do tipo ```geometry_msgs```, usada na publicação de velocidades. 

Documentação oficial: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html

### Mensagens de estado 
        self.drone_state = State()
Mensagem do tipo ```mavros_msgs```, usada para receber os estados do drone.

Documentação oficial: http://docs.ros.org/melodic/api/mavros_msgs/html/msg/State.html

## Funções

### Funções "set"
São usadas para mudar instãncias do drone diretamente. 

#### ```set_position```
Configura posições locais em x, y e z.
#### ```set_vel```
Configura velocidades lineares em x, y e z e velocidades angulares em x, y e z.
#### ```set_mode```
Configura o modo/estado do drone para OFFBOARD, e faz uma checagem que garante que o estado é mantido em condições normais. 

### Funções de movimento
São responsáveis por traçar trajetórias, utilizando, para isso, as funções "set". 
#### ```takeoff```
Configura a posição do drone para uma altura determinada e arma o drone, se ele já não estiver armado. A trajetória de decolagem é 
parametrizada na prórpia classe, por posição, com um polinômio calculado, em que as velocidades inicial e final são nulas e a aceleração cresce e decresce de forma suave.  
#### ```RTL```
Responsável por fazer o drone retornar para as coordenadas locais (0, 0, 0), de onde foi feita a decolagem e fazer uma checagem, a partir das 
mensagens recebidas de ```ExtendedState```, se o drone está pousado, para desarmá-lo em seguida. 
#### ```hold```
Responsável por manter o drone estático por um tempo determinado.
#### ```land```
Responsável por pousar o drone na posição atual e fazer uma checagem, a partir das mensagens recebidas de ```ExtendedState```, 
se o drone está pousado, para desarmá-lo em seguida. A trajetória é parametrizada da mesma forma que na função ```takeoff```. 
#### ```disarm```
Desarma o drone, checando se o drone está pousado para que seja desarmado. 
#### ```go_gps_target```
Responsável por levar o drone a uma posição global determinada. Configura as posições globais na própria função, com latitude, longitude, altitude, orientação em x, y e z 
e velocidade angular em z (yaw). 

A trajetória é parametrizada utilizando posições e por meio de controle proporcional, em que a constante de controle do movimento recebe valores crescentes até atingir uma constante determinada. Além disso, na parametrização, é utilizada a biblioteca ```LatLon``` do Python, para conversão de distâncias em latitude e longitude para distâncias metrificadas. 
#### ```set_altitude```
Mantém o drone na mesma posição e configura uma altura determinada, sem pousá-lo. A trajetória é parametrizada da mesma forma que na função ```takeoff```. 

# Sobre a Classe SWARM

Similar à classe MAV, a SWARM compila as funções básicas para controle de um swarm de drones. A ideia é ser um controle genérico de n drones. 

## Classe ```Bee```

 Quando um objeto da SWARM é criado, ele leva um argumento de quantos drones serão controlados, e cada um é inicializado como um objeto de uma outra classe, a ```Bee```. Assim, cada objeto da classe ```Bee```, por sua vez, carrega as características especificas do drone que representa, como a posição, a velocidade e o estado, que são recebidas pelas funções de callback. 

Ela é usada, portanto, em conjunto com a classe SWARM apenas como forma de guardar as informações individuais dos drones, enquanto a SWARM recebe como objeto um vetor que contém esses drones e ela é que é responsável por mudar essas informações de modo conjunto entre todos os veículos.


## Funções

As funções da classe SWARM são muito similares às da classe MAV, com a diferença de que os comandos são feitos para o vetor de drones que é objeto da classe. Ou seja, quando setamos uma posição com a função ```set_position```, por exemplo, setamos essa posição para todos os drones do vetor, sendo que ela é relativa para cada drone, o que evita que todos acabem na mesma posição e colidam (em outras palavras, cada um se move em relação ao seu próprio sistema de coordenadas).

### ```run_delivery```

Essa é a única função que é única da classe SWARM e não está presente na MAV. A ideia dela é realizar um trajeto cooperativo entre os drones, inspirado em uma entrega cooperativa de um pacote. Ela recebe uma latitude e uma longitude e esses argumentos são passados como posições globais para as quais o drone 0 deve ir, o qual é tido como referência para o vetor de drones que é objeto da classe. Essa posição é transformada em coordenadas cartesianas e é passada como argumento para a posição final desse drone. 
