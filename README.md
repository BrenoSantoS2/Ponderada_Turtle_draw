# Ponderada_Turtle_draw

Como executar:

Primeiramente é necessário que você rode alguns códigos para ter certeza que vai funcionar certinho:

colcon build

source install/local_setup.bash


Com isso agora iremos rodar o código do turtlesim:

ros2 run turtlesim turtlesim_node

Agora rodademos o código do desenho, que infelizmente esta como arquivo python pois tentei de tudo e não consigo compilar para ser executado como ros2 run de maneira alguma, então você vai entrar pelo terminal no diretório do projeto, pelo caminho:

./src/turtle_draw/turtle_draw/

E lá você vai executar o comando:

python3 drawing.py

E agora o desenho será feito!!
