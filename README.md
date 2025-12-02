# Algoritmo de busca em labirinto

## Introdução

&emsp; Este repositório foi criado para atender a uma atividade de programação do Instituto de Tecnologia e Liderança. O objetivo da atividade era dividido em duas partes:

1. Navegação com mapa — tendo acesso ao mapa, desenvolver um algoritmo para encontrar a rota otimizada até o alvo.

2. Mapeamento do labirinto: O algoritmo desenvolvido deve navegar pelo mapa, mapeando-o no processo. A seguir, deve-se comprovar que o mapa criado é suficiente para reproduzir a rota da parte 1. (NICOLA, 2025)

&emsp; Para tanto, utilizei um pacote ROS da culling games: um simulador de labirinto feito em pygame. Para saber mais sobre o simulador, acesse o seguinte repositório: [https://github.com/rmnicola/culling_games](https://github.com/rmnicola/culling_games)


## Funcionamento dos algoritmos 

&emsp; Nesta atividade, criou-se um pacote com dois executáveis que se comunicam com o simulador do labirinto. Esses algoritmos estão em ``src/meu_pacote/src/algoritmo1`` e ``src/meu_pacote/src/algoritmo2``. Eles atendem à parte 1 e parte 2 da atividade, respectivamente.

&emsp; O primeiro algoritmo,
1. Recebe o mapa planificado
2. Formata-o como uma matriz
3. Usa A* para procurar o target de forma otimizada*
4. Move o robô até o target

&emsp; O segundo algoritmo,
1. Usa uma versão adaptada do A* para procurar o target de forma otimizada
2. Move o robô até o target

&emsp; A diferença entre os dois reside, principalmente, em como saber se uma célula está bloqueada ou não. No primeiro, isso é feito via mapa. Portanto, o algoritmo tem uma velocidade avassaladora. Já no segundo, o robô deve ir até a célula mapeada para descobrir se os seus sucessores estão bloqueados. Isso cria a necessidade de 
1. **resetar o mapa a cada interação**;
2. mover o robô;
3. receber a mensagem no tópico de sensores;
4. e aferir se a célula é ou não bloqueada. 

&emsp; Apesar de conceitualmente pequena, essa mudança causa uma grande diferença tanto na implementação no algoritmo quanto na velocidade do encontro da saída. Para se comprovar que ambos os algoritmos seguem a mesma rota, usa-se recursos visuais e logs.

## Principais implementações

&emsp; O _core_ dos três algoritmos estão na classe, na função ``busca_por_a_estrela`` e na função encontra_caminho. Em ambos os algortimos, implementou-se uma classe para comunicação via ROS. Ademais, a função de busca A* funciona de modo parecido em ambos. Ademais, criou-se o método encontra caminho para mover o robô pelo caminho mais otimizado.

&emsp; Apesar de fundamentais, as classes de cada algoritmo funciona de modo diferente. No primeiro algoritmo, há a implementação do cliente para o serviço *get_map* (que, como diz o nome, pega o mapa do serviço do labirinto) e do cliente para o serviço *move_command*. Já no segundo algoritmo, há a implementação do cliente *move_command*, *reset* e do subscriber no tópico "/culling_games/robot_sensors". Essa diferença se deve a natureza de navegação no escuro da segunda parte.

&emsp; Já na função ``busca_por_a_estrela``, as diferenças são mínimas. No *corpus* da função, cria-se duas listas: uma fechada e uma aberta. Na lista aberta são inseridos pontos que foram vistos, mas cujos sucessores não foram ainda visitados. Já na lista fechada, inclui-se células visitadas e cujos sucessores já foram visitados. O algoritmo funciona em formato de pilha e inicia com a posição inicial do robô. Então, ele visita células nos pontos cardeais e calcula seu valor h (a distância até o target). Se o valor calculado é menor do que estava previamente na célula, então, nossa posição é definida como (pais) dessa célula, criando uma estrutura de grafo acíclico dirigido em direção ao target que melhora com o mapeamento. Isso funciona até que o robô chegue no target. A diferença entre os dois algoritmos é apenas o método de mapeamento como foi explicado anteriormente.

&emsp; Por fim, a função ``encontra_caminho`` ganha um novo significado com o segundo algoritmo. No primeiro, ela é usada apenas após o encontro da melhor rota. Então, o robô segue essa rota em formato de pilha, que começa no sucessor e vai passando pelos pais até o robô. Já no segundo algoritmo, dada a necessidade de sensoriamento, essa função é utilizada ininterruptamente até que se encontre o target. Então, dá-se uma pausa para visualizar e comparar as rotas, e então ela é acionada novamente para levar o robô até o target.

## Como rodar a solução

&emsp; Para rodar essa solução, é necessário ter o ROS2 instalado em seu computador.

1. Após cumprir este pré requisito, clone o repositório da culling_games e este.
2. Seguindo o tutorial do repositório da [culling_games](https://github.com/rmnicola/culling_games), gere um mapa.
3. Em outro terminal, vá até o local onde o repositório foi clonado e rode os seguintes comandos para ver o funcionamento da primeira parte:

```bash
source opt/ros/<sua versão de ros>/setup.bash
colcon build
source install/setup.bash
ros2 run meu_pacote algoritmo_parte_1
```

4. Para a segunda parte, reinicie o mapa e rode:

```bash
ros2 run meu_pacote algoritmo_parte_2
```

E se divirta vendo o robô encontrar uma rota otimizada em outros mapas.

## Demonstração 


* Inspiração: GEEKSFORGEEKS. A* Search Algorithm. Disponível em: <https://www.geeksforgeeks.org/dsa/a-search-algorithm/>. 
