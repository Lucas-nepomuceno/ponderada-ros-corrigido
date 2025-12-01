# Algoritmo de busca em labirinto

## Introdução

&emsp; Este repositório foi criado para atender a uma atividade de programação do Instituto de Tecnologia e Liderança. O objetivo da atividade era dividido em duas partes:

```
"A parte 1 consiste em navegação com mapa — tendo acesso ao mapa, você deverá desenvolver um algoritmo para encontrar a rota otimizada até o alvo.

A parte 2 envolve o mapeamento do labirinto. O algoritmo desenvolvido deve navegar pelo mapa, mapeando-o no processo. A seguir, deve-se comprovar que o mapa criado é suficiente para reproduzir a rota da parte 1." (NICOLA, 2025)

```

&emsp; Para tanto, utilizei um pacote ROS que é um simulador de labirinto feito em pygame. Para saber mais sobre o simulador, acesse o seguinte repositório: [https://github.com/rmnicola/culling_games](https://github.com/rmnicola/culling_games)


## Funcionamento dos algoritmos 

&emsp; Nesta atividade, cria-se um pacote com dois algoritmos que se comunicam com o simulador simulador. Esses algoritmos estão em ``src/meu_pacote/src/algoritmo1`` e ``src/meu_pacote/src/algoritmo2``. Eles atendem à parte 1 e parte 2 da atividade, respectivamente.

&emsp; O primeiro algoritmo,
1. Recebe o mapa planificado
2. Formata-o como uma matriz
3. Usa A* para procurar o target de forma otimizada(como desenvolvido pelo [https://www.geeksforgeeks.org/dsa/a-search-algorithm/](https://www.geeksforgeeks.org/dsa/a-search-algorithm/))
4. Move o robô até o target

&emsp; O segundo algoritmo,
1. Usa A*-adaptado para procurar o target de forma otimizada
2. Move o robô até o target

&emsp; A diferença entre os dois reside, principalmente, em um único aspecto: como saber se uma célula está bloqueada ou não. No primeiro, isso é feito via mapa. Portanto, o algoritmo tem uma velocidade avassaladora. Já no segundo, o robô deve ir até a célula mapeada para descobrir se os seus sucessores estão bloqueados. Isso cria a necessidade de **resetar o mapa a cada interação**,mover o robô, receber a mensagem no tópico de sensores e aferir se a célula é ou não bloqueada. Apesar de pequena, essa mudança causa uma grande diferença tanto na implementação no algoritmo quanto na 


