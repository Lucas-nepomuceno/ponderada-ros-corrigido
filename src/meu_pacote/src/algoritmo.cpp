#include <bits/stdc++.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
using namespace std;
#define LINHA 29
#define COLUNA 29


class AlgoritmoParte1 : public rclcpp::Node
{
public:
    AlgoritmoParte1()
    : Node("algoritmo_parte_1")
    {
        client = this->create_client<cg_interfaces::srv::GetMap>("get_map");

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Aguardando serviço get_map...");
        }

        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        result_future = client->async_send_request(request).future.share();
    }

    std::vector<std::string> getMapa()
    {
        if (rclcpp::spin_until_future_complete(
                this->shared_from_this(), 
                result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Peguei o mapa");

            return result_future.get()->occupancy_grid_flattened;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Falhou ao chamar get_map");
            return {};
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture result_future;
};

typedef pair<int, int> Par;

//estrutura com os parametro f e as coordenadas para uma celula
typedef pair<double, pair<int,int>> pPar;

// Estrutura de mapa 29x29
struct mapa_normal
{
	char mapa[LINHA][COLUNA];
};

//estrutura que guarda os pais e os parametros do A*
struct celula {
    int pai_x, pai_y;
    char comando;
    double f, g, h;
};

// Função utilitária para devolver o mapa normal (29x29)
struct mapa_normal deixa_em_formato_de_mapa(const std::vector<std::string>& mapa) {
    struct mapa_normal mapa_normal;

    int indice_mapa_flattened = 0;

    for (int i = 0; i < LINHA; i++) {
        for (int j = 0; j < COLUNA; j++) {
            mapa_normal.mapa[i][j] = mapa[indice_mapa_flattened][0];
            indice_mapa_flattened++;
        }
    }

    return mapa_normal;
}

// Referência: geeksforgeeks.org/dsa/a-search-algorithm
//Função utilitária para checar validade da celula.
bool eh_valida(int linha, int coluna)
{

    return (linha >= 0) && (linha < LINHA) && (coluna >= 0)
           && (coluna < COLUNA);
}

// Função utilitária para ver se a célula tá bloqueada
bool eh_bloqueada(int linha, int coluna, mapa_normal mapa) {
    if (mapa.mapa[linha][coluna] == 'b') {
        return true;
    }

    return false;
}

// Função utilitária para ver se a célula é o target
bool eh_target(int linha, int coluna, Par target) {
    if (!(linha == target.first && coluna == target.second)) {
        return false;
    }

    return true;
}

// Função utilitária para calcular o parâmetro h do A* usando a fórmula de distância
double calcula_valor_de_h(int linha, int coluna, Par target) {
    double distancia_x = (linha - target.first);
    double distancia_y = (coluna - target.second);

    return ((double)sqrt((distancia_x*distancia_x + distancia_y*distancia_y)));
}

// Função utilitária para fazer o caminho do robô
void encontra_caminho(celula detalhes_das_celulas[LINHA][COLUNA], Par target)
{
    //Começa do alvo
    int linha = target.first;
    int coluna = target.second;

    stack<char> Caminho;

    //Até chegar no robô
    while (!(detalhes_das_celulas[linha][coluna].pai_x == linha && detalhes_das_celulas[linha][coluna].pai_y == coluna)) {
        if (detalhes_das_celulas[linha][coluna].comando != '-')
            Caminho.push(detalhes_das_celulas[linha][coluna].comando);

        int temp_linha = detalhes_das_celulas[linha][coluna].pai_x;
        int temp_coluna = detalhes_das_celulas[linha][coluna].pai_y;
        linha = temp_linha;
        coluna = temp_coluna;
    }

    if (detalhes_das_celulas[linha][coluna].comando != '-')
     Caminho.push(detalhes_das_celulas[linha][coluna].comando);

    while (!Caminho.empty()) {
        char p = Caminho.top();
        Caminho.pop();
        switch(p) {
            case 'u':
                cout << "Move up" << endl;
                break;
            case 'd':
                cout << "Move down" << endl;
                break;
            case 'l':
                cout << "Move left" << endl;
                break;
            case 'r':
                cout << "Move right" << endl;
                break;
        }
    }

    return;
}

void busca_por_a_estrela (mapa_normal mapa_a_ser_buscado, Par robo, Par target) {
    bool lista_fechada[LINHA][COLUNA];
    memset(lista_fechada, false, sizeof(lista_fechada));

    celula detalhes_das_celulas[LINHA][COLUNA];

    int i, j;

    for (i = 0; i < LINHA; i++) {
        for (j = 0; j < COLUNA; j++) {
            detalhes_das_celulas[i][j].f = FLT_MAX; //FLT_MAX é infinito
            detalhes_das_celulas[i][j].g = FLT_MAX;
            detalhes_das_celulas[i][j].h = FLT_MAX;
            detalhes_das_celulas[i][j].pai_x = -1;
            detalhes_das_celulas[i][j].pai_y = -1;
        }
    }

    // Iniciando os parametros da robo
    i = robo.first, j = robo.second;
    detalhes_das_celulas[i][j].f = 0.0;
    detalhes_das_celulas[i][j].g = 0.0;
    detalhes_das_celulas[i][j].h = 0.0;
    detalhes_das_celulas[i][j].comando = '-';
    detalhes_das_celulas[i][j].pai_x = i;
    detalhes_das_celulas[i][j].pai_y = j; //ele é o proprio pai

    set<pPar> lista_aberta; 

    
    lista_aberta.insert(make_pair(0.0, make_pair(i, j))); //coloca o robo na lista_aberta

    bool achou_o_target = false;

    while (!lista_aberta.empty()) {
        pPar primeiro_da_lista = *lista_aberta.begin();

        lista_aberta.erase(lista_aberta.begin());

        // Define este vertice como verdadeiro na lista fechada
        i = primeiro_da_lista.second.first;
        j = primeiro_da_lista.second.second;
        lista_fechada[i][j] = true;

        double gNovo, hNovo, fNovo;

        //Primeiro Sucessor: cima
        if (eh_valida(i - 1, j)) {
            
            if (eh_target(i - 1, j, target)) {
                detalhes_das_celulas[i - 1][j].pai_x = i;
                detalhes_das_celulas[i - 1][j].pai_y = j;
                printf("Encontramos o target\n");
                encontra_caminho(detalhes_das_celulas, target);
                achou_o_target = true;
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i - 1][j] == false
                     && !eh_bloqueada(i - 1, j, mapa_a_ser_buscado)) {
                gNovo = detalhes_das_celulas[i][j].g + 1.0;
                hNovo = calcula_valor_de_h(i - 1, j, target);
                fNovo = gNovo + hNovo;

                bool esta_na_lista_aberta = detalhes_das_celulas[i - 1][j].f != FLT_MAX;
                bool caminho_antigo_eh_pior = detalhes_das_celulas[i - 1][j].f > fNovo;

                if (!esta_na_lista_aberta
                    || caminho_antigo_eh_pior) {
                    lista_aberta.insert(make_pair(
                        fNovo, make_pair(i - 1, j)));

                    // Atualiza detalhes da célula
                    detalhes_das_celulas[i - 1][j].f = fNovo;
                    detalhes_das_celulas[i - 1][j].g = gNovo;
                    detalhes_das_celulas[i - 1][j].h = hNovo;
                    detalhes_das_celulas[i - 1][j].comando = 'u';
                    detalhes_das_celulas[i - 1][j].pai_x = i;
                    detalhes_das_celulas[i - 1][j].pai_y = j;
                }
            }
        }

        //Segundo Sucessor: baixo
        if (eh_valida(i + 1, j)) {
            
            if (eh_target(i + 1, j, target)) {
                detalhes_das_celulas[i + 1][j].pai_x = i;
                detalhes_das_celulas[i + 1][j].pai_y = j;
                printf("Encontramos o target\n");
                encontra_caminho(detalhes_das_celulas, target);
                achou_o_target = true;
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i + 1][j] == false
                     && !eh_bloqueada(i + 1, j, mapa_a_ser_buscado)) {
                gNovo = detalhes_das_celulas[i][j].g + 1.0;
                hNovo = calcula_valor_de_h(i + 1, j, target);
                fNovo = gNovo + hNovo;

                bool esta_na_lista_aberta = detalhes_das_celulas[i + 1][j].f != FLT_MAX;
                bool caminho_antigo_eh_pior = detalhes_das_celulas[i + 1][j].f > fNovo;

                if (!esta_na_lista_aberta
                    || caminho_antigo_eh_pior) {
                    lista_aberta.insert(make_pair(
                        fNovo, make_pair(i + 1, j)));

                    // Atualiza detalhes da célula
                    detalhes_das_celulas[i + 1][j].f = fNovo;
                    detalhes_das_celulas[i + 1][j].g = gNovo;
                    detalhes_das_celulas[i + 1][j].h = hNovo;
                    detalhes_das_celulas[i + 1][j].comando = 'd';
                    detalhes_das_celulas[i + 1][j].pai_x = i;
                    detalhes_das_celulas[i + 1][j].pai_y = j;
                }
            }
        }

        //Terceiro Sucessor: esquerda
        if (eh_valida(i, j - 1)) {
            
            if (eh_target(i, j - 1, target)) {
                detalhes_das_celulas[i][j - 1].pai_x = i;
                detalhes_das_celulas[i][j - 1].pai_y = j;
                printf("Encontramos o target\n");
                encontra_caminho(detalhes_das_celulas, target);
                achou_o_target = true;
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i][j - 1] == false
                     && !eh_bloqueada(i, j - 1, mapa_a_ser_buscado)) {
                gNovo = detalhes_das_celulas[i][j].g + 1.0;
                hNovo = calcula_valor_de_h(i, j - 1, target);
                fNovo = gNovo + hNovo;

                bool esta_na_lista_aberta = detalhes_das_celulas[i][j - 1].f != FLT_MAX;
                bool caminho_antigo_eh_pior = detalhes_das_celulas[i][j - 1].f > fNovo;

                if (!esta_na_lista_aberta
                    || caminho_antigo_eh_pior) {
                    lista_aberta.insert(make_pair(
                        fNovo, make_pair(i, j - 1)));

                    // Atualiza detalhes da célula
                    detalhes_das_celulas[i][j- 1].f = fNovo;
                    detalhes_das_celulas[i][j- 1].g = gNovo;
                    detalhes_das_celulas[i][j- 1].h = hNovo;
                    detalhes_das_celulas[i][j- 1].comando = 'l';
                    detalhes_das_celulas[i][j- 1].pai_x = i;
                    detalhes_das_celulas[i][j- 1].pai_y = j;
                }
            }
        }   
        
        //Quarto Sucessor: direita
        if (eh_valida(i, j + 1)) {
            
            if (eh_target(i, j + 1, target)) {
                detalhes_das_celulas[i][j + 1].pai_x = i;
                detalhes_das_celulas[i][j + 1].pai_y = j;
                printf("Encontramos o target\n");
                encontra_caminho(detalhes_das_celulas, target);
                achou_o_target = true;
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i][j + 1] == false
                     && !eh_bloqueada(i, j + 1, mapa_a_ser_buscado)) {
                gNovo = detalhes_das_celulas[i][j].g + 1.0;
                hNovo = calcula_valor_de_h(i, j + 1, target);
                fNovo = gNovo + hNovo;

                bool esta_na_lista_aberta = detalhes_das_celulas[i][j + 1].f != FLT_MAX;
                bool caminho_antigo_eh_pior = detalhes_das_celulas[i][j + 1].f > fNovo;

                if (!esta_na_lista_aberta
                    || caminho_antigo_eh_pior) {
                    lista_aberta.insert(make_pair(
                        fNovo, make_pair(i, j + 1)));

                    // Atualiza detalhes da célula
                    detalhes_das_celulas[i][j+ 1].f = fNovo;
                    detalhes_das_celulas[i][j+ 1].g = gNovo;
                    detalhes_das_celulas[i][j+ 1].h = hNovo;
                    detalhes_das_celulas[i][j+ 1].comando = 'r';
                    detalhes_das_celulas[i][j+ 1].pai_x = i;
                    detalhes_das_celulas[i][j+ 1].pai_y = j;
                }
            }
        } 

    }

    if (!achou_o_target)
        printf("A célula target não foi encontrada\n");

    return;
}

int main(int argc, char **argv) {    
    rclcpp::init(argc, argv);

    auto no = std::make_shared<AlgoritmoParte1>();
    rclcpp::spin_some(no);
    // Espera o mapa
    auto mapa_flattened = no->getMapa();

    struct mapa_normal mapa_a_ser_buscado;
    mapa_a_ser_buscado = deixa_em_formato_de_mapa(mapa_flattened);
    Par robo;
    Par target;

    for (int i = 0; i < LINHA; i++) {
        for (int j = 0; j < COLUNA; j++){
            if(mapa_a_ser_buscado.mapa[i][j] == 'r'){
                robo = make_pair(i,j);
            }
        }
    }

    for (int i = 0; i < LINHA; i++) {
        for (int j = 0; j < COLUNA; j++){
            if(mapa_a_ser_buscado.mapa[i][j] == 't'){
                target = make_pair(i,j);
            }
        }
    }

    cout << "o mapa a ser buscado é:" << endl;
    for (int i = 0; i < LINHA; i++) {
        for(int j = 0; j < COLUNA; j++){
            cout << mapa_a_ser_buscado.mapa[i][j];
        }
        cout << endl;
    }

    busca_por_a_estrela(mapa_a_ser_buscado, robo, target);

    rclcpp::shutdown();

    return (0);
}
