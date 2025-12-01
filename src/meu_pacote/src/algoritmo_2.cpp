#include <bits/stdc++.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
using namespace std;
#define LINHA 29
#define COLUNA 29


class AlgoritmoParte2 : public rclcpp::Node
{
public:
    AlgoritmoParte2()
    : Node("algoritmo_parte_2")
    {
        move_robot_client = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");
        reset_client = this->create_client<cg_interfaces::srv::Reset>("reset");

        subscriber =
            this->create_subscription<cg_interfaces::msg::RobotSensors>(
                "/culling_games/robot_sensors",
                rclcpp::SensorDataQoS(),
                std::bind(&AlgoritmoParte2::topic_callback, this, std::placeholders::_1)
            );    
    }

    int move(char direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();

        switch(direction) {
            case 'u': request->direction = "up"; break;
            case 'd': request->direction = "down"; break;
            case 'l': request->direction = "left"; break;
            case 'r': request->direction = "right"; break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Direção inválida");
                return -1;
        }

        if (!move_robot_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Serviço move_command não disponível");
            return -1;
        }

        auto future = move_robot_client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            switch(direction) {
            case 'u': RCLCPP_INFO(this->get_logger(), "Movido para cima!"); break;
            case 'd': RCLCPP_INFO(this->get_logger(), "Movido para baixo!"); break;
            case 'l': RCLCPP_INFO(this->get_logger(), "Movido para esquerda!"); break;
            case 'r': RCLCPP_INFO(this->get_logger(), "Movido para direita!"); break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Direção inválida");
                return -1;
            }
            rclcpp::sleep_for(5ms);
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Falha ao mover");
            return -1;
        }
    }

    int reset()
    {
        auto request = std::make_shared<cg_interfaces::srv::Reset::Request>();

        if (!reset_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Serviço reset não disponível");
            return -1;
        }

        auto future = reset_client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Falha ao resetar mapa");
            return -1;
        }
    }

    cg_interfaces::msg::RobotSensors getUltimoSensorStatus(){
        return ultimo_sensor_status;
    }


    bool temDirecaoValida() const {
        return ultima_direcao_recebida;
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_robot_client;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr subscriber; 
    cg_interfaces::msg::RobotSensors ultimo_sensor_status;
    bool ultima_direcao_recebida;

    void topic_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg)
    {
        ultimo_sensor_status = *msg;
        ultima_direcao_recebida = true;
    }
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

bool eh_bloqueada(char direction, AlgoritmoParte2* no) {
    auto dado_do_sensor = no->getUltimoSensorStatus();

    switch(direction) {
        case 'u': 
            return dado_do_sensor.up == "b";
        case 'd': 
            return dado_do_sensor.down == "b";

        case 'l': 
            return dado_do_sensor.left == "b";

        case 'r': 
            return dado_do_sensor.right == "b";

        default:
            std::cerr << "Direção inválida\n";
            return true;   // considere inválido como bloqueado
    }
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
void encontra_caminho(celula detalhes_das_celulas[LINHA][COLUNA], Par target, AlgoritmoParte2* no)
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
        no->move(p);
    }

    rclcpp::sleep_for(300ms);
    rclcpp::spin_some(no->get_node_base_interface());

    return;
}

void busca_por_a_estrela (Par robo, Par target, AlgoritmoParte2* no) {
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

        Par lugar = make_pair(i,j);

        if (!achou_o_target) {
            no->reset();
            rclcpp::sleep_for(1000ms);
            encontra_caminho(detalhes_das_celulas, lugar, no);
            rclcpp::sleep_for(100ms);
        }

        double gNovo, hNovo, fNovo;

        //Primeiro Sucessor: cima
        if (eh_valida(i - 1, j)) {
            
            if (eh_target(i - 1, j, target)) {
                detalhes_das_celulas[i - 1][j].pai_x = i;
                detalhes_das_celulas[i - 1][j].pai_y = j;
                detalhes_das_celulas[i - 1][j].comando = 'u';
                printf("Encontramos o target\n");
                achou_o_target = true;
                no->reset();
                rclcpp::sleep_for(3000ms);
                encontra_caminho(detalhes_das_celulas, target, no);
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i - 1][j] == false
                     && !eh_bloqueada('u', no)) {
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
                detalhes_das_celulas[i + 1][j].comando = 'd';
                printf("Encontramos o target\n");
                achou_o_target = true;
                no->reset();
                rclcpp::sleep_for(3000ms);
                encontra_caminho(detalhes_das_celulas, target, no);
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i + 1][j] == false
                     && !eh_bloqueada('d', no)) {
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
                detalhes_das_celulas[i][j- 1].comando = 'l';
                printf("Encontramos o target\n");
                achou_o_target = true;
                no->reset();
                rclcpp::sleep_for(3000ms);
                encontra_caminho(detalhes_das_celulas, target, no);
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i][j - 1] == false
                     && !eh_bloqueada('l', no)) {
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
                detalhes_das_celulas[i][j+ 1].comando = 'r';
                printf("Encontramos o target\n");
                achou_o_target = true;
                no->reset();
                rclcpp::sleep_for(3000ms);
                encontra_caminho(detalhes_das_celulas, target, no);
                return;
            }
            //Se não está na lista fechada e não é bloqueada
            else if (lista_fechada[i][j + 1] == false
                     && !eh_bloqueada('r', no)) {
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

    no->reset();
    rclcpp::sleep_for(3000ms);
    return;
}

int main(int argc, char **argv) {    
    rclcpp::init(argc, argv);

    auto no = std::make_shared<AlgoritmoParte2>();

    // Espera até receber a primeira mensagem do sensor
    while (rclcpp::ok() && !no->temDirecaoValida()) {
        rclcpp::spin_some(no);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    Par robo = make_pair(1,1);
    Par target = make_pair(14,14);

    busca_por_a_estrela(robo, target, no.get());

    rclcpp::shutdown();

    return (0);
}
