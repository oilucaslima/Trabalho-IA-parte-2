#include "src/functions.h"

int main(){
    auto inicio3 = chrono::high_resolution_clock::now();
    call_astar();
    auto fim3 = chrono::high_resolution_clock::now();
    auto duracao3 = ::chrono::duration_cast<std::chrono::nanoseconds>(fim3 - inicio3);
    cout << "Tempo de execução - A Estrela: " << duracao3.count() << " ns" << endl;

    auto inicio = chrono::high_resolution_clock::now();
    call_greedy();
    auto fim = chrono::high_resolution_clock::now();
    auto duracao = ::chrono::duration_cast<std::chrono::nanoseconds>(fim - inicio);
    cout << "Tempo de execução - Guloso: " << duracao.count() << " ns" << endl;

    return 0;
}