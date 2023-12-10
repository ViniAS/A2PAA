# A2PAA

Trabalho da disciplina Análise e Projeto de Algoritmos do curso de 
Ciência de Dado e Inteligência Artificial (CDIA) da FGV EMAp.

## Como compilar e executar

Primeiro é preciso que você tenha instalado o [CMake](https://cmake.org/) (versão mínima 3.11) em seu sistema.

É recomendável criar uma nova pasta para a compilação do projeto, para isso, execute o seguinte comando:

```bash
mkdir build
cd build
```

Agora, dentro da pasta build, execute o seguinte comando para gerar os arquivos de compilação:

```bash
cmake ..
```

Por fim, execute o seguinte comando para compilar o projeto:

```bash
make
```

Se a compilação for bem sucedida, três executáveis serão gerados. A2PAA é a interface ASCII para interagir com o CityGraph, A2PAA_chart gera os arquivos csvs com tempos de execução e A2PAA_test para testar os algoritmos. Para rodar os arquivos basta utilizar os seguintes comandos

```bash
./A2PAA
```
```bash
./A2PAA_chart
```
```bash
./A2PAA_test
```


