import time
import networkx as nx
from networkx.readwrite import graphml
import matplotlib.pyplot as plt

class Grafo:
    def __init__(self, vertices):
        self.vertices = vertices
        self.matriz_adjacencia = [[0] * vertices for _ in range(vertices)]
        self.lista_adjacencia = {i: set() for i in range(vertices)}
        self.pesos_arestas = {}
        self.rotulos_arestas = {}
        self.pesos_vertices = {}
        self.rotulos_vertices = {}

    def cria_aresta(self, vertice1, vertice2):
        if not self._verifica_vertice(vertice1) or not self._verifica_vertice(vertice2):
            return

        self.matriz_adjacencia[vertice1][vertice2] = 1
        self.matriz_adjacencia[vertice2][vertice1] = 1
        self.lista_adjacencia[vertice1].add(vertice2)
        self.lista_adjacencia[vertice2].add(vertice1)
        print(f"Aresta entre {vertice1} e {vertice2} criada.")
        self.exibe_grafo()

    def remove_aresta(self, vertice1, vertice2):
        if not self._verifica_vertice(vertice1) or not self._verifica_vertice(vertice2):
            return

        if self.matriz_adjacencia[vertice1][vertice2] == 1:
            self.matriz_adjacencia[vertice1][vertice2] = 0
            self.matriz_adjacencia[vertice2][vertice1] = 0
            self.lista_adjacencia[vertice1].remove(vertice2)
            self.lista_adjacencia[vertice2].remove(vertice1)
            self.pesos_arestas.pop((vertice1, vertice2), None)
            self.rotulos_arestas.pop((vertice1, vertice2), None)
            print(f"Aresta entre {vertice1} e {vertice2} removida.")
        else:
            print(f"Aresta entre {vertice1} e {vertice2} não existe.")
        self.exibe_grafo()

    def _verifica_vertice(self, vertice):
        if 0 <= vertice < self.vertices:
            return True
        print("Vértice fora da faixa válida.")
        return False

    def pondera_rotula_vertice(self, vertice, peso, rotulo):
        if not self._verifica_vertice(vertice):
            return

        self.pesos_vertices[vertice] = peso
        self.rotulos_vertices[vertice] = rotulo
        print(f"Vértice {vertice} ponderado com peso {peso} e rotulado como '{rotulo}'.")

    def checa_adjacencia_vertice(self, vertice1, vertice2):
        if not self._verifica_vertice(vertice1) or not self._verifica_vertice(vertice2):
            return False

        resultado = self.matriz_adjacencia[vertice1][vertice2] == 1
        print(f"Vértice {vertice1} é adjacente a {vertice2}.") if resultado else print(f"Vértice {vertice1} não é adjacente a {vertice2}.")
        return resultado

    def pondera_rotula_aresta(self, vertice1, vertice2, peso, rotulo):
        if 0 <= vertice1 < self.vertices and 0 <= vertice2 < self.vertices:
            if self.matriz_adjacencia[vertice1][vertice2] == 1:
                self.pesos_arestas[(vertice1, vertice2)] = peso
                self.rotulos_arestas[(vertice1, vertice2)] = rotulo
                print(f"Aresta entre {vertice1} e {vertice2} ponderada com peso {peso} e rotulada como '{rotulo}'.")
            else:
                print("Aresta não existe.")
        else:
            print("Vértice fora da faixa válida.")
        self.exibe_grafo()

    def checa_adjacencia_aresta(self, vertice1, vertice2):
        if 0 <= vertice1 < self.vertices and 0 <= vertice2 < self.vertices:
            resultado = self.matriz_adjacencia[vertice1][vertice2] == 1
            print(f"Aresta entre {vertice1} e {vertice2} é adjacente.") if resultado else print(f"Aresta entre {vertice1} e {vertice2} não é adjacente.")
            return resultado
        else:
            print("Vértice fora da faixa válida.")
            return False

    def checa_quantidade_vertices_arestas(self):
        num_arestas = sum(sum(row) for row in self.matriz_adjacencia) // 2
        vertices, arestas = self.vertices, num_arestas
        print(f"O grafo tem {vertices} vértices e {arestas} arestas.")
        return vertices, arestas

    def checa_grafo_vazio(self):
        resultado = all(all(cell == 0 for cell in row) for row in self.matriz_adjacencia)
        print("O grafo está vazio.") if resultado else print("O grafo não está vazio.")
        return resultado

    def checa_grafo_completo(self):
        completo = all(sum(row) == self.vertices - 1 for row in self.matriz_adjacencia)
        reflexivo = all(self.matriz_adjacencia[i][i] == 0 for i in range(self.vertices))
        resultado = completo and reflexivo
        print("O grafo é completo.") if resultado else print("O grafo não é completo.")
        return resultado

    def exibe_grafo(self):
        print("\nMatriz de Adjacência:")
        for row in self.matriz_adjacencia:
            print(row)

        print("\nLista de Adjacência:")
        for vertice, vizinhos in self.lista_adjacencia.items():
            print(f"{vertice}: {list(vizinhos)}")

        print("\nPesos das Arestas:")
        for aresta, peso in self.pesos_arestas.items():
            vertice1, vertice2 = aresta
            print(f"Aresta entre {vertice1} e {vertice2}: Peso {peso}")

        print("\nRótulos das Arestas:")
        for aresta, rotulo in self.rotulos_arestas.items():
            vertice1, vertice2 = aresta
            print(f"Aresta entre {vertice1} e {vertice2}: Rótulo {rotulo}")

        print("\nPesos dos Vértices:")
        for vertice, peso in self.pesos_vertices.items():
            print(f"Vértice {vertice}: Peso {peso}")

        print("\nRótulos dos Vértices:")
        for vertice, rotulo in self.rotulos_vertices.items():
            print(f"Vértice {vertice}: Rótulo {rotulo}")

    def salvar_grafo_graphml(self, nome_arquivo):
        G = nx.Graph()

        G.add_nodes_from(range(self.vertices))

        for aresta in self.lista_adjacencia.keys():
            vertice1 = aresta
            for vertice2 in self.lista_adjacencia[aresta]:
                peso = self.pesos_arestas.get((vertice1, vertice2))
                rotulo_aresta = self.rotulos_arestas.get((vertice1, vertice2), "")

                if peso is not None:
                    G.add_edge(vertice1, vertice2, weight=peso, rotulo_aresta=rotulo_aresta)
                else:
                    G.add_edge(vertice1, vertice2, rotulo_aresta=rotulo_aresta)

        nx.set_node_attributes(G, self.rotulos_vertices, 'rotulo_vertice')

        graphml.write_graphml(G, nome_arquivo + ".graphml")
        pos = nx.spring_layout(G)
        nx.draw(G, pos, with_labels=True, node_size=700, node_color="skyblue", font_size=8, font_color="black", font_weight="bold", edge_color="gray", linewidths=0.5, arrowsize=10)

        plt.savefig(f"{nome_arquivo}_grafo.png")
        plt.show()



    def carregar_grafo_graphml(self, nome_arquivo):
        G = graphml.read_graphml(nome_arquivo)
        self.vertices = len(G.nodes)
        self.matriz_adjacencia = [[0] * self.vertices for _ in range(self.vertices)]
        self.lista_adjacencia = {i: set() for i in range(self.vertices)}
        self.pesos_arestas = {}
        self.rotulos_arestas = {}
        self.pesos_vertices = {}
        self.rotulos_vertices = {}

        for vertice1, vertice2, data in G.edges(data=True):
            vertice1, vertice2 = int(vertice1), int(vertice2)
            peso = data.get('weight', '')
            self.cria_aresta(vertice1, vertice2)
            self.pesos_arestas[(vertice1, vertice2)] = peso

            rotulo_aresta = data.get('rotulo_aresta', '')
            self.rotulos_arestas[(vertice1, vertice2)] = rotulo_aresta

        for vertice, data in G.nodes(data=True):
            vertice = int(vertice)
            peso_vertice = data.get('weight', '')
            self.pondera_rotula_vertice(vertice, peso_vertice, '')

            rotulo_vertice = data.get('rotulo_vertice', '')
            self.rotulos_vertices[vertice] = rotulo_vertice

        print(f"Grafo carregado de {nome_arquivo} no formato GraphML.")

        G = graphml.read_graphml(nome_arquivo)
        self.vertices = len(G.nodes)
        self.matriz_adjacencia = [[0] * self.vertices for _ in range(self.vertices)]
        self.lista_adjacencia = {i: set() for i in range(self.vertices)}
        self.pesos_arestas = {}
        self.rotulos_arestas = {}
        self.pesos_vertices = {}
        self.rotulos_vertices = {}

        for vertice1, vertice2, data in G.edges(data=True):
            vertice1, vertice2 = int(vertice1), int(vertice2)
            peso = data.get('weight', 1.0)
            self.cria_aresta(vertice1, vertice2)
            self.pesos_arestas[(vertice1, vertice2)] = peso

        for vertice, data in G.nodes(data=True):
            vertice = int(vertice)
            peso = data.get('weight', 1.0)
            self.pondera_rotula_vertice(vertice, peso, '')

        print(f"Grafo carregado de {nome_arquivo} no formato GraphML.")

def ler_entrada_int(mensagem):
    while True:
        try:
            return int(input(mensagem))
        except ValueError:
            print("Por favor, insira um número inteiro válido.")

def ler_entrada_float(mensagem):
    while True:
        try:
            return float(input(mensagem))
        except ValueError:
            print("Por favor, insira um número válido.")

def menu():
    print("\nEscolha uma opção:")
    print("1. Criar aresta")
    print("2. Remover aresta")
    print("3. Ponderar e rotular vértice")
    print("4. Ponderar e rotular aresta")
    print("5. Checar adjacência entre vértices")
    print("6. Checar adjacência entre arestas")
    print("7. Checar quantidade de vértices e arestas")
    print("8. Checar se o grafo está vazio")
    print("9. Checar se o grafo é completo")
    print("10. Exibir grafo")
    print("11. Salvar grafo em GraphML")
    print("12. Carregar grafo em GraphML")
    print("0. Sair")

if __name__ == "__main__":
    num_vertices = ler_entrada_int("Digite o número de vértices do grafo: ")
    grafo = Grafo(num_vertices)

    while True:
        menu()

        try:
            escolha = ler_entrada_int("Digite o número da opção desejada: ")
        except ValueError:
            print("Por favor, insira um número inteiro válido.")
            continue

        if escolha == 1:
            try:
                vertice1 = ler_entrada_int("Digite o primeiro vértice: ")
                vertice2 = ler_entrada_int("Digite o segundo vértice: ")
            except ValueError:
                print("Por favor, insira números inteiros válidos.")
                continue
            grafo.cria_aresta(vertice1, vertice2)
        elif escolha == 2:
            try:
                vertice1 = ler_entrada_int("Digite o primeiro vértice: ")
                vertice2 = ler_entrada_int("Digite o segundo vértice: ")
            except ValueError:
                print("Por favor, insira números inteiros válidos.")
                continue
            grafo.remove_aresta(vertice1, vertice2)
        elif escolha == 3:
            try:
                vertice = ler_entrada_int("Digite o vértice a ser ponderado: ")
                peso = ler_entrada_float("Digite o peso do vértice: ")
                rotulo = input("Digite o rótulo do vértice: ")
            except ValueError:
                print("Por favor, insira números e texto válidos.")
                continue
            grafo.pondera_rotula_vertice(vertice, peso, rotulo)
        elif escolha == 4:
            try:
                vertice1 = ler_entrada_int("Digite o primeiro vértice da aresta: ")
                vertice2 = ler_entrada_int("Digite o segundo vértice da aresta: ")
                peso = ler_entrada_float("Digite o peso da aresta: ")
                rotulo = input("Digite o rótulo da aresta: ")
            except ValueError:
                print("Por favor, insira números e texto válidos.")
                continue
            grafo.pondera_rotula_aresta(vertice1, vertice2, peso, rotulo)
        elif escolha == 5:
            try:
                vertice1 = ler_entrada_int("Digite o primeiro vértice: ")
                vertice2 = ler_entrada_int("Digite o segundo vértice: ")
            except ValueError:
                print("Por favor, insira números inteiros válidos.")
                continue
            grafo.checa_adjacencia_vertice(vertice1, vertice2)
        elif escolha == 6:
            try:
                vertice1 = ler_entrada_int("Digite o primeiro vértice: ")
                vertice2 = ler_entrada_int("Digite o segundo vértice: ")
            except ValueError:
                print("Por favor, insira números inteiros válidos.")
                continue
            grafo.checa_adjacencia_aresta(vertice1, vertice2)
        elif escolha == 7:
            grafo.checa_quantidade_vertices_arestas()
        elif escolha == 8:
            grafo.checa_grafo_vazio()
        elif escolha == 9:
            grafo.checa_grafo_completo()
        elif escolha == 10:
            grafo.exibe_grafo()
        elif escolha == 11:
            nome_arquivo = input("Digite o nome do arquivo para salvar o grafo (com extensão .graphml): ")
            grafo.salvar_grafo_graphml(nome_arquivo)
        elif escolha == 12:
            nome_arquivo = input("Digite o nome do arquivo para carregar o grafo (com extensão .graphml): ")
            grafo.carregar_grafo_graphml(nome_arquivo)

        time.sleep(3)
        if escolha == 0:
            break
