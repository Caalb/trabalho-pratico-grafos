import sys
import timeit
import random
import networkx as nx
import matplotlib.pyplot as plt
from collections import defaultdict, deque

class GrafoDirecionado:
    def __init__(self):
        self.lista_adjacencia = defaultdict(list)
        self.vertices = 0

    def adiciona_vertice(self, vertice):
        if vertice not in self.lista_adjacencia:
            self.lista_adjacencia[vertice] = []

    def cria_aresta(self, vertice_origem, vertice_destino, peso=1):
        if(vertice_origem == vertice_destino):
            return

        self.adiciona_vertice(vertice_origem)
        self.adiciona_vertice(vertice_destino)
        self.lista_adjacencia[vertice_origem].append((vertice_destino, peso))

    def remove_aresta(self, vertice_origem, vertice_destino):
        self.lista_adjacencia[vertice_origem] = [(v, p) for v, p in self.lista_adjacencia[vertice_origem] if v != vertice_destino]

    def plotar_grafo(self, nome_arquivo):
        G = nx.DiGraph(self.lista_adjacencia)

        pos = nx.spring_layout(G)
        nx.draw(
            G,
            pos,
            with_labels=True,
            node_size=300,
            node_color="skyblue",
            font_size=5,
            font_color="black",
            font_weight="bold",
            edge_color="gray",
            linewidths=0.5,
            arrows=True,
            connectionstyle="arc3,rad=0.1"
        )

        plt.savefig(f"{nome_arquivo}_grafo.png")


    def dfs(self, vertice, visitados, parent, tempo, low, descoberto):
        visitados.add(vertice)
        tempo[0] += 1
        descoberto[vertice] = low[vertice] = tempo[0]
        for vizinho, _ in self.lista_adjacencia[vertice]:
            if vizinho not in visitados:
                parent[vizinho] = vertice
                self.dfs(vizinho, visitados, parent, tempo, low, descoberto)
                low[vertice] = min(low[vertice], low[vizinho])
            elif vizinho != parent[vertice]:
                low[vertice] = min(low[vertice], descoberto[vizinho])
            
    def dfs_inicial(self, vertice, visitados):
        visitados.add(vertice)
        for vizinho, _ in self.lista_adjacencia[vertice]:
            if vizinho not in visitados:
                self.dfs_inicial(vizinho, visitados)

    def esta_conectado_usando_visitados(self, _, visitados):
        if len(visitados) != len(self.lista_adjacencia):
            return False
        for v in visitados:
            if v not in self.lista_adjacencia or len(self.lista_adjacencia[v]) == 0:
                return False
        return True


    def identifica_pontes_naive(self):
        pontes = []
        visitados_inicial = set()
        self.dfs_inicial(next(iter(self.lista_adjacencia)), visitados_inicial)
        
        for v in self.lista_adjacencia:
            for u, _ in self.lista_adjacencia[v]:
                self.remove_aresta(v, u)
                visitados = visitados_inicial.copy()
                if not self.esta_conectado_usando_visitados(v, visitados):
                    pontes.append((v, u))
                self.cria_aresta(v, u)
        print("Pontes identificadas (naive):", len(pontes))
        return len(pontes)
    
    def identifica_pontes_tarjan(self):
        pontes = []
        visitados = set()
        parent = defaultdict(lambda: -1)
        tempo = [0]
        low = defaultdict(lambda: float("inf"))
        descoberto = defaultdict(lambda: float("inf"))
        for vertice in self.lista_adjacencia:
            if vertice not in visitados:
                self.dfs(vertice, visitados, parent, tempo, low, descoberto)

        for v in self.lista_adjacencia:
            for u, _ in self.lista_adjacencia[v]:
                if low[u] > descoberto[v]:
                    pontes.append((v, u))

        print("Pontes identificadas (Tarjan):", len(pontes))
        return len(pontes)

    def encontra_caminho_euleriano(self):
        if not self.checa_grafo_completo():
            print("O grafo não é euleriano.")
            return

        visitados = set()
        caminho = []

        def fleury(vertice):
            for vizinho, _ in self.lista_adjacencia[vertice]:
                aresta = (vertice, vizinho)
                if aresta in self.pesos_arestas:
                    self.remove_aresta(vertice, vizinho)
                    caminho.append(aresta)
                    fleury(vizinho)

        vertice_inicial = next(iter(self.lista_adjacencia.keys()))
        fleury(vertice_inicial)

        print("Caminho Euleriano:")
        for aresta in caminho:
            print(aresta)

    def checa_grafo_completo(self):
        return all(len(neighbors) % 2 == 0 for neighbors in self.lista_adjacencia.values())

    def gera_grafo_aleatorio(self, num_vertices, densidade_arestas=0.1):
        self.lista_adjacencia = defaultdict(list)
        self.vertices = num_vertices

        start_time = timeit.default_timer()

        for i in range(num_vertices):
            for j in range(i + 1, num_vertices):
                if random.random() < densidade_arestas / (num_vertices ** 0.5):
                    self.cria_aresta(i, j)

        elapsed_time = timeit.default_timer() - start_time
        print(f"Tempo para gerar grafo aleatório: {elapsed_time:.8f} segundos")

    def salvar_grafo_graphml(self, nome_arquivo):
        G = nx.DiGraph(self.lista_adjacencia)
        nx.write_graphml(G, f"{nome_arquivo}.graphml")

    def teste_tempos_computacionais(self):
        tamanhos_grafos = [100, 1000]
        for num_vertices in tamanhos_grafos:
            print(f"\nTestando para um grafo com {num_vertices} vértices:")

            self.gera_grafo_aleatorio(num_vertices)

            time_naive = timeit.timeit(self.identifica_pontes_naive, number=1)
            print(f"Tempo para identificar pontes (naive): {time_naive:.8f} segundos")

            time_tarjan = timeit.timeit(self.identifica_pontes_tarjan, number=1)
            print(f"Tempo para identificar pontes (Tarjan): {time_tarjan:.8f} segundos")

            time_fleury = timeit.timeit(self.encontra_caminho_euleriano, number=1)
            print(f"Tempo para encontrar caminho euleriano: {time_fleury:.8f} segundos")
            self.salvar_grafo_graphml(f"grafo_{num_vertices}")
            self.plotar_grafo(f"grafo_{num_vertices}")

if __name__ == "__main__":
    sys.setrecursionlimit(10**7)
    grafo = GrafoDirecionado()
    grafo.teste_tempos_computacionais()
