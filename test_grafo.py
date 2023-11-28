import unittest
from unittest.mock import patch
from io import StringIO
import sys
import time

from main import Grafo

class TestGrafoMethods(unittest.TestCase):

    def setUp(self):
        self.held_output = None
        self.num_vertices = 5
        self.grafo = Grafo(self.num_vertices)
        self.redirect_output()

    def redirect_output(self):
        self.held_output = StringIO()
        sys.stdout = self.held_output

    def release_output(self):
        output = self.held_output.getvalue().strip()
        self.held_output.close()
        sys.stdout = sys.__stdout__

        return output

    def test_cria_aresta(self):
        self.grafo.cria_aresta(1, 2)
        output = self.release_output()
        self.assertIn("Aresta entre 1 e 2 criada.", output)

    def test_remove_aresta(self):
        self.grafo.cria_aresta(1, 2)
        self.grafo.remove_aresta(1, 2)
        output = self.release_output()
        self.assertIn("Aresta entre 1 e 2 removida.", output)

    def test_pondera_rotula_vertice(self):
        self.grafo.pondera_rotula_vertice(1, 3.5, "A")
        output = self.release_output()
        self.assertIn("Vértice 1 ponderado com peso 3.5 e rotulado como 'A'.", output)

    def test_pondera_rotula_aresta(self):
        self.grafo.cria_aresta(1, 2)
        self.grafo.pondera_rotula_aresta(1, 2, 2.0, "B")
        output = self.release_output()
        self.assertIn("Aresta entre 1 e 2 ponderada com peso 2.0 e rotulada como 'B'.", output)

    def test_checa_adjacencia_aresta(self):
        self.grafo.cria_aresta(1, 2)
        self.assertTrue(self.grafo.checa_adjacencia_aresta(1, 2))

    def test_checa_grafo_vazio(self):
        self.assertTrue(self.grafo.checa_grafo_vazio())

    def test_checa_grafo_completo(self):
        self.grafo.cria_aresta(0, 1)
        self.grafo.cria_aresta(0, 2)
        self.grafo.cria_aresta(0, 3)
        self.grafo.cria_aresta(0, 4)
        self.grafo.cria_aresta(1, 2)
        self.grafo.cria_aresta(1, 3)
        self.grafo.cria_aresta(1, 4)
        self.grafo.cria_aresta(2, 3)
        self.grafo.cria_aresta(2, 4)
        self.grafo.cria_aresta(3, 4)

        self.assertTrue(self.grafo.checa_grafo_completo())

    def test_checa_quantidade_vertices_arestas(self):
        vertices, arestas = self.grafo.checa_quantidade_vertices_arestas()
        self.assertEqual(vertices, self.num_vertices)
        self.assertEqual(arestas, 0)

    def test_checa_adjacencia_aresta_invalida(self):
        self.assertFalse(self.grafo.checa_adjacencia_aresta(1, 6))

    def test_checa_grafo_nao_vazio(self):
        self.grafo.cria_aresta(0, 1)
        self.assertFalse(self.grafo.checa_grafo_vazio())

    def test_checa_grafo_nao_completo(self):
        self.grafo.cria_aresta(0, 1)
        self.assertFalse(self.grafo.checa_grafo_completo())

    def test_checa_adjacencia_vertice(self):
        self.grafo.cria_aresta(1, 2)
        self.assertTrue(self.grafo.checa_adjacencia_vertice(1, 2))

    def test_checa_adjacencia_vertice_inexistente(self):
        self.assertFalse(self.grafo.checa_adjacencia_vertice(1, 3))

    def test_checa_adjacencia_aresta(self):
        self.grafo.cria_aresta(1, 2)
        self.assertTrue(self.grafo.checa_adjacencia_aresta(1, 2))

    def test_checa_adjacencia_aresta_inexistente(self):
        self.assertFalse(self.grafo.checa_adjacencia_aresta(1, 3))

    def test_remove_aresta_inexistente(self):
        self.grafo.remove_aresta(1, 2)
        output = self.release_output()
        self.assertIn("Aresta entre 1 e 2 não existe.", output)

    def test_pondera_rotula_vertice_invalido(self):
        self.grafo.pondera_rotula_vertice(6, 3.5, "A")
        output = self.release_output()
        self.assertIn("Vértice fora da faixa válida.", output)

    def test_pondera_rotula_aresta_invalida(self):
        self.grafo.pondera_rotula_aresta(1, 6, 2.0, "B")
        output = self.release_output()
        self.assertIn("Vértice fora da faixa válida.", output)

    def test_exibe_grafo(self):
        self.grafo.cria_aresta(1, 2)
        self.grafo.exibe_grafo()
        output = self.release_output()
        self.assertIn("Matriz de Adjacência:", output)
        self.assertIn("Lista de Adjacência:", output)

    def test_checa_adjacencia_vertice_a_si_mesmo(self):
        self.assertFalse(self.grafo.checa_adjacencia_vertice(1, 1))

    def test_checa_adjacencia_aresta_a_si_mesma(self):
        self.assertFalse(self.grafo.checa_adjacencia_aresta(1, 1))

    def test_aresta_com_peso_e_rotulo(self):
        self.grafo.cria_aresta(1, 2)
        self.grafo.pondera_rotula_aresta(1, 2, 2.0, "B")
        output = self.release_output()
        self.assertIn("Aresta entre 1 e 2 ponderada com peso 2.0 e rotulada como 'B'.", output)

    def test_vertice_com_peso_e_rotulo(self):
        self.grafo.pondera_rotula_vertice(1, 3.5, "A")
        output = self.release_output()
        self.assertIn("Vértice 1 ponderado com peso 3.5 e rotulado como 'A'.", output)

    def test_aresta_invalida_com_peso_e_rotulo(self):
        self.grafo.pondera_rotula_aresta(1, 6, 2.0, "B")
        output = self.release_output()
        self.assertIn("Vértice fora da faixa válida.", output)

    def test_vertice_invalido_com_peso_e_rotulo(self):
        self.grafo.pondera_rotula_vertice(6, 3.5, "A")
        output = self.release_output()
        self.assertIn("Vértice fora da faixa válida.", output)

    def test_saida_padrao_casos_extras(self):
        self.grafo.cria_aresta(1, 2)
        self.grafo.pondera_rotula_aresta(1, 2, 2.0, "B")
        self.grafo.pondera_rotula_vertice(1, 3.5, "A")
        self.grafo.exibe_grafo()
        output = self.release_output()
        self.assertIn("Aresta entre 1 e 2 criada.", output)
        self.assertIn("Aresta entre 1 e 2 ponderada com peso 2.0 e rotulada como 'B'.", output)
        self.assertIn("Vértice 1 ponderado com peso 3.5 e rotulado como 'A'.", output)

if __name__ == '__main__':
    unittest.main()
