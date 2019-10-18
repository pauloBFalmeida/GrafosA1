import operator

class Graph:
    def __init__(self, arquivo):
        file = open(arquivo, 'r')
        linhaInicial = file.readline().split()
        nVertices = int(linhaInicial[1])
        self.V = nVertices
        self.E = 0
        self.v_adj = {}
        for i in range(1, self.V + 1):
            self.v_adj[i] = []
        self.arestas = {}
        self.rotulos = {}

        for i in range(nVertices):
            linha = file.readline().split(' ', 1)
            numero = int(linha[0])
            rotulo = linha[1][:-1]
            self.addVertice(numero, rotulo)
        linhaEdges = file.readline()
        arcos = False
        if (linhaEdges == "*arcs"):
            arcos = True
        if (linhaEdges == "*edges"):
            arcos = False
        while True:
            try:
                linha = file.readline()
                if (linha == ""): break
                a, b, peso = linha.split()
                a, b = map(int, [a, b])
                peso = float(peso)
                if (arcos):
                    self.addArco(a, b, peso)
                else:
                    self.addAresta(a, b, peso)
            except EOFError:
                break
        file.close()


    def addVertice(self, num, rotulo):
        self.rotulos[num] = rotulo

    def addAresta(self, a, b, peso):
        self.v_adj[a].append(b)
        self.v_adj[b].append(a)
        self.arestas[(a, b)] = peso
        self.arestas[(b, a)] = peso
        self.E += 1

    def addArco(self, a, b, peso):
        self.v_adj[a].append(b)
        self.arestas[(a, b)] = peso
        self.E += 1

    def qtdVertices(self):
        return self.V

    def qtdArestas(self):
        return self.E

    def grau(self, v):
        return len(self.v_adj[v])

    def rotulo(self, v):
        return self.rotulos[v]

    def vizinhos(self, v):
        return self.v_adj[v].copy()

    def haAresta(self, u, v):
        if (v in self.v_adj[u]):
            return True
        return False

    def peso(self, u, v):
        if (v in self.v_adj[u]):
            return self.arestas[(u, v)]
        return (float("inf"))

    tempo = 0

    def dfs(self):
        visitado = [False] * (self.V + 1)
        tempo_entrada = [(float("inf"))] * (self.V + 1)
        tempo_saida = [(float("inf"))] * (self.V + 1)
        ancestrais = [None] * (self.V + 1)

        tempo = 0;

        for u in range(1, self.V + 1):
            if not visitado[u]:
                self.dfs_visit(u, visitado, tempo_entrada, ancestrais, tempo_saida)

        return visitado, tempo_entrada, ancestrais, tempo_saida

    def dfs_visit(self, v, visitado, tempo_entrada, ancestrais, tempo_saida):
        visitado[v] = True
        tempo += 1
        tempo_entrada[v] = tempo

        for u in vizinhos(v):
            if not visitado[u]:
                ancestrais[u] = v
                self.dfs_visit(u, visitado, tempo_entrada, ancestrais, tempo_saida)

        tempo += 1
        tempo_saida = tempo


    def find(self, raizes, i):
        if (raizes[i] == i):
            return i
        raizes[i] = self.find(raizes, raizes[i])
        return raizes[i]

    def union(self, raizes, rank, xs, ys):
        x = self.find(raizes, xs)
        y = self.find(raizes, ys)
        if (rank[x] > rank[y]):
            raizes[y] = x
        elif (rank[x] < rank[y]):
            raizes[x] = y
        else:
            raizes[x] = y
            rank[y] = rank[y] + 1

    def kruskal(self):
        raizes = [i for i in range(self.V + 1)]
        weight = 0;
        rank = [0] * (self.V + 1)

        arv_min = []
        arestas_ord = sorted(self.arestas.items(), key=operator.itemgetter(1))
        print(arestas_ord)

        for i in range(len(arestas_ord)):
            a, b = arestas_ord[i][0]
            if (self.find(raizes, a) == self.find(raizes, b)):
                # print ("Ciclo detec: " + str(a) + " " + str(b))
                continue
            arv_min.append((a,b));
            weight += arestas_ord[i][1]
            self.union(raizes, rank, a, b)

        return arv_min, weight


# ler objeto
nome_do_arquivo = input()
grafo = Graph(nome_do_arquivo)

arv, w = grafo.kruskal()
print ("Arvore: " + str(arv))
print ("Peso: " + str(w))
