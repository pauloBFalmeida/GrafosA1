class Graph:
    def __init__(self, arquivo):
        file = open(arquivo, 'r')
        linhaInicial = file.readline().split()
        nVertices = int(linhaInicial[1])
        self.V = nVertices
        self.E = 0
        self.adj = []
        for _ in range(self.V + 1):
            self.adj.append([float("inf")] * (self.V + 1))
        self.rotulos = {}

        for i in range(nVertices):
            linha = file.readline().split()
            numero = int(linha[0])
            rotulo = linha[1]
            self.addVertice(numero, rotulo)
        linhaEdges = file.readline()
        while True:
            try:
                linha = file.readline()
                if (linha == ""): break
                a, b, peso = linha.split()
                a, b = map(int, [a, b])
                peso = float(peso)
                self.addEdge(a, b, peso)
            except EOFError:
                break
        file.close()


    def addVertice(self, num, rotulo):
        self.rotulos[num] = rotulo

    def addEdge(self, a, b, peso):
        self.adj[a][b] = peso
        self.adj[b][a] = peso
        self.E += 1

    def qtdVertices(self):
        return self.V

    def qtdArestas(self):
        return self.E

    def grau(self, v):
        grau = 0
        for u in range(1, self.V + 1):
            if (self.adj[v][u] < float("inf")):
                grau += 1
        return grau

    def rotulo(self, v):
        return self.rotulos[v]

    def vizinhos(self, v):
        vizinhos = []
        for u in range(1, self.V + 1):
            if (self.adj[v][u] < float("inf")):
                vizinhos.append(u)
        return vizinhos

    def haAresta(self, u, v):
        if (self.adj[u][v] < float("inf")):
            return True
        return False

    def peso(self, u, v):
        return self.adj[u][v]

    # ExErCiCiO DoIs
    def BFS(self, source):
        visited = [False] * (self.V + 1)
        distances = [-1] * (self.V + 1)
        parents = [None] * (self.V + 1)
        visited[source] = True
        distances[source] = 0
        queue = []
        niveis = {0 : [source], }
        queue.append(source)
        while (len(queue) != 0):
            u = queue.pop(0)
            vizinhos = self.vizinhos(u)
            for v in vizinhos:
                if not visited[v]:
                    visited[v] = True
                    distances[v] = distances[u] + 1
                    if (distances[u] + 1 not in niveis.keys()):
                        niveis[distances[u] + 1] = [v]
                    else:
                        niveis[distances[u] + 1].append(v)
                    parents[v] = u
                    queue.append(v)
        for i in sorted(niveis.keys()):
            result = str(i) + ": "
            first = True
            for v in niveis[i]:
                if (first):
                    result += str(v)
                    first = False
                else:
                    result += "," + str(v)
            print(result)



# ler objeto
nome_do_arquivo = input()
grafo = Graph(nome_do_arquivo)

# print(grafo.qtdArestas())
# print(grafo.rotulos)
print(grafo.BFS(1))
# print(grafo.qtdArestas)
