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
    # def BFS(self, source):
    #     visited = [False] * (self.V + 1)
    #     distances = [-1] * (self.V + 1)
    #     parents = [None] * (self.V + 1)
    #     visited[source] = True
    #     distances[source] = 0
    #     queue = []
    #     niveis = {0 : [source], }
    #     queue.append(source)
    #     while (len(queue) != 0):
    #         u = queue.pop(0)
    #         vizinhos = self.vizinhos(u)
    #         for v in vizinhos:
    #             if not visited[v]:
    #                 visited[v] = True
    #                 distances[v] = distances[u] + 1
    #                 if (distances[u] + 1 not in niveis.keys()):
    #                     niveis[distances[u] + 1] = [v]
    #                 else:
    #                     niveis[distances[u] + 1].append(v)
    #                 parents[v] = u
    #                 queue.append(v)
    #     for i in sorted(niveis.keys()):
    #         result = str(i) + ": "
    #         first = True
    #         for v in niveis[i]:
    #             if (first):
    #                 result += str(v)
    #                 first = False
    #             else:
    #                 result += "," + str(v)
    #         print(result)


    def menor(self, distancias, visitados):
        max = float("Inf")
        max_v = -1
        for i in range(len(distancias)):
            if ((not visitados[i]) and distancias[i] < max):
                max = distancias[i]
                max_v = i
        return max_v


    def dijkstra(self, s):

        ancestrais = [None] * (self.V + 1)
        visitados = [False] * (self.V + 1)
        distancias = [float ("Inf")] * (self.V + 1)
        distancias[s] = 0

        while (True):
            v = self.menor(distancias, visitados)
            if (v == -1):
                break
            visitados[v] = True

            for u in self.vizinhos(v):
                if (distancias[u] > distancias[v] + self.peso(v, u)):
                    distancias[u] = distancias[v] + self.peso(v, u)
                    ancestrais[u] = v

        # printa as informações
        for i in range(1, self.V + 1):
            linha = str(i) + "; d=" + str(distancias[i])
            vertice = i

            while (ancestrais[vertice] != None):
                vertice = ancestrais[vertice]
                linha = str(vertice) + "," + linha
            linha = str(i) + ": " + linha

            print(linha)

    def floydWarshall(self):
        # vamos usar apenas duas matrizes para o problema
        matriz0 = []
        matriz1 = []
        for _ in range(self.V + 1):
            matriz0.append([float("inf")] * (self.V + 1))
            matriz1.append([float("inf")] * (self.V + 1))
        for i in range(len(matriz0)):
            for j in range(len(matriz0)):
                if (i == j):
                    matriz0[i][j] = 0
                else:
                    matriz0[i][j] = self.adj[i][j]
        distancias = [matriz0, matriz1]
        for k in range(1, self.V + 1):
            m = k % 2
            for u in range(1, self.V + 1):
                for v in range(1, self.V + 1):
                    distancias[m][u][v] = min(distancias[1-m][u][v],
                                          distancias[1-m][u][k] + distancias[1-m][k][v])
        m_res = self.V % 2
        resposta = ""
        for s in range(1, self.V + 1):
            resposta = str(s) + ":"
            for t in range(1, self.V + 1):
                if (t == self.V):
                    resposta = resposta + str(distancias[m_res][s][t])
                else:
                    resposta = resposta + str(distancias[m_res][s][t]) + ", "
            print(resposta)





# ler objeto
nome_do_arquivo = input()
grafo = Graph(nome_do_arquivo)
grafo.dijkstra(2)
grafo.floydWarshall()
# print (grafo.adj)
# print(grafo.qtdArestas())
# print(grafo.rotulos)
# print(grafo.BFS(1))
# print(grafo.qtdArestas)
