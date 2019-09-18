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

    def bfs(self, s):
        visitados = [False] * (self.V + 1)
        distancia = [(float("inf"))] * (self.V + 1)

        visitados[s] = True
        distancia[s] = 0

        fila = []
        fila.append(s)

        linha = "0: " + str(s) + ","
        nivel_atual = 0
        while (len(fila) != 0):
            u = fila.pop(0)
            for v in self.vizinhos(u):
                if (not visitados[v]):
                    visitados[v] = True
                    distancia[v] = distancia[u] + 1
                    if (distancia[v] > nivel_atual):
                        nivel_atual += 1
                        print(linha[:-1])
                        linha = str(nivel_atual) + ": "
                    linha = linha + str(v) + ","
                    fila.append(v)
        print(linha[:-1])

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

    def hierholzer(self):
        arestas = {}

        for i in range(1, self.V + 1):
            arestas[i] = self.vizinhos(i)

        n_impares = 0
        ultimo_impar = 1
        for i in range(1, self.V + 1):
            if (len(arestas[i]) % 2 != 0):
                n_impares += 1
                ultimo_impar = i

        if (n_impares > 2 or n_impares == 1):
            print (0)
            return

        caminho_atual = []

        caminho_atual.append(ultimo_impar)
        v_atual = ultimo_impar

        ciclo = []

        while (len(caminho_atual) != 0):
            if (len(arestas[v_atual]) > 0):
                caminho_atual.append(v_atual)

                v_prox = arestas[v_atual].pop(0)

                arestas[v_prox].remove(v_atual)

                v_atual = v_prox
            else:
                ciclo.append(v_atual)

                v_atual = caminho_atual.pop()

        resultado = ""
        for i in range(len(ciclo)-1, -1, -1):
            if (i != 0):
                resultado = resultado + str(ciclo[i]) + ","
            else:
                resultado = resultado + str(ciclo[i])
        print (resultado)


    def floydWarshall(self):
        # vamos usar apenas duas matrizes para o problema
        matriz0 = []
        matriz1 = []
        for _ in range(self.V + 1):
            matriz0.append([float("inf")] * (self.V + 1))
            matriz1.append([float("inf")] * (self.V + 1))

        # inicializamos a primeira matriz tal como a função W
        for i in range(len(matriz0)):
            for j in range(len(matriz0)):
                if (i == j):
                    matriz0[i][j] = 0
                else:
                    matriz0[i][j] = self.adj[i][j]
        distancias = [matriz0, matriz1]

        # fazemos o loop principal de Floyd Warshall
        for k in range(1, self.V + 1):
            m = k % 2
            for u in range(1, self.V + 1):
                for v in range(1, self.V + 1):
                    distancias[m][u][v] = min(distancias[1-m][u][v],
                                          distancias[1-m][u][k] + distancias[1-m][k][v])

        # a matriz com as respostas é a m_res
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
grafo.bfs(4)
# grafo.dijkstra(2)
# grafo.floydWarshall()
# grafo.hierholzer()
# print (grafo.adj)
# print(grafo.qtdArestas())
# print(grafo.rotulos)
# print(grafo.BFS(1))
# print(grafo.qtdArestas)
