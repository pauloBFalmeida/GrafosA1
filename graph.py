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
        if (linhaEdges == "*arcs\n"):
            arcos = True
        if (linhaEdges == "*edges\n"):
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

    def bipartir(self, s):
        # conjuntos 0 e 1

        conjuntos = [-1] * (self.V + 1)

        fila = []
        fila.append(s)
        conjuntos[s] = 0

        while len(fila) != 0:
            v = fila.pop(0)

            for u in self.v_adj[v]:
                if conjuntos[u] == -1:
                    conjuntos[u] = 1 - conjuntos[v]
                    fila.append(u)
                elif conjuntos[u] == conjuntos[v]:
                    print ("Grafo não bipartido!")
                    return
        X = [i for i in range(1, self.V + 1) if conjuntos[i] == 0 or conjuntos[i] == -1]
        Y = [i for i in range(1, self.V + 1) if conjuntos[i] == 1]
        return X, Y



    def hopcroft_karp(self):
        dist = [float("inf")] * (self.V + 1)
        mate = [None] * (self.V + 1)

        m = 0
        X, Y = self.bipartir(1)
        while self.bfs_hk(X, mate, dist):
            #dividir o grafo em X
            for x in X:
                if mate[x] == None:
                    if self.dfs_hk(X, mate, x, dist):
                        m += 1

        # return m, mate
        print ("Emparelhamento máximo: " + str(m))

        str_arestas = ""
        for i in range(self.V + 1):
            if mate[i] != None and mate[i] > i:
                str_arestas += ("(" + str(i) + "," + str(mate[i]) + "),")
        print (str_arestas[:-1])

    def bfs_hk(self, X, mate, dist):
        fila = []

        for x in X:
            if mate[x] == None:
                dist[x] = 0
                fila.append(x)
            else:
                dist[x] = float("inf")
        dist[0] = float("inf")

        while len(fila) != 0:
            x = fila.pop(0)

            if (dist[x] < dist[0]):
                for y in self.v_adj[x]:
                    if mate[y] == None:
                        dist[0] = dist[x] + 1

                    elif dist[mate[y]] == float("inf"):
                        dist[mate[y]] = dist[x] + 1
                        fila.append(mate[y])
        return dist[0] != float("inf")

    def dfs_hk(self, X, mate, x, dist):
        if x != None:
            for y in self.v_adj[x]:
                if (mate[y] == None and dist[0] == dist[x] + 1) or dist[mate[y]] == dist[x] + 1:
                    if (self.dfs_hk(X, mate, mate[y], dist)):
                        mate[y] = x
                        mate[x] = y
                        return True
            dist[x] = float("inf")
            return False
        return True


    def edmonds_karp(self, s, t):
        rede_residual = {}
        rr_adj = {}
        for (u, v) in self.arestas.keys():
            rede_residual[(u, v)] = self.arestas[(u, v)]
            rede_residual[(v, u)] = 0
        # print (self.bfs_ek(2, 5, rede_residual))
        # print(rede_residual)
        p = self.bfs_ek(s, t, rede_residual)
        while (p != None):
            print(p)
            u = p[0]
            v = -1
            fluxo_total = float("inf")
            for i in range(1, len(p)):
                v = p[i]
                fluxo_total = min(fluxo_total, rede_residual[(u, v)])
                u = v

            print(fluxo_total)
            u = p[0]
            v = -1
            for i in range(1, len(p)):

                v = p[i]

                # print((u, v))
                # if (u, v) in self.arestas.keys():
                rede_residual[(u, v)] = rede_residual[(u, v)] - fluxo_total
                rede_residual[(v, u)] = rede_residual[(v, u)] + fluxo_total
                # else:
                    # rede_residual[(u, v)] = rede_residual[(u, v)] - fluxo_total
                    # rede_residual[(v, u)] = rede_residual[(v, u)] + fluxo_total
                u = v
            # p = None
            p = self.bfs_ek(s, t, rede_residual)
            # print (rede_residual)
        print(rede_residual)
        fluxo_total = 0
        for i in range(1, self.V + 1):
            if (t, i) in rede_residual.keys() and not (t, i) in self.arestas.keys():
                fluxo_total = fluxo_total + rede_residual[(t, i)]
        print(fluxo_total)

    def bfs_ek(self, s, t, rede_residual):
        visitados = [False] * (self.V + 1)
        ancestrais = [None] * (self.V + 1)

        visitados[s] = True
        fila = []

        fila.append(s)

        while len(fila) != 0:
            u = fila.pop(0)
            for v in range(1, self.V + 1):
                if (u, v) in rede_residual.keys() and rede_residual[(u, v)] > 0 and not visitados[v]:
                    visitados[v] = True
                    ancestrais[v] = u
                    if v == t:
                        p = [t]
                        w = t
                        while w != s:
                            w = ancestrais[w]
                            p.insert(0, w)
                        return p
                    fila.append(v)
        return None

    def all_subsets(self, n):
        if (n == 0):
            return [[]]
        conj = self.all_subsets(n-1)

        saida = []
        for i in range(len(conj)):
            saida.append(conj[i] + [n])
            saida.append(conj[i])
        return saida

    def ord_function(self, set):
        ind = 0
        for i in range(1, self.V+1):
            if i in set:
                ind += 2**(i-1)
        return ind


    def lawler(self):
        x = [0] * (2**(self.V))
        SS = sorted(self.all_subsets(self.V), key = len)
        # print(S)
        for S in SS:
            s = self.ord_function(S)
            x[s] = float("inf")
            #criar novo Grafo
            g_vert = S.copy()
            g_arest = []
            for u in S:
                for v in S:
                    if u != v and (u, v) in self.arestas.keys():
                        g_arest.append((u, v))


# ler objeto
nome_do_arquivo = input()
grafo = Graph(nome_do_arquivo)

# grafo.hopcroft_karp()
# s = int(input())
# t = int(input())
# grafo.edmonds_karp(s, t)

# print(grafo.all_subsets(t))
# grafo.lawler()
grafo.ord_function([])
while (True):
    l = list(map(int, input().split()))
    print(grafo.ord_function(l))
