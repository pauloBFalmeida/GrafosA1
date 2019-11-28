import operator

class Graph:
    def __init__(self, arquivo):
        file = open(arquivo, 'r')
        linhaInicial = file.readline().split()
        if linhaInicial[0] == "vertices":
            nVertices = int(linhaInicial[1])
            self.V = nVertices
            self.E = 0
            self.v_adj = {}
            for i in range(1, self.V + 1):
                self.v_adj[i] = []
            self.arestas = {}
            self.rotulos = {}
            self.flow_source = 1
            self.flow_sink = self.V

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
        else:
            # lê o formato dos arquivos de fluxo/emparelhamento
            nLinhas = 0
            nLinhasDet = False
            while (True):
                try:
                    linha = file.readline().split()
                    if (nLinhas == 0 and nLinhasDet):
                        break
                    if linha == []:
                        continue
                    if linha[0] == "c" and linha[2] == "vertices":
                        self.V = int(linha[1])
                        self.E = 0
                        self.v_adj = {}
                        for i in range(1, self.V + 1):
                            self.v_adj[i] = []
                        self.arestas = {}
                        self.rotulos = {}
                        self.flow_source = 1
                        self.flow_sink = self.V
                    elif linha[0] == "p" and linha[1] == "edge":
                        self.V = int(linha[2])
                        self.E = 0
                        self.v_adj = {}
                        for i in range(1, self.V + 1):
                            self.v_adj[i] = []
                        self.arestas = {}
                        self.rotulos = {}
                        nLinhasDet = True
                        nLinhas -= int(linha[3])
                    elif linha[0] == "p":
                        nLinhasDet = True
                        nLinhas -= int(linha[3])

                    elif linha[0] == "a":
                        a, b, c = map(int, linha[1:])
                        nLinhas += 1
                        self.addArco(a, b, c)
                    elif linha[0] == "e":
                        a, b = map(int, linha[1:])
                        nLinhas += 1
                        self.addAresta(a, b, 1)
                    elif linha[0] == "n":
                        if linha[2] == "s":
                            self.flow_source = int(linha[1])
                        elif linha[2] == "t":
                            self.flow_sink = int(linha[1])
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


    def hopcroft_karp(self):
        dist = [float("inf")] * (self.V + 1)
        mate = [None] * (self.V + 1)

        m = 0
        # definimos a bipartição esperada
        X = [i for i in range(1, self.V//2 + 1)]
        Y = [i for i in range(self.V//2 + 1, self.V + 1)]

        # enquanto achamos um modo de aumentar o número de emparelhamentos
        while self.bfs_hk(X, mate, dist):
            for x in X:
                # atualizamos
                if mate[x] == None:
                    if self.dfs_hk(X, mate, x, dist):
                        m += 1

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

        # usamos 0 como o vértice nulo
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
                if (mate[y] == None and dist[0] == dist[x] + 1) or (mate[y] != None and dist[mate[y]] == dist[x] + 1):
                    if (self.dfs_hk(X, mate, mate[y], dist)):
                        mate[y] = x
                        mate[x] = y
                        return True
            dist[x] = float("inf")
            return False
        return True


    def edmonds_karp(self):
        rede_residual = {}

        s = self.flow_source
        t = self.flow_sink

        # inicializamos a rede residual
        for (u, v) in self.arestas.keys():
            rede_residual[(u, v)] = self.arestas[(u, v)]
            rede_residual[(v, u)] = 0

        # repetimos enquanto acharmos um caminho aumentante
        p = self.bfs_ek(s, t, rede_residual)
        while (p != None):
            u = p[0]
            v = -1
            fluxo_total = float("inf")
            for i in range(1, len(p)):
                v = p[i]
                fluxo_total = min(fluxo_total, rede_residual[(u, v)])
                u = v

            u = p[0]
            v = -1
            for i in range(1, len(p)):

                v = p[i]

                # atualizamos diretamente na rede residual
                rede_residual[(u, v)] = rede_residual[(u, v)] - fluxo_total
                rede_residual[(v, u)] = rede_residual[(v, u)] + fluxo_total

                u = v
            p = self.bfs_ek(s, t, rede_residual)

        # obtemos o fluxo total somando o fluxo que entrou no sorvedouro
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
                    # pelos arcos onde ainda pode passar fluxo
                    visitados[v] = True
                    ancestrais[v] = u
                    if v == t: # caminho encontrado
                        p = [t]
                        w = t
                        # armazenamos o caminho
                        while w != s:
                            w = ancestrais[w]
                            p.insert(0, w)
                        return p
                    fila.append(v)
        return None


    def all_subsets(self, lst):
        # todos os subconjuntos de um conjunto representado por uma lista
        if lst == []:
            return [[]]
        conj = self.all_subsets(lst[:-1])

        saida = []
        for i in range(len(conj)):
            saida.append(conj[i] + [lst[-1]])
            saida.append(conj[i])
        return saida

    def is_independent_set(self, v_set):
        # checa se o conjunto de vértices forma um conjunto independente
        for i in range(len(v_set)):
            for j in range(len(v_set)):
                if j != i:
                    if (v_set[i], v_set[j]) in self.arestas.keys():
                        return False
        return True

    def all_max_indepedent_sets(self, vert):
        # arranjamos todos os subconjuntos do maior para o menor
        vert_sets = sorted(self.all_subsets(vert), key = len, reverse=True)

        max = -1 # cardinalidade do subconjunto independente máximo

        ind_sets = []

        for i in range(len(vert_sets)):
            # depois de acharmos um, ignoramos os menores
            if (len(vert_sets[i]) < max):
                break
            if self.is_independent_set(vert_sets[i]):
                max = len(vert_sets[i])
                ind_sets.append(vert_sets[i])

        return ind_sets

    # essa função mapeia os subconjuntos para sua posição no vetor
    def ord_function(self, set):
        ind = 0
        for i in range(1, self.V+1):
            if i in set:
                ind += 2**(i-1)
        return ind
    # e esta mapeia de volta
    def inv_function(self, n):
        set = []
        for i in range(self.V, 0, -1):
            if n >= 2**(i-1):
                set.insert(0, i)
                n -= 2**(i-1)
        return set

    # uma subtração típica de conjuntos
    def set_delete(self, S, I):
        new_set = []
        for i in S:
            if not i in I:
                new_set.append(i)
        return new_set

    def lawler(self):
        x = [0] * (2**(self.V))
        cor_set = [0] * (2**self.V)
        # obtemos todos os subconjuntos, ordenados em tamanho
        SS = sorted(self.all_subsets([i for i in range(1, self.V + 1)]), key = len)

        # para cada subconjunto
        for S in SS:
            if S == []:
                continue

            # obtemos a posição do subconjunto
            s = self.ord_function(S)
            x[s] = float("inf")

            # descobrimos todos os conjuntos independentes maximais do subgrafo
            g_vert = S.copy()
            ind_sets = self.all_max_indepedent_sets(g_vert)

            # para cada um dos conjuntos, checamos se temos um modo de colorir com menos cores o conjunto S
            for I in ind_sets:
                i = self.ord_function(self.set_delete(S, I))
                if x[i] + 1 < x[s]:
                    x[s] = x[i] + 1
                    cor_set[s] = self.ord_function(I)

        print (x[2**(self.V) - 1])
        S = SS[-1]
        cor = 1
        cores = [0] * (self.V + 1)
        while S != []:
            for i in self.inv_function(cor_set[self.ord_function(S)]):
                cores[i] = cor
            cor += 1
            S = self.set_delete(S, self.inv_function(cor_set[self.ord_function(S)]))
        for i in range(1, self.V + 1):
            print(self.rotulos[i] + ": " + str(cores[i]))



# ler objeto
nome_do_arquivo = input()
grafo = Graph(nome_do_arquivo)

# grafo.edmonds_karp()

# grafo.hopcroft_karp()

# grafo.lawler()
