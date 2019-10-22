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


    def dfs(self):
        visitado = [False] * (self.V + 1)
        tempo_entrada = [(float("inf"))] * (self.V + 1)
        tempo_saida = [(float("inf"))] * (self.V + 1)
        ancestrais = [None] * (self.V + 1)

        tempo = [0];

        for u in range(1, self.V + 1):
            if not visitado[u]:
                self.dfs_visit(u, visitado, tempo_entrada, ancestrais, tempo_saida, tempo)

        return visitado, tempo_entrada, ancestrais, tempo_saida

    def dfs_visit(self, v, visitado, tempo_entrada, ancestrais, tempo_saida, tempo):
        visitado[v] = True
        tempo[0] += 1
        tempo_entrada[v] = tempo[0]

        for u in self.vizinhos(v):
            if not visitado[u]:
                ancestrais[u] = v
                self.dfs_visit(u, visitado, tempo_entrada, ancestrais, tempo_saida, tempo)

        tempo[0] += 1
        tempo_saida[v] = tempo[0]


    def cfc(self):

        visitado, tempo_entrada, ancestrais_linha, tempo_saida = self.DFS()

        arestas_t = []

        for i in self.arestas.keys():
            x, y = i
            arestas_t.append((y, x))

        visitado_t, tempo_entrada_t, ancestrais_t, tempo_saida = self.dfs_adapt(tempo_saida, arestas_t)

        return ancestrais_t


    def dfs_adapt(self, tempo_saida, arestas_t):
        visitado_t = [False] * (self.V + 1)
        tempo_entrada_t = [(float("inf"))] * (self.V + 1)
        tempo_saida_t = [(float("inf"))] * (self.V + 1)
        ancestrais_t = [None] * (self.V + 1)

        vertices_f = []

        for i in range(1, self.V + 1):
            vertices_f.append((tempo_saida[i], i))

        vertices_f.sort(reverse=True)

        tempo = [0]

        for i in range(len(vertices_f)):
            if not visitado_t[vertices_f[i][1]]:
                dfs_adapt_visit(vertices_f[i][1], visitado_t, tempo_entrada_t, tempo_saida_t, ancestrais_t, arestas_t, tempo)

        return visitado_t, tempo_entrada_t, ancestrais_t, tempo_saida_t

    def dfs_adapt_visit(self, v, visitado_t, tempo_entrada_t, tempo_saida_t, ancestrais_t, arestas_t, tempo):
        visitado_t[v] = True
        tempo[0] += 1
        tempo_entrada_t[v] = tempo[0]

        for x in range(len(arestas_t)):
            if arestas_t[x][0] == v:
                if not visitado_t[arestas_t[x][1]]:
                    ancestrais_t[arestas_t[x][1]] = v
                    self.dfs_adapt_visit(arestas_t[x][1], visitado_t, tempo_entrada_t, ancestrais_t, tempo_saida_t, tempo)

        tempo[0] += 1
        tempo_saida_t[v] = tempo[0]


    def ord_topologica(self):
        visitado = [False] * (self.V + 1)
        tempo_entrada = [(float("inf"))] * (self.V + 1)
        tempo_saida = [(float("inf"))] * (self.V + 1)
        ancestrais = [None] * (self.V + 1)

        tempo = [0]
        pilha = []

        ciclico = [False]

        for i in range(1, self.V + 1):
            if not visitado[i]:
                self.ord_topologica_aux(i, visitado, tempo_entrada, tempo_saida, pilha, tempo, ciclico)

        if (ciclico[0]):
            print("Impossivel fazer a ordenação topológica");
            return
        resultado = ""
        for i in range(len(pilha)-1, -1, -1):
            resultado = resultado + self.rotulo(pilha[i])
            if (i != 0):
                resultado = resultado + " -> "

        print (resultado)

    def ord_topologica_aux(self, i, visitado, tempo_entrada, tempo_saida, pilha, tempo, ciclico):
        visitado[i] = True
        tempo[0] += 1
        tempo_entrada[i] = tempo[0]

        for u in self.vizinhos(i):
            if (tempo_entrada[u] < float("inf") and tempo_saida[u] == float("inf")):
                ciclico[0] = True
                return
            if not visitado[u]:
                self.ord_topologica_aux(u, visitado, tempo_entrada, tempo_saida, pilha, tempo, ciclico)
        if (ciclico[0]):
            return
        tempo[0] += 1
        tempo_saida[i] = tempo[0]

        pilha.insert(0, i)



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

        for i in range(len(arestas_ord)):
            a, b = arestas_ord[i][0]
            if (self.find(raizes, a) == self.find(raizes, b)):
                # print ("Ciclo detec: " + str(a) + " " + str(b))
                continue
            arv_min.append((a,b));
            weight += arestas_ord[i][1]
            self.union(raizes, rank, a, b)
        # return arv_min, weight
        print(weight)
        saida = ""
        for i in range(len(arv_min)):
            saida = saida + str(arv_min[i][0]) + '-' + str(arv_min[i][1])
            if i != len(arv_min) - 1:
                saida += ", "
        print(saida)


# ler objeto
nome_do_arquivo = input()
grafo = Graph(nome_do_arquivo)

# print("Arestas:")
# print(grafo.arestas)
# grafo.ord_topologic
grafo.kruskal()
# print ("Arvore: " + str(arv))
# print ("Peso: " + str(w))
