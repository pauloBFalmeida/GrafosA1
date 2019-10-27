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


    # Uma busca em profundidade padrão
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


    # Componentes fortemente conexas
    def cfc(self):

        # Rodamos uma busca em profundidade para obtermos os tempos de saída
        visitado, tempo_entrada, ancestrais_linha, tempo_saida = self.dfs()

        # Para transpormos o grafo, criamos uma lista para as arestas transpostas
        # e usamos isso como as arestas na busca adaptada
        arestas_t = []
        for i in self.arestas.keys():
            x, y = i
            arestas_t.append((y, x))

        self.dfs_adapt(tempo_saida, arestas_t)


    def dfs_adapt(self, tempo_saida, arestas_t):
        visitado_t = [False] * (self.V + 1)
        tempo_entrada_t = [(float("inf"))] * (self.V + 1)
        tempo_saida_t = [(float("inf"))] * (self.V + 1)
        ancestrais_t = [None] * (self.V + 1)

        vertices_f = []

        for i in range(1, self.V + 1):
            vertices_f.append((tempo_saida[i], i))

        # Obtemos os vértices ordenados por tempo de saída
        vertices_f.sort(reverse=True)

        tempo = [0]

        # Imprimimos os componentes separadamente
        componente = []

        for i in range(len(vertices_f)):
            if not visitado_t[vertices_f[i][1]]:
                componente = []
                self.dfs_adapt_visit(vertices_f[i][1], componente, visitado_t, tempo_entrada_t, tempo_saida_t, ancestrais_t, arestas_t, tempo)
                resultado = ""
                for i in range(len(componente)):
                    resultado = resultado + str(componente[i])
                    if not i == len(componente) - 1:
                        resultado = resultado + ","
                print (resultado)

    def dfs_adapt_visit(self, v, componente, visitado_t, tempo_entrada_t, tempo_saida_t, ancestrais_t, arestas_t, tempo):
        visitado_t[v] = True
        componente.append(v)
        tempo[0] += 1
        tempo_entrada_t[v] = tempo[0]

        for x in range(len(arestas_t)):
            if arestas_t[x][0] == v:
                if not visitado_t[arestas_t[x][1]]:
                    ancestrais_t[arestas_t[x][1]] = v
                    self.dfs_adapt_visit(arestas_t[x][1], componente, visitado_t, tempo_entrada_t, tempo_saida_t, ancestrais_t, arestas_t, tempo)

        tempo[0] += 1
        tempo_saida_t[v] = tempo[0]


    def ord_topologica(self):
        visitado = [False] * (self.V + 1)
        tempo_entrada = [(float("inf"))] * (self.V + 1)
        tempo_saida = [(float("inf"))] * (self.V + 1)
        ancestrais = [None] * (self.V + 1)

        tempo = [0]
        lista = []

        ciclico = [False]

        # Fazemos uma busca em profundidade
        for i in range(1, self.V + 1):
            if not visitado[i]:
                self.ord_topologica_aux(i, visitado, tempo_entrada, tempo_saida, lista, tempo, ciclico)

        if (ciclico[0]):
            print("Impossivel fazer a ordenação topológica");
            return
        resultado = ""
        for i in range(len(lista)):
            resultado = resultado + self.rotulo(lista[i])
            if (i != len(lista)-1):
                resultado = resultado + " -> "

        print (resultado)

    def ord_topologica_aux(self, i, visitado, tempo_entrada, tempo_saida, lista, tempo, ciclico):
        visitado[i] = True
        tempo[0] += 1
        tempo_entrada[i] = tempo[0]

        for u in self.vizinhos(i):
            # Se há um arco de retorno, há um ciclo no grafo e a ordenação é impossível
            if (tempo_entrada[u] < float("inf") and tempo_saida[u] == float("inf")):
                ciclico[0] = True
                return
            if not visitado[u]:
                self.ord_topologica_aux(u, visitado, tempo_entrada, tempo_saida, lista, tempo, ciclico)
        if (ciclico[0]):
            return
        tempo[0] += 1
        tempo_saida[i] = tempo[0]

        # Adicionamos no início da lista quando terminamos um vértice
        lista.insert(0, i)


    # Usamos conjuntos disjuntos para detectar ciclos
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
        # Ordenamos as arestas baseado no peso
        arestas_ord = sorted(self.arestas.items(), key=operator.itemgetter(1))

        # Percorremos as arestas ordenadas
        for i in range(len(arestas_ord)):
            a, b = arestas_ord[i][0]
            # Se essa aresta formar um ciclo, simplesmente não adicionamos a aresta
            if (self.find(raizes, a) == self.find(raizes, b)):
                continue

            # Adicionamos na árvore mínima
            arv_min.append((a,b));
            weight += arestas_ord[i][1]
            self.union(raizes, rank, a, b)

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

# Componentes fortemente conexas
# grafo.cfc()

# Ordenação topológica
# grafo.ord_topologica()

# Árvore geradora mínima (Kruskal)
# grafo.kruskal()
