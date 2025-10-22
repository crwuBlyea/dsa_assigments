#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
template <typename V, typename E>
class Edge;

template <typename V, typename E>
class Vertex {
public:
    V element;
    std::list<Edge<V, E>*> outgoing;
    std::list<Edge<V, E>*> incoming;

    Vertex(const V& elem) : element(elem) {}
};

template <typename V, typename E>
class Edge {
public:
    E element;
    Vertex<V, E>* endpoints[2];

    Edge(Vertex<V, E>* u, Vertex<V, E>* v, const E& elem) : element(elem) {
        endpoints[0] = u;
        endpoints[1] = v;
    }
};

template <typename V, typename E>
class Graph {
public:
    bool isDirected;
    std::list<Vertex<V, E>*> vertexList;
    std::list<Edge<V, E>*> edgeList;

    Graph(bool directed) : isDirected(directed) {}

    ~Graph() {
        for (Edge<V, E>* e : edgeList) {
            delete e;
        }
        for (Vertex<V, E>* v : vertexList) {
            delete v;
        }
    }

    Vertex<V, E>* addVertex(const V& elem) {
        Vertex<V, E>* v = new Vertex<V, E>(elem);
        vertexList.push_back(v);
        return v;
    }

    Edge<V, E>* addEdge(Vertex<V, E>* u, Vertex<V, E>* v, const E& elem) {
        Edge<V, E>* e = new Edge<V, E>(u, v, elem);
        edgeList.push_back(e);
        u->outgoing.push_back(e);
        v->incoming.push_back(e);
        if (!isDirected) {
            u->incoming.push_back(e);
            v->outgoing.push_back(e);
        }
        return e;
    }

    Vertex<V, E>* getVertex(const V& elem) const {
        for (Vertex<V, E>* v : vertexList) {
            if (v->element == elem) {
                return v;
            }
        }
        return nullptr;
    }
};

template <typename E>
class Partition {
public:
    class Locator {
    public:
        E element;
        int size;
        Locator* parent;

        Locator(E elem) : element(elem), size(1), parent(this) {}
        E getElement() const { return element; }
    };

    Locator* makeCluster(const E& e) {
        Locator* loc = new Locator(e);
        locators.push_back(loc);
        return loc;
    }

    Locator* find(Locator* p) {
        if (p->parent != p)
            p->parent = find(p->parent);
        return p->parent;
    }

    void unionClusters(Locator* p, Locator* q) {
        Locator* a = find(p);
        Locator* b = find(q);
        if (a != b) {
            if (a->size > b->size) {
                b->parent = a;
                a->size += b->size;
            }
            else {
                a->parent = b;
                b->size += a->size;
            }
        }
    }

    int countUniqueLeadersById(int N, const std::vector<Locator*>& nodes) {
        std::vector<bool> visited(N + 1, false);
        int count = 0;
        for (int i = 1; i <= N; ++i) {
            Locator* leader = find(nodes[i]);
            int id = leader->getElement();
            if (!visited[id]) {
                visited[id] = true;
                count++;
            }
        }
        return count;
    }

    ~Partition() {
        for (Locator* ptr : locators)
            delete ptr;
    }

private:
    std::vector<Locator*> locators;
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    int n, m;
    std::cin >> n >> m;

    Graph<int, int> g(false);

    std::vector<Vertex<int, int>*> vertexPtrs(n + 1);
    for (int i = 1; i <= n; ++i) {
        vertexPtrs[i] = g.addVertex(i);
    }

    for (int i = 0; i < m; ++i) {
        int u, v;
        std::cin >> u >> v;
        g.addEdge(vertexPtrs[u], vertexPtrs[v], 0);
    }

    Partition<int> dsu;
    std::vector<Partition<int>::Locator*> nodes(n + 1);

    for (int i = 1; i <= n; ++i) {
        nodes[i] = dsu.makeCluster(i);
    }

    for (Edge<int, int>* edge : g.edgeList) {
        Vertex<int, int>* u_vertex = edge->endpoints[0];
        Vertex<int, int>* v_vertex = edge->endpoints[1];
        int u = u_vertex->element;
        int v = v_vertex->element;
        dsu.unionClusters(nodes[u], nodes[v]);
    }

    std::cout << dsu.countUniqueLeadersById(n, nodes) << std::endl;

    return 0;
}
