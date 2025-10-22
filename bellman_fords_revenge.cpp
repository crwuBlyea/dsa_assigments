#include <iostream>
#include <list>
#include <vector>
#include <unordered_map>
#include <limits>
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
private:
    bool isDirected;
    std::list<Vertex<V, E>*> vertexList;
    std::list<Edge<V, E>*> edgeList;

public:
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
        return e;
    }

    std::list<Vertex<V, E>*> vertices() {
        return vertexList;
    }

    std::list<Edge<V, E>*> edges() {
        return edgeList;
    }

    std::vector<V> IashChaukhan_bellman_ford(V src) {
        int max_id = 0;
        for (auto v : vertexList) {
            if (v->element > max_id)
                max_id = v->element;
        }

        int n = vertexList.size();
        const int INF = std::numeric_limits<int>::max();

        std::vector<int> dist(max_id + 1, INF);
        std::vector<int> pred(max_id + 1, -1);
        dist[src] = 0;
        for (int i = 1; i < n; ++i) {
            for (auto e : edgeList) {
                int u = e->endpoints[0]->element;
                int v = e->endpoints[1]->element;
                int w = e->element;
                if (dist[u] != INF && dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    pred[v] = u;
                }
            }
        }
        int start = -1;
        for (auto e : edgeList) {
            int u = e->endpoints[0]->element;
            int v = e->endpoints[1]->element;
            int w = e->element;
            if (dist[u] != INF && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w; 
                pred[v] = u;
                start = v;
            }
        }

        if (start == -1)
            return {};
        int x = start;
        for (int i = 0; i < n; ++i) {
            x = pred[x];
        }

        std::vector<V> cycle;
        int cur = x;
        do {
            cycle.push_back(cur);
            cur = pred[cur];
            if (cur == -1) break;
        } while (cur != x);

        return cycle;
    }




    Vertex<V, E>* getVertex(const V& elem) const {
        for (Vertex<V, E>* v : vertexList) {
            if (v->element == elem) {
                return v;
            }
        }
        return nullptr;
    }
    Edge<V, E>* getEdge(const E& elem) const {
        for (Edge<V, E>* e : edgeList) {
            if (e->element == elem) {
                return e;
            }
        }
        return nullptr;
    }



};

int main() {
    int n;
    std::cin >> n;
    std::vector<std::vector<int>> matrix(n+1, std::vector<int>(n+1));
    Graph<int, int> g(true);
    for (size_t i = 1; i < n+1; i++)
    {
        g.addVertex(i);
        for (size_t j = 1; j < n+1; j++) {
            std::cin >> matrix[i][j];
        }
    }
    for (size_t i = 1; i < n+1; i++)
    {
        for (size_t j = 1; j < n+1; j++)
        {
            if (matrix[i][j] != 0) {
                g.addEdge(g.getVertex(j), g.getVertex(i), matrix[i][j]);
            }
        }
    }
    std::vector<int> cycle = g.IashChaukhan_bellman_ford(1);
    if (cycle.empty()) {
        std::cout << "NO" << std::endl;
    }
    else {
        std::cout << "YES" << std::endl;
        std::cout << cycle.size() << std::endl;
        for (int v : cycle) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }




}
