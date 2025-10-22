#include <iostream>
#include <list>
#include <vector>
#include <queue>
#include <limits>
#include <unordered_map>
#include <functional>



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
    std::unordered_map<V, Vertex<V, E>*> vertexMap;
 
public:
    Graph(bool directed) : isDirected(directed) {}
 
    ~Graph() {
        for (Edge<V, E>* e : edgeList) delete e;
        for (Vertex<V, E>* v : vertexList) delete v;
    }
 
    Vertex<V, E>* addVertex(const V& elem) {
        Vertex<V, E>* v = new Vertex<V, E>(elem);
        vertexList.push_back(v);
        vertexMap[elem] = v; 
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
        auto it = vertexMap.find(elem);
        return it != vertexMap.end() ? it->second : nullptr;
    }
 
    E IashChaukhan_dijkstra(V src, V target) {
        std::unordered_map<V, long long> dist;
        std::unordered_map<V, long long> bandwidth;
 
        for (auto v : vertexList) {
            dist[v->element] = std::numeric_limits<long long>::max();
            bandwidth[v->element] = 0;
        }
 
        if (dist.find(src) == dist.end() || dist.find(target) == dist.end()) {
            return { std::numeric_limits<long long>::max(), 0 };
        }
 
        using QueueElement = std::pair<std::pair<long long, long long>, V>;
        auto comp = [](const QueueElement& a, const QueueElement& b) {
            if (a.first.first != b.first.first) return a.first.first > b.first.first;
            return a.first.second < b.first.second;
        };
 
        std::priority_queue<QueueElement, std::vector<QueueElement>, decltype(comp)> pq(comp);
 
        dist[src] = 0;
        bandwidth[src] = std::numeric_limits<long long>::max();
        pq.push({ {0, bandwidth[src]}, src });
 
        while (!pq.empty()) {
            long long curDist = pq.top().first.first;
            long long curBw = pq.top().first.second;
            V u = pq.top().second;
            pq.pop();
 
            if (curDist > dist[u] || (curDist == dist[u] && curBw < bandwidth[u])) continue;
 
            Vertex<V, E>* node = getVertex(u);
            if (!node) continue;
 
            for (Edge<V, E>* edge : node->outgoing) {
                Vertex<V, E>* u1 = edge->endpoints[0];
                Vertex<V, E>* u2 = edge->endpoints[1];
                V v = (u1->element == u) ? u2->element : u1->element;
 
                long long weight = edge->element.first;
                long long edgeBw = edge->element.second;
 
                long long newDist = dist[u] + weight;
                long long newBw = std::min(bandwidth[u], edgeBw);
 
                if (newDist < dist[v] || (newDist == dist[v] && newBw > bandwidth[v])) {
                    dist[v] = newDist;
                    bandwidth[v] = newBw;
                    pq.push({ {newDist, newBw}, v });
                }
            }
        }
 
        return { dist[target], bandwidth[target] };
    }
};
 
int main() {
    long long n, m;
    std::cin >> n >> m;
 
    Graph<long, std::pair<long long, long long>> g(false); 
 
    for (int i = 0; i < m; ++i) {
        long long u, v, d, b;
        std::cin >> u >> v >> d >> b;
 
        Vertex<long, std::pair<long long, long long>>* uVertex = g.getVertex(u);
        Vertex<long, std::pair<long long, long long>>* vVertex = g.getVertex(v);
        if (!uVertex) uVertex = g.addVertex(u);
        if (!vVertex) vVertex = g.addVertex(v);
 
        g.addEdge(uVertex, vVertex, { d, b });
    }
 
    long long s, t;
    std::cin >> s >> t;
 
    auto [shortestDist, maxBandwidth] = g.IashChaukhan_dijkstra(s, t);
    std::cout << shortestDist << " " << maxBandwidth << std::endl;
 
    return 0;
}
