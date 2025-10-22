#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <queue>
#include <limits>
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
private:
    bool isDirected;
    std::list<Vertex<V, E>*> vertexList;
    std::list<Edge<V, E>*> edgeList;
 
    void dfs(Vertex<V, E>* current, Vertex<V, E>* parent, std::unordered_map<Vertex<V, E>*, bool>& visited) {
        visited[current] = true;
        for (Edge<V, E>* e : current->outgoing) {
            Vertex<V, E>* neighbor = (e->endpoints[0] == current) ? e->endpoints[1] : e->endpoints[0];
            if (!visited[neighbor]) {
                std::cout << current->element.first << ":" << neighbor->element.first << " ";
                dfs(neighbor, current, visited);
            }
        }
    }
 
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
 
    Graph<V, E>* IashChaukhan_mst_prim() {
        Graph<V, E>* mst = new Graph<V, E>(isDirected);
        if (vertexList.empty()) return mst;
 
        std::vector<Vertex<V, E>*> vertices;
        std::unordered_map<Vertex<V, E>*, size_t> vertexToIndex;
        size_t idx = 0;
        for (auto v : vertexList) {
            vertices.push_back(v);
            vertexToIndex[v] = idx++;
        }
        size_t numVertices = vertices.size();
 
        std::vector<bool> inMST(numVertices, false);
        std::vector<E> key(numVertices, std::numeric_limits<E>::max());
        std::vector<Edge<V, E>*> parent(numVertices, nullptr);
 
        auto cmp = [](const std::pair<E, size_t>& a, const std::pair<E, size_t>& b) { return a.first > b.first; };
 
        std::unordered_map<Vertex<V, E>*, Vertex<V, E>*> originalToMst;
        for (Vertex<V, E>* orig_v : vertexList) {
            Vertex<V, E>* mst_v = mst->addVertex(orig_v->element);
            originalToMst[orig_v] = mst_v;
        }
 
        for (size_t i = 0; i < numVertices; ++i) {
            if (!inMST[i]) {
                key[i] = E();
                std::priority_queue<std::pair<E, size_t>, std::vector<std::pair<E, size_t>>, decltype(cmp)> pq(cmp);
                pq.push({ key[i], i });
 
                while (!pq.empty()) {
                    size_t uIdx = pq.top().second;
                    pq.pop();
 
                    if (inMST[uIdx]) continue;
                    inMST[uIdx] = true;
 
                    Vertex<V, E>* u = vertices[uIdx];
 
                    for (Edge<V, E>* e : u->outgoing) {
                        Vertex<V, E>* v = (e->endpoints[0] == u) ? e->endpoints[1] : e->endpoints[0];
                        auto it = vertexToIndex.find(v);
                        if (it == vertexToIndex.end()) continue;
                        size_t vIdx = it->second;
 
                        if (!inMST[vIdx] && e->element < key[vIdx]) {
                            key[vIdx] = e->element;
                            parent[vIdx] = e;
                            pq.push({ key[vIdx], vIdx });
                        }
                    }
                }
            }
        }
 
        for (size_t i = 0; i < numVertices; ++i) {
            if (parent[i] != nullptr) {
                Vertex<V, E>* orig_u = parent[i]->endpoints[0];
                Vertex<V, E>* orig_v = parent[i]->endpoints[1];
                Vertex<V, E>* mst_u = originalToMst[orig_u];
                Vertex<V, E>* mst_v = originalToMst[orig_v];
                if (mst_u && mst_v) {
                    mst->addEdge(mst_u, mst_v, parent[i]->element);
                }
            }
        }
 
        return mst;
    }
    void print_dfs() {
        if (vertexList.empty()) return;
        std::unordered_map<Vertex<V, E>*, bool> visited;
        for (auto v : vertexList) {
            if (!visited[v]) {
                dfs(v, nullptr, visited);
                visited[v] = true;
            }
        }
    }
};
 
std::vector<std::string> split(std::string s, const std::string& delimiter) {
    std::vector<std::string> tokens;
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        if (!token.empty()) tokens.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    if (!s.empty()) tokens.push_back(s);
    return tokens;
}
 
int main() {
    int n;
    std::cin >> n;
    std::cin.ignore();
    Graph<std::pair<std::string, long double>, long double> g(false);
    std::unordered_map<std::string, std::pair<std::string, long double>> stallMap;
    for (int i = 0; i < n; i++) {
        std::string line;
        std::getline(std::cin, line);
        std::vector<std::string> splittedLine = split(line, " ");
        std::string command = splittedLine[0];
 
        if (command == "ADD") {
            std::string name = splittedLine[1];
            long double value = std::stod(splittedLine[2]);
            stallMap[name] = { name, value };
            g.addVertex({ name, value });
        }
        else if (command == "CONNECT") {
            std::string uName = splittedLine[1];
            std::string vName = splittedLine[2];
            long double weight = std::stod(splittedLine[3]);
            auto uIt = stallMap.find(uName);
            auto vIt = stallMap.find(vName);
            if (uIt != stallMap.end() && vIt != stallMap.end()) {
                long double cost = weight / (uIt->second.second + vIt->second.second);
                Vertex<std::pair<std::string, long double>, long double>* u = g.getVertex(uIt->second);
                Vertex<std::pair<std::string, long double>, long double>* v = g.getVertex(vIt->second);
                if (u && v) {
                    g.addEdge(u, v, cost);
                }
            }
        }
        else if (command == "PRINT_MIN") {
            Graph<std::pair<std::string, long double>, long double>* mst = g.IashChaukhan_mst_prim();
            mst->print_dfs();
            std::cout << std::endl;
            delete mst;
        }
    }
    return 0;
}
