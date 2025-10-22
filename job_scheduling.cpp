
#include <iostream>
#include <unordered_map>
#include <vector>

template<typename T, typename G> class Pair {
private:
	T _first;
	G _second;
public:
	Pair() = default; //https://en.cppreference.com/w/cpp/language/default_constructor
	Pair(T first, G second) {
		_first = first;
		_second = second;
	}
	T getFirst() {
		return _first;
	}
	G getSecond() {
		return _second;
	}
	~Pair() {}
};

template<typename T> void swap(T& a, T& b) {
	T temp = a;
	a = b;
	b = temp;
}

void selectionSort(std::vector<Pair<int, Pair<int, int>>>& vec) {
    for (int i = 0; i < (int)vec.size(); ++i) {
        int minIndex = i;
        for (int j = i + 1; j < (int)vec.size(); ++j) {
            if (vec[j].getSecond().getSecond() < vec[minIndex].getSecond().getSecond()) {
                minIndex = j;
            }
        }
        swap(vec[i], vec[minIndex]);
    }
}

//used idea from the tutorial slides
int maxValBottomUp(std::vector<Pair<int, Pair<int, int>>>& vec, int value) {
    std::vector<int> results(value);
    results[0] = vec[0].getFirst();
    for (int i = 1; i < (int)vec.size(); i++) {
        int incl = vec[i].getFirst();
        int maxIndex = -1;
        for (int j = i - 1; j >= 0; j--) {
            if (vec[j].getSecond().getSecond() <= vec[i].getSecond().getFirst()) {
                maxIndex = j;
                break;
            }
        }
        if (maxIndex != -1) {
            incl += results[maxIndex];
        }
        int excl = results[i - 1];
        results[i] = std::max(incl, excl);
    }
    return results[value - 1];
}

int main() {
    int n;
    std::cin >> n;
    std::vector<int> starts(n), finishes(n), values(n);
    for (int i = 0; i < n; i++) {
        std::cin >> starts[i];
    }
    for (int i = 0; i < n; i++) {
        std::cin >> finishes[i];
    }
    for (int i = 0; i < n; i++) {
        std::cin >> values[i];
    }
    std::vector<Pair<int, Pair<int, int>>> jobs;
    jobs.reserve(n);
    for (int i = 0; i < n; i++) {
        Pair<int, int> p(starts[i], finishes[i]);
        jobs.push_back(Pair<int, Pair<int, int>>(values[i], p));
    }
    selectionSort(jobs);
    std::cout << maxValBottomUp(jobs, n) << std::endl;
    return 0;
}
