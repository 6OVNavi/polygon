#include <iostream>
#include <map>
#include <string>
#include <algorithm>
#include <vector>
using namespace std;

bool compare(pair<char, int> i, pair<char, int> j) {
	return i.second < j.second;
}
bool compare2(pair<char, int> i, pair<char, int> j) {
	return i.second > j.second;
}
int main() {
	int n;
	string s;
	cin >> n;
	cin >> s;
	map<char, int>u;
	pair<char, int>min;
	pair<char, int>max;
	int min_ = 1e9;
	int size = u.size();
	for (int i = 0; i < n; i++) {
		u[s[i]] = i;
		if (size != u.size()) {
			size = u.size();
			min_ = 1e9;
		}


		pair<char, int> min = *min_element(u.begin(), u.end(), compare);
		pair<char, int> max = *max_element(u.begin(), u.end(), compare);
		if (max.second - min.second < min_ && max.second - min.second != 0) {
			min_ = max.second - min.second;
		}
	}
	if (min_ == 1e9) {
		cout << 1;
		return 0;
	}
	cout << min_ + 1;
	return 0;
}