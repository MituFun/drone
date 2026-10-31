#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
using namespace std;

const double TIME_FOR_PICKUP = 100;		// 取货时间
const double TIME_FOR_OFFLOAD = 100;	// 卸货时间
const double DRONE_SPEED = 8;			// 无人机速度
const double BATTERY_USED_PER_M = 0.01; // 每米电量消耗
const double SAFETY_BATTERY = 5.0;		// 无人机最低电量限制
const double falpha = 0.5;
const double fbeta = 0.2;
const double fgamma = 0.3;
const double galpha = 0.5;
const double gbeta = 0.3;
const double ggamma = 0.2;
double mapX, mapY; // 地图尺寸，每个块面积为 sqrt(mapX)*sqrt(mapY)

struct $Point {
	double x, y;
	$Point(double xx, double yy) { x = xx, y = yy; }
	$Point() { x = 0, y = 0; }
};
struct $Order {
	int id;
	$Point pickup;
	$Point delivery;
	double deadLine;
	double weight;
	bool assigned;
};
struct $Drone {
	int id;
	$Point location;
	double maxWeight;
	double battery;
	// IDLE=空闲；DELIVERY=配送；TO_SPOT=去待命区；TO_CHARGE=充电中；
	string status;
	double doneTime;	 // 任务结束时间
	$Point taskLocation; // 任务目的地
};

const int maxMapSize = 1e5;
int hashCnt;
unordered_map<int, $Point> realId;
struct PairHash {
	size_t operator()(const pair<double, double> &p) const {
		return hash<double>()(p.first) ^ (hash<double>()(p.second) << 1);
	}
};
unordered_map<pair<double, double>, int, PairHash> hashId;
int getId($Point x) {
	pair<double, double> tmp;
	tmp.first = x.x;
	tmp.second = x.y;
	if (!hashId.count(tmp)) {
		hashId[tmp] = ++hashCnt;
		realId[hashCnt] = x;
		return hashCnt;
	} else {
		return hashId[tmp];
	}
}
vector<pair<int, double>> g[maxMapSize];
void mapInit() {
	int n;
	$Point u, v;
	double w;
	cin >> n;
	for (int i = 1; i <= n; i++) {
		cin >> u.x >> u.y;
		cin >> v.x >> v.y;
		mapX = max(mapX, max(u.x, v.x));
		mapY = max(mapY, max(u.y, v.y));
		cin >> w;
		g[getId(u)].push_back(make_pair(getId(v), w));
		g[getId(v)].push_back(make_pair(getId(u), w));
	}
}

vector<$Drone> drones;
void droneInit() {
	int n;
	cin >> n;
	for (int i = 1; i <= n; i++) {
		$Drone tmp;
		tmp.id = i;
		cin >> tmp.location.x >> tmp.location.y;
		cin >> tmp.maxWeight;
		cin >> tmp.battery;
		tmp.status = "IDLE";
		tmp.doneTime = -1;
		drones.push_back(tmp);
	}
}

int ordersCnt;
vector<$Order> orders;
void orderInit() {
	$Order tmp;
	tmp.id = ++ordersCnt;
	cin >> tmp.pickup.x >> tmp.pickup.y;
	cin >> tmp.delivery.x >> tmp.delivery.y;
	cin >> tmp.deadLine;
	cin >> tmp.weight;
	tmp.assigned = false;
	orders.push_back(tmp);
}

double h(int u, int t) {
	double dx = realId[u].x - realId[t].x;
	double dy = realId[u].y - realId[t].y;
	return sqrt(dx * dx + dy * dy);
}
double astar($Point a, $Point b) {
	pair<double, double> pa = make_pair(a.x, a.y);
	pair<double, double> pb = make_pair(b.x, b.y);

	if (!hashId.count(pa) || !hashId.count(pb))
		return -1;

	int s = hashId[pa], t = hashId[pb];

	const double INF = 1e100;
	vector<double> dist(hashCnt + 1, INF);
	priority_queue<pair<double, int>, vector<pair<double, int>>,
				   greater<pair<double, int>>>
		q;
	dist[s] = 0;
	q.push(make_pair(h(s, t), s));
	while (!q.empty()) {
		int u = q.top().second;
		double f = q.top().first;
		q.pop();
		if (f > dist[u] + h(u, t))
			continue;
		if (u == t)
			return dist[u];
		for (int i = 0; i < (int)g[u].size(); i++) {
			int v = g[u][i].first;
			double w = g[u][i].second;
			if (dist[v] > dist[u] + w) {
				dist[v] = dist[u] + w;
				q.push(make_pair(dist[v] + h(v, t), v));
			}
		}
	}
	return -1;
}

double getCurrentTime() {
	return std::chrono::duration<double>(
			   std::chrono::system_clock::now().time_since_epoch())
		.count();
}

int main() {
	mapInit();
	droneInit();
	while (1) {
		orderInit();
		double now = getCurrentTime();
		for (auto &drone : drones) {
			if (drone.doneTime <= now &&
				(drone.status == "DELIVERY" || drone.status == "TO_SPOT")) {
				drone.status = "IDLE";
				drone.location = drone.taskLocation;
			}
			if (drone.battery <= SAFETY_BATTERY) {
				drone.status = "TO_CHARGE";
			}
		}
		// 分配订单
		for (auto &drone : drones) {
			if (drone.status != "IDLE")
				continue;
			vector<pair<double, pair<int, pair<int, double>>>> fs;
			for (auto &order : orders) {
				if (order.assigned)
					continue;
				if (order.weight > drone.maxWeight)
					continue;
				double d1 = astar(drone.location, order.pickup);
				double d2 = astar(order.pickup, order.delivery);
				if (d1 == -1 || d2 == -1)
					continue;
				double d = d1 + d2;
				double t = now;
				double T = TIME_FOR_PICKUP + d / DRONE_SPEED + TIME_FOR_OFFLOAD;
				double s = order.deadLine - t - T;
				if (s < 0)
					continue;
				double E = d * BATTERY_USED_PER_M;
				if (drone.battery - E < SAFETY_BATTERY) {
					continue;
				}
				double f =
					falpha / (1.0 + d) + fbeta / (1.0 + s) + fgamma / (1.0 + E);
				fs.push_back(
					make_pair(f, make_pair(drone.id, make_pair(order.id, T))));
			}
			if (fs.empty())
				continue;
			sort(fs.begin(), fs.end());
			cout << "DRONE: " << drone.id << "->"
				 << " ORDER: " << fs[fs.size() - 1].second.second.first;
			drone.status = "DELIVERY";
			drone.doneTime = fs[fs.size() - 1].second.second.second + now;
			double taskTime = fs[fs.size() - 1].second.second.second;
			drone.battery -= (taskTime - TIME_FOR_PICKUP - TIME_FOR_OFFLOAD) *
							 DRONE_SPEED * BATTERY_USED_PER_M;
			for (auto &order : orders) {
				if (order.id == fs[fs.size() - 1].second.second.first) {
					order.assigned = 1;
					drone.taskLocation = order.delivery;
				}
			}
		}

		// 对空闲无人机进行处理
		for (auto &drone : drones) {
			if (drone.status == "IDLE") {
				double blockX = max(1.0, sqrt(mapX));
				double blockY = max(1.0, sqrt(mapY));
				int rows = int(mapX / blockX) + 1;
				int cols = int(mapY / blockY) + 1;
				vector<vector<int>> n(rows, vector<int>(cols));
				vector<vector<int>> q(rows, vector<int>(cols));
				vector<vector<int>> spot(rows, vector<int>(cols));
				for (int v = 1; v <= hashCnt; v++) {
					int x = int(realId[v].x / blockX);
					int y = int(realId[v].y / blockY);
					if (x >= 0 && x < rows && y >= 0 && y < cols &&
						spot[x][y] == 0)
						spot[x][y] = v;
				}
				for (auto &tmpdrone : drones) {
					if (tmpdrone.id == drone.id)
						continue;
					$Point pos;
					if (tmpdrone.status == "IDLE")
						pos = tmpdrone.location;
					else if (tmpdrone.status == "TO_SPOT")
						pos = tmpdrone.taskLocation;
					else
						continue;
					int x = int(pos.x / blockX), y = int(pos.y / blockY);
					if (pos.x >= 0 && pos.y >= 0 && x < rows && y < cols)
						n[x][y]++;
				}
				for (auto &order : orders) {
					double x = order.pickup.x;
					double y = order.pickup.y;
					int row = int(x / blockX), col = int(y / blockY);
					if (x >= 0 && y >= 0 && row < rows && col < cols)
						q[row][col]++;
				}
				vector<pair<double, pair<int, int>>> gs;
				for (int i = 0; i < rows; i++) {
					for (int j = 0; j < cols; j++) {
						if (spot[i][j] == 0)
							continue;
						double d = astar(drone.location, realId[spot[i][j]]);
						if (d == -1 || drone.battery - d * BATTERY_USED_PER_M <
										   SAFETY_BATTERY)
							continue;
						double g = galpha * q[i][j] + gbeta / (1.0 + n[i][j]) +
								   ggamma / (1 + d);
						gs.push_back(make_pair(g, make_pair(i, j)));
					}
				}
				if (gs.empty())
					continue;
				sort(gs.begin(), gs.end());
				auto t = gs[gs.size() - 1];
				cout << "DRONE: " << drone.id << "->"
					 << " SPOT: " << t.second.first << ',' << t.second.second
					 << endl;
				drone.status = "TO_SPOT";
				drone.taskLocation =
					realId[spot[t.second.first][t.second.second]];
				double d = astar(drone.location, drone.taskLocation);
				drone.battery -= d * BATTERY_USED_PER_M;
				drone.doneTime = d / DRONE_SPEED + now;
			}
		}
	}
	return 0;
}
