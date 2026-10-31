#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>
using namespace std;

const int MAXN = 1005, MAXV = 1000000;
const double INF = 1e100, EPS = 1e-8;
const double SPEED = 1, SERVICE = 1;
const double RESERVE = 2, CHARGE_RATE = 2;
const double WINDOW = 30;

struct Point {
	double x, y, z;
	Point(double a = 0, double b = 0, double c = 0) {
		x = a;
		y = b;
		z = c;
	}
};
struct Path {
	vector<Point> p;
	double len;
	Path() { len = INF; }
};
struct Leg {
	Point a, b;
	double begin, end;
};
struct Order {
	int id;
	Point pickup, delivery;
	double deadline, weight;
	bool assigned;
	double arrival, takeoff, finish;
};
enum State { IDLE, DELIVERY, TO_CHARGER, CHARGING, TO_HOTSPOT, GROUNDED };
struct Drone {
	int id;
	Point position;
	double maxLoad, battery;
	int state;
	double taskFinishTime;
	double capacity;
	int version, target;
	bool settled;
	vector<Leg> legs;
};
struct Region {
	int xmin, xmax, ymin, ymax;
	Point standby;
};
enum Type { ARRIVAL, FINISH, REACH_CHARGER, CHARGED, REACH_HOTSPOT };
struct Event {
	double time;
	int type, drone, order, version;
};
struct EventCmp {
	bool operator()(Event a, Event b) const { return a.time > b.time; }
};
struct Node {
	int id;
	double f;
	Node(int a, double b) {
		id = a;
		f = b;
	}
};
struct NodeCmp {
	bool operator()(Node a, Node b) const { return a.f > b.f; }
};

Order order[MAXN];
Drone drone[MAXN];
Region region[MAXN];
Point charger[MAXN];
bool blocked[MAXV];
double dis[MAXV];
int pre[MAXV];
int X, Y, Z, B, C, R, N, M;
double now, horizon, energyUsed;
priority_queue<Event, vector<Event>, EventCmp> events;
int dx[6] = {1, -1, 0, 0, 0, 0};
int dy[6] = {0, 0, 1, -1, 0, 0};
int dz[6] = {0, 0, 0, 0, 1, -1};

void readPoint(Point &p) { cin >> p.x >> p.y >> p.z; }
double distance3(Point a, Point b) { // 三维坐标
	double x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
	return sqrt(x * x + y * y + z * z);
}
int id(int x, int y, int z) { return (z * Y + y) * X + x; }
Point point(int u) { return Point(u % X, u / X % Y, u / (X * Y)); }
bool valid(int x, int y, int z) {
	return x >= 0 && x < X && y >= 0 && y < Y && z >= 0 && z < Z &&
		   !blocked[id(x, y, z)];
}
bool inside(Point p, int k) {
	return p.x >= region[k].xmin && p.x <= region[k].xmax &&
		   p.y >= region[k].ymin && p.y <= region[k].ymax;
}
double snap(double x) {
	double nearest = floor(x + 0.5);
	if (fabs(x - nearest) < EPS)
		return nearest;
	return x;
}

Path astar(Point start, Point goal) {
	int i, u, v, k, x, y, z;
	for (i = 0; i < X * Y * Z; i++) {
		dis[i] = INF;
		pre[i] = -1;
	}
	priority_queue<Node, vector<Node>, NodeCmp> q;
	start = Point(snap(start.x), snap(start.y), snap(start.z));
	Point endpoint[2];
	endpoint[0] = Point(floor(start.x), floor(start.y), floor(start.z));
	endpoint[1] = Point(ceil(start.x), ceil(start.y), ceil(start.z));
	// 改道时可能位于栅格边中间，连接这条边的两个端点，也就允许掉头。
	for (i = 0; i < 2; i++) {
		Point p = endpoint[i];
		if (!valid((int)p.x, (int)p.y, (int)p.z))
			continue;
		u = id((int)p.x, (int)p.y, (int)p.z);
		dis[u] = distance3(start, p);
		q.push(Node(u, dis[u] + distance3(p, goal)));
	}
	int target = id((int)goal.x, (int)goal.y, (int)goal.z);
	while (!q.empty()) {
		Node top = q.top();
		q.pop();
		u = top.id;
		Point p = point(u);
		if (top.f > dis[u] + distance3(p, goal) + EPS)
			continue;
		if (u == target) {
			Path result;
			result.len = dis[u];
			for (v = u; v != -1; v = pre[v])
				result.p.push_back(point(v));
			reverse(result.p.begin(), result.p.end());
			if (distance3(start, result.p[0]) > 0)
				result.p.insert(result.p.begin(), start);
			return result;
		}
		for (k = 0; k < 6; k++) {
			x = (int)p.x + dx[k];
			y = (int)p.y + dy[k];
			z = (int)p.z + dz[k];
			if (!valid(x, y, z))
				continue;
			v = id(x, y, z);
			if (dis[u] + 1 < dis[v]) {
				dis[v] = dis[u] + 1;
				pre[v] = u;
				q.push(Node(v, dis[v] + distance3(point(v), goal)));
			}
		}
	}
	return Path();
}
Path nearestCharger(Point p) {
	Path best;
	for (int i = 1; i <= C; i++) {
		Path path = astar(p, charger[i]);
		if (path.len < best.len)
			best = path;
	}
	return best;
}
void addEvent(double t, int type, int i = 0, int j = 0) {
	Event e;
	e.time = t;
	e.type = type;
	e.drone = i;
	e.order = j;
	e.version = i == 0 ? 0 : drone[i].version;
	events.push(e);
}
bool current(Event e) {
	return e.drone == 0 || e.version == drone[e.drone].version;
}
bool waiting(int j) { return order[j].arrival <= now && !order[j].assigned; }

// 时间跳到 t：沿当前航线更新位置，只扣实际飞过的那部分电量。
void advance(double t) {
	for (int i = 1; i <= N; i++) {
		Drone &d = drone[i];
		if (d.state == CHARGING)
			d.battery = min(d.capacity, d.battery + (t - now) * CHARGE_RATE);
		for (int j = 0; j < (int)d.legs.size(); j++) {
			Leg leg = d.legs[j];
			double dt = max(0.0, min(t, leg.end) - max(now, leg.begin));
			double used = distance3(leg.a, leg.b) * dt / (leg.end - leg.begin);
			d.battery -= used;
			energyUsed += used;
			if (t >= leg.end)
				d.position = leg.b;
			else if (t >= leg.begin) {
				double f = (t - leg.begin) / (leg.end - leg.begin);
				d.position = Point(leg.a.x + (leg.b.x - leg.a.x) * f,
								   leg.a.y + (leg.b.y - leg.a.y) * f,
								   leg.a.z + (leg.b.z - leg.a.z) * f);
				break;
			} else
				break;
		}
	}
	now = t;
}
void beginTask(int i, int state) {
	drone[i].version++; // 改道后，原来的抵达事件因版本不同而失效
	drone[i].state = state;
	drone[i].legs.clear();
	drone[i].taskFinishTime = now;
	drone[i].target = 0;
	drone[i].settled = false;
}
void fly(int i, const Path &path) {
	for (int j = 1; j < (int)path.p.size(); j++) {
		Leg leg;
		leg.a = path.p[j - 1];
		leg.b = path.p[j];
		leg.begin = drone[i].taskFinishTime;
		leg.end = leg.begin + distance3(leg.a, leg.b) / SPEED;
		if (leg.end > leg.begin)
			drone[i].legs.push_back(leg);
		drone[i].taskFinishTime = leg.end;
	}
}
void service(int i, Point p) {
	Leg leg;
	leg.a = p;
	leg.b = p;
	leg.begin = drone[i].taskFinishTime;
	leg.end = leg.begin + SERVICE;
	drone[i].legs.push_back(leg);
	drone[i].taskFinishTime = leg.end;
}

// 返回 f 分数；不可行时返回 -1。a、b 同时保存取货和送货路线。
double evaluate(int i, int j, double battery, Path &a, Path &b) {
	if (order[j].weight > drone[i].maxLoad)
		return -1;
	a = astar(drone[i].position, order[j].pickup);
	b = astar(order[j].pickup, order[j].delivery);
	double E = a.len + b.len;
	double s = order[j].deadline - now - E / SPEED - 2 * SERVICE;
	if (E >= INF || s < -EPS)
		return -1;
	double back = nearestCharger(order[j].delivery).len;
	if (E + back + RESERVE > battery + EPS)
		return -1;
	return 0.4 / (1 + a.len / 10) + 0.4 / (1 + max(0.0, s) / 30) +
		   0.2 / (1 + E / 30);
}
void assignOrders() {
	while (true) {
		int di = 0, oi = 0;
		double best = -1;
		Path first, second;
		for (int i = 1; i <= N; i++) {
			if (drone[i].state != IDLE && drone[i].state != TO_HOTSPOT)
				continue;
			for (int j = 1; j <= M; j++)
				if (waiting(j)) {
					Path a, b;
					double score = evaluate(i, j, drone[i].battery, a, b);
					if (score > best) {
						best = score;
						di = i;
						oi = j;
						first = a;
						second = b;
					}
				}
		}
		if (di == 0)
			break;
		beginTask(di, DELIVERY);
		fly(di, first);
		service(di, order[oi].pickup);
		fly(di, second);
		service(di, order[oi].delivery);
		order[oi].assigned = true;
		order[oi].takeoff = now;
		cout << "assign " << now << " drone=" << di << " order=" << oi
			 << " finish=" << drone[di].taskFinishTime << '\n';
		addEvent(drone[di].taskFinishTime, FINISH, di, oi);
	}
}

void parkDrones() {
	for (int i = 1; i <= N; i++) {
		if (drone[i].state != IDLE)
			continue;
		bool low = drone[i].battery < 0.3 * drone[i].capacity ||
				   drone[i].battery < RESERVE;
		if (drone[i].battery < drone[i].capacity - EPS) {
			for (int j = 1; j <= M; j++)
				if (waiting(j)) {
					Path a, b;
					if (evaluate(i, j, drone[i].capacity, a, b) >= 0)
						low = true;
				}
		}
		if (low) {
			Path path = nearestCharger(drone[i].position);
			if (path.len >= INF ||
				(path.len > EPS &&
				 path.len + RESERVE > drone[i].battery + EPS)) {
				drone[i].state = GROUNDED;
				continue;
			}
			beginTask(i, TO_CHARGER);
			fly(i, path);
			addEvent(drone[i].taskFinishTime, REACH_CHARGER, i);
			continue;
		}
		if (drone[i].settled)
			continue;
		int target = 0;
		double best = -1;
		Path bestPath;
		for (int k = 1; k <= R; k++) {
			int q = 0, n = 0;
			for (int j = 1; j <= M; j++)
				if (order[j].arrival <= now &&
					order[j].arrival > now - WINDOW &&
					inside(order[j].pickup, k))
					q++;
			for (int j = 1; j <= N; j++)
				if (j != i) {
					if ((drone[j].state == IDLE &&
						 inside(drone[j].position, k)) ||
						(drone[j].state == TO_HOTSPOT && drone[j].target == k))
						n++;
				}
			Path path = astar(drone[i].position, region[k].standby);
			double back = nearestCharger(region[k].standby).len;
			if (path.len + back + RESERVE > drone[i].battery + EPS)
				continue;
			double g = 0.5 * (q / 5.0) + 0.3 / (1 + n / 3.0) +
					   0.2 / (1 + path.len / 10);
			if (g > best) {
				best = g;
				target = k;
				bestPath = path;
			}
		}
		if (target == 0)
			continue;
		if (bestPath.len < EPS) {
			drone[i].settled = true;
			continue;
		}
		beginTask(i, TO_HOTSPOT);
		drone[i].target = target;
		fly(i, bestPath);
		addEvent(drone[i].taskFinishTime, REACH_HOTSPOT, i);
	}
}

void handle(Event e) {
	if (e.type == ARRIVAL) {
		for (int j = 1; j <= N; j++)
			if (drone[j].state == IDLE)
				drone[j].settled = false;
		return;
	}
	int i = e.drone;
	drone[i].legs.clear();
	if (e.type == FINISH) {
		order[e.order].finish = now;
		drone[i].state = IDLE;
	} else if (e.type == REACH_CHARGER) {
		beginTask(i, CHARGING);
		drone[i].taskFinishTime =
			now + max(0.0, drone[i].capacity - drone[i].battery) / CHARGE_RATE;
		addEvent(drone[i].taskFinishTime, CHARGED, i);
	} else {
		drone[i].state = IDLE;
		if (e.type == CHARGED)
			drone[i].battery = drone[i].capacity;
		if (e.type == REACH_HOTSPOT) {
			drone[i].settled = true;
			drone[i].target = 0;
		}
	}
}

int main() {
	ios::sync_with_stdio(false);
	cin.tie(0);
	cout << fixed << setprecision(3);
	// 输入保证合法：格子数 <= MAXV，其余各数量 < MAXN。
	if (!(cin >> X >> Y >> Z >> B >> C >> R >> N >> M >> horizon))
		return 0;
	for (int i = 1; i <= B; i++) {
		int x, y, z;
		cin >> x >> y >> z;
		blocked[id(x, y, z)] = true;
	}
	for (int i = 1; i <= C; i++)
		readPoint(charger[i]);
	for (int i = 1; i <= R; i++) {
		cin >> region[i].xmin >> region[i].xmax >> region[i].ymin >>
			region[i].ymax;
		readPoint(region[i].standby);
	}
	for (int i = 1; i <= N; i++) {
		drone[i].id = i;
		readPoint(drone[i].position);
		cin >> drone[i].maxLoad >> drone[i].battery >> drone[i].capacity;
	}
	for (int j = 1; j <= M; j++) {
		order[j].id = j;
		cin >> order[j].arrival;
		readPoint(order[j].pickup);
		readPoint(order[j].delivery);
		cin >> order[j].deadline >> order[j].weight;
		order[j].takeoff = -1;
		order[j].finish = -1;
		addEvent(order[j].arrival, ARRIVAL, 0, j);
	}
	while (true) {
		// 同一时刻的事件一起处理，再统一进行调度。
		while (!events.empty() && events.top().time == now) {
			Event e = events.top();
			events.pop();
			if (current(e))
				handle(e);
		}
		assignOrders();
		parkDrones();
		while (!events.empty() && !current(events.top()))
			events.pop();
		if (events.empty() || events.top().time > horizon) {
			advance(horizon);
			break;
		}
		advance(events.top().time);
	}
	int arrived = 0, done = 0;
	double total = 0;
	for (int j = 1; j <= M; j++) {
		if (order[j].arrival <= horizon)
			arrived++;
		if (order[j].finish >= 0) {
			done++;
			total += order[j].finish - order[j].arrival;
		}
		cout << "order " << j << " takeoff=" << order[j].takeoff
			 << " finish=" << order[j].finish << '\n';
	}
	cout << "arrived=" << arrived << " completed=" << done
		 << " energy=" << energyUsed
		 << " avg_time=" << (done ? total / done : 0) << '\n';
	return 0;
}
