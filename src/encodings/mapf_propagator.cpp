#include "mapf_propagator.hpp"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>

using namespace std;

/****************************/
// MARK: construction
/****************************/

MapfPropagator::MapfPropagator(const MapfTegSpec& spec) : S(spec)
{
	A = S.agents;
	V = S.vertices;
	L = S.layers;
	N = S.num_vars;

	node_agent.assign(N + 1, -1);
	node_vertex.assign(N + 1, -1);
	node_time.assign(N + 1, -1);
	agents_at_start.assign(V + 1, 0);

	for (int a = 0; a < A; a++)
	{
		for (int v = 0; v < V; v++)
		{
			int k = a * V + v;
			if (!S.first_var[k])
				continue;
			agents_at_start[v + 1]++;
			for (int t = S.first_t[k]; t <= S.last_t[k]; t++)
			{
				int x = S.first_var[k] + (t - S.first_t[k]);
				assert(x >= 1 && x <= N);
				node_agent[x] = a;
				node_vertex[x] = v;
				node_time[x] = t;
			}
		}
	}

	for (int v = 0; v < V; v++)
		agents_at_start[v + 1] += agents_at_start[v];
	agents_at_list.resize(agents_at_start[V]);
	{
		vector<int> fill(agents_at_start.begin(), agents_at_start.end() - 1);
		for (int a = 0; a < A; a++)
			for (int v = 0; v < V; v++)
				if (S.first_var[a * V + v])
					agents_at_list[fill[v]++] = a;
	}

	if (S.single)
	{
		layer_start.assign((size_t)A * L + 1, 0);
		for (int x = 1; x <= N; x++)
			layer_start[(size_t)node_agent[x] * L + node_time[x] + 1]++;
		for (size_t i = 0; i < (size_t)A * L; i++)
			layer_start[i + 1] += layer_start[i];
		layer_list.resize(N);
		vector<int> fill(layer_start.begin(), layer_start.end() - 1);
		for (int x = 1; x <= N; x++)
			layer_list[fill[(size_t)node_agent[x] * L + node_time[x]]++] = x;
	}

	agent_last.assign(A, 0);
	for (int x = 1; x <= N; x++)
		agent_last[node_agent[x]] = max(agent_last[node_agent[x]], node_time[x]);
	ltrue.assign((size_t)A * L, 0);
	frontier.assign(A, 0);

	val.assign(N + 1, 0);
	dead.assign(N + 1, 0);
	reason_off.assign(N + 1, -1);
	occ.assign((size_t)V * L, 0);
}

/****************************/
// MARK: propagation rules
/****************************/

// Queue -m, explained by the clause (-m, o1, o2, o3); the o's are false now.
void MapfPropagator::kill(int m, int o1, int o2, int o3)
{
	if (!m || dead[m] || val[m] < 0)
		return;
	dead[m] = 1;
	undo.push_back({UNDO_DEAD, m, 0});
	reason_off[m] = (int)pool.size();
	pool.push_back(2 + (o2 != 0) + (o3 != 0));
	pool.push_back(-m);
	pool.push_back(o1);
	if (o2)
		pool.push_back(o2);
	if (o3)
		pool.push_back(o3);
	outq.push_back(-m);
}

// Agent a moves from -> to between t and t+1 (both nodes true). A different
// agent must not move to -> from at the same time.
void MapfPropagator::swap_move(int a, int from, int to, int t, int lit_from, int lit_to)
{
	int c = occupant(to, t);
	if (c >= 0 && c != a)
	{
		stats.kills_swap++;
		kill(var_at(c, from, t + 1), -lit_from, -lit_to, -var_at(c, to, t));
	}
	c = occupant(from, t + 1);
	if (c >= 0 && c != a)
	{
		stats.kills_swap++;
		kill(var_at(c, to, t), -lit_from, -lit_to, -var_at(c, from, t + 1));
	}
}

void MapfPropagator::on_true(int n)
{
	const int a = node_agent[n], v = node_vertex[n], t = node_time[n];

	size_t lk = (size_t)a * L + t;
	if (!ltrue[lk])
	{
		ltrue[lk] = n;
		undo.push_back({UNDO_LTRUE, (int)lk, 0});
		if (frontier[a] == t)
		{
			int f = t;
			while (f <= agent_last[a] && ltrue[(size_t)a * L + f])
				f++;
			undo.push_back({UNDO_FRONTIER, a, frontier[a]});
			frontier[a] = f;
		}
	}

	size_t cell = (size_t)v * L + t;
	if (!occ[cell])
	{
		occ[cell] = a + 1;
		undo.push_back({UNDO_OCC, (int)cell, 0});
	}

	// vertex conflicts, and the pebble rule on the previous and next timestep
	for (int i = agents_at_start[v]; i < agents_at_start[v + 1]; i++)
	{
		int b = agents_at_list[i];
		if (b == a)
			continue;
		int m = var_at(b, v, t);
		if (m && !dead[m] && val[m] >= 0)
		{
			stats.kills_vertex++;
			kill(m, -n);
		}
		if (S.pebble)
		{
			m = var_at(b, v, t - 1);
			if (m && !dead[m] && val[m] >= 0)
			{
				stats.kills_pebble++;
				kill(m, -n);
			}
			m = var_at(b, v, t + 1);
			if (m && !dead[m] && val[m] >= 0)
			{
				stats.kills_pebble++;
				kill(m, -n);
			}
		}
	}

	// at most one position per agent and timestep
	if (S.single)
	{
		size_t k = (size_t)a * L + t;
		for (int i = layer_start[k]; i < layer_start[k + 1]; i++)
		{
			int m = layer_list[i];
			if (m != n && !dead[m] && val[m] >= 0)
			{
				stats.kills_layer++;
				kill(m, -n);
			}
		}
	}

	if (S.pebble)	// pebble motion subsumes swapping conflicts
		return;

	// (1) n completes a move of agent a, forbid the opposite move for others
	for (int d = 1; d < 5; d++)
	{
		int u = S.nbr[v * 5 + d];
		if (u < 0)
			continue;
		if (occupant(u, t - 1) == a)
			swap_move(a, u, v, t - 1, var_at(a, u, t - 1), n);
		if (occupant(u, t + 1) == a)
			swap_move(a, v, u, t, n, var_at(a, u, t + 1));
	}

	// (2) n is one end of the opposite move of an established move of agent c
	int c_prev = occupant(v, t - 1);	// c moves v -> x between t-1 and t
	int c_next = occupant(v, t + 1);	// c moves x -> v between t and t+1
	for (int d = 1; d < 5; d++)
	{
		int x = S.nbr[v * 5 + d];
		if (x < 0)
			continue;
		if (c_next >= 0 && c_next != a && occupant(x, t) == c_next)
		{
			stats.kills_swap++;
			kill(var_at(a, x, t + 1), -n, -var_at(c_next, x, t), -var_at(c_next, v, t + 1));
		}
		if (c_prev >= 0 && c_prev != a && occupant(x, t) == c_prev)
		{
			stats.kills_swap++;
			kill(var_at(a, x, t - 1), -n, -var_at(c_prev, v, t - 1), -var_at(c_prev, x, t));
		}
	}
}

/****************************/
// MARK: IPASIR-UP callbacks
/****************************/

void MapfPropagator::notify_assignment(const vector<int>& lits)
{
	for (int lit : lits)
	{
		int x = abs(lit);
		if (x > N)
			continue;
		int8_t s = lit > 0 ? 1 : -1;
		if (val[x] == s)	// root literals are notified again after some restarts
			continue;
		assert(!val[x]);
		val[x] = s;
		undo.push_back({UNDO_VAL, x, 0});
		if (s > 0)
		{
			stats.notified_true++;
			if (!dead[x])	// otherwise the queued -x turns into a conflict
				on_true(x);
		}
		else
			stats.notified_false++;
	}
}

void MapfPropagator::notify_new_decision_level()
{
	marks.push_back({undo.size(), outq.size(), pool.size()});
}

void MapfPropagator::notify_backtrack(size_t new_level)
{
	stats.backtracks++;
	while (marks.size() > new_level)
	{
		Mark m = marks.back();
		marks.pop_back();
		while (undo.size() > m.undo)
		{
			Undo u = undo.back();
			undo.pop_back();
			if (u.type == UNDO_VAL)
				val[u.idx] = 0;
			else if (u.type == UNDO_DEAD)
				dead[u.idx] = 0;
			else if (u.type == UNDO_OCC)
				occ[u.idx] = 0;
			else if (u.type == UNDO_LTRUE)
				ltrue[u.idx] = 0;
			else
				frontier[u.idx] = u.old;
		}
		outq.resize(m.outq);
		pool.resize(m.pool);
	}
	if (head > outq.size())
		head = outq.size();
	reason_cur = reason_end = -1;
}

int MapfPropagator::cb_propagate()
{
	while (head < outq.size())
	{
		int lit = outq[head++];
		if (val[abs(lit)] == (lit > 0 ? 1 : -1))	// already known to the solver
			continue;
		stats.propagated++;
		return lit;
	}
	return 0;
}

// Extend the path of one agent: from its last fixed position, choose the free
// successor closest to the goal (largest last timestep).
int MapfPropagator::cb_decide()
{
	if (!S.decide)
		return 0;
	int start = 0, end = A;
	if (S.decide == 2)	// the incomplete agent with the shortest fixed prefix
	{
		int best = -1;
		for (int a = 0; a < A; a++)
			if (frontier[a] <= agent_last[a] && (best < 0 || frontier[a] < frontier[best]))
				best = a;
		if (best < 0)
			return 0;
		start = best;
		end = best + 1;
	}
	for (int a = start; a < end; a++)
	{
		int f = frontier[a];
		if (f > agent_last[a] || f == 0)
			continue;
		int prev = ltrue[(size_t)a * L + f - 1];
		if (!prev)
			continue;
		int v = node_vertex[prev];
		int best = 0, best_last = -1;
		for (int d = 0; d < 5; d++)
		{
			int u = S.nbr[v * 5 + d];
			if (u < 0)
				continue;
			int m = var_at(a, u, f);
			if (!m || val[m] || dead[m])
				continue;
			int lt = S.last_t[(size_t)a * V + u];
			if (lt > best_last)
			{
				best = m;
				best_last = lt;
			}
		}
		if (best)
		{
			stats.decisions++;
			return best;
		}
	}
	return 0;
}

int MapfPropagator::cb_add_reason_clause_lit(int propagated_lit)
{
	if (reason_cur < 0)
	{
		int x = abs(propagated_lit);
		assert(dead[x] && reason_off[x] >= 0);
		assert(pool[reason_off[x] + 1] == propagated_lit);
		reason_cur = reason_off[x] + 1;
		reason_end = reason_cur + pool[reason_off[x]];
		stats.reasons++;
	}
	if (reason_cur < reason_end)
		return pool[reason_cur++];
	reason_cur = reason_end = -1;
	return 0;
}

bool MapfPropagator::cb_check_found_model(const vector<int>& model)
{
	stats.model_checks++;
	if (!find_violation(model))
		return true;
	// Every violation should have been propagated during search already.
	stats.model_rejects++;
	cerr << "c mapf propagator: rejected a model, adding the violated clause" << endl;
	ext_pos = 0;
	ext_pending = true;
	return false;
}

bool MapfPropagator::cb_has_external_clause(bool& is_forgettable)
{
	is_forgettable = false;
	return ext_pending;
}

int MapfPropagator::cb_add_external_clause_lit()
{
	if (ext_pos < ext.size())
		return ext[ext_pos++];
	ext_pending = false;
	ext.clear();
	ext_pos = 0;
	return 0;
}

// Independent check of all constraints owned by the propagator on a full
// model. On a violation the violated clause is stored in ext.
bool MapfPropagator::find_violation(const vector<int>& model)
{
	vector<int> cell((size_t)V * L, 0);	// a true node at (v,t)
	vector<int> layer;					// a true node of (a,t), single mode only
	if (S.single)
		layer.assign((size_t)A * L, 0);
	vector<int> trues;

	for (int lit : model)
		if (lit > 0 && lit <= N)
			trues.push_back(lit);

	for (int x : trues)
	{
		int a = node_agent[x], v = node_vertex[x], t = node_time[x];
		int& y = cell[(size_t)v * L + t];
		if (y && node_agent[y] != a)
		{
			ext = {-x, -y};
			return true;
		}
		y = x;
		if (S.single)
		{
			int& z = layer[(size_t)a * L + t];
			if (z && z != x)
			{
				ext = {-x, -z};
				return true;
			}
			z = x;
		}
	}

	for (int x : trues)
	{
		int a = node_agent[x], v = node_vertex[x], t = node_time[x];
		if (S.pebble)
		{
			int y = t > 0 ? cell[(size_t)v * L + t - 1] : 0;
			if (y && node_agent[y] != a)
			{
				ext = {-x, -y};
				return true;
			}
			continue;
		}
		if (t + 1 >= L)
			continue;
		for (int d = 1; d < 5; d++)	// a moves v -> w, b moves w -> v
		{
			int w = S.nbr[v * 5 + d];
			if (w < 0)
				continue;
			int y = cell[(size_t)w * L + t + 1];
			if (!y || node_agent[y] != a)
				continue;
			int z = cell[(size_t)w * L + t];
			if (!z || node_agent[z] == a)
				continue;
			int zz = cell[(size_t)v * L + t + 1];
			if (zz && node_agent[zz] == node_agent[z])
			{
				ext = {-x, -y, -z, -zz};
				return true;
			}
		}
	}
	return false;
}
