#include <argparse/argparse.hpp>
#include <cctype>
#include <iostream>
#include <lacam.hpp>

namespace
{
struct GraphInputData
{
  std::vector<int> node_ids;
  std::vector<std::array<int, 3>> node_positions;
  std::vector<std::pair<int, int>> edges;
  std::vector<std::array<int, 3>> agents;  // agent_id, start_node_id, goal_node_id
  int seed = 0;
};

bool starts_with(const std::string &text, const std::string &prefix)
{
  return text.size() >= prefix.size() &&
         text.compare(0, prefix.size(), prefix) == 0;
}

std::vector<std::string> split_csv(const std::string &text)
{
  std::vector<std::string> tokens;
  std::string token;
  for (char c : text) {
    if (c == ',') {
      tokens.push_back(token);
      token.clear();
      continue;
    }
    token.push_back(c);
  }
  tokens.push_back(token);
  return tokens;
}

std::string trim(std::string text)
{
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) {
    text.erase(text.begin());
  }
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) {
    text.pop_back();
  }
  return text;
}

bool load_graph_input(const std::string &path, GraphInputData &out, std::string &error)
{
  std::ifstream file(path);
  if (!file) {
    error = "graph input file not found: " + path;
    return false;
  }

  std::string line;
  while (getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    line = trim(line);
    if (line.empty() || starts_with(line, "#")) continue;

    if (starts_with(line, "seed=")) {
      out.seed = std::stoi(line.substr(5));
      continue;
    }

    if (starts_with(line, "node=")) {
      auto tokens = split_csv(line.substr(5));
      if (tokens.size() < 4) {
        error = "invalid node line: " + line;
        return false;
      }
      out.node_ids.push_back(std::stoi(tokens[0]));
      out.node_positions.push_back(
          {std::stoi(tokens[1]), std::stoi(tokens[2]), std::stoi(tokens[3])});
      continue;
    }

    if (starts_with(line, "edge=")) {
      auto tokens = split_csv(line.substr(5));
      if (tokens.size() < 2) {
        error = "invalid edge line: " + line;
        return false;
      }
      out.edges.emplace_back(std::stoi(tokens[0]), std::stoi(tokens[1]));
      continue;
    }

    if (starts_with(line, "agent=")) {
      auto tokens = split_csv(line.substr(6));
      if (tokens.size() < 3) {
        error = "invalid agent line: " + line;
        return false;
      }
      out.agents.push_back(
          {std::stoi(tokens[0]), std::stoi(tokens[1]), std::stoi(tokens[2])});
      continue;
    }
  }

  if (out.node_ids.empty()) {
    error = "graph input has no nodes";
    return false;
  }
  if (out.agents.empty()) {
    error = "graph input has no agents";
    return false;
  }

  return true;
}

Graph *build_graph(
    const GraphInputData &input,
    std::unordered_map<int, int> &internal_id_by_original_id,
    std::vector<int> &original_id_by_internal_id)
{
  auto *graph = new Graph();
  graph->V.clear();
  graph->U.clear();
  graph->width = 0;
  graph->height = 0;

  internal_id_by_original_id.clear();
  original_id_by_internal_id.clear();
  original_id_by_internal_id.reserve(input.node_ids.size());

  for (size_t idx = 0; idx < input.node_ids.size(); ++idx) {
    const int original_id = input.node_ids[idx];
    if (internal_id_by_original_id.find(original_id) !=
        internal_id_by_original_id.end()) {
      delete graph;
      return nullptr;
    }

    const int internal_id = static_cast<int>(graph->V.size());
    internal_id_by_original_id[original_id] = internal_id;
    original_id_by_internal_id.push_back(original_id);

    const auto &pos = input.node_positions[idx];
    // Existing LaCAM internals rely on x/y only. For 3D inputs we project z into y.
    auto *vertex = new Vertex(internal_id, original_id, pos[0], pos[2]);
    graph->V.push_back(vertex);
  }

  for (const auto &edge : input.edges) {
    auto from_it = internal_id_by_original_id.find(edge.first);
    auto to_it = internal_id_by_original_id.find(edge.second);
    if (from_it == internal_id_by_original_id.end() ||
        to_it == internal_id_by_original_id.end()) {
      delete graph;
      return nullptr;
    }

    auto *from = graph->V[from_it->second];
    auto *to = graph->V[to_it->second];
    if (from == nullptr || to == nullptr) {
      delete graph;
      return nullptr;
    }
    from->neighbor.push_back(to);
  }

  return graph;
}

void write_graph_log(
    const std::string &output_name,
    const std::vector<int> &agent_ids,
    const std::vector<int> &original_id_by_internal_id,
    const Solution &solution,
    bool solved)
{
  std::ofstream log(output_name, std::ios::out);
  log << "agents=" << agent_ids.size() << "\n";
  log << "solver=lacam3-graph\n";
  log << "solved=" << (solved ? 1 : 0) << "\n";
  if (!solved || solution.empty()) {
    return;
  }

  log << "solution=\n";
  for (size_t i = 0; i < agent_ids.size(); ++i) {
    log << agent_ids[i] << ":";
    for (size_t t = 0; t < solution.size(); ++t) {
      auto *v = solution[t][i];
      if (v == nullptr || v->id < 0 ||
          static_cast<size_t>(v->id) >= original_id_by_internal_id.size()) {
        continue;
      }
      log << original_id_by_internal_id[v->id];
      if (t + 1 < solution.size()) log << ",";
    }
    log << "\n";
  }
}
}  // namespace

int main(int argc, char *argv[])
{
  argparse::ArgumentParser program("lacam3-graph", "0.1.0");
  program.add_argument("--graph").help("graph input file").required();
  program.add_argument("-N", "--num")
      .help("number of agents (optional, defaults to all)")
      .default_value(std::string("-1"));
  program.add_argument("-s", "--seed")
      .help("seed")
      .default_value(std::string("0"));
  program.add_argument("-v", "--verbose")
      .help("verbose")
      .default_value(std::string("0"));
  program.add_argument("-t", "--time_limit_sec")
      .help("time limit sec")
      .default_value(std::string("3"));
  program.add_argument("-o", "--output")
      .help("output file")
      .default_value(std::string("./build/result.txt"));

  // solver parameters (kept compatible with main.cpp)
  program.add_argument("--no-all").default_value(false).implicit_value(true);
  program.add_argument("--no-star").default_value(false).implicit_value(true);
  program.add_argument("--random-insert-prob1")
      .default_value(std::string("0.001"));
  program.add_argument("--random-insert-prob2")
      .default_value(std::string("0.01"));
  program.add_argument("--random-insert-init-node")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--no-swap").default_value(false).implicit_value(true);
  program.add_argument("--no-multi-thread")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--pibt-num").default_value(std::string("10"));
  program.add_argument("--no-scatter").default_value(false).implicit_value(true);
  program.add_argument("--scatter-margin").default_value(std::string("10"));
  program.add_argument("--no-refiner").default_value(false).implicit_value(true);
  program.add_argument("--refiner-num").default_value(std::string("4"));
  program.add_argument("--recursive-rate").default_value(std::string("0.2"));
  program.add_argument("--recursive-time-limit")
      .default_value(std::string("1"));
  program.add_argument("--checkpoints-duration")
      .default_value(std::string("5"));

  try {
    program.parse_known_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  const auto verbose = std::stoi(program.get<std::string>("verbose"));
  const auto time_limit_sec =
      std::stoi(program.get<std::string>("time_limit_sec"));
  const auto output_name = program.get<std::string>("output");
  const auto num_agents_limit = std::stoi(program.get<std::string>("num"));
  auto seed = std::stoi(program.get<std::string>("seed"));
  const auto graph_path = program.get<std::string>("graph");

  GraphInputData graph_input;
  std::string parse_error;
  if (!load_graph_input(graph_path, graph_input, parse_error)) {
    std::cerr << parse_error << std::endl;
    return 1;
  }
  if (seed == 0) seed = graph_input.seed;

  std::unordered_map<int, int> internal_id_by_original_id;
  std::vector<int> original_id_by_internal_id;
  auto *graph = build_graph(
      graph_input, internal_id_by_original_id, original_id_by_internal_id);
  if (graph == nullptr) {
    std::cerr << "failed to build graph from input" << std::endl;
    return 1;
  }

  std::vector<std::array<int, 3>> selected_agents;
  if (num_agents_limit > 0 &&
      static_cast<size_t>(num_agents_limit) < graph_input.agents.size()) {
    selected_agents.assign(
        graph_input.agents.begin(),
        graph_input.agents.begin() + num_agents_limit);
  } else {
    selected_agents = graph_input.agents;
  }

  Config starts;
  Config goals;
  std::vector<int> agent_ids;
  starts.reserve(selected_agents.size());
  goals.reserve(selected_agents.size());
  agent_ids.reserve(selected_agents.size());

  for (const auto &entry : selected_agents) {
    const int agent_id = entry[0];
    const int start_original = entry[1];
    const int goal_original = entry[2];
    auto s_it = internal_id_by_original_id.find(start_original);
    auto g_it = internal_id_by_original_id.find(goal_original);
    if (s_it == internal_id_by_original_id.end() ||
        g_it == internal_id_by_original_id.end()) {
      std::cerr << "agent references unknown node id" << std::endl;
      delete graph;
      return 1;
    }

    starts.push_back(graph->V[s_it->second]);
    goals.push_back(graph->V[g_it->second]);
    agent_ids.push_back(agent_id);
  }

  const auto flg_no_all = program.get<bool>("no-all");
  Planner::FLG_SWAP = !program.get<bool>("no-swap") && !flg_no_all;
  Planner::FLG_STAR = !program.get<bool>("no-star") && !flg_no_all;
  Planner::FLG_MULTI_THREAD =
      !program.get<bool>("no-multi-thread") && !flg_no_all;
  Planner::PIBT_NUM =
      flg_no_all ? 1 : std::stoi(program.get<std::string>("pibt-num"));
  Planner::FLG_REFINER = !program.get<bool>("no-refiner") && !flg_no_all;
  Planner::REFINER_NUM = std::stoi(program.get<std::string>("refiner-num"));
  Planner::FLG_SCATTER = !program.get<bool>("no-scatter") && !flg_no_all;
  Planner::SCATTER_MARGIN =
      std::stoi(program.get<std::string>("scatter-margin"));
  Planner::RANDOM_INSERT_PROB1 =
      flg_no_all ? 0
                 : std::stof(program.get<std::string>("random-insert-prob1"));
  Planner::RANDOM_INSERT_PROB2 =
      flg_no_all ? 0
                 : std::stof(program.get<std::string>("random-insert-prob2"));
  Planner::FLG_RANDOM_INSERT_INIT_NODE =
      program.get<bool>("random-insert-init-node") && !flg_no_all;
  Planner::RECURSIVE_RATE =
      flg_no_all ? 0 : std::stof(program.get<std::string>("recursive-rate"));
  Planner::RECURSIVE_TIME_LIMIT =
      flg_no_all
          ? 0
          : std::stof(program.get<std::string>("recursive-time-limit")) * 1000;
  Planner::CHECKPOINTS_DURATION =
      std::stof(program.get<std::string>("checkpoints-duration")) * 1000;

  Instance ins(graph, starts, goals, static_cast<uint>(starts.size()));
  if (!ins.is_valid(verbose)) {
    delete graph;
    return 1;
  }

  const auto deadline = Deadline(time_limit_sec * 1000);
  const auto solution = solve(ins, verbose - 1, &deadline, seed);
  const auto comp_time_ms = deadline.elapsed_ms();

  const bool solved = !solution.empty();
  if (!solved) info(1, verbose, &deadline, "failed to solve");
  if (solved && !is_feasible_solution(ins, solution, verbose)) {
    info(0, verbose, &deadline, "invalid solution");
    delete graph;
    return 1;
  }
  if (solved) print_stats(verbose, &deadline, ins, solution, comp_time_ms);

  write_graph_log(output_name, agent_ids, original_id_by_internal_id, solution, solved);
  delete graph;
  return solved ? 0 : 1;
}
