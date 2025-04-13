// Ce programme de path-finding utilise la méthode du "travel salesman problem" (tsp) :
//	il s'agit de créer une boucle passant par tous les points en parcourant la plus petite distance possible
// Les points d'un graphe sont inscrit dans un fichier noeuds.yaml
// les points que l'on veut relié avec le tsp sont écrit dans le fichier objectifs.yaml
// Ce programme lie les deux fichiers, utilise le graphe pour créer une matrice de distances entre les différents objectifs souhaités
// puis le tsp renvoie l'ordre des points qu'il faut suivre pour parcourir la plus petite distance
// Le chemin final est envoyé dans un topic "tsp_path" pour être utiliser plus tard

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <algorithm>

using namespace std;

// Stucture pour stocker les noeuds
struct Noeud {
    string nom;
    double x, y;
    vector<string> voisins;
};

// Fonction pour calculer la distance entre les noeuds à partir des coordonnées
double distance(const Noeud& a, const Noeud& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

//Algorithme de Dijkstra pour trouver le plus court chemin entre 2 sommets
unordered_map<string, double> dijkstra(
    const unordered_map<string, Noeud>& graph,
    const string& start,
    unordered_map<string, unordered_map<string, double>>& precomputed) {

    using P = pair<double, string>;
    priority_queue<P, vector<P>, greater<P>> pq;
    unordered_map<string, double> dist;

    for (const auto& p : graph) dist[p.first] = numeric_limits<double>::infinity();
    dist[start] = 0;
    pq.emplace(0.0, start);

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;

        for (const auto& v : graph.at(u).voisins) {
            double alt = dist[u] + distance(graph.at(u), graph.at(v));
            if (alt < dist[v]) {
                dist[v] = alt;
                pq.emplace(alt, v);
            }
        }
    }

    precomputed[start] = dist;
    return dist;
}

vector<int> solve_tsp(const vector<vector<double>>& dist) {
    const int N = dist.size();
    const int FULL_MASK = (1 << N);
    const double INF = numeric_limits<double>::infinity();
    vector<vector<double>> dp(FULL_MASK, vector<double>(N, INF));
    vector<vector<int>> parent(FULL_MASK, vector<int>(N, -1));
    dp[1][0] = 0;
    
    // Teste tous les trajets possible
    for (int mask = 1; mask < FULL_MASK; ++mask) {
        for (int u = 0; u < N; ++u) {
            if (!(mask & (1 << u))) continue;
            for (int v = 0; v < N; ++v) {
                if (mask & (1 << v)) continue;
                int next_mask = mask | (1 << v);
                double new_cost = dp[mask][u] + dist[u][v];
                if (new_cost < dp[next_mask][v]) {
                    dp[next_mask][v] = new_cost;
                    parent[next_mask][v] = u;
                }
            }
        }
    }
    
    // Cherche quel trajet a le cout le plus faible
    double best_cost = INF;
    int last = -1;
    for (int i = 1; i < N; ++i) {
        double cost = dp[FULL_MASK - 1][i] + dist[i][0];
        if (cost < best_cost) {
            best_cost = cost;
            last = i;
        }
    }
    
    // Construit un vecteur avec le trajet selectionné précédement
    vector<int> path;
    int mask = FULL_MASK - 1;
    while (last != -1) {
        path.push_back(last);
        int temp = parent[mask][last];
        mask ^= (1 << last);
        last = temp;
    }

    path.push_back(0);
    reverse(path.begin(), path.end());
    return path;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tsp_solver");
    ros::NodeHandle nh;
    // Publisher pour le résultat
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("tsp_path", 10);
    ros::Rate loop_rate(1);
    
    // Les chemins des fichiers .yaml ont été rentré manuelement car c'est la seule manière qui fonctiionnait.
    // Il faut les modifier si le programme est utiliser depuis un autre ordinateur
    string noeuds_path = "/home/thomas/catkin_ws/src/thomas/config/noeuds.yaml";
    string objectifs_path = "/home/thomas/catkin_ws/src/thomas/config/objectifs.yaml";
    YAML::Node noeuds_yaml = YAML::LoadFile(noeuds_path);
    YAML::Node objectifs_yaml = YAML::LoadFile(objectifs_path);

    unordered_map<string, Noeud> noeud_map;
    unordered_map<string, int> index_global_map;
    vector<string> objectifs;
    
    // Chargement des noeuds
    int idx = 0;
    for (const auto& item : noeuds_yaml) {
        Noeud n;
        n.nom = item.first.as<string>();
        n.x = item.second["x"].as<double>();
        n.y = item.second["y"].as<double>();
        if (item.second["voisins"]) {
            for (const auto& v : item.second["voisins"]) {
                n.voisins.push_back(v.as<string>());
            }
        }
        noeud_map[n.nom] = n;
        index_global_map[n.nom] = idx++;
    }
    
    // Chargement des objectifs
    for (const auto& obj : objectifs_yaml["objectifs"]) {
        objectifs.push_back(obj.as<string>());
    }
    int n = objectifs.size();
    if (n == 0) {
        ROS_ERROR("Aucune ville sélectionnée !");
        return 1;
    }

    // Crée la matrice de distanc entre chaque objectif
    unordered_map<string, unordered_map<string, double>> shortest_paths;
    for (const auto& from : objectifs) {
        dijkstra(noeud_map, from, shortest_paths);
    }
    vector<vector<double>> dist(n, vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            dist[i][j] = shortest_paths[objectifs[i]][objectifs[j]];

    // résout le tsp
    vector<int> chemin = solve_tsp(dist);

    // Nettoyage doublons
    vector<int> cleaned_path;
    for (int i = 0; i < chemin.size(); ++i) {
        if (i == 0 || chemin[i] != chemin[i - 1]) {
            cleaned_path.push_back(chemin[i]);
        }
    }

    // Ajouter retour au point de départ
    cleaned_path.push_back(cleaned_path.front());

    // Affichage
    cout << "Ordre optimal des villes : ";
    for (int i : cleaned_path)
        cout << objectifs[i] << " -> ";
    cout << "(retour)" << endl;

    // Publication avec indices globaux YAML, ces indices correspondent aux identifiants du fichier goals.yaml utilisé dans d'autre noeuds
    std_msgs::Int32MultiArray msg;
    for (int i : cleaned_path) {
        string ville = objectifs[i];
        msg.data.push_back(index_global_map[ville]);
    }

    while (ros::ok()) {
        pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}

