#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &(model.FindClosestNode(start_x, start_y));
    this->end_node = &(model.FindClosestNode(end_x, end_y));
    this->start_node->g_value = 0.0f;
    this->start_node->h_value = CalculateHValue(this->start_node);
    this->start_node->visited = true;
    open_list.push_back(this->start_node);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node)); 
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors){
        if (neighbor->visited == false){     
            neighbor->parent = current_node;
            neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
            neighbor->h_value = neighbor->distance(*(this->end_node));
            neighbor->visited = true;
            this->open_list.push_back(neighbor);
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    const auto comparison_function = [](const RouteModel::Node *left, const RouteModel::Node *right){
        const auto left_f = left->g_value + left->h_value;
        const auto right_f = right->g_value + right->h_value;
        return left_f > right_f;
        };

    std::sort(open_list.begin(), open_list.end(), comparison_function);
    auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    this->distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != this->start_node){
        path_found.push_back(*current_node);
        this->distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent; 
    }
    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());
    this->distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    while (current_node != this->end_node && !open_list.empty()){
        current_node = NextNode();
        AddNeighbors(current_node);
    }

    m_Model.path = ConstructFinalPath(current_node);

}