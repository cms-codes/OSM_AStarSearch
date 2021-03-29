#include "route_planner.h"
#include <algorithm>
#include <limits>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*this->end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    float f_value = 0.0f;
    current_node->FindNeighbors();

    for ( RouteModel::Node *i : current_node->neighbors )
    {
        if (i->visited == false)
        {
            i->h_value = CalculateHValue(i);
            f_value = i->distance(*current_node);
            i->g_value = current_node->g_value + f_value;
            
            i->visited = true;
            i->parent = current_node;
            this->open_list.emplace_back(i);
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() 
{
    RouteModel::Node *shortest = nullptr;

    // declare lambda for std::sort comparator
    auto open_list_sortLambda = [] (const RouteModel::Node *a, const RouteModel::Node *b) -> bool
    {
        // sort in descending order so we can pop the last element
        return std::isgreater( a->g_value + a->h_value, b->g_value + b->h_value );
    };

    std::sort(this->open_list.begin(), this->open_list.end(), open_list_sortLambda);
    shortest = this->open_list.back();

    this->open_list.pop_back();
    return shortest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *node_i = current_node;
    int counter = 0;

    // loop through the tree until we arrive at the starting node
    while ( node_i != nullptr )
    {
        if (node_i->parent != nullptr) distance += node_i->distance(*node_i->parent);

        path_found.push_back(*node_i);
        node_i = node_i->parent;
    }

    // reverse the order so the path is in order from start to finish
    std::reverse ( path_found.begin(), path_found.end() );
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = this->start_node;
    current_node->parent = nullptr;
    current_node->visited = true;
    this->open_list.push_back(current_node);

    while ( !this->open_list.empty() )
    {

        current_node = this->NextNode();

        if ( current_node == this->end_node )
        {
            this->m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        else
        {
            this->AddNeighbors(current_node);
        }
    }
}