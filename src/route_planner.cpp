#include "route_planner.h"
#include <algorithm>
#include <limits>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{

    return node->distance(*this->end_node);

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //std::cout << "Starting FindNeighbors()\n";
    float f_value = 0.0f;
    current_node->FindNeighbors();

    for ( RouteModel::Node *i : current_node->neighbors )
    {
        if (i->visited == false)
        {
            //std::cout << "CalculateHValue(i)\n";
            i->h_value = CalculateHValue(i);
            //std::cout << "distance(i)\n";
            f_value = i->distance(*current_node);
            i->g_value = current_node->g_value + f_value;
            
            i->visited = true;
            //std::cout << "new node in AddNeighbors: " << i << ", with parent " << current_node << "\n";
            i->parent = current_node;

            //std::cout << "push_back(i)\n";
            this->open_list.push_back(i);
        }

    }
    //std::cout << "Finishing AddNeighbors()\n";
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() 
{
    //std::cout << "Starting NextNode()\n";
    RouteModel::Node *shortest = nullptr;

    // sort with a lambda comparison function as a std::sort comparator

    auto open_list_sortLambda = [] (const RouteModel::Node *a, const RouteModel::Node *b) -> bool
    {
        // sort in descending order so we can pop the last element
        return std::isgreater( ( a->g_value + a->h_value ), ( b->g_value + b->h_value ) );
    };

    std::sort(this->open_list.begin(), this->open_list.end(), open_list_sortLambda);
    shortest = this->open_list.back();

    //std::cout << "Current open_list:\n";
    for ( RouteModel::Node *i : this->open_list )
    {
        //std::cout << i << " distance: " << i->h_value + i->g_value << "\n";
    }

    this->open_list.pop_back();

    
    //std::cout << "shortest: " << shortest << " parent " << shortest->parent << "\n";
    return shortest;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *node_i = current_node;
    int counter = 0;

    // TODO: Implement your solution here.

    // loop through the tree until we arrive at the starting node
    while ( node_i != nullptr )
    {
        //std::cout << "begin # " << counter << "\n";
        //std::cout << "node: " << node_i << " parent " << node_i->parent << "\n";
        
        if (node_i->parent != nullptr) 
        {
            //std::cout << "adding distance\n";
            distance += node_i->distance(*node_i->parent);
        }
        //std::cout << "adding obj to path_found \n";
        path_found.push_back(*node_i);
        //std::cout << "increment: ";
        node_i = node_i->parent;
        //std::cout << "done.\n";
        counter++;
        //std::cout << "end.\n";
    }
    
    //distance += path_found.back().distance(*this->start_node);
    //path_found.push_back(*this->start_node);
    //std::cout << "Finished ConstructFinalPath() with " << counter << " nodes.\n";

    // reverse the order so the path is in order from start to finish
    std::reverse ( path_found.begin(), path_found.end() );
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    int counter = 0;
    // TODO: Implement your solution here.

    current_node = this->start_node;
    current_node->parent = nullptr;
    current_node->visited = true;
    this->open_list.push_back(current_node);
    //std::cout << "start_node: " << current_node << " parent: " << this->start_node->parent << "\n";

    //std::cout << "Starting AStarSearch()\n";
    while ( this->open_list.size() > 0 )
    {
        //std::cout << "AStar next node # " << counter << "\n";

        current_node = this->NextNode();
        //this->open_list.clear();
        counter++;

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
    //std::cout << "Finishing AStarSearch() with " << counter << " passes.\n";
    
    //std::cout << "Finished AStarSearch()\n";
}