#include <optional>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

float getInput(std::string prompt, float min_input, float max_input);

std::vector<float> getMapCoordinates();

bool badRange(float user_input, float min_input, float max_input);

std::vector<float> getMapCoordinates()
{
    float start_x{};
    float start_y{};
    float end_x{};
    float end_y{};

    std::vector<float> input;

    std::cout << "\nEnter start x and y coordinates (0-100):\n";
    start_x = getInput("x: ", 0, 100);
    input.emplace_back(start_x);
    start_y = getInput("y: ", 0, 100);
    input.emplace_back(start_y);

    std::cout << "Enter destination x and y coordinates (0-100):\n";
    end_x = getInput("x: ", 0, 100);
    input.emplace_back(end_x);
    end_y = getInput("y: ", 0, 100);
    input.emplace_back(end_y);

    return input;
}

float getInput(std::string prompt, float min_input, float max_input)
{
    std::string input_str = "";
    float input_float = 0.0f;
    std::size_t index = 0;

    do
    {
        input_str = "";
        input_float = 0.0f;
        index = 0;
        do
        {
            std::cout << prompt;
            std::getline(std::cin, input_str);
            std::stringstream s(input_str);
            if (sizeof(s) > 0)
            {
                try
                {
                    input_float = std::stof(input_str, &index);
                }
                catch (...)
                {
                    index = -1;
                }
            }
            
        }
        while ( index != input_str.length() );

    }
    while (badRange(input_float, min_input, max_input));

    return input_float;
}

bool badRange(float user_input, float min_input, float max_input)
{
    if ( (std::isless(user_input, min_input) ) || (std::isgreater(user_input, max_input) ) )
        return true;
    else
        return false;
}

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // Collect user input.
    std::vector<float> user_input;

    user_input = getMapCoordinates();

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, user_input[0], user_input[1], user_input[2], user_input[3]};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
