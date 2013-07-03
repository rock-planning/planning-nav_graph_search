#include "terrain_classes.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace std;
using namespace nav_graph_search;

std::list<TerrainClass> TerrainClass::load(std::string const& path)
{
    list<TerrainClass> classes;
    ifstream class_file(path.c_str());

    if (class_file.fail())
        throw std::runtime_error("cannot load " + path);

    while (!class_file.eof())
    {
        char line[256];
        class_file.getline(line, 256);

        if (line[0] == '#' || !*line)
            continue;

        istringstream class_desc(line);
        TerrainClass klass;
        class_desc >> klass.in >> klass.out >> klass.cost >> klass.margin >> klass.name;
        classes.push_back(klass);
    }

    return classes;
}

std::string TerrainClass::toString(std::list<TerrainClass> classes) {
    std::stringstream ss;
    std::list<TerrainClass>::iterator it = classes.begin();
    ss << "Terrain classes, format: in out cost margin name" << std::endl;
    for(; it != classes.end(); ++it) {
        ss << it->in << " " << it->out << " " << it->cost << " " << it->margin <<
                " " << it->name << std::endl;
    }
    return ss.str();
}

