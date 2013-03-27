#ifndef NAV_TERRAIN_CLASSES_HH
#define NAV_TERRAIN_CLASSES_HH

#include <list>
#include <string>

namespace nav_graph_search
{
    /** This structure is a terrain class as described in a on-disk map */
    struct TerrainClass
    {
	TerrainClass() : in(0), out(0), cost(-1), margin(0)
	{
	}
	
        int in; //! The class value on disk
        int out; //! The class value to be used in TraversabilityMap
        float cost; //! The cost of this class
        float margin; //! The needed geometrical margin so that to use the maximum speed
        std::string name; //! The class name

        static std::list<TerrainClass> load(std::string const& path);
	
	/**
	 * The class number is the value that 
	 * expected in the envire::TraversabilityMap
	 * */
	size_t getNumber() const
	{
	    return out;
	}
	
	/**
	 * Cost for traversing terrain belonging to this class
	 * */
	double getCost() const
	{
	    return cost;
	}

	/**
	 * Name of the terrain class
	 * */
	std::string getName() const
	{
	    return name;
	}
	
	/**
	 * Returns wether this class is traversable.
	 * */
	bool isTraversable() const
	{
	    return cost > 0;
	}
	
    };

    typedef std::list<TerrainClass> TerrainClasses;

}

#endif

