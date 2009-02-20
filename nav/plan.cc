#include "plan.hh"
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>
#include <algorithm>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

// #include <CGAL/Cartesian.h>
// #include <CGAL/convex_hull_2.h>

const int Nav::Plan::USEFUL;
const int Nav::Plan::NOT_USEFUL;

using namespace std;
using namespace Nav;
using namespace boost;

Plan::Plan() {}
Plan::Plan(PointID start, PointID end, GridGraph const& nav_function)
    : m_start(start), m_end(end), m_nav_function(nav_function) {}

PointID Plan::getStartPoint() const { return m_start; }
PointID Plan::getEndPoint() const { return m_end; }
GridGraph const& Plan::getNavigationFunction() const { return m_nav_function; }

void Plan::setStartPoint(PointID const& p) { m_start = p; }
void Plan::setEndPoint(PointID const& p) { m_end = p; }
void Plan::setNavigationFunction(GridGraph const& nav_function)
{ m_nav_function = nav_function; }

void Plan::clear()
{ corridors.clear(); }

void Plan::removeCorridor(int idx, int replace_by)
{
    corridors.erase(corridors.begin() + idx);
    for (corridor_iterator corridor = corridors.begin(); corridor != corridors.end(); ++corridor)
    {
        Corridor::Connections& connections = corridor->connections;
        Corridor::connection_iterator it = connections.begin();
        while (it != connections.end())
        {
            int const target_idx = it->get<1>();
            if (replace_by == -1 && target_idx == idx)
                connections.erase(it++);
            else
            {
                if (target_idx == idx)
                    it->get<1>() = replace_by;
                else if (target_idx > idx)
                    it->get<1>()--;

                ++it;
            }
        }
    }
}

void Plan::addAdjacentBorders(MedianPoint const& p0, MedianPoint const& p1, set<PointID>& result) const
{
    for (list<PointVector>::const_iterator p0_border_it = p0.borders.begin(); p0_border_it != p0.borders.end(); ++p0_border_it)
    {
        for (PointVector::const_iterator p0_it = p0_border_it->begin(); p0_it != p0_border_it->end(); ++p0_it)
        {
            if (p1.isBorderAdjacent(*p0_it))
                result.insert(*p0_it);
        }
    }
}

void Plan::concat(Plan const& other)
{
    int merge_start = corridors.size();
    copy(other.corridors.begin(), other.corridors.end(), back_inserter(corridors));

    for (vector<Corridor>::iterator it = corridors.begin() + merge_start; it != corridors.end(); ++it)
        for (Corridor::connection_iterator c = it->connections.begin(); c != it->connections.end(); ++c)
            c->get<1>() += merge_start;
}

void Plan::moveConnections(int into_idx, int from_idx)
{
    Corridor::Connections& from = corridors[from_idx].connections;
    Corridor::Connections& into = corridors[into_idx].connections;
    Corridor::Connections::iterator conn_it;
    for (conn_it = from.begin(); conn_it != from.end(); ++conn_it)
    {
        int target_idx = conn_it->get<1>();
        if (target_idx == into_idx)
            continue;

        into.push_back(*conn_it);
        Corridor::Connections& target = corridors[target_idx].connections;
        Corridor::Connections::iterator target_it;
        for (target_it = target.begin(); target_it != target.end(); ++target_it)
        {
            if (target_it->get<1>() == from_idx)
                target_it->get<1>() = into_idx;
        }
    }
}

void Plan::simplify()
{
    vector<int> useful_corridors;
    useful_corridors.resize(corridors.size(), 0);
    useful_corridors[0] = USEFUL;

    size_t original_count = corridors.size();

    markEndpointCorridors(useful_corridors);
    markNullCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);

    markUselessCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);

    mergeSimpleCrossroads();

    // We changed something. Re-run the simplification process.
    if (original_count != corridors.size())
        simplify();
    else if (!m_nav_function.empty())
    {
        removeBackToBackConnections();

        markUselessCorridors(useful_corridors);
        removeUselessCorridors(useful_corridors);
        mergeSimpleCrossroads_directed();
    }
}

void Plan::removeBackToBackConnections()
{
    // Mark, for each corridor, which endpoints are "front" and which are "back"
    // based on the cost. Note that endpoint corridors have either all front or
    // all back (based on wether they are start or end points).

    static const int FRONT_LINE = 0;
    static const int BACK_LINE = 1;
    static const int BIDIR_LINE = 2;
    size_t start_idx = findStartCorridor();
    size_t end_idx   = findEndCorridor();

    typedef map< pair<int, PointID>, int> EndpointTypemap; 
    EndpointTypemap types;

    {
        Corridor& corridor = corridors[start_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();
        for (; it != end; ++it)
            types[ make_pair(start_idx, it->get<0>()) ] = FRONT_LINE;
    }
    {
        Corridor& corridor = corridors[end_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();
        for (; it != end; ++it)
            types[ make_pair(end_idx, it->get<0>()) ] = BACK_LINE;
    }

    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
    {
        if (corridor_idx == start_idx || corridor_idx == end_idx)
            continue;

        Corridor& corridor = corridors[corridor_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();

        // Gather a cost value for the endpoints of each existing connections
        list< tuple<int, PointID, float> > endpoint_costs;
        float min_value = std::numeric_limits<float>::max(),
              max_value = std::numeric_limits<float>::min();
        for (; it != end; ++it)
        {
            PointID target_p = it->get<2>();
            float value = m_nav_function.getValue(target_p.x, target_p.y);

            endpoint_costs.push_back(make_tuple( corridor_idx, it->get<0>(), value));
            min_value = min(value, min_value);
            max_value = max(value, max_value);
        }

        float min_bound = (max_value +  min_value) / 2;
        float max_bound = (max_value + min_value) / 2;
        for (list< tuple<int, PointID, float> >::const_iterator it = endpoint_costs.begin(); it != endpoint_costs.end(); ++it)
        {
            float cost = it->get<2>();

            int type = -1;
            if (cost < min_bound)
                type = FRONT_LINE;
            else if (cost > max_bound)
                type = BACK_LINE;
            else
            {
                cerr << "min_bound = " << min_bound << ", max_bound = " << max_bound << ", cost = " << cost << endl;
                throw std::runtime_error("cost in [min_bound, max_bound]");
            }
            EndpointTypemap::key_type tuple = make_pair(it->get<0>(), it->get<1>());
            EndpointTypemap::iterator type_it = types.find(tuple);
            if (type_it == types.end())
                types[tuple] = type;
            else if (type_it->second != type)
                type_it->second = BIDIR_LINE;
        }
    }

    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();

        while (it != end)
        {
            PointID endpoint = it->get<0>();
            int type = types[ make_pair(corridor_idx, endpoint) ];

            if (type == BACK_LINE)
                connections.erase(it++);
            else
            {
                // Check the direction of the other endpoint. If it is also
                // front, then remove
                int target_idx = it->get<1>();
                PointID target_endpoint = it->get<2>();
                if (types[ make_pair(target_idx, target_endpoint) ] == FRONT_LINE)
                    connections.erase(it++);
                else
                    ++it;
            }
        }
    }
}

void Plan::markNullCorridors(vector<int>& useful)
{
    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        list<PointSet> end_regions = corridor.endRegions();

        if (useful[corridor_idx] == USEFUL || end_regions.size() > 1)
            continue;
        else if (corridor.connections.size() <= 1)
        {
            useful[corridor_idx] = NOT_USEFUL;
        }
        else
        {
            // We still have to check that there is no two corridors that
            // are connected only through this one, i.e. that we don't have
            //
            // A => B => C
            //
            // but no direct A => C connection, in which case we can't remove B
            Corridor::Connections& connections = corridor.connections;
            Corridor::Connections::const_iterator conn_it = connections.begin();
            while (conn_it != connections.end())
            {
                int conn_idx = conn_it->get<1>();
                Corridor& conn_corridor = corridors[conn_idx];

                Corridor::Connections::const_iterator target_it = connections.begin();
                for (; target_it != connections.end(); ++target_it)
                {
                    int target_idx = target_it->get<1>();
                    if (target_idx == conn_idx)
                        continue;
                    if (!conn_corridor.isConnectedTo(target_it->get<1>()))
                        break;
                }

                // increment first, since we may invalidate the iterator by
                // removing the connections
                if (target_it == connections.end())
                {
                    // we can remove this connection
                    corridor.removeConnectionsTo(conn_idx);
                    conn_corridor.removeConnectionsTo(corridor_idx);

                    // we don't know where is the next iterator
                    // (removeConnectionsTo can remove more than one
                    // connection). Just start again ...
                    conn_it = connections.begin();
                }
                else ++conn_it;
            }

            if (corridor.connections.size() <= 1)
            {
                //cerr << "  " << corridor_idx << " is a null corridor" << endl;
                useful[corridor_idx] = NOT_USEFUL;
            }
        }
    }
}

int Plan::findStartCorridor() const
{
    int owner = findCorridorOf(m_start);
    if (owner == -1)
        throw runtime_error("no corridor for start point");
    return owner;
}
int Plan::findEndCorridor() const
{
    int owner = findCorridorOf(m_end);
    if (owner == -1)
        throw runtime_error("no corridor for end point");
    return owner;
}
int Plan::findCorridorOf(PointID const& endp) const
{
    float min_distance = -1;
    int owner = -1;

    for (size_t i = 1; i < corridors.size(); ++i)
    {
        Corridor const& corridor = corridors[i];
        if (!corridor.bbox.isNeighbour(endp))
            continue;

        for (MedianLine::const_iterator median_it = corridor.median.begin(); median_it != corridor.median.end(); ++median_it)
        {
            float d = endp.distanceTo2(median_it->center);
            if (min_distance == -1 || min_distance > d)
            {
                min_distance = d;
                owner = i;
            }
        }
    }
    return owner;
}

void Plan::markEndpointCorridors(vector<int>& useful)
{
    // Mark as useful the corridors that contain endpoints. What we assume here
    // is that there is at most one corridor which contains the endpoints.
    PointID endpoints[2] = { m_start, m_end };
    for (int endp_idx = 0; endp_idx < 2; ++endp_idx)
    {
        PointID endp = endpoints[endp_idx];
        int owner = findCorridorOf(endp);
        if (owner == -1)
            throw std::runtime_error("no owner for endpoint " + lexical_cast<string>(endp));

        useful[owner] = USEFUL;
    }
}

void Plan::markUselessCorridors(vector<int>& useful)
{
    // Now, do a depth-first search. The useful corridors are the ones that
    // helps connecting an endpoint corridor to another endpoint corridor
    vector<int> dfs_stack;
    for (size_t i = 1; i < corridors.size(); ++i)
    {
        if (useful[i] == USEFUL)
        {
            dfs_stack.clear();
            dfs_stack.push_back(i);
            markNextCorridors(dfs_stack, i, useful);
        }
    }

    for (size_t i = 1; i < useful.size(); ++i)
    {
        if (useful[i] == 0)
            useful[i] = NOT_USEFUL;
    }

}

void Plan::removeUselessCorridors(vector<int>& useful)
{
    // Now remove the not useful corridors
    for (size_t i = corridors.size() - 1; i > 0; --i)
    {
        if (useful[i] == NOT_USEFUL)
        {
            //cerr << "  corridor " << i << " is not useful" << endl;
            removeCorridor(i);
            useful.erase(useful.begin() + i);
        }
        //else if (useful[i] == USEFUL)
        //    //cerr << "  corridor " << i << " is useful" << endl;
        //else
        //    //cerr << "  corridor " << i << " is undetermined" << endl;
    }
}

void Plan::mergeSimpleCrossroads_directed()
{
    vector<int> in_connectivity(corridors.size(), 0);
    vector<int> out_connectivity(corridors.size(), 0);

    for (size_t i = 1; i < corridors.size(); ++i)
    {
        set<int> connectivity = corridors[i].connectivity();
        for (set<int>::const_iterator it = connectivity.begin();
                it != connectivity.end(); ++it)
            in_connectivity[*it]++;

        out_connectivity[i] = connectivity.size();
    }

    vector<bool> simple_corridor(corridors.size());
    for (size_t i = 1; i < corridors.size(); ++i) 
        simple_corridor[i] = (out_connectivity[i] <= 1 && in_connectivity[i] <= 1);

    for (int i = 1; i < (int)corridors.size(); ++i)
    {
        if (!simple_corridor[i]) continue;

        int target_idx = corridors[i].connections.front().get<1>();
        if (!simple_corridor[target_idx]) continue;

        Corridor const& source = corridors[i];
        Corridor& target = corridors[target_idx];
        cerr << "merging " << target_idx << " " << i << endl;
        target.merge(source);
        removeCorridor(i, target_idx);
        simple_corridor.erase(simple_corridor.begin() + i);
        --i;
    }
}

void Plan::mergeSimpleCrossroads()
{
    // Finally, remove crossroads that connects only two corridors together (by
    // contrast with those that connect more, which are obviously real
    // crossroads).
    list<PointSet> connection_zones;
    map<PointID, int> ownerships;
    PointSet seen;
    for (size_t i = 1; i < corridors.size(); ++i)
    {
        Corridor::Connections const& connections = corridors[i].connections;
        Corridor::Connections::const_iterator conn_it;
        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            size_t target_idx = conn_it->get<1>();
            if (target_idx < i)
                continue; // already done (connections are symmetric)

            PointID source = conn_it->get<0>();
            PointID target = conn_it->get<2>();
            //cerr << source << " " << target << endl;
            ownerships[source] = i;
            ownerships[target] = target_idx;

            list<PointSet>::iterator source_set = find_if(connection_zones.begin(), connection_zones.end(),
                    bind(&PointSet::count, _1, source));
            list<PointSet>::iterator target_set = find_if(connection_zones.begin(), connection_zones.end(),
                    bind(&PointSet::count, _1, target));

            if (source_set == connection_zones.end())
            {
                if (target_set == connection_zones.end())
                {
                    PointSet new_set;
                    new_set.insert(source);
                    new_set.insert(target);
                    connection_zones.push_back(new_set);
                }
                else
                    target_set->insert(source);
            }
            else if (target_set == connection_zones.end())
                source_set->insert(target);
            else if (source_set != target_set)
            {
                source_set->insert(target_set->begin(), target_set->end());
                connection_zones.erase(target_set);
            }
        }
    }

    //cerr << connection_zones.size() << " crossroads found" << endl;

    // Now that we have clustered the connection points, merge the corridors
    // which are connected by a simple crossroad
    list<PointSet>::const_iterator zone_it;
    for (zone_it = connection_zones.begin(); zone_it != connection_zones.end(); ++zone_it)
    {
        //cerr << *zone_it << endl;

        PointSet const& points = *zone_it;
        set<int> connected;
        for (PointSet::const_iterator p_it = points.begin(); p_it != points.end(); ++p_it)
        {
            connected.insert(ownerships[*p_it]);
            if (connected.size() > 2)
                break;
        }
        if (connected.size() == 2)
        {
            // That is a simple crossroad. Merge the two corridors, update the
            // connections and don't forget to update the ownerships as well --
            // needed for the suite of this loop.
            int into_idx = *connected.begin();
            int from_idx = *(++connected.begin());
            //cerr << "simple: " << into_idx << " and " << from_idx << endl;
            Corridor& into = corridors[into_idx];
            Corridor& from = corridors[from_idx];
            into.merge(from);

            // Remove the connections in +into+ that link to +from+
            { Corridor::Connections& connections = from.connections;
                Corridor::Connections::iterator conn_it;
                for (conn_it = connections.begin(); conn_it != connections.end(); )
                {
                    if (conn_it->get<1>() == from_idx)
                        conn_it = connections.erase(conn_it);
                    else ++conn_it;
                }
            }

            // Update the ownership of points that are in \c from
            {
                map<PointID, int>::iterator owner_it;
                for (owner_it = ownerships.begin(); owner_it != ownerships.end(); ++owner_it)
                {
                    int target_idx = owner_it->second;
                    if (target_idx == from_idx)
                        owner_it->second = into_idx;
                    else if (target_idx > from_idx)
                        --owner_it->second;
                }
            }

            // Move the connections of +target+ into +source+. This
            // does not copy the connections between the two and updates the
            // other corridors
            moveConnections(into_idx, from_idx);

            // And now remove
            removeCorridor(from_idx);
        }
    }
}

int Plan::markNextCorridors(vector<int>& stack, int corridor_idx, vector<int>& useful) const
{
    Corridor const& c = corridors[corridor_idx];
    Corridor::Connections::const_iterator conn_it;

    stack.push_back(corridor_idx);
    for (conn_it = c.connections.begin(); conn_it != c.connections.end(); ++conn_it)
    {
        int target_idx = conn_it->get<1>();
        if (target_idx == 0 || find(stack.begin(), stack.end(), target_idx) != stack.end())
            continue;

        if (useful[target_idx] == USEFUL || markNextCorridors(stack, target_idx, useful) == USEFUL)
            useful[corridor_idx] = USEFUL;
    }

    stack.pop_back();
    return useful[corridor_idx];
}

ostream& Nav::operator << (ostream& io, Plan const& plan)
{
    int const NO_OWNER = plan.corridors.size();
    int const width    = plan.width;
    int const height   = plan.height;

    if (!plan.pixel_map.empty())
    {
        io << "\nPixel map\n";
        io << "  ";
        for (int x = 0; x < width; ++x)
            io << " " << std::setw(2) << x;
        io << endl;
        for (int y = 0; y < height; ++y)
        {
            io << std::setw(2) << y;
            for (int x = 0; x < width; ++x)
            {
                int owner = plan.pixel_map[ x + y * width ];
                if (owner != NO_OWNER)
                    io << " " << std::setw(2) << owner;
                else
                    io << "  -";
            }
            io << endl;
        }
    }

    for (vector<Corridor>::const_iterator it = plan.corridors.begin(); it != plan.corridors.end(); ++it)
    {
        io << "\n==== Corridor " << it - plan.corridors.begin() << "====\n";
        io << *it << endl;
    }
    return io;
}

