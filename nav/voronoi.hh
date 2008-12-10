#ifndef NAV_VORONOI_HH
#define NAV_VORONOI_HH
#include <list>
#include <map>
#include <iosfwd>
#include "point.hh"

namespace Nav
{
    /** Descriptor for one point of the voronoi diagram. This descriptor
     * maintains the association between the point and the associated border
     * points
     */
    struct MedianPoint
    {
        typedef std::list< PointSet > BorderList;

        /** MedianPoint maintains a list of borders, each borders being a set of
         * adjacent points. This means that:
         * <ul>
         * <li> in each element of \c borders, the point form a connected graph
         * <li> there is no connection between two different elements of \c * borders
         * </ul>
         *
         * These contraints are maintained by addBorderPoint and mergeBorders
         */
        BorderList borders;

        /** The distance between the point and the borders. Note that there is
         * one unique distance by construction
         */
        int distance;

        MedianPoint() : distance(0)  {}

        /** Add \c p to the borders. See borders for more details */
        void addBorderPoint(PointID const& p);
        /** Merge the borders of \c p into the ones of \c this */
        void mergeBorders(MedianPoint const& p);

        /** Display the borders in a human-readable form on \c io */
        void displayBorders(std::ostream& io) const;
        /** Returns true if for each border of \c p there is at least one
         * touching border in \c this */
        bool isAdjacent(MedianPoint const& p) const;
    };
    typedef std::map<PointID, MedianPoint> MedianLine;

    void displayMedianLine(std::ostream& io, MedianLine const& skel, int w, int h);
}

#endif

