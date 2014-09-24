#include "updateentry.h"

UpdateEntry::UpdateEntry(int updateNumber, int minx, int miny, int maxx, int maxy)
{
    _updateNumber= updateNumber;
    if(minx > maxx)
        ROS_ERROR("minx > maxx in UpdateEntry()");
    if(miny > maxy)
        ROS_ERROR("miny > maxy in UpdateEntry()");
    _minx = minx;
    _miny = miny;
    _maxx = maxx;
    _maxy = maxy;
}
int UpdateEntry::getMaxX()
{
    return _maxx;
}
int UpdateEntry::getMaxY()
{
    return _maxy;
}
int UpdateEntry::getMinX()
{
    return _minx;
}
int UpdateEntry::getMinY()
{
    return _miny;
}
