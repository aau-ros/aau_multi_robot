#ifndef UPDATEENTRY_H
#define UPDATEENTRY_H
#include "ros/console.h"
class UpdateEntry
{
private:
    int _minx,_miny,_maxx,_maxy;
    int _updateNumber;
public:
    UpdateEntry(int updateNumber,int minx,int miny,int maxx,int maxy);
    int getMaxX();
    int getMaxY();
    int getMinX();
    int getMinY();
};

#endif // UPDATEENTRY_H
