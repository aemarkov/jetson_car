//
// Created by garrus on 11.01.19.
//

#ifndef JETSON_CAR_GRIDCOORD_H
#define JETSON_CAR_GRIDCOORD_H

/**
 * Integer two-dimension coordinate in grid
 */
struct GridCoord
{
    union {
        struct {
            int row, col;
        };
        struct {
            int y, x;
        };
        struct {
            int height, width;
        };
    };

    GridCoord(){}
    GridCoord(int row, int col)
    {
        this->row = row;
        this->col = col;
    }
};

#endif //JETSON_CAR_GRIDCOORD_H
