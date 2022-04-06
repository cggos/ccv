//
// Created by gordon on 18-1-3.
//

#ifndef CGOCV_NONMAX_SUPPRESSION_H
#define CGOCV_NONMAX_SUPPRESSION_H

#include "cvkit/cv/image.h"

namespace cg {

    class NonMaxSuppression {

        struct Greater
        {
            static bool Compare(int a, int b)
            {
                return a > b;
            }
        };

        struct GreaterEqual
        {
            static bool Compare(int a, int b)
            {
                return a >= b;
            }
        };

    public:

        /**
         * @tparam Score
         * @tparam ReturnType
         * @tparam Collector
         * @param corners
         * @param scores
         * @param nonmax_corners
         * @verbatim
         * This function has been moved from fast_corner.cxx. Look there fast_nonmax_t is templated so you can have either of:
         * 1: A vector of ImageRefs of the nonmax corners
         * 2: A vector of <ImageRef, int> pairs of the corners and their scores.
         * And
         * 1: Non-strict (neighbours of the same score allowed)
         * 2: Strict (must be larger than the neighbours)
         * It's internal, the user-visible functions instantiate it below..
         * @endverbatim
         */
        template<class Score, class ReturnType, class Collector>
        static inline void nonmax_suppression_t(const std::vector<cg::Point2D<int> >& corners, const std::vector<Score>& scores, std::vector<ReturnType> &nonmax_corners)
        {
            nonmax_corners.clear();
            nonmax_corners.reserve(corners.size());

            if(corners.size() < 1)
                return;

            // Find where each row begins
            // (the corners are output in raster scan order). A beginning of -1 signifies
            // that there are no corners on that row.
            int last_row = corners.back().y;
            std::vector<int> row_start(last_row + 1, -1);

            int prev_row = -1;
            for(unsigned int i=0; i< corners.size(); i++)
                if(corners[i].y != prev_row)
                {
                    row_start[corners[i].y] = i;
                    prev_row = corners[i].y;
                }

            //Point above points (roughly) to the pixel above the one of interest, if there
            //is a feature there.
            int point_above = 0;
            int point_below = 0;

            const int sz = (int)corners.size();

            for(int i=0; i < sz; i++)
            {
                Score score = scores[i];
                cg::Point2D<int> pos = corners[i];

                //Check left
                if(i > 0)
                    if(corners[i-1] == pos-cg::Point2D<int>(1,0) && Greater::Compare(scores[i-1], score))
                        continue;

                //Check right
                if(i < (sz - 1))
                    if(corners[i+1] == pos+cg::Point2D<int>(1,0) && Greater::Compare(scores[i+1], score))
                        continue;

                //Check above (if there is a valid row above)
                if(pos.y != 0 && row_start[pos.y - 1] != -1)
                {
                    //Make sure that current point_above is one
                    //row above.
                    if(corners[point_above].y < pos.y - 1)
                        point_above = row_start[pos.y-1];

                    //Make point_above point to the first of the pixels above the current point,
                    //if it exists.
                    for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
                    {}


                    for(int i=point_above; corners[i].y < pos.y && corners[i].x <= pos.x + 1; i++)
                    {
                        int x = corners[i].x;
                        if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Greater::Compare(scores[i], score))
                            goto cont;
                    }

                }

                //Check below (if there is anything below)
                if(pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < sz) //Nothing below
                {
                    if(corners[point_below].y < pos.y + 1)
                        point_below = row_start[pos.y+1];

                    // Make point below point to one of the pixels belowthe current point, if it
                    // exists.
                    for(; point_below < sz && corners[point_below].y == pos.y+1 && corners[point_below].x < pos.x - 1; point_below++)
                    {}

                    for(int i=point_below; i < sz && corners[i].y == pos.y+1 && corners[i].x <= pos.x + 1; i++)
                    {
                        int x = corners[i].x;
                        if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Greater::Compare(scores[i],score))
                            goto cont;
                    }
                }

                nonmax_corners.push_back(Collector::collect(corners[i],scores[i]));

                cont:
                ;
            }
        }

    };

}

#endif //CGOCV_NONMAX_SUPPRESSION_H
