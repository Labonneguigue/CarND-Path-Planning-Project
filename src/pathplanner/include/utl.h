#ifndef UTL_H
#define UTL_H

#include <math.h>
#include <string>

namespace utl
{
    // For converting back and forth between radians and degrees.
    template <typename T>
    constexpr T pi() {
        return M_PI;
    }

    template <typename T>
    constexpr T deg2rad(T x)
    {
        return x * pi<T>() / 180;
    }

    template <typename T>
    constexpr T rad2deg(double x)
    {
        return x * 180 / pi<T>();
    }

    template <typename T>
    constexpr T sqr(T x)
    { return x*x; }

    template <typename T>
    constexpr T mph2ms(T mph)
    { return mph*0.44704; }

    template <typename T>
    constexpr T ms2mph(T mph)
    { return mph/0.44704; }

    template <typename T>
    constexpr T distance(T x1, T y1, T x2, T y2)
    {
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    template <typename T>
    int ClosestWaypoint(T x, T y, const std::vector<T> &maps_x, const std::vector<T> &maps_y)
    {

        T closestLen = std::numeric_limits<T>::max();
        int closestWaypoint = 0;

        for(int i = 0; i < maps_x.size(); i++)
        {
            T map_x = maps_x[i];
            T map_y = maps_y[i];
            T dist = distance(x,y,map_x,map_y);
            if(dist < closestLen)
            {
                closestLen = dist;
                closestWaypoint = i;
            }

        }

        return closestWaypoint;
    }

    template <typename T>
    int NextWaypoint(T x, T y, T theta, const std::vector<T> &maps_x, const std::vector<T> &maps_y)
    {

        int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

        T map_x = maps_x[closestWaypoint];
        T map_y = maps_y[closestWaypoint];

        T heading = atan2((map_y-y),(map_x-x));

        T angle = fabs(theta-heading);
        angle = std::min(2 * pi<T>() - angle, angle);

        if( angle > pi<T>()/4 )
        {
            closestWaypoint++;
            if (closestWaypoint == maps_x.size())
            {
                closestWaypoint = 0;
            }
        }

        return closestWaypoint;
    }

    /** Transform from Cartesian x,y coordinates to Frenet s,d coordinates
     *
     */
    template <typename T>
    std::vector<T> getFrenet(T x, T y, T theta, const std::vector<T> &maps_x, const std::vector<T> &maps_y)
    {
        int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

        int prev_wp;
        prev_wp = next_wp-1;
        if(next_wp == 0)
        {
            prev_wp  = maps_x.size()-1;
        }

        T n_x = maps_x[next_wp]-maps_x[prev_wp];
        T n_y = maps_y[next_wp]-maps_y[prev_wp];
        T x_x = x - maps_x[prev_wp];
        T x_y = y - maps_y[prev_wp];

        // find the projection of x onto n
        T proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        T proj_x = proj_norm*n_x;
        T proj_y = proj_norm*n_y;

        T frenet_d = distance(x_x,x_y,proj_x,proj_y);

        //see if d value is positive or negative by comparing it to a center point

        T center_x = 1000-maps_x[prev_wp];
        T center_y = 2000-maps_y[prev_wp];
        T centerToPos = distance(center_x,center_y,x_x,x_y);
        T centerToRef = distance(center_x,center_y,proj_x,proj_y);

        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }

        // calculate s value
        T frenet_s = 0;
        for(int i = 0; i < prev_wp; i++)
        {
            frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
        }

        frenet_s += distance<T>(0,0,proj_x,proj_y);

        return {frenet_s,frenet_d};

    }

    /**
     Transform from Frenet s,d coordinates to Cartesian x,y

     @param s s component using the Frenet coordinates space (longitudinal)
     @param d d component using the Frenet coordinates space (orthogonal)
     @param maps_s <#maps_s description#>
     @param maps_x <#maps_x description#>
     @param maps_y <#maps_y description#>
     @return <#return value description#>
     */
    template <typename T>
    std::vector<T> getXY(T s, T d, const std::vector<T> &maps_s, const std::vector<T> &maps_x, const std::vector<T> &maps_y)
    {
        int prev_wp = -1;
        
        while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
            prev_wp++;
        }
        
        int wp2 = (prev_wp+1)%maps_x.size();
        
        T heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        T seg_s = (s-maps_s[prev_wp]);
        
        T seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        T seg_y = maps_y[prev_wp]+seg_s*sin(heading);
        
        T perp_heading = heading-pi<T>()/2;
        
        T x = seg_x + d*cos(perp_heading);
        T y = seg_y + d*sin(perp_heading);
        
        return {x,y};
    }

    /**
     Returns the d component in the Frenet coordinates space for a particular lane

     @param double laneNumber Lane number. 0 is left, 1 is center and 2 is right lane
     @return d Frenet component
     */
    template <typename T>
    T getDFromLane(T laneNumber)
    {
        static const T laneWidth = 4;
        static const T laneBias = 2;
        return (laneBias + (laneWidth * laneNumber));
    }

    /**
     Checks whether a car is a specified lane or not

     @param lane Number of the lane I'm in: 0 is left, 1 is center and 2 is right lane
     @param othercar_d Frenet d coordinate of the suspected car
     @return True if the car is in my lane, False otherwise
     
     @tparam T type of the othercar_d frenet coordinate
     */
    template <typename T>
    bool isCarInLane(int lane, T othercar_d)
    {
        static const int halfLaneWidth = 2;
        return ((othercar_d < (getDFromLane(lane) + halfLaneWidth)) &&
                (othercar_d > (getDFromLane(lane) - halfLaneWidth))) ? true : false;
    }
}

#endif //UTL_H
