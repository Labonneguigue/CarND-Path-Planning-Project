#ifndef UTL_H
#define UTL_H

#include <math.h>
#include <string>

namespace utl
{
    // For converting back and forth between radians and degrees.
    constexpr double pi() { return M_PI; }
    double deg2rad(double x) { return x * pi() / 180; }
    double rad2deg(double x) { return x * 180 / pi(); }

    template <typename T>
    T sqr(T x)
    { return x*x; }

    template <typename T>
    T mph2ms(T mph)
    { return mph*0.44704; }

    // Checks if the SocketIO event has JSON data.
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.
    std::string hasData(std::string s) {
        auto found_null = s.find("null");
        auto b1 = s.find_first_of("[");
        auto b2 = s.find_first_of("}");
        if (found_null != std::string::npos) {
            return "";
        } else if (b1 != std::string::npos && b2 != std::string::npos) {
            return s.substr(b1, b2 - b1 + 2);
        }
        return "";
    }

    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
    {

        double closestLen = std::numeric_limits<double>::max();
        int closestWaypoint = 0;

        for(int i = 0; i < maps_x.size(); i++)
        {
            double map_x = maps_x[i];
            double map_y = maps_y[i];
            double dist = distance(x,y,map_x,map_y);
            if(dist < closestLen)
            {
                closestLen = dist;
                closestWaypoint = i;
            }

        }

        return closestWaypoint;

    }

    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
    {

        int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

        double map_x = maps_x[closestWaypoint];
        double map_y = maps_y[closestWaypoint];

        double heading = atan2((map_y-y),(map_x-x));

        double angle = fabs(theta-heading);
        angle = std::min(2 * pi() - angle, angle);

        if( angle > pi()/4 )
        {
            closestWaypoint++;
            if (closestWaypoint == maps_x.size())
            {
                closestWaypoint = 0;
            }
        }

        return closestWaypoint;
    }

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
    {
        int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

        int prev_wp;
        prev_wp = next_wp-1;
        if(next_wp == 0)
        {
            prev_wp  = maps_x.size()-1;
        }

        double n_x = maps_x[next_wp]-maps_x[prev_wp];
        double n_y = maps_y[next_wp]-maps_y[prev_wp];
        double x_x = x - maps_x[prev_wp];
        double x_y = y - maps_y[prev_wp];

        // find the projection of x onto n
        double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        double proj_x = proj_norm*n_x;
        double proj_y = proj_norm*n_y;

        double frenet_d = distance(x_x,x_y,proj_x,proj_y);

        //see if d value is positive or negative by comparing it to a center point

        double center_x = 1000-maps_x[prev_wp];
        double center_y = 2000-maps_y[prev_wp];
        double centerToPos = distance(center_x,center_y,x_x,x_y);
        double centerToRef = distance(center_x,center_y,proj_x,proj_y);

        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }

        // calculate s value
        double frenet_s = 0;
        for(int i = 0; i < prev_wp; i++)
        {
            frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
        }

        frenet_s += distance(0,0,proj_x,proj_y);

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
    std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
    {
        int prev_wp = -1;
        
        while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
            prev_wp++;
        }
        
        int wp2 = (prev_wp+1)%maps_x.size();
        
        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);
        
        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
        
        double perp_heading = heading-pi()/2;
        
        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);
        
        return {x,y};
        
    }

    /**
     Returns the d component in the Frenet coordinates space for a particular lane

     @param double laneNumber Lane number. 0 is left, 1 is center and 2 is right lane
     @return d Frenet component
     */
    double getDFromLane(double laneNumber)
    {
        static const double laneWidth = 4;
        static const double laneBias = 2;
        return (laneBias + (laneWidth * laneNumber));
    }
}

#endif //UTL_H
